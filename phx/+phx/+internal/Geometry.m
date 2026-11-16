classdef Geometry
%phx.internal.Geometry PHX geometric Library

%   Copyright 1998-2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    methods (Static)
        function [Mass, I0, Icm, cm] = meshMass(vertices, faces, rho)
        %meshMass Mass and inertia tensor of a closed triangle mesh.
        %
        %   [Mass, I0] = meshMass(vertices, faces, rho) integrates the solid
        %   bounded by the mesh at density rho; I0 is the inertia tensor about
        %   the origin of the vertex coordinates. With rho = 1, Mass is the
        %   volume.
        %
        %   [Mass, I0, Icm, cm] = meshMass(___) also returns the centre of
        %   mass cm (1x3) and the inertia tensor Icm about it.

            a = vertices(faces(:, 1), :);
            b = vertices(faces(:, 2), :);
            c = vertices(faces(:, 3), :);

            % Signed volume of each tetrahedron
            sv = dot(a, cross(b, c, 2), 2) / 6;
            Mass  = abs(sum(sv)) * rho;

            % Center of mass: weighted average of the centers of the tetrahedra
            cm = sum(((a + b + c) / 4) .* sv, 1) / sum(sv);

            % Matrix quadratic moments: sum_k sv(k) * A_k' * M * A_k
            %   where A_k = [a(k, :); b(k, :); c(k, :)]  and  M = ones(3) + eye(3)
            M = ones(3) + eye(3);   % weight matrix
            S = zeros(3);
            for k = 1:size(faces, 1)
                A  = [a(k, :); b(k, :); c(k, :)];
                S  = S + sv(k) * (A' * M * A);
            end
            S = sign(sum(sv)) * S / 20;

            % Inertia tensor at the origin: I0_ij = rho*(trace(S)*delta_ij - S_ij)
            I0 = rho * (trace(S) * eye(3) - S);

            % Shift to the center of mass using Steiner's theorem
            Icm = I0 - Mass * (dot(cm, cm) * eye(3) - cm' * cm);
        end

        function [points, dV] = voxelize(vertices, faces, resolution)
        %voxelize Deterministic interior sampling of a closed triangle mesh.
        %
        %   [points, dV] = voxelize(vertices, faces, resolution) covers the
        %   axis-aligned bounding box of the mesh with a regular grid of
        %   resolution^3 cell centers and keeps the points lying inside the
        %   mesh. points is an Nx3 matrix of interior points and dV is the
        %   volume represented by each of them (total mesh volume divided
        %   by the number of interior points), so N*dV equals the exact
        %   signed-tetrahedron volume of the mesh. The grid depends only on
        %   the mesh and the resolution, so the result is fully reproducible.

            [bmin, bmax] = bounds(vertices);
            span = bmax - bmin;
            if any(span <= 0)
                points = zeros(0, 3);
                dV = 0;
                return
            end

            % Cell centers of a resolution^3 grid over the bounding box
            cellSize = span/resolution;
            x = bmin(1) + ((1:resolution) - 0.5)*cellSize(1);
            y = bmin(2) + ((1:resolution) - 0.5)*cellSize(2);
            z = bmin(3) + ((1:resolution) - 0.5)*cellSize(3);
            [gx, gy, gz] = ndgrid(x, y, z);
            grid = [gx(:) gy(:) gz(:)];

            inside = phx.internal.Geometry.pointsInMesh(grid, vertices, faces);
            points = grid(inside, :);

            volume = phx.internal.Geometry.meshMass(vertices, faces, 1);
            dV = volume/max(size(points, 1), 1);
        end

        function inside = pointsInMesh(points, vertices, faces)
        %pointsInMesh Inside test of points against a closed triangle mesh.
        %
        %   inside = pointsInMesh(points, vertices, faces) returns a logical
        %   column marking the points (Nx3) that lie inside the mesh. A ray
        %   is cast from every point in a fixed skewed direction and the
        %   crossing parity is counted (Moller-Trumbore), so non-convex
        %   meshes work too. Open meshes give undefined results.

            % Fixed non-axis-aligned direction avoids grazing axis-aligned
            % edges and keeps the test deterministic
            dir = [0.2810846 0.5389028 0.7940817];
            dir = dir/norm(dir);

            v0 = vertices(faces(:, 1), :);
            e1 = vertices(faces(:, 2), :) - v0;
            e2 = vertices(faces(:, 3), :) - v0;

            count = zeros(size(points, 1), 1);
            for k = 1:size(faces, 1)
                h = cross(dir, e2(k, :));
                det = e1(k, :)*h';
                if abs(det) < 1e-12
                    continue % ray parallel to the triangle
                end
                s = points - v0(k, :);
                u = (s*h')/det;
                q = [s(:, 2)*e1(k, 3) - s(:, 3)*e1(k, 2), ...
                     s(:, 3)*e1(k, 1) - s(:, 1)*e1(k, 3), ...
                     s(:, 1)*e1(k, 2) - s(:, 2)*e1(k, 1)];
                v = (q*dir')/det;
                t = (q*e2(k, :)')/det;
                count = count + (u >= 0 & v >= 0 & u + v <= 1 & t > 0);
            end
            inside = mod(count, 2) == 1;
        end

        function [V, N] = switchZAxis(axis, V, N)
            switch axis
                case "x"
                    V = circshift(V, [0 1]);
                    if nargin == 3
                        N = circshift(N, [0 1]);
                    end
                case "y"
                    V = circshift(V, [0 2]);
                    if nargin == 3
                        N = circshift(N, [0 2]);
                    end
            end
        end

        function [V, N, F, T] = triBox(Size)
            % Vertexes
            V = repmat(([0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1] - 0.5).*Size, [3, 1]);

            % Normal vectors
            N = [0 0 -1; 0 0 -1; 0 0 -1; 0 0 -1; 0 0 1; 0 0 1; 0 0 1; 0 0 1;...
                -1 0 0; 1 0 0; 1 0 0; -1 0 0; -1 0 0; 1 0 0; 1 0 0; -1 0 0;...
                0 -1 0; 0 -1 0; 0 1 0; 0 1 0; 0 -1 0; 0 -1 0; 0 1 0; 0 1 0];

            % Texture coordinates
            T = [0 0; 1 0; 1 1; 0 1; 0 1; 1 1; 1 0; 0 0;...  % bottom/top
                1 1; 0 1; 1 1; 0 1; 1 0; 0 0; 1 0; 0 0;...  % right/left
                0 1; 1 1; 0 1; 1 1; 0 0; 1 0; 0 0; 1 0];    % front/back

            % Face indexes
            %    bottom         front                right               back                 left               top
            F = [1 3 4; 1 2 3;  17 21 22; 22 18 17;  10 14 11; 11 14 15;  19 23 20; 20 23 24;  12 16 9; 9 16 13;  5 8 6; 6 8 7];
        end

        function [V, N, F] = quadBox(Size)
            V = ([0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 1; 1 0 1; 1 1 1; 0 1 1] -0.5).*Size;
            F = [6 2 1 5; 6 7 3 2; 7 8 4 3; 8 5 1 4; 1 2 3 4; 8 7 6 5];
            N = [0 -1 0; 1 0 0; 0 1 0; -1 0 0; 0 0 -1; 0 0 1];
        end

        function [V, N, F] = icosphere(Subdivision)
            % Basic geometry
            fi = (1 + sqrt(5))/2;
            V = [0 +1 +fi; 0 +1 -fi; 0 -1 +fi; 0 -1 -fi];
            V = [V; circshift(V, [0 -1]); circshift(V, [0 -2])];

            % Faces
            F = [1 9 3; 9 6 3; 6 8 3; 8 10 3; 10 1 3; ...
                 7 2 5; 5 2 11; 11 2 4; 4 2 12; 12 2 7; ...
                 1 7 5; 5 9 1; 9 5 11; 11 6 9; 6 11 4; ...
                 4 8 6; 8 4 12; 12 10 8; 10 12 7; 7 1 10];

            % Subdivision
            for j = 2:Subdivision
                N = length(F);
                F2 = zeros(4*N, 3);
                V2 = zeros(3*N, 3);
                off = length(V);
                for i = 1:N
                    f1 = F(i, :);
                    v1 = V(f1, :);
                    v2 = [v1(1, :) + v1(2, :); v1(2, :) + v1(3, :); v1(1, :) + v1(3, :)]./2;
                    id = (i*3 - 2):i*3;
                    V2(id, :) = v2;
                    k = off + i*3;
                    f12 = [f1 (k - 2):k];
                    f2 = [f12(1) f12(4) f12(6); f12(4) f12(2) f12(5); f12(6) f12(5) f12(3); f12(6) f12(4) f12(5)];
                    id = (i*4 - 3):i*4;
                    F2(id, :) = f2;
                end
                F = F2;
                V = [V; V2];
            end

            % Normalization and scale
            V = V./sqrt(sum(V.^2, 2));
            N = V;
        end

        function [V, F] = geodesicSphere(Frequency)
            % Unit sphere of the regular icosahedron with each edge cut into
            % Frequency parts and each face into Frequency^2 triangles,
            % projected on the sphere; the vertices are shared and the faces
            % counter-clockwise seen from outside
            fi = (1 + sqrt(5))/2;
            C = [0 +1 +fi; 0 +1 -fi; 0 -1 +fi; 0 -1 -fi];
            C = [C; circshift(C, [0 -1]); circshift(C, [0 -2])];
            T = [1 9 3; 9 6 3; 6 8 3; 8 10 3; 10 1 3; ...
                 7 2 5; 5 2 11; 11 2 4; 4 2 12; 12 2 7; ...
                 1 7 5; 5 9 1; 9 5 11; 11 6 9; 6 11 4; ...
                 4 8 6; 8 4 12; 12 10 8; 10 12 7; 7 1 10];

            % Grid of one face, point (i, j) at A + i/n*(B - A) + j/n*(C - A),
            % and its triangles pointing up and down, wound as the face
            n = Frequency;
            [i, j] = ndgrid(0:n);
            in = i + j <= n;
            id = zeros(n + 1);
            id(in) = 1:nnz(in);
            i = i(in);
            j = j(in);
            [a, b] = ndgrid(0:n - 1);
            up = a + b <= n - 1;
            down = a + b <= n - 2;
            at = @(p, q) reshape(id(sub2ind([n + 1, n + 1], p + 1, q + 1)), [], 1);
            G = [at(a(up), b(up)) at(a(up) + 1, b(up)) at(a(up), b(up) + 1); ...
                at(a(down) + 1, b(down)) at(a(down) + 1, b(down) + 1) at(a(down), b(down) + 1)];

            % Each face of the icosahedron turned outwards and filled with
            % the grid; the vertices on its edges are merged afterwards
            m = numel(i);
            g = size(G, 1);
            V = zeros(20*m, 3);
            F = zeros(20*g, 3);
            for k = 1:20
                P = C(T(k, :), :);
                if dot(cross(P(2, :) - P(1, :), P(3, :) - P(1, :)), sum(P, 1)) < 0
                    P = P([1 3 2], :);
                end
                V((k - 1)*m + (1:m), :) = P(1, :) + i/n.*(P(2, :) - P(1, :)) + j/n.*(P(3, :) - P(1, :));
                F((k - 1)*g + (1:g), :) = (k - 1)*m + G;
            end
            [V, ~, map] = uniquetol(V, 1e-9, "ByRows", true);
            F = map(F);
            V = V./sqrt(sum(V.^2, 2));
        end

        function H = gridHexes(n)
            % Corner indices of the cells of a grid of n(1) x n(2) x n(3)
            % cells whose points are numbered with the first axis fastest;
            % the corners of a cell are numbered as the bits of their steps
            % along the three axes, plus 1
            [i, j, k] = ndgrid(1:n(1), 1:n(2), 1:n(3));
            first = sub2ind(n + 1, i(:), j(:), k(:));
            dy = n(1) + 1;
            dz = dy*(n(2) + 1);
            H = first + [0, 1, dy, 1 + dy, dz, 1 + dz, dy + dz, 1 + dy + dz];
        end

        function T = hexTetrahedra(H)
            % Six tetrahedra of each hexahedron (rows of H, corners numbered
            % as gridHexes does) around its diagonal from corner 1 to corner
            % 8, of positive volume when the axes of the numbering are
            % right-handed. Every face is cut along the diagonal from its
            % lowest to its highest corner, so neighbours numbered along the
            % same directions share their face diagonals.
            split = [1 2 4 8; 1 6 2 8; 1 4 3 8; 1 3 7 8; 1 5 6 8; 1 7 5 8];
            T = reshape(H(:, split), [], 4);
        end

        function [V, T, ends] = quadraticTetrahedra(V, T)
            % Kx10 quadratic tetrahedra of Kx4 ones: a vertex in the middle
            % of each edge, appended to V, the corners first and then the
            % edges 12 23 13 14 24 34; ends lists the corners of the edges in
            % the order of the appended vertices
            ends = sort([reshape(T(:, [1 2 1 1 2 3]), [], 1) reshape(T(:, [2 3 3 4 4 4]), [], 1)], 2);
            [ends, ~, j] = unique(ends, "rows");
            T = [T size(V, 1) + reshape(j, [], 6)];
            V = [V; (V(ends(:, 1), :) + V(ends(:, 2), :))/2];
        end

        function volume = tetrahedraVolumes(V, T)
            % Volumes of Kx4 tetrahedra, or of Kx10 quadratic ones (corners,
            % then the edges 12 23 13 14 24 34) whose edges may be curved,
            % integrated exactly by a rule of degree 3; positive whatever
            % the order of the corners
            if size(T, 2) == 4
                a = V(T(:, 1), :);
                volume = abs(dot(cross(V(T(:, 2), :) - a, V(T(:, 3), :) - a, 2), V(T(:, 4), :) - a, 2))/6;
                return
            end
            points = [1/4 1/4 1/4; 1/6 1/6 1/6; 1/2 1/6 1/6; 1/6 1/2 1/6; 1/6 1/6 1/2];
            weights = [-2/15 3/40 3/40 3/40 3/40];
            pairs = [1 2; 2 3; 1 3; 1 4; 2 4; 3 4];
            dL = [-1 -1 -1; eye(3)];
            volume = zeros(size(T, 1), 1);
            for p = 1:5
                % Derivatives of the ten shape functions at the point
                L = [1 - sum(points(p, :)), points(p, :)];
                dN = zeros(10, 3);
                for c = 1:4
                    dN(c, :) = (4*L(c) - 1)*dL(c, :);
                end
                for e = 1:6
                    dN(4 + e, :) = 4*(L(pairs(e, 2))*dL(pairs(e, 1), :) + L(pairs(e, 1))*dL(pairs(e, 2), :));
                end
                J = zeros(size(T, 1), 3, 3);
                for n = 1:10
                    J = J + V(T(:, n), :).*reshape(dN(n, :), 1, 1, 3);
                end
                d = J(:, 1, 1).*(J(:, 2, 2).*J(:, 3, 3) - J(:, 2, 3).*J(:, 3, 2)) ...
                    - J(:, 1, 2).*(J(:, 2, 1).*J(:, 3, 3) - J(:, 2, 3).*J(:, 3, 1)) ...
                    + J(:, 1, 3).*(J(:, 2, 1).*J(:, 3, 2) - J(:, 2, 2).*J(:, 3, 1));
                volume = volume + weights(p)*d;
            end
            volume = abs(volume);
        end

        function [V, T] = cubedCylinder(nc, m, nz, a, radii)
            % Cylinder of unit radius and unit height along z, centred at the
            % origin, filled with tetrahedra: the disk of the equator of
            % cubedBall (a square core of half side a with nc cells along
            % each side, and four blocks of m layers of cells between its
            % sides and the circle, whose points are spaced at equal angles)
            % stacked in nz layers. The cells are split by hexTetrahedra; in
            % each block the direction along the side follows its global
            % axis, the second one z and the third points outwards, so the
            % faces of all neighbouring cells match. The tetrahedra are
            % turned to a positive volume.
            %
            % With radii = [r1 r2] the radius changes linearly from r1 at
            % the bottom to r2 at the top: a truncated cone, or a sharp one
            % when a radius is 0. The end layer of a sharp cone collapses
            % into its apex, where the tetrahedra left without volume are
            % dropped; each of its cells keeps two, a pyramid.
            arguments
                nc
                m
                nz
                a
                radii (1, 2) double = [1 1]
            end
            g = linspace(-1, 1, nc + 1);
            h = linspace(-0.5, 0.5, nz + 1);
            [x, y, z] = ndgrid(g, g, h);
            P = cell(1, 5);
            H = cell(1, 5);
            P{1} = [a*x(:) a*y(:) z(:)];
            H{1} = phx.internal.Geometry.gridHexes([nc nc nz]);

            % Blocks on the sides +-x and +-y: the normal axis, then the one
            % along the side
            [u, v, w] = ndgrid(g, h, linspace(0, 1, m + 1));
            hexes = phx.internal.Geometry.gridHexes([nc nz m]);
            order = [1 2; 2 1];
            offset = size(P{1}, 1);
            b = 1;
            for k = 1:2
                for side = [-1 1]
                    inner = zeros(numel(u), 2);
                    inner(:, order(k, :)) = [repmat(side, numel(u), 1) u(:)];
                    outer = zeros(numel(u), 2);
                    outer(:, order(k, :)) = [repmat(side, numel(u), 1) tan(pi/4*u(:))];
                    outer = outer./sqrt(sum(outer.^2, 2));
                    b = b + 1;
                    P{b} = [(1 - w(:)).*a.*inner + w(:).*outer, v(:)];
                    H{b} = offset + hexes;
                    offset = offset + numel(u);
                end
            end
            V = vertcat(P{:});
            V(:, 1:2) = V(:, 1:2).*(radii(1) + (radii(2) - radii(1))*(V(:, 3) + 0.5));
            T = phx.internal.Geometry.hexTetrahedra(vertcat(H{:}));

            % The points the core and the blocks share merged, the end layer
            % of a sharp cone into one, and the tetrahedra that lost a corner
            % so dropped
            [V, ~, map] = uniquetol(V, 1e-12, "ByRows", true, "DataScale", 1);
            T = map(T);
            S = sort(T, 2);
            T = T(all(diff(S, 1, 2) > 0, 2), :);
            e = V(T(:, 1), :);
            negative = dot(cross(V(T(:, 2), :) - e, V(T(:, 3), :) - e, 2), V(T(:, 4), :) - e, 2) < 0;
            T(negative, [2 3]) = T(negative, [3 2]);
        end

        function [V, F] = ringSurface(profile, spacing)
            % Closed triangle mesh of the surface of revolution about z of
            % a profile of [r z] points, from the axis back to the axis, or a
            % closed loop (its last point the first) for a hollow solid,
            % counter-clockwise in the (r, z) plane, so that the solid lies
            % on its left; its vertices spaced about evenly. Each segment of
            % the profile is cut into parts about spacing(1) long, each point
            % becomes a ring of vertices about spacing(end) apart (a point on
            % the axis a single vertex), and neighbouring rings are joined by
            % a band of triangles, counter-clockwise seen from outside.
            along = spacing(1);
            around = spacing(end);

            % The profile cut into parts; a part point too near the axis for
            % a ring of three vertices is left out, so that the vertices do
            % not crowd towards an apex
            points = profile(1, :);
            for k = 2:size(profile, 1)
                parts = max(1, round(norm(profile(k, :) - profile(k - 1, :))/along));
                t = (1:parts)'/parts;
                part = profile(k - 1, :) + t.*(profile(k, :) - profile(k - 1, :));
                inner = (1:parts)' < parts & round(2*pi*part(:, 1)/around) < 3;
                points = [points; part(~inner, :)]; %#ok<AGROW>
            end

            % Neighbouring points, the last one joined to the first in a
            % closed loop, but not two points on the axis
            n = numel(points(:, 1));
            pairs = [(1:n - 1)' (2:n)'];
            if norm(points(end, :) - points(1, :)) <= 1e-12*max(abs(profile(:)))
                n = n - 1;
                points(end, :) = [];
                pairs(end, 2) = 1;
            end
            onAxis = points(:, 1) < 1e-12*max(profile(:, 1));
            pairs(all(onAxis(pairs), 2), :) = [];

            % A ring of vertices for each point, turned by half a step on
            % every other ring
            rings = cell(n, 1);
            angles = cell(n, 1);
            V = zeros(0, 3);
            for k = 1:n
                r = points(k, 1);
                if onAxis(k)
                    a = 0;
                else
                    c = max(3, round(2*pi*r/around));
                    a = 2*pi*((0:c - 1)' + mod(k, 2)/2)/c;
                end
                rings{k} = size(V, 1) + (1:numel(a))';
                angles{k} = a;
                V = [V; r*cos(a) r*sin(a) repmat(points(k, 2), numel(a), 1)]; %#ok<AGROW>
            end

            % Bands between neighbouring rings, merged by angle
            F = zeros(0, 3);
            for p = 1:size(pairs, 1)
                A = rings{pairs(p, 1)};
                B = rings{pairs(p, 2)};
                if isscalar(A) || isscalar(B)
                    % A fan to a vertex on the axis
                    ring = A;
                    tip = B;
                    if isscalar(A)
                        ring = B;
                        tip = A;
                    end
                    F = [F; ring circshift(ring, -1) repmat(tip, numel(ring), 1)]; %#ok<AGROW>
                    continue
                end
                a = angles{pairs(p, 1)};
                b = angles{pairs(p, 2)};
                [~, j0] = min(abs(angle(exp(1i*(b - a(1))))));
                order = mod(j0 - 1 + (0:numel(b))', numel(b)) + 1;
                bu = unwrap(b(order));
                bu = bu - 2*pi*round((bu(1) - a(1))/(2*pi));
                au = [a; a(1) + 2*pi];
                ia = [A; A(1)];
                ib = B(order);
                i = 1;
                j = 1;
                while i < numel(au) || j < numel(bu)
                    if j == numel(bu) || (i < numel(au) && au(i + 1) <= bu(j + 1))
                        F = [F; ia(i) ia(i + 1) ib(j)]; %#ok<AGROW>
                        i = i + 1;
                    else
                        F = [F; ia(i) ib(j + 1) ib(j)]; %#ok<AGROW>
                        j = j + 1;
                    end
                end
            end

            % Each triangle turned to face away from the solid: the outward
            % normal of its profile segment is the tangent turned clockwise
            segment = zeros(size(F, 1), 1);
            count = 0;
            for p = 1:size(pairs, 1)
                A = rings{pairs(p, 1)};
                B = rings{pairs(p, 2)};
                m = numel(A) + numel(B);
                if isscalar(A) || isscalar(B)
                    m = max(numel(A), numel(B));
                end
                segment(count + (1:m)) = p;
                count = count + m;
            end
            tangent = points(pairs(segment, 2), :) - points(pairs(segment, 1), :);
            C = (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :))/3;
            radial = C(:, 1:2)./max(sqrt(sum(C(:, 1:2).^2, 2)), eps);
            outward = [tangent(:, 2).*radial, -tangent(:, 1)];
            normal = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
            inward = dot(normal, outward, 2) < 0;
            F(inward, [2 3]) = F(inward, [3 2]);
        end

        function [V, T] = polarTube(nr, nt, nz, bore, thickness, height)
            % Tube wall along z, centred at the origin, filled with
            % tetrahedra: a polar grid of nr cells across the wall, nt around
            % it and nz along it. The bore radius changes linearly from
            % bore(1) at the bottom to bore(2) at the top, the wall keeps its
            % thickness. The cells are split by hexTetrahedra with the
            % directions around, along and outwards, the same in every cell,
            % so neighbours share their face diagonals across the seam too. A
            % bore closed at an end (radius 0) collapses its inner edge there
            % onto the axis, where the tetrahedra left without volume are
            % dropped. The tetrahedra are turned to a positive volume.
            [i, j, k] = ndgrid(0:nt - 1, 0:nz, 0:nr);
            angle = 2*pi*i(:)/nt;
            z = height*(j(:)/nz - 0.5);
            r = bore(1) + (bore(2) - bore(1))*(z/height + 0.5) + thickness*k(:)/nr;
            V = [r.*cos(angle) r.*sin(angle) z];

            % Corners of each cell, numbered as the bits of their steps
            % around, along and outwards, plus 1; around wraps
            [i, j, k] = ndgrid(0:nt - 1, 0:nz - 1, 0:nr - 1);
            index = @(di, dj, dk) 1 + mod(i(:) + di, nt) + nt*((j(:) + dj) + (nz + 1)*(k(:) + dk));
            H = [index(0, 0, 0) index(1, 0, 0) index(0, 1, 0) index(1, 1, 0) ...
                index(0, 0, 1) index(1, 0, 1) index(0, 1, 1) index(1, 1, 1)];
            T = phx.internal.Geometry.hexTetrahedra(H);

            % A closed bore merges its points on the axis; the tetrahedra
            % that lost a corner so are dropped
            [V, ~, map] = uniquetol(V, 1e-12, "ByRows", true, "DataScale", max([bore + thickness, height]));
            T = map(T);
            S = sort(T, 2);
            T = T(all(diff(S, 1, 2) > 0, 2), :);
            e = V(T(:, 1), :);
            negative = dot(cross(V(T(:, 2), :) - e, V(T(:, 3), :) - e, 2), V(T(:, 4), :) - e, 2) < 0;
            T(negative, [2 3]) = T(negative, [3 2]);
        end

        function [V, T] = cubedHalfBall(nc, kc, m, a)
            % Upper half of the unit ball (z >= 0) filled with tetrahedra as
            % cubedBall fills a ball: a core of half side a, nc cells across
            % it and kc up to its top, and blocks of m layers of cells
            % reaching the sphere, four on its sides and one on its top. Its
            % flat face matches the end of cubedCylinder with the same nc, m
            % and a, so the two join into a capsule. The tetrahedra are
            % turned to a positive volume.
            g = linspace(-1, 1, nc + 1);
            up = linspace(0, 1, kc + 1);
            [x, y, z] = ndgrid(g, g, up);
            P = cell(1, 6);
            H = cell(1, 6);
            P{1} = a*[x(:) y(:) z(:)];
            H{1} = phx.internal.Geometry.gridHexes([nc nc kc]);
            offset = size(P{1}, 1);

            % Blocks on the sides +-x and +-y (along the side, then up) and
            % on the top (along x, then y); the third direction outwards
            b = 1;
            blocks = {[1 2 3], -1, up; [1 2 3], 1, up; [2 1 3], -1, up; [2 1 3], 1, up; [3 1 2], 1, g};
            for k = 1:size(blocks, 1)
                [order, side, second] = blocks{k, :};
                [u, v, w] = ndgrid(g, second, linspace(0, 1, m + 1));
                inner = zeros(numel(u), 3);
                inner(:, order) = [repmat(side, numel(u), 1) u(:) v(:)];
                outer = zeros(numel(u), 3);
                outer(:, order) = [repmat(side, numel(u), 1) tan(pi/4*u(:)) tan(pi/4*v(:))];
                outer = outer./sqrt(sum(outer.^2, 2));
                b = b + 1;
                P{b} = (1 - w(:)).*a.*inner + w(:).*outer;
                H{b} = offset + phx.internal.Geometry.gridHexes([nc numel(second) - 1 m]);
                offset = offset + numel(u);
            end
            V = vertcat(P{:});
            T = phx.internal.Geometry.hexTetrahedra(vertcat(H{:}));

            % The points the core and the blocks share merged
            [V, ~, map] = uniquetol(V, 1e-12, "ByRows", true, "DataScale", 1);
            T = map(T);
            e = V(T(:, 1), :);
            negative = dot(cross(V(T(:, 2), :) - e, V(T(:, 3), :) - e, 2), V(T(:, 4), :) - e, 2) < 0;
            T(negative, [2 3]) = T(negative, [3 2]);
        end

        function [V, T] = cubedBall(nc, m, a)
            % Ball of unit radius filled with tetrahedra: a cube core of half
            % side a with nc cells along each edge, and six blocks of m
            % layers of cells between its faces and the sphere, whose
            % surface points form an equiangular cubed sphere. The cells are
            % split by hexTetrahedra; in each block the two directions along
            % the face follow the global axes and the third points outwards,
            % so the faces of all neighbouring cells match. The tetrahedra
            % are turned to a positive volume.
            g = linspace(-1, 1, nc + 1);
            [x, y, z] = ndgrid(g, g, g);
            P = cell(1, 7);
            H = cell(1, 7);
            P{1} = a*[x(:) y(:) z(:)];
            H{1} = phx.internal.Geometry.gridHexes([nc nc nc]);

            % Blocks on the faces +-x, +-y, +-z: the normal axis, then the
            % two along the face
            [u, v, w] = ndgrid(g, g, linspace(0, 1, m + 1));
            hexes = phx.internal.Geometry.gridHexes([nc nc m]);
            order = [1 2 3; 2 1 3; 3 1 2];
            offset = size(P{1}, 1);
            b = 1;
            for k = 1:3
                for side = [-1 1]
                    inner = zeros(numel(u), 3);
                    inner(:, order(k, :)) = [repmat(side, numel(u), 1) u(:) v(:)];
                    outer = zeros(numel(u), 3);
                    outer(:, order(k, :)) = [repmat(side, numel(u), 1) tan(pi/4*u(:)) tan(pi/4*v(:))];
                    outer = outer./sqrt(sum(outer.^2, 2));
                    b = b + 1;
                    P{b} = (1 - w(:)).*a.*inner + w(:).*outer;
                    H{b} = offset + hexes;
                    offset = offset + numel(u);
                end
            end
            V = vertcat(P{:});
            T = phx.internal.Geometry.hexTetrahedra(vertcat(H{:}));

            % The points the core and the blocks share merged
            [V, ~, map] = uniquetol(V, 1e-12, "ByRows", true, "DataScale", 1);
            T = map(T);
            e = V(T(:, 1), :);
            negative = dot(cross(V(T(:, 2), :) - e, V(T(:, 3), :) - e, 2), V(T(:, 4), :) - e, 2) < 0;
            T(negative, [2 3]) = T(negative, [3 2]);
        end

        function [V, N, F, T] = rock(ASize, Roundness)
            % Basic geometry
            fi = (1 + sqrt(5))/2;
            V = [0 +1 +fi; 0 +1 -fi; 0 -1 +fi; 0 -1 -fi];
            V = [V; circshift(V, [0 -1]); circshift(V, [0 -2])];

            % Faces
            F = [1 9 3; 9 6 3; 6 8 3; 8 10 3; 10 1 3; ...
                 7 2 5; 5 2 11; 11 2 4; 4 2 12; 12 2 7; ...
                 1 7 5; 5 9 1; 9 5 11; 11 6 9; 6 11 4; ...
                 4 8 6; 8 4 12; 12 10 8; 10 12 7; 7 1 10];

            % Normalization and scale
            V = V./sqrt(sum(V.^2, 2));
            f = F';
            N = V(f(:), :);
            N = (N(1:3:end, :) + N(2:3:end, :) + N(3:3:end, :))/3;
            V = V.*ASize/2;

            % Deformation
            V(1:4:end) = V(1:4:end)*Roundness;

            % Texture coordinates: each face carries its own corners and
            % takes the box projection along its undeformed normal. The
            % flat faces hide the jumps at the edges; the radius bounds the
            % deformed vertices, so the coordinates stay within the image.
            V = V(f(:), :);
            F = reshape(1:numel(f), 3, [])';
            [V, F, T] = phx.internal.Geometry.boxUV(V, F, ASize, Normals = N);
        end

        function [V, F, T, idx] = boxUV(V, F, Span, Options)
        %boxUV Box-projected texture coordinates of a triangle mesh.
        %
        %   [V, F, T] = boxUV(V, F, Span) gives each face the image projected
        %   along the coordinate axis closest to its normal, unmirrored when
        %   seen from outside. The image spans a cube of side Span centred
        %   at the origin, the same on all six sides. A vertex shared by
        %   faces projected along different axes is split, so V and F come
        %   back with the duplicated vertices and T holds one row per
        %   vertex of the new V.
        %
        %   [V, F, T, idx] = boxUV(___) also returns the rows of the input V
        %   the new vertices come from, so per-vertex data such as normals
        %   follow with N(idx, :).
        %
        %   [___] = boxUV(___, Center = c) centres the cube at c (1x3).
        %
        %   [___] = boxUV(___, Normals = FN) selects the axes by the face
        %   normals FN (one row per face) instead of the geometric normals
        %   of counterclockwise faces.

            arguments
                V (:, 3) double
                F (:, 3) double
                Span (1, 1) double {mustBePositive}
                Options.Center (1, 3) double = [0 0 0]
                Options.Normals double = []
            end

            FN = Options.Normals;
            if isempty(FN)
                FN = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
            end

            % Side of the box for every face: 1-3 = +x +y +z, 4-6 = -x -y -z
            [~, k] = max(abs(FN), [], 2);
            negative = FN(sub2ind(size(FN), (1:size(FN, 1))', k)) < 0;
            side = k + 3*negative;

            % One vertex per original vertex and side
            corners = reshape(F', [], 1);
            [~, first, F] = unique([corners repelem(side, 3, 1)], "rows", "stable");
            idx = corners(first);
            V = V(idx, :);
            F = reshape(F, 3, [])';
            vertexSide = side(ceil(first/3));

            T = zeros(size(V, 1), 2);
            for s = 1:6
                axis = zeros(1, 3);
                axis(mod(s - 1, 3) + 1) = 1 - 2*(s > 3);
                up = [0 0 1];
                if s == 3 || s == 6
                    up = [0 1 0];
                end
                right = cross(-axis, up);
                in = vertexSide == s;
                T(in, :) = 0.5 + (V(in, :) - Options.Center)*[right' -up']/Span;
            end
        end

        function [V, N, F, T] = revolution(ZX, Segments, beginCap, endCap)
            if beginCap
                ZX = [ZX(1, :).*[1 0]; ZX(1, :); ZX];
            end
            if endCap
                ZX = [ZX; ZX(end, :); ZX(end, :).*[1 0]];
            end

            % Construct vertices
            v = [ZX(:, 2) ZX(:, 2)*0 ZX(:, 1)];

            % Construct normals
            dv = diff(v);
            n = [dv(1, :); dv(1:end - 1, :) + dv(2:end, :); dv(end, :)];
            n = [n(:, 3) n(:, 2) -n(:, 1)];
            n = n./sqrt(sum(n.^2, 2));

            % Construct texture coordinates
            lv = cumsum(sqrt(sum(dv.^2, 2)));
            lv = [0; lv/lv(end)];
            t = [lv*0 1 - lv];

            % Construct faces for one stripe
            r = size(n, 1);
            f = [0:r - 2; 1:r - 1; r:2*r - 2]';
            f = [f; f + [r 0 1]] + 1;

            % Complete geometry
            nv = size(v, 1);
            nf = size(f, 1);
            V = zeros(nv*Segments, 3);
            V(1:nv, :) = v;
            N = zeros(nv*Segments, 3);
            N(1:nv, :) = n;
            T = zeros(nv*Segments, 2);
            T(1:nv, :) = t;
            F = zeros(nf*Segments, 3);
            i = 0;
            for alfa = [1:Segments - 1 0]*2*pi/Segments
                i = i + 1;
                s = sin(-alfa);
                c = cos(-alfa);
                M = [c s 0; -s c 0; 0 0 1];
                V(nv*i+1:nv*i+nv, :) = v*M;
                N(nv*i+1:nv*i+nv, :) = n*M;
                T(nv*i+1:nv*i+nv, :) = t + [i/Segments 0];
                F(nf*i-nf+1:nf*i, :) = f + (i - 1)*r;
            end

            % The rings advance clockwise seen from +z; run u the other way
            % so that the texture reads unmirrored from outside
            T(:, 1) = 1 - T(:, 1);

            % Remove degenerated faces. A face is degenerate only when two of
            % its vertices coincide, which is what the zero-radius rings of
            % caps, poles and closed tips produce. The tolerance is relative
            % to the profile so that it stays a test of coincidence at any
            % scale instead of culling small but valid faces.
            tol = 1e-9*max([max(ZX, [], 1) - min(ZX, [], 1), eps]);
            v1 = V(F(:, 1), :);
            v2 = V(F(:, 2), :);
            v3 = V(F(:, 3), :);
            b12 = sum(abs(v1 - v2), 2) < tol;
            b23 = sum(abs(v2 - v3), 2) < tol;
            b31 = sum(abs(v3 - v1), 2) < tol;
            b = or(or(b12, b23), b31);
            F(b, :) = [];
        end

        function [V, N, F, T] = extrusion(Spine, Scale, Profile, BeginCap, EndCap)
            V = [];
            N = [];
            F = [];
            T = [];

            if size(Spine, 1) < 2
                return
            end
            % One scale row per spine point; a single row applies to all of them
            if size(Scale, 1) ~= size(Spine, 1)
                Scale = repmat(Scale(1, :), size(Spine, 1), 1);
            end
            if size(Scale, 2) == 1
                Scale = repmat(Scale, [1 3]);
            else
                Scale = [ones(size(Scale, 1), 1) Scale(:, [1 2])];
            end

            % Prepare segment profile
            v0 = [Profile(:, 1)*0 Profile(:, 1:2)];

            % Compute segment normals. The normal at a profile point is the
            % chord between its two neighbours - a central difference, centred
            % on the point itself - rotated by -90 deg in the profile plane.
            % A closed profile repeats its first point, so at
            % the seam the neighbours are taken across it, not onto the
            % duplicate, and the two coincident points get the same normal.
            nP = size(v0, 1);
            prev = [1, 1:nP - 1];
            next = [2:nP, nP];
            if nP > 2 && norm(v0(1, :) - v0(nP, :)) <= 1e-9*max(1, norm(v0(1, :)))
                prev(1) = nP - 1;
                next(nP) = 2;
            end
            n0 = v0(next, :) - v0(prev, :);
            n0 = [n0(:, 1) n0(:, 3) -n0(:, 2)];
            n0 = n0./vecnorm(n0, 2, 2);

            % Compute segment texture coordinates
            Segments = size(Profile, 1) - 1;
            t0 = [linspace(0, 1, Segments + 1)', v0(:, 1)];

            % Face indexes for first part. The winding runs so that a triangle
            % normal, (spine step) x (profile tangent), comes out parallel to
            % the vertex normals above, so that the shading and the collision
            % mesh both face out.
            f = (1:Segments)';
            f0 = [f, f + 1, f + Segments + 2; f, f + Segments + 2, f + Segments + 1];

            % Texture Y-coord through spine. It runs over the spine points
            % alone; each cap inherits the coordinates of the ring it sits on.
            % The image top lies at the last spine point, as the top of a
            % revolution lies at its last profile point, so that the
            % texture reads unmirrored from outside.
            nS = size(Spine, 1);
            tY = linspace(1, 0, nS);

            % Main part: one cross-section per spine point. Row 1 of R is the
            % local tangent, so the profile plane is truly perpendicular to the
            % curve; rows 2-3 orient the profile within that plane by referencing
            % a fixed world up-direction. Built from vectors rather than angles,
            % the frame stays regular for a segment in any direction, including
            % one with a world-Z component.
            % (row-vector convention: local basis -> rows of R)
            lp = Segments + 1;

            % Per-point tangents: at interior vertices the normalized average of
            % the incoming and outgoing segment directions (a miter), so the plane
            % bisects each bend instead of leaning fully onto one segment. That
            % halves the tilt of a section at a sharp corner and stops a steeply
            % tilted ring from crossing its neighbour and pinching the swept tube
            % into a blade. End points use their single adjacent segment.
            D = Spine(2:nS, :) - Spine(1:nS - 1, :);
            D = D./vecnorm(D, 2, 2);                 % unit segment directions
            Tg = [D(1, :); zeros(nS - 2, 3); D(nS - 1, :)];
            for i = 2:nS - 1
                t = D(i - 1, :) + D(i, :);
                if norm(t) < 1e-9
                    t = D(i, :);                     % ~180 deg reversal fallback
                end
                Tg(i, :) = t/norm(t);
            end

            % Build each frame from its tangent and a fixed world up-reference
            % (+Z, falling back to +X where the tangent runs along +-Z). Anchoring
            % the roll to a global direction - rather than parallel-transporting
            % it point to point - keeps a non-circular profile from slowly rolling
            % about the tangent along a curved or helical spine, so e.g. a
            % screw flight stays a regular screw instead of a warped one. Then
            % place a scaled, translated ring per point, and its normals
            % through the same frame.
            for i = 1:nS
                p = Tg(i, :);
                ref = [0 0 1];
                if abs(p*ref') > 1 - 1e-6
                    ref = [1 0 0];                   % tangent along world +-Z
                end
                row2 = cross(ref, p);
                row2 = row2/norm(row2);
                row3 = cross(p, row2);
                R = [p; row2; row3];
                V = vertcat(V, v0.*Scale(i, :)*R + Spine(i, :));
                N = vertcat(N, n0*R);
                T = vertcat(T, t0 + [0 tY(i)]);
                if i < nS
                    F = vertcat(F, f0);
                    f0 = f0 + lp;
                end
            end

            % Close the ends with a real triangulation of the profile outline,
            % which keeps the cap inside the solid for any outline, and without
            % zero-area triangles. Each cap gets its own copy of the ring it
            % sits on, so it carries the flat cap normal while inheriting that
            % ring's texture coordinates, and the texture runs on across the rim.
            if BeginCap || EndCap
                cf = phx.internal.Geometry.capFaces(Profile);
            end
            if BeginCap
                V = [V(1:lp, :); V];
                N = [repmat(-Tg(1, :), lp, 1); N];
                T = [T(1:lp, :); T];
                F = [cf(:, [1 3 2]); F + lp];      % faces away from the spine
            end
            if EndCap
                nV = size(V, 1);
                V = [V; V(nV - lp + 1:nV, :)];
                N = [N; repmat(Tg(nS, :), lp, 1)];
                T = [T; T(nV - lp + 1:nV, :)];
                F = [F; cf + nV];
            end
        end

        function F = capFaces(Profile)
            %capFaces Triangulate a closed profile outline into cap faces
            %
            %   Indexes refer to the rows of Profile and the winding is
            %   counter-clockwise in profile coordinates, which puts the face
            %   normal along the local sweep tangent.

            F = zeros(0, 3);
            P = Profile;
            if size(P, 1) > 1 && norm(P(1, :) - P(end, :)) <= ...
                    1e-9*max(max(P, [], 1) - min(P, [], 1))
                P(end, :) = [];                    % drop the repeated closing point
            end
            if size(P, 1) < 3
                return
            end

            % KeepCollinearPoints keeps the outline's own vertices, so the cap
            % shares them with the ring it closes instead of introducing points
            % that carry no texture coordinate of their own.
            tr = triangulation(polyshape(P, "Simplify", false, "KeepCollinearPoints", true));
            [~, loc] = ismembertol(tr.Points, P, 1e-12, "ByRows", true, "DataScale", 1);
            F = loc(tr.ConnectivityList);
        end

        function [V, N, F, T] = sphere(ASize, Segments)
            % Main geometry: Segments meridians and a parallel at the same
            % angular step, so that the cells at the equator are square
            a = linspace(-90, 90, round(Segments/2) + 1)';
            z = sind(a)*0.5;
            x = cosd(a)*0.5;
            [V, N, F, T] = phx.internal.Geometry.revolution([z x], Segments, false, false);

            % Scale
            V = V.*ASize;
        end

        function [V, N, F, T] = terrain(ASize, Height)
            % Vertexes
            x = linspace(0, 1, size(Height, 2));
            y = linspace(0, 1, size(Height, 1));
            [gx, gy] = meshgrid(x, y);
            V = [gx(:) - 0.5, gy(:) - 0.5, Height(:)];

            % Faces
            F = delaunay(gx, gy);
            F = F(:, [1 3 2]);

            % Normal vectors
            tr = triangulation(F, V);
            N = -tr.vertexNormal;

            % Texture coordinates
            T = [gx(:) gy(:)];

            % Scale
            V = V.*ASize;
        end
    end

end