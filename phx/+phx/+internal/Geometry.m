classdef Geometry
%phx.internal.Geometry PHX geometric Library

%   Copyright 1998-2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    methods (Static)
        function [Mass, I0, Icm, cm] = meshMass(vertices, faces, rho)
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

        function [V, N, F] = rock(ASize, Roundness)
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
            % N = V;
            V = V.*ASize/2;

            % Deformation
            V(1:4:end) = V(1:4:end)*Roundness;
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
            if size(Spine, 1) ~= size(Scale, 1)
                Scale = ones(size(Spine, 1), 3)*Scale(1);
            else
                if size(Scale, 2) == 1
                    Scale = repmat(Scale, [1 3]);
                else
                    Scale = [ones(size(Scale, 1), 1) Scale(:, [1 2])];
                end
            end

            % Prepare segment profile
            v0 = [Profile(:, 1)*0 Profile(:, 1:2)];

            % Compute segment normals. The normal at a profile point is the
            % chord between its two neighbours - a central difference, centred
            % on the point itself - rotated by -90 deg in the profile plane.
            % The earlier v0(k) - v0(k-2) spanned the same chord but was centred
            % on k-1, which rotated the shading of every extrusion by one
            % profile segment (45 deg on an 8-segment profile, 2.5 deg on a
            % 144-segment one). A closed profile repeats its first point, so at
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
            % the vertex normals above rather than against them. The two used to
            % disagree, which left every caller a choice between shading that
            % faces out and a collision mesh that faces out - never both.
            f = (1:Segments)';
            f0 = [f, f + 1, f + Segments + 2; f, f + Segments + 2, f + Segments + 1];

            % Texture Y-coord through spine. The caps no longer ride on the
            % spine, so this runs over the real spine points alone; each cap
            % inherits the coordinates of the ring it sits on.
            nS = size(Spine, 1);
            tY = linspace(0, 1, nS);

            % Main part: one cross-section per spine point. Row 1 of R is the
            % local tangent, so the profile plane is truly perpendicular to the
            % curve; rows 2-3 orient the profile within that plane by referencing
            % a fixed world up-direction. This replaces the old zero-roll atan2
            % aim, which mis-oriented (and was singular for) any segment with a
            % world-Z component. (row-vector convention: local basis -> rows of R)
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
            % about the tangent along a curved or helical spine, so e.g. the
            % flighting of phxex_screwconv stays a regular screw instead of a
            % warped one; for a straight or planar spine it reproduces the
            % historical frame exactly. Then place a scaled, translated ring per
            % point, and its normals through the same frame.
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

            % Close the ends with a real triangulation of the profile outline.
            % Each cap gets its own copy of the ring it sits on, so it carries
            % the flat cap normal while inheriting that ring's texture
            % coordinates - the texture runs across the rim exactly as before.
            % The outline used to be collapsed onto a point instead, which put
            % zero-area triangles into roughly 60 % of the faces and fanned the
            % cap from the profile origin, so any outline not star-shaped about
            % that origin got cap triangles hanging outside the solid.
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
            % Main geometry
            a = linspace(-90, 90, Segments)';
            z = sind(a)*0.5;
            x = cosd(a)*0.5;
            [V, N, F, T] = phx.internal.Geometry.revolution([z x], Segments, false, false);

            % Scale
            V = V.*ASize;
        end

        function [V, N, F, T] = terrain(ASize, Height, PlanarNormals)
            % Vertexes
            x = linspace(0, 1, size(Height, 2));
            y = linspace(0, 1, size(Height, 1));
            [gx, gy] = meshgrid(x, y);
            V = [gx(:) - 0.5, gy(:) - 0.5, Height(:)];

            % Faces
            F = delaunay(gx, gy);
            F = F(:, [1 3 2]);

            % Normal vectors
            if PlanarNormals
                N = repmat([0 0 1], [size(V, 1) 1]);
            else
                tr = triangulation(F, V);
                N = -tr.vertexNormal;
            end

            % Texture coordinates
            T = [gx(:) gy(:)];

            % Scale
            V = V.*ASize;
        end
    end

end