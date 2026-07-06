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

            % Remove degenerated faces
            v1 = V(F(:, 1), :);
            v2 = V(F(:, 2), :);
            v3 = V(F(:, 3), :);
            b12 = sum(abs(v1 - v2), 2) < 0.001;
            b23 = sum(abs(v2 - v3), 2) < 0.001;
            b31 = sum(abs(v3 - v1), 2) < 0.001;
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

            % Prepare for caps
            if BeginCap
                bcShift = Spine(2, :) - Spine(1, :);
                Scale = [0 0 0; Scale(1, :); Scale];
                Spine = [Spine(1, :) - 2*bcShift; Spine(1, :) - bcShift; Spine];
            end
            if EndCap
                ecShift = Spine(end, :) - Spine(end - 1, :);
                Scale = [Scale; Scale(end, :); 0 0 0];
                Spine = [Spine; Spine(end, :) + ecShift; Spine(end, :) + 2*ecShift];
            end

            % Prepare segment profile
            v0 = [Profile(:, 1)*0 Profile(:, 1:2)];

            % Compute segment normals
            n0 = v0 - circshift(v0, [2 0]);
            n0 = [n0(:, 1) n0(:, 3) -n0(:, 2)];
            for i = 1:size(n0, 1)
                n0(i, :) = n0(i, :)/norm(n0(i, :));
            end

            % Compute segment texture coordinates
            Segments = size(Profile, 1) - 1;
            t0 = [linspace(0, 1, Segments + 1)', v0(:, 1)];

            % Face indexes for first part
            f = (1:Segments)';
            f0 = [f, f + Segments + 2, f + 1; f, f + Segments + 1, f + Segments + 2];

            % Texture Y-coord through spine
            nS = size(Spine, 1);
            if ~BeginCap && ~EndCap
                tY = linspace(0, 1, nS);
            elseif ~BeginCap && EndCap
                tY = [linspace(0, 1, nS - 2) 1 1];
            elseif BeginCap && ~EndCap
                tY = [0 0 linspace(0, 1, nS - 2)];
            elseif BeginCap && EndCap
                tY = [0 0 linspace(0, 1, nS - 4) 1 1];
            end

            % Main part (from second point to end)
            Ptot = Spine(1, :);
            for i = 2:nS
                P = Spine(i, :) - Spine(i - 1, :);
                p = P/norm(P);
                Z = -atan2(p(2), p(1));
                Y = atan2(p(3), sqrt(p(1)^2 + p(2)^2));
                X = 0;
                R = phx.internal.Math.rot321([X Y Z]);
                Ptot = Ptot + P;
                V = vertcat(V, v0.*Scale(i, :)*R + Ptot);
                N = vertcat(N, n0*R);
                F = vertcat(F, f0);
                f0 = f0 + Segments + 1;
                T = vertcat(T, t0 + [0 tY(i)]);
            end

            % Initial part (from first to second point)
            lp = Segments + 1;
            V = [(V(1:lp, :) - Spine(2, :)).*Scale(1, :)./Scale(2, :) + Spine(1, :); V];
            N = [N(1:lp, :); N];
            T = [t0; T];

            % Modify caps segments
            if BeginCap
                V(1:lp*2, :) = V(1:lp*2, :) + bcShift;
                V(1:lp, :) = V(1:lp, :) + bcShift;
                N(1:lp*2, :) = zeros(lp*2, 3) - bcShift/norm(bcShift);
            end
            if EndCap
                V(end - lp*2 + 1:end, :) = V(end - lp*2 + 1:end, :) - ecShift;
                V(end - lp + 1:end, :) = V(end - lp + 1:end, :) - ecShift;
                N(end - lp*2 + 1:end, :) = zeros(lp*2, 3) + ecShift/norm(ecShift);
            end
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