classdef tShapeMesh < PhxTestCase
%tShapeMesh Meshes that shapes generate for flexible and soft bodies.
%
%   The mesh generators are pure methods on the value-class shapes, so
%   these tests need neither the physics engine nor a graphics session.
%   They check how ElementResolution divides a shape, that the volumetric
%   mesh fills the shape exactly with matching faces between neighbouring
%   tetrahedra, and that the surface mesh closes around it facing out.
%
%   See also phx.shape.Box, phx.FlexibleBody, phx.SoftBody

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function boxScalarResolutionCutsTheSmallestSide(tc)
            % N cells across the smallest side, the others as close to cubes
            % as whole numbers allow, whichever axis the smallest side is on
            tc.verifyEqual(phx.shape.Box("Size", [1.06 0.12 0.1], "ElementResolution", 2).ElementDivisions, [21 2 2]);
            tc.verifyEqual(phx.shape.Box("Size", [0.12 0.1 1.06], "ElementResolution", 2).ElementDivisions, [2 2 21]);
            tc.verifyEqual(phx.shape.Box("Size", [1 1 1], "ElementResolution", 1).ElementDivisions, [1 1 1]);
            tc.verifyEqual(phx.shape.Box("Size", [0.4 0.2 0.1]).ElementDivisions, [8 4 2], ...
                "Two cells across the smallest side by default.");
        end

        function boxVectorResolutionIsTakenAsGiven(tc)
            s = phx.shape.Box("Size", [0.4 0.2 0.1], "ElementResolution", [5 1 3]);
            tc.verifyEqual(s.ElementDivisions, [5 1 3]);
            [~, T] = s.generateVolumetricMesh;
            tc.verifySize(T, [6*5*1*3 4], "Six tetrahedra per cell.");
        end

        function boxResolutionIsValidated(tc)
            for bad = {[1 2], [1 2 3 4], 0, 1.5, [2 2 -1], NaN}
                tc.verifyError(@() phx.shape.Box("ElementResolution", bad{1}), "phx:Box:badResolution");
            end
        end

        function boxMeshFillsTheBox(tc)
            % The tetrahedra are positive, add up to the volume of the box,
            % span it exactly and use every vertex
            sz = [0.5 0.2 0.15];
            [V, T] = phx.shape.Box("Size", sz, "ElementResolution", 2).generateVolumetricMesh;
            volume = tc.volumes(V, T);
            tc.verifyGreaterThan(min(volume), 0);
            tc.verifyEqual(sum(volume), prod(sz), "RelTol", 1e-12);
            [lo, hi] = bounds(V, 1);
            tc.verifyEqual([lo; hi], [-sz; sz]/2, "AbsTol", 1e-15);
            tc.verifyEqual(unique(T(:))', 1:size(V, 1));
        end

        function boxMeshFacesMatch(tc)
            % Inner faces are shared by exactly two tetrahedra; the surface
            % is two triangles per face of each boundary cell
            s = phx.shape.Box("Size", [0.5 0.2 0.15], "ElementResolution", [4 3 2]);
            [~, T] = s.generateVolumetricMesh;
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [~, ~, k] = unique(F, "rows");
            uses = accumarray(k, 1);
            n = s.ElementDivisions;
            tc.verifyLessThanOrEqual(max(uses), 2);
            tc.verifyEqual(nnz(uses == 1), 4*(n(1)*n(2) + n(1)*n(3) + n(2)*n(3)));
        end

        function boxSurfaceMeshClosesAroundTheBox(tc)
            % The surface vertices of the grid alone, every edge shared by
            % two triangles, and the triangles counter-clockwise from outside:
            % the signed volume they enclose is the volume of the box
            sz = [0.5 0.2 0.15];
            s = phx.shape.Box("Size", sz, "ElementResolution", [4 3 2]);
            [V, F] = s.generateSurfaceMesh;
            n = s.ElementDivisions;
            tc.verifyEqual(size(V, 1), prod(n + 1) - prod(n - 1), "No inner vertices.");
            tc.verifyEqual(size(F, 1), 4*(n(1)*n(2) + n(1)*n(3) + n(2)*n(3)));
            tc.verifyEqual(unique(F(:))', 1:size(V, 1));
            E = sort([F(:, [1 2]); F(:, [2 3]); F(:, [3 1])], 2);
            [~, ~, k] = unique(E, "rows");
            tc.verifyEqual(unique(accumarray(k, 1))', 2, "A closed surface.");
            enclosed = sum(dot(V(F(:, 1), :), cross(V(F(:, 2), :), V(F(:, 3), :), 2), 2))/6;
            tc.verifyEqual(enclosed, prod(sz), "RelTol", 1e-12);
            [lo, hi] = bounds(V, 1);
            tc.verifyEqual([lo; hi], [-sz; sz]/2, "AbsTol", 1e-15);
        end

        function sphereSurfaceMeshIsGeodesic(tc)
            % The icosahedron with its edges cut into n parts: 10n^2 + 2
            % vertices on the sphere and 20n^2 triangles, edges about
            % Diameter/ElementResolution long
            for c = [2 1; 4 2; 30 16]'
                s = phx.shape.Sphere("Diameter", 0.6, "ElementResolution", c(1));
                [V, F] = s.generateSurfaceMesh;
                n = c(2);
                tc.verifySize(V, [10*n^2 + 2 3]);
                tc.verifySize(F, [20*n^2 3]);
                tc.verifyEqual(vecnorm(V, 2, 2), repmat(0.3, size(V, 1), 1), "AbsTol", 1e-12);
                E = sort([F(:, [1 2]); F(:, [2 3]); F(:, [3 1])], 2);
                L = vecnorm(V(E(:, 1), :) - V(E(:, 2), :), 2, 2);
                tc.verifyEqual(mean(L), 0.6/c(1), "RelTol", 0.25);
            end
        end

        function sphereSurfaceMeshClosesOutwards(tc)
            [V, F] = phx.shape.Sphere("Diameter", 2, "ElementResolution", 8).generateSurfaceMesh;
            E = sort([F(:, [1 2]); F(:, [2 3]); F(:, [3 1])], 2);
            [~, ~, k] = unique(E, "rows");
            tc.verifyEqual(unique(accumarray(k, 1))', 2, "A closed surface.");
            tc.verifyEqual(unique(F(:))', 1:size(V, 1));
            enclosed = sum(dot(V(F(:, 1), :), cross(V(F(:, 2), :), V(F(:, 3), :), 2), 2))/6;
            tc.verifyGreaterThan(enclosed, 0.95*4/3*pi, "Wound outwards, close to the sphere.");
            tc.verifyLessThan(enclosed, 4/3*pi);
        end

        function sphereVolumetricMeshFillsTheSphere(tc)
            % Quadratic tetrahedra, positive and with matching faces; the
            % corners and the middles of the edges on the surface lie on the
            % sphere, the other middles halfway along their edges
            for N = [2 5 9]
                [V, T] = phx.shape.Sphere("Diameter", 0.6, "ElementResolution", N).generateVolumetricMesh;
                tc.verifyEqual(size(T, 2), 10);
                tc.verifyGreaterThan(min(tc.volumes(V, T)), 0);
                F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
                [faces, ~, k] = unique(F, "rows");
                uses = accumarray(k, 1);
                tc.verifyLessThanOrEqual(max(uses), 2);
                outer = faces(uses == 1, :);
                surface = unique(sort([outer(:, [1 2]); outer(:, [2 3]); outer(:, [1 3])], 2), "rows");
                ends = [1 2; 2 3; 1 3; 1 4; 2 4; 3 4];
                E = sort([reshape(T(:, ends(:, 1)), [], 1) reshape(T(:, ends(:, 2)), [], 1)], 2);
                middle = reshape(T(:, 5:10), [], 1);
                on = ismember(E, surface, "rows");
                tc.verifyEqual(vecnorm(V([unique(outer); middle(on)], :), 2, 2), ...
                    repmat(0.3, numel(unique(outer)) + nnz(on), 1), "AbsTol", 1e-12);
                tc.verifyEqual(V(middle(~on), :), (V(E(~on, 1), :) + V(E(~on, 2), :))/2, "AbsTol", 1e-15);
            end
        end

        function sphereCurvedMeshHasTheVolumeOfTheSphere(tc)
            % The curved tetrahedra follow the sphere, so even a coarse
            % mesh has close to its volume, unlike the straight corners
            ball = 4/3*pi*0.3^3;
            [V, T] = phx.shape.Sphere("Diameter", 0.6, "ElementResolution", 5).generateVolumetricMesh;
            curved = sum(phx.internal.Geometry.tetrahedraVolumes(V, T));
            straight = sum(phx.internal.Geometry.tetrahedraVolumes(V, T(:, 1:4)));
            tc.verifyEqual(curved, ball, "RelTol", 0.005);
            tc.verifyLessThan(straight, 0.9*ball);
        end

        function cylinderVolumetricMeshFillsTheCylinder(tc)
            % Quadratic tetrahedra, positive and with matching faces, inside
            % the cylinder; the middles of the edges on its curved side lie
            % on it, the other middles halfway along their edges, and the
            % curved tetrahedra have close to its volume
            r = 0.2;
            h = 0.5;
            [V, T] = phx.shape.Cylinder("Diameter", 2*r, "Height", h, "ElementResolution", 3).generateVolumetricMesh;
            tc.verifyEqual(size(T, 2), 10);
            tc.verifyGreaterThan(min(tc.volumes(V, T)), 0);
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [~, ~, k] = unique(F, "rows");
            tc.verifyLessThanOrEqual(max(accumarray(k, 1)), 2);
            rho = vecnorm(V(:, 1:2), 2, 2);
            tc.verifyLessThan(max(rho), r*(1 + 1e-12));
            tc.verifyLessThan(max(abs(V(:, 3))), h/2*(1 + 1e-12));
            ends = [1 2; 2 3; 1 3; 1 4; 2 4; 3 4];
            E = [reshape(T(:, ends(:, 1)), [], 1) reshape(T(:, ends(:, 2)), [], 1)];
            middle = reshape(T(:, 5:10), [], 1);
            side = abs(rho(E(:, 1)) - r) < 1e-12 & abs(rho(E(:, 2)) - r) < 1e-12;
            tc.verifyEqual(rho(middle(side)), repmat(r, nnz(side), 1), "AbsTol", 1e-12);
            tc.verifyEqual(V(middle(~side), :), (V(E(~side, 1), :) + V(E(~side, 2), :))/2, "AbsTol", 1e-15);
            tc.verifyEqual(sum(phx.internal.Geometry.tetrahedraVolumes(V, T)), pi*r^2*h, "RelTol", 0.005);
        end

        function cylinderResolutionCutsTheThinnerSize(tc)
            % A scalar is the number of cells across the smaller of the
            % diameter and the height, the other cut into cells about as
            % long; a vector gives the cells across the diameter and along
            % the height. Counted here by the layers along the axis.
            tc.verifyEqual(tc.layers(phx.shape.Cylinder("Diameter", 0.1, "Height", 1, "ElementResolution", 2)), 20);
            tc.verifyEqual(tc.layers(phx.shape.Cylinder("Diameter", 1, "Height", 0.1, "ElementResolution", 2)), 2);
            tc.verifyEqual(tc.layers(phx.shape.Cylinder("Diameter", 0.5, "Height", 1, "ElementResolution", [4 6])), 6);
        end

        function cylinderMeshFollowsItsAxis(tc)
            [V, T] = phx.shape.Cylinder("Diameter", 0.4, "Height", 1, "Axis", "x").generateVolumetricMesh;
            [lo, hi] = bounds(V, 1);
            tc.verifyEqual([lo; hi], [-0.5 -0.2 -0.2; 0.5 0.2 0.2], "AbsTol", 1e-12);
            tc.verifyGreaterThan(min(tc.volumes(V, T)), 0, "Turned, not mirrored.");
        end

        function cylinderResolutionIsValidated(tc)
            for bad = {[1 2 3], 0, 1.5, [2 0], NaN}
                tc.verifyError(@() phx.shape.Cylinder("ElementResolution", bad{1}), "phx:Cylinder:badResolution");
            end
        end

        function sphereResolutionIsValidated(tc)
            for bad = {[1 2], 0, 1.5, NaN, []}
                tc.verifyError(@() phx.shape.Sphere("ElementResolution", bad{1}), "phx:Sphere:badResolution");
            end
        end

        function shapesWithoutAMeshSayNo(tc)
            tc.verifyError(@() phx.shape.Cone().generateVolumetricMesh, "phx:Shape:noVolumetricMesh");
            tc.verifyError(@() phx.shape.Cone().generateSurfaceMesh, "phx:Shape:noSurfaceMesh");
        end
    end

    methods (Static, Access = private)
        function n = layers(shape)
            % Layers of cells along z: the distinct heights of the corners
            [V, T] = shape.generateVolumetricMesh;
            n = numel(uniquetol(V(unique(T(:, 1:4)), 3), 1e-9)) - 1;
        end

        function v = volumes(V, T)
            a = V(T(:, 1), :);
            v = dot(cross(V(T(:, 2), :) - a, V(T(:, 3), :) - a, 2), V(T(:, 4), :) - a, 2)/6;
        end
    end

end
