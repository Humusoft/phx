classdef tShapeMesh < PhxTestCase
%tShapeMesh Meshes of tetrahedra that shapes generate for flexible bodies.
%
%   generateMesh is a pure method on the value-class shapes, so these tests
%   need neither the physics engine nor a graphics session. They check how
%   ElementResolution divides a shape and that the mesh fills the shape
%   exactly, with matching faces between neighbouring tetrahedra.
%
%   See also phx.shape.Box, phx.FlexibleBody

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
            [~, T] = s.generateMesh;
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
            [V, T] = phx.shape.Box("Size", sz, "ElementResolution", 2).generateMesh;
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
            [~, T] = s.generateMesh;
            F = sort([T(:, [1 2 3]); T(:, [1 2 4]); T(:, [1 3 4]); T(:, [2 3 4])], 2);
            [~, ~, k] = unique(F, "rows");
            uses = accumarray(k, 1);
            n = s.ElementDivisions;
            tc.verifyLessThanOrEqual(max(uses), 2);
            tc.verifyEqual(nnz(uses == 1), 4*(n(1)*n(2) + n(1)*n(3) + n(2)*n(3)));
        end
    end

    methods (Static, Access = private)
        function v = volumes(V, T)
            a = V(T(:, 1), :);
            v = dot(cross(V(T(:, 2), :) - a, V(T(:, 3), :) - a, 2), V(T(:, 4), :) - a, 2)/6;
        end
    end

end
