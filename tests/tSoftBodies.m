classdef tSoftBodies < PhxTestCase
%tSoftBodies Technical tests for phx.SoftBody, phx.FlexibleBody and phx.IntersectionWeld.
%
%   The physics of soft and flexible bodies is still being tuned, so these
%   tests check only their technical contract: the documented errors, the
%   rest shape and the quantities derived from the mesh before a simulation
%   exists, and, with the engine ("Engine" tag), that a simulation takes
%   the bodies in, moves them and welds a body into them.
%
%   See also phx.SoftBody, phx.FlexibleBody, phx.IntersectionWeld

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        Half = 0.3   % half the edge of the test cube
    end

    % --- Configuration (no engine) ----------------------------------------
    methods (Test)
        function softBodyNeedsAMesh(tc)
            tc.verifyError(@() phx.SoftBody([]), "phx:SoftBody:noMesh");
        end

        function softBodyRestShapeIsPlaced(tc)
            [V, F] = tc.cube;
            R = [cos(0.5) -sin(0.5) 0; sin(0.5) cos(0.5) 0; 0 0 1];
            s = phx.SoftBody([], "Vertices", V, "Faces", F, "Position", [1 2 3], "Orientation", R);
            tc.verifyEqual(s.CurrentVertices, V*R' + [1 2 3], "AbsTol", 1e-12);
            [lo, hi] = bounds(V*R' + [1 2 3], 1);
            tc.verifyEqual(s.boundingBox, [lo; hi], "AbsTol", 1e-12);
        end

        function flexibleBodyNeedsAMesh(tc)
            tc.verifyError(@() phx.FlexibleBody([]), "phx:FlexibleBody:noMesh");
            V = tc.cube;
            tc.verifyError(@() phx.FlexibleBody([], "Vertices", V, "Tetrahedra", ones(2, 5)), ...
                "phx:FlexibleBody:badTetrahedra");
        end

        function flexibleBodyDerivesSurfaceAndMass(tc)
            V = tc.cube;
            f = phx.FlexibleBody([], "Vertices", V, "Tetrahedra", delaunay(V), "Density", 500);
            tc.verifySize(f.Faces, [12 3], "The surface of a cube is 12 triangles.");
            tc.verifyEqual(f.Mass, 500*(2*tc.Half)^3, "RelTol", 1e-9);
            tc.verifyEqual(f.CurrentVertices, V, "AbsTol", 1e-12);
        end

        function quadraticElementsAddEdgeVertices(tc)
            V = tc.cube;
            T = delaunay(V);
            edges = unique(sort([T(:, [1 2]); T(:, [2 3]); T(:, [1 3]); T(:, [1 4]); T(:, [2 4]); T(:, [3 4])], 2), "rows");
            f = phx.FlexibleBody([], "Vertices", V, "Tetrahedra", T, "ElementOrder", "quadratic");
            tc.verifySize(f.CurrentVertices, [size(V, 1) + size(edges, 1) 3]);
            tc.verifyEqual(f.CurrentVertices(1:size(V, 1), :), V, "AbsTol", 1e-12, ...
                "The corner vertices come first.");
        end

        function weldJoinsASoftBodyAndABody(tc)
            [V, F] = tc.cube;
            s = phx.SoftBody([], "Vertices", V, "Faces", F);
            b = tc.spawnBody([0 0 0]);
            tc.verifyError(@() phx.IntersectionWeld(b, b), "MATLAB:validators:mustBeA");
            w = phx.IntersectionWeld(s, b);
            tc.verifyEqual(w.VertexCount, 0, "Nothing is welded before the simulation starts.");
            delete(b);
            tc.verifyFalse(isvalid(w), "A weld without its body must go with it.");
        end
    end

    % --- Simulation (engine) ------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function softBodyFallsOntoTheFloor(tc)
            tc.requireEngine;
            [V, F] = tc.cube;
            s = phx.SoftBody([], "Vertices", V, "Faces", F, "Position", [0 0 1], "Compliance", 1e-4);
            sim = tc.simulate({tc.floor, s});
            sim.step(1, 200, -1);
            Vnow = s.CurrentVertices;
            tc.verifySize(Vnow, size(V));
            tc.verifyLessThan(mean(Vnow(:, 3)), 0.9, "The soft body must fall.");
            tc.verifyGreaterThan(min(Vnow(:, 3)), -0.05, "The floor must stop it.");
        end

        function flexibleBodyFallsAndReportsStress(tc)
            tc.requireEngine;
            V = tc.cube;
            f = phx.FlexibleBody([], "Vertices", V, "Tetrahedra", delaunay(V), "Position", [0 0 1]);
            sim = tc.simulate({tc.floor, f});
            sim.step(1, 200, -1);
            Vnow = f.CurrentVertices;
            tc.verifyLessThan(mean(Vnow(:, 3)), 0.9, "The flexible body must fall.");
            tc.verifyGreaterThan(min(Vnow(:, 3)), -0.05, "The floor must stop it.");
            tc.verifySize(f.VonMisesStress, [size(V, 1) 1]);
            tc.verifyTrue(all(isfinite(f.VonMisesStress)));
        end

        function weldHoldsTheOverlappedVertices(tc)
            % A static slab over the top face of the cube holds its four
            % top vertices, and the hanging cube stays up
            tc.requireEngine;
            [V, F] = tc.cube;
            s = phx.SoftBody([], "Vertices", V, "Faces", F, "Position", [0 0 1], "Compliance", 1e-4);
            slab = tc.spawnBody([0 0 1 + tc.Half], "Type", "static", "Shape", {"Box", "Size", [1 1 0.2]});
            w = phx.IntersectionWeld(s, slab);
            sim = tc.simulate({slab, s});
            sim.step(0.5, 100, -1);
            tc.verifyEqual(w.VertexCount, 4);
            tc.verifyGreaterThan(min(s.CurrentVertices(:, 3)), 0.4, "The weld must hold the cube up.");
        end
    end

    methods (Access = private)
        function [V, F] = cube(tc)
            [x, y, z] = ndgrid([-1 1]*tc.Half);
            V = [x(:) y(:) z(:)];
            F = convhull(V);
        end

        function b = floor(tc)
            b = tc.spawnBody([0 0 -0.5], "Type", "static", "Shape", {"Box", "Size", [4 4 1]});
        end

        function sim = simulate(tc, objects)
            sim = phx.Simulation(objects);
            tc.addTeardown(@() tc.deleteIfValid(sim));
        end
    end

end
