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
            tc.verifySize(f.Faces, [48 3], "The surface of a cube is 12 triangles, each split into four.");
            tc.verifyEqual(f.Mass, 500*(2*tc.Half)^3, "RelTol", 1e-9);
            tc.verifyEqual(f.CurrentVertices(1:size(V, 1), :), V, "AbsTol", 1e-12);
        end

        function flexibleBodyMassSetsTheDensity(tc)
            % Mass given before the mesh is still taken after it
            V = tc.cube;
            f = phx.FlexibleBody([], "Mass", 2, "Vertices", V, "Tetrahedra", delaunay(V));
            tc.verifyEqual(f.Mass, 2, "RelTol", 1e-9);
            tc.verifyEqual(f.Density, 2/(2*tc.Half)^3, "RelTol", 1e-9);
            f.Vertices = 2*V;
            tc.verifyEqual(f.Mass, 16, "RelTol", 1e-9, "A new mesh keeps the Density.");
        end

        function linearMeshGetsEdgeVertices(tc)
            V = tc.cube;
            T = delaunay(V);
            edges = unique(sort([T(:, [1 2]); T(:, [2 3]); T(:, [1 3]); T(:, [1 4]); T(:, [2 4]); T(:, [3 4])], 2), "rows");
            f = phx.FlexibleBody([], "Vertices", V, "Tetrahedra", T);
            tc.verifySize(f.CurrentVertices, [size(V, 1) + size(edges, 1) 3]);
            tc.verifyEqual(f.CurrentVertices(1:size(V, 1), :), V, "AbsTol", 1e-12, ...
                "The corner vertices come first.");
        end

        function meshSetsVerticesAndTetrahedra(tc)
            % A mesh in the form PDE Toolbox gives it: Nodes 3xN, Elements
            % 4xK or 10xK; an object whose Mesh is such a mesh works alike
            V = tc.cube;
            T = delaunay(V);
            f = phx.FlexibleBody([], "Mesh", struct("Nodes", V', "Elements", T'));
            tc.verifyEqual(f.Vertices, V);
            tc.verifyEqual(f.Tetrahedra, T);
            tc.verifyEqual(f.Mesh.Elements, T');
            g = phx.FlexibleBody([], "Mesh", struct("Mesh", struct("Nodes", V', "Elements", T')));
            tc.verifyEqual(g.Tetrahedra, T);
            tc.verifyError(@() phx.FlexibleBody([], "Mesh", struct("Nodes", V', "Elements", T(:, 1:3)')), ...
                "phx:FlexibleBody:badMesh", "A mesh of triangles is not a solid.");
            tc.verifyError(@() phx.FlexibleBody([], "Mesh", V), "phx:FlexibleBody:badMesh");
        end

        function meshTakesAPdeToolboxMesh(tc)
            tc.assumeTrue(exist("fegeometry", "file") > 0, "Needs PDE Toolbox.");
            gm = generateMesh(fegeometry(multicuboid(0.4, 0.2, 0.1)), Hmax=0.1);
            f = phx.FlexibleBody([], "Mesh", gm, "Density", 1000);
            tc.verifySize(f.Tetrahedra, [size(gm.Mesh.Elements, 2) 10], "A quadratic mesh by default.");
            tc.verifyEqual(f.Mass, 1000*0.4*0.2*0.1, "RelTol", 1e-9);
        end

        function shapeSetsTheMesh(tc)
            % A shape object and the cell shorthand give the mesh the shape
            % generates
            box = phx.shape.Box("Size", [0.4 0.2 0.1], "ElementResolution", 1);
            [V, T] = box.generateVolumetricMesh;
            f = phx.FlexibleBody([], "Shape", box, "Density", 1100);
            tc.verifyEqual(f.Vertices, V);
            tc.verifyEqual(f.Tetrahedra, T);
            tc.verifyEqual(f.Mass, 1100*0.4*0.2*0.1, "RelTol", 1e-9);
            g = phx.FlexibleBody([], "Shape", {"Box", "Size", [0.4 0.2 0.1], "ElementResolution", 1});
            tc.verifyEqual(g.Tetrahedra, T);
        end

        function shapeDensityOfPHX2WarnsAndIsTaken(tc)
            % A density given to the shape (PHX 2.0) warns; the body takes it
            % unless given its own Density or Mass, before or after the shape
            box = phx.shape.Box("Size", [0.4 0.2 0.1], "Density", 1100);
            f = tc.verifyWarning(@() phx.FlexibleBody([], "Shape", box), "phx:Shape:densityMoved");
            tc.verifyEqual(f.Density, 1100);
            f = tc.verifyWarning(@() phx.FlexibleBody([], "Density", 500, "Shape", box), "phx:Shape:densityMoved");
            tc.verifyEqual(f.Density, 500);
            f = tc.verifyWarning(@() phx.FlexibleBody([], "Shape", box, "Density", 500), "phx:Shape:densityMoved");
            tc.verifyEqual(f.Density, 500);
            f = tc.verifyWarning(@() phx.FlexibleBody([], "Mass", 2, "Shape", box), "phx:Shape:densityMoved");
            tc.verifyEqual(f.Mass, 2, "RelTol", 1e-9);
        end

        function flexibleBodyHasItsVolumeAndItsOwnDensity(tc)
            % The shape gives the mesh, the body the material: Volume of the
            % tetrahedra, Mass = Density*Volume, 1000 kg/m^3 by default;
            % Density and Mass together are refused
            shape = {"Box", "Size", [0.4 0.2 0.1], "ElementResolution", 1};
            f = phx.FlexibleBody([], "Shape", shape);
            tc.verifyEqual(f.Volume, 0.4*0.2*0.1, "RelTol", 1e-9);
            tc.verifyEqual(f.Density, 1000);
            f.Density = 800;
            tc.verifyEqual(f.Mass, 800*f.Volume, "RelTol", 1e-14);
            tc.verifyError(@() phx.FlexibleBody([], "Shape", shape, "Density", 800, "Mass", 2), ...
                "phx:FlexibleBody:densityAndMass");
        end

        function curvedMeshGivesTheMass(tc)
            % The tetrahedra count the volume of their curved edges
            sphere = phx.shape.Sphere("Diameter", 0.5);
            q = phx.FlexibleBody([], "Shape", sphere);
            tc.verifyEqual(q.Volume, sphere.computeVolume, "RelTol", 0.005);
        end

        function shapeMustGenerateAMesh(tc)
            V = tc.cube;
            tc.verifyError(@() phx.FlexibleBody([], "Shape", {"Box"}, "Vertices", V, "Tetrahedra", delaunay(V)), ...
                "phx:FlexibleBody:shapeAndMesh");
            tc.verifyError(@() phx.FlexibleBody([], "Shape", {"Box"}, "Mesh", struct("Nodes", V', "Elements", delaunay(V)')), ...
                "phx:FlexibleBody:shapeAndMesh");
            tc.verifyError(@() phx.FlexibleBody([], "Shape", {"Mesh", "Vertices", eye(4, 3), "Faces", [1 3 2; 1 2 4; 1 4 3; 2 3 4]}), "phx:Shape:noVolumetricMesh");
            tc.verifyError(@() phx.FlexibleBody([], "Shape", V), "phx:FlexibleBody:badShape");
        end

        function softBodyShapeSetsTheSurfaceNotTheMass(tc)
            % A shape object and the cell shorthand give the surface mesh the
            % shape generates; the mass stays that of the soft body
            box = phx.shape.Box("Size", [0.4 0.2 0.1], "ElementResolution", 1);
            [V, F] = box.generateSurfaceMesh;
            s = phx.SoftBody([], "Shape", box, "Mass", 3);
            tc.verifyEqual(s.Vertices, V);
            tc.verifyEqual(s.Faces, F);
            tc.verifyEqual(s.Mass, 3);
            g = tc.verifyWarning(@() phx.SoftBody([], "Shape", {"Box", "Size", [0.4 0.2 0.1], ...
                "Density", 1100, "ElementResolution", 1}), "phx:Shape:densityMoved");
            tc.verifyEqual(g.Faces, F);
            tc.verifyEqual(g.Mass, 1, "The default mass, not the density of the shape (PHX 2.0).");
        end

        function softBodyShapeMustGenerateAMesh(tc)
            [V, F] = tc.cube;
            tc.verifyError(@() phx.SoftBody([], "Shape", {"Box"}, "Vertices", V, "Faces", F), ...
                "phx:SoftBody:shapeAndMesh");
            tc.verifyError(@() phx.SoftBody([], "Shape", {"Mesh", "Vertices", eye(4, 3), "Faces", [1 3 2; 1 2 4; 1 4 3; 2 3 4]}), "phx:Shape:noSurfaceMesh");
            tc.verifyError(@() phx.SoftBody([], "Shape", V), "phx:SoftBody:badShape");
        end

        function flexibleBodyColormapKeepsColorAcrossSaveAndLoad(tc)
            % The colormap paints the surface while Color keeps its value;
            % both survive save and load, and an emptied colormap gives the
            % surface its Color back
            V = tc.cube;
            f = phx.FlexibleBody([], "Vertices", V, "Tetrahedra", delaunay(V), ...
                "Color", [0 1 0], "Colormap", turbo(8));
            tc.verifyEqual(f.Color, [0 1 0]);

            file = fullfile(tc.tempFolder, "flexible.mat");
            save(file, "f");
            g = load(file).f;
            tc.verifyEqual(g.Colormap, turbo(8));
            tc.verifyEqual(g.Color, [0 1 0]);
            surface = findobj(g.Graphics, "Type", "patch");
            tc.verifyEqual(surface.FaceColor, 'interp');

            g.Colormap = [];
            tc.verifyEqual(surface.FaceColor, [0 1 0]);
        end

        function restOrientationHasTheViewsOfABody(tc)
            % EulerAngles, AxisAngle and Quaternion set and read the
            % Orientation of the rest shape, as they do on phx.Body
            [V, F] = tc.cube;
            bodies = {phx.SoftBody([], "Vertices", V, "Faces", F, "EulerAngles", [0.1 0.2 0.3]), ...
                phx.FlexibleBody([], "Vertices", V, "Tetrahedra", delaunay(V), "EulerAngles", [0.1 0.2 0.3])};
            for i = 1:2
                b = bodies{i};
                tc.verifyEqual(b.Orientation, phx.internal.Math.rot321([0.1 0.2 0.3]), "AbsTol", 1e-12);
                tc.verifyEqual(b.EulerAngles, [0.1 0.2 0.3], "AbsTol", 1e-12);
                b.AxisAngle = [0 0 1 pi/2];
                tc.verifyEqual(b.Orientation, [0 -1 0; 1 0 0; 0 0 1], "AbsTol", 1e-12);
                tc.verifyEqual(b.AxisAngle, [0 0 1 pi/2], "AbsTol", 1e-12);
                b.Quaternion = [cos(pi/8) sin(pi/8) 0 0];
                tc.verifyEqual(b.Orientation, phx.internal.Math.rotAA([1 0 0], pi/4), "AbsTol", 1e-12);
                tc.verifyEqual(b.Quaternion, [cos(pi/8) sin(pi/8) 0 0], "AbsTol", 1e-12);
                tc.verifyEqual(b.CurrentVertices(1:size(V, 1), :), V*b.Orientation', "AbsTol", 1e-12, ...
                    "The rest shape is turned.");
            end
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
            tc.verifySize(f.VonMisesStress, [size(Vnow, 1) 1]);
            tc.verifyTrue(all(isfinite(f.VonMisesStress)));
        end

        function lastVerticesOutliveTheSimulation(tc)
            % After the run CurrentVertices read the last state, the rest
            % shape stays the Vertices; a change of the rest shape (here
            % its Position) brings the rest shape back, and a new run
            % starts from it
            tc.requireEngine;
            [V, F] = tc.cube;
            bodies = {phx.SoftBody([], "Vertices", V, "Faces", F, "Position", [0 0 1], "Compliance", 1e-4), ...
                phx.FlexibleBody([], "Vertices", V, "Tetrahedra", delaunay(V), "Position", [0 0 1])};
            for k = 1:2
                b = bodies{k};
                rest = b.CurrentVertices;
                sim = tc.simulate({tc.floor, b});
                sim.step(0.5, 100, -1);
                last = b.CurrentVertices;
                delete(sim);
                tc.verifyEqual(b.CurrentVertices, last, class(b));
                tc.verifyLessThan(mean(last(:, 3)), mean(rest(:, 3)) - 0.05, class(b) + " fell.");
                tc.verifyEqual(b.Vertices, V, class(b) + " keeps its rest shape.");
                b.Position = [0 0 2];
                tc.verifyEqual(b.CurrentVertices, rest + [0 0 1], "AbsTol", 1e-12, class(b));
            end
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
