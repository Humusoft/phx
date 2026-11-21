classdef tSphereShell < PhxTestCase
%tSphereShell Tests for the phx.shape.SphereShell geometry.
%
%   The size and mass tests are pure - computeMass is a method on a value
%   class. The drawing tests need a graphics session because the faces are
%   read back off the drawn primitive, and the body tests need the engine
%   (the engine's own contract of the shell is in tEngineProtocol).
%
%   See also phx.shape.SphereShell, tEngineProtocol, tTube

%   Copyright 2026 HUMUSOFT s.r.o.

    % --- Sizes and mass ---------------------------------------------------
    methods (Test)
        function sizesFollowTheOuterDiameterAndTheWall(tc)
            % Diameter is the outside; the wall grows inwards from it
            s = phx.shape.SphereShell("Diameter", 1, "WallThickness", 0.1);
            tc.verifyEqual(s.Radius, 0.5);
            tc.verifyEqual(s.InnerDiameter, 0.8, "RelTol", 1e-12);
            s.Radius = 1;
            tc.verifyEqual(s.Diameter, 2);
            tc.verifyEqual(s.InnerDiameter, 1.8, "RelTol", 1e-12);
            tc.verifyEqual(s.WallThickness, 0.1);
        end

        function wallMustLeaveACavity(tc)
            % The wall is checked where the shape is used, so the sizes may
            % be set in any order on the way to a valid shell
            s = phx.shape.SphereShell("Diameter", 1);
            s.WallThickness = 2;
            tc.verifyError(@() s.computeMass, "phx:SphereShell:badWallThickness");
            s.Diameter = 5;
            tc.verifyWarningFree(@() s.computeMass);
            s.WallThickness = 2.5;
            tc.verifyError(@() s.computeMass, "phx:SphereShell:badWallThickness");
            tc.verifyError(@() phx.shape.SphereShell("WallThickness", 0), "MATLAB:validators:mustBePositive");
        end

        function massMatchesTheShellFormulas(tc)
            s = phx.shape.SphereShell("Diameter", 1, "WallThickness", 0.1, "Density", 1000);
            [m, I, c] = s.computeMass;
            ro = 0.5; ri = 0.4;
            tc.verifyEqual(m, 1000*4/3*pi*(ro^3 - ri^3), "RelTol", 1e-12);
            tc.verifyEqual(I, [1 1 1]*2/5*m*(ro^5 - ri^5)/(ro^3 - ri^3), "RelTol", 1e-12);
            tc.verifyEqual(c, [0 0 0]);
            % A thin wall tends to the surface's 2/3 m r^2
            s.WallThickness = 1e-4;
            [m, I] = s.computeMass;
            tc.verifyEqual(I, [1 1 1]*2/3*m*0.5^2, "RelTol", 1e-3);
        end
    end

    % --- Drawing ----------------------------------------------------------
    methods (Test, TestTags = {'Graphics'})
        function drawsTheOuterFaceAndTheInnerTurnedToTheCavity(tc)
            % Every vertex lies on one of the two faces, and the faces enclose
            % the wall: outward on the outside, towards the cavity inside
            ax = tc.prepareAxes;
            s = phx.shape.SphereShell("Diameter", 1, "WallThickness", 0.1, "Segments", 24);
            s.drawTo(ax);
            p = findobj(ax, "Type", "patch");
            tc.assertNumElements(p, 1);
            V = p.Vertices;
            F = p.Faces;
            r = vecnorm(V, 2, 2);
            onOuter = abs(r - 0.5) < 1e-6;
            onInner = abs(r - 0.4) < 1e-6;
            tc.verifyTrue(all(onOuter | onInner), "Every vertex lies on a face.");
            tc.verifyTrue(any(onInner) && any(onOuter));
            n = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
            centre = (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :))/3;
            outward = dot(n, centre, 2);
            proper = vecnorm(n, 2, 2) > 1e-12;   % not the slivers at the poles
            inner = all(onInner(F), 2);
            tc.verifyTrue(all(outward(proper & ~inner) > 0), "The outer face faces outwards.");
            tc.verifyTrue(all(outward(proper & inner) < 0), "The inner face faces the cavity.");
            N = p.VertexNormals;
            tc.verifyTrue(all(dot(N(onOuter, :), V(onOuter, :), 2) > 0), "Outer normals point out.");
            tc.verifyTrue(all(dot(N(onInner, :), V(onInner, :), 2) < 0), "Inner normals point to the centre.");
        end

        function alphaIsTheOpacityOfTheWholeWall(tc)
            % A line of sight through the wall crosses both faces, so each
            % takes 1 - sqrt(1 - Alpha); the shape keeps the Alpha given
            a = 0.5;
            face = 1 - sqrt(1 - a);
            ax = tc.prepareAxes;
            b = phx.Body(ax, "Type", "static", "Shape", phx.shape.SphereShell("Alpha", a));
            p = tc.primitiveOf(b);
            tc.verifyEqual(p.FaceAlpha, face, "AbsTol", 1e-12, "Patch drawing");
            tc.verifyEqual(tc.bodyShape(b).Alpha, a);

            setappdata(ax, "phxAxes", true);   % the world-primitive path of the Viewer
            b = phx.Body(ax, "Type", "static", "Shape", phx.shape.SphereShell("Alpha", a));
            p = tc.primitiveOf(b);
            tc.verifyEqual(string(p.ColorType), "truecoloralpha");
            tc.verifyEqual(double(p.ColorData(4)), round(255*face), "World-primitive drawing");
            b = phx.Body(ax, "Type", "static", "Shape", phx.shape.SphereShell);
            tc.verifyEqual(string(tc.primitiveOf(b).ColorType), "truecolor", "An opaque shell stays truecolor.");
        end
    end

    % --- Bodies -----------------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function shellHoldsABallAndRestsOnAFloor(tc)
            % A ball thrown about in a static shell stays in its cavity; a
            % dynamic shell rests on a floor on its outer face and weighs
            % what computeMass gives
            tc.requireEngine;
            ax = tc.prepareAxes;
            phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], "Shape", {"Box", "Size", [6 6 0.1]});
            tank = phx.shape.SphereShell("Diameter", 2, "WallThickness", 0.05);
            phx.Body(ax, "Type", "static", "Position", [0 0 1.2], "Shape", tank);
            ball = phx.Body(ax, "Position", [0.3 0 1.2], "Shape", {"Sphere", "Radius", 0.1});
            ball.LinearVelocity = [3 2 1];
            shell = phx.shape.SphereShell("Diameter", 0.4, "WallThickness", 0.02);
            free = phx.Body(ax, "Position", [2 0 0.5], "Shape", shell);
            sim = phx.Simulation(ax);
            tc.addTeardown(@() delete(sim));
            farthest = 0;
            for k = 1:200
                sim.step(0.01, 2, -1);
                farthest = max(farthest, norm(ball.Position - [0 0 1.2]));
            end
            tc.verifyLessThan(farthest, 0.95 - 0.1 + 5e-3, "Farthest the ball's centre got from the shell's (m)");
            tc.verifyEqual(free.Position(3), 0.2, "AbsTol", 2e-3, "Height of the resting shell (m)");
            tc.verifyEqual(free.Mass, shell.computeMass, "RelTol", 1e-12);
        end
    end

    methods (Access = private)
        function p = primitiveOf(~, body)
            % The primitive the body's shape drew
            p = [];
            for ch = body.Graphics.Children'
                if ~isempty(getappdata(ch, "phxShape"))
                    p = ch;
                    return
                end
            end
        end
    end

end
