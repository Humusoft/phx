classdef tCapsuleShell < PhxTestCase
%tCapsuleShell Tests for the phx.shape.CapsuleShell geometry.
%
%   The size and mass tests are pure - computeMass is a method on a value
%   class. The drawing tests need a graphics session because the faces are
%   read back off the drawn primitive, and the body test needs the engine
%   (the engine's own contract of the shell is in tEngineProtocol).
%
%   See also phx.shape.CapsuleShell, tSphereShell, tEngineProtocol

%   Copyright 2026 HUMUSOFT s.r.o.

    % --- Sizes and mass ---------------------------------------------------
    methods (Test)
        function sizesFollowTheOuterDiameterAndTheWall(tc)
            % Diameter is the outside; the wall grows inwards from it
            s = phx.shape.CapsuleShell("Diameter", 1, "Height", 2, "WallThickness", 0.1);
            tc.verifyEqual(s.Radius, 0.5);
            tc.verifyEqual(s.InnerDiameter, 0.8, "RelTol", 1e-12);
            s.Radius = 1;
            tc.verifyEqual(s.Diameter, 2);
            tc.verifyEqual(s.InnerDiameter, 1.8, "RelTol", 1e-12);
            tc.verifyEqual([s.Height s.WallThickness], [2 0.1]);
        end

        function wallMustLeaveACavity(tc)
            % The wall is checked where the shape is used, so the sizes may
            % be set in any order on the way to a valid shell
            s = phx.shape.CapsuleShell("Diameter", 1);
            s.WallThickness = 2;
            tc.verifyError(@() s.computeMass, "phx:CapsuleShell:badWallThickness");
            s.Diameter = 5;
            tc.verifyWarningFree(@() s.computeMass);
            tc.verifyError(@() phx.shape.CapsuleShell("WallThickness", 0), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() phx.shape.CapsuleShell("Height", -1), "MATLAB:validators:mustBeNonnegative");
        end

        function massIsTheCapsuleLessItsCavity(tc)
            % The solid capsule of the outer face less that of the cavity, as
            % phx.shape.Capsule computes them, its axial moment along Axis
            for axis = ["x" "y" "z"]
                s = phx.shape.CapsuleShell("Diameter", 1, "Height", 2, "WallThickness", 0.1, "Axis", axis);
                [m, I, c] = s.computeMass;
                [mo, Io] = phx.shape.Capsule("Diameter", 1, "Height", 2, "Axis", axis).computeMass;
                [mi, Ii] = phx.shape.Capsule("Diameter", 0.8, "Height", 2, "Axis", axis).computeMass;
                tc.verifyEqual(m, mo - mi, "RelTol", 1e-12);
                tc.verifyEqual(I, Io - Ii, "RelTol", 1e-12);
                tc.verifyEqual(c, [0 0 0]);
            end
            % A thin wall weighs its surface times the thickness
            s = phx.shape.CapsuleShell("Diameter", 0.6, "Height", 1, "WallThickness", 1e-4);
            r = 0.3 - 0.5e-4;
            tc.verifyEqual(s.computeMass, 1000*1e-4*(2*pi*r*1 + 4*pi*r^2), "RelTol", 1e-6);
        end
    end

    % --- Drawing ----------------------------------------------------------
    methods (Test, TestTags = {'Graphics'})
        function drawsTheOuterFaceAndTheInnerTurnedToTheCavity(tc)
            % Every vertex lies on one of the two faces, around the segment of
            % the cylindrical part along Axis; the outer face faces out, the
            % inner one the cavity
            ax = tc.prepareAxes;
            s = phx.shape.CapsuleShell("Diameter", 1, "Height", 1, "WallThickness", 0.1, "Axis", "x");
            s.drawTo(ax);
            p = findobj(ax, "Type", "patch");
            tc.assertNumElements(p, 1);
            V = p.Vertices;
            F = p.Faces;
            Q = [max(min(V(:, 1), 0.5), -0.5) zeros(size(V, 1), 2)];
            d = vecnorm(V - Q, 2, 2);
            onOuter = abs(d - 0.5) < 1e-6;
            onInner = abs(d - 0.4) < 1e-6;
            tc.verifyTrue(all(onOuter | onInner), "Every vertex lies on a face.");
            tc.verifyTrue(any(onInner) && any(onOuter));
            n = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
            centre = (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :))/3;
            Qc = [max(min(centre(:, 1), 0.5), -0.5) zeros(size(centre, 1), 2)];
            outward = dot(n, centre - Qc, 2);
            proper = vecnorm(n, 2, 2) > 1e-12;   % not the slivers at the poles
            inner = all(onInner(F), 2);
            tc.verifyTrue(all(outward(proper & ~inner) > 0), "The outer face faces outwards.");
            tc.verifyTrue(all(outward(proper & inner) < 0), "The inner face faces the cavity.");
            N = p.VertexNormals;
            tc.verifyTrue(all(dot(N(onOuter, :), V(onOuter, :) - Q(onOuter, :), 2) > 0), "Outer normals point out.");
            tc.verifyTrue(all(dot(N(onInner, :), V(onInner, :) - Q(onInner, :), 2) < 0), "Inner normals point to the axis.");
        end

        function alphaIsTheOpacityOfTheWholeWall(tc)
            % A line of sight through the wall crosses both faces, so each
            % takes 1 - sqrt(1 - Alpha); the shape keeps the Alpha given
            a = 0.5;
            ax = tc.prepareAxes;
            b = phx.Body(ax, "Type", "static", "Shape", phx.shape.CapsuleShell("Alpha", a));
            p = findobj(b.Graphics, "Type", "patch");
            tc.verifyEqual(p.FaceAlpha, 1 - sqrt(1 - a), "AbsTol", 1e-12);
            tc.verifyEqual(tc.bodyShape(b).Alpha, a);
        end
    end

    % --- Bodies -----------------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function drumHoldsABallAndRestsOnAFloor(tc)
            % A ball thrown about in a static upright shell stays in its
            % cavity; a dynamic lying shell rests on a floor on its outer face
            % and weighs what computeMass gives
            tc.requireEngine;
            ax = tc.prepareAxes;
            phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], "Shape", {"Box", "Size", [6 6 0.1]});
            phx.Body(ax, "Type", "static", "Position", [0 0 1.5], ...
                "Shape", phx.shape.CapsuleShell("Diameter", 1, "Height", 1, "WallThickness", 0.04));
            ball = phx.Body(ax, "Position", [0.2 0 1.5], "Shape", {"Sphere", "Radius", 0.08});
            ball.LinearVelocity = [3 2 4];
            shell = phx.shape.CapsuleShell("Diameter", 0.4, "Height", 0.6, "Axis", "x", "WallThickness", 0.02);
            drum = phx.Body(ax, "Position", [2 0 0.5], "Shape", shell);
            sim = phx.Simulation(ax);
            tc.addTeardown(@() delete(sim));
            farthest = 0;
            for k = 1:200
                sim.step(0.01, 2, -1);
                p = ball.Position - [0 0 1.5];
                farthest = max(farthest, norm(p - [0 0 max(min(p(3), 0.5), -0.5)]));
            end
            tc.verifyLessThan(farthest, 0.46 - 0.08 + 5e-3, "Farthest the ball's centre got from the shell's segment (m)");
            tc.verifyEqual(drum.Position(3), 0.2, "AbsTol", 2e-3, "Height of the resting drum (m)");
            tc.verifyEqual(drum.Mass, shell.computeMass, "RelTol", 1e-12);
        end
    end

end
