classdef tBoxShell < PhxTestCase
%tBoxShell Tests for the phx.shape.BoxShell geometry.
%
%   The size and mass tests are pure - computeMass is a method on a value
%   class. The drawing tests need a graphics session because the faces are
%   read back off the drawn primitive, and the body test needs the engine
%   (the engine's own contract of the shell is in tEngineProtocol).
%
%   See also phx.shape.BoxShell, tSphereShell, tCapsuleShell, tEngineProtocol

%   Copyright 2026 HUMUSOFT s.r.o.

    % --- Sizes and mass ---------------------------------------------------
    methods (Test)
        function innerSizeIsTheOuterLessTwoWalls(tc)
            s = phx.shape.BoxShell("Size", [1.2 0.8 1], "WallThickness", 0.04);
            tc.verifyEqual(s.InnerSize, [1.12 0.72 0.92], "AbsTol", 1e-12);
            tc.verifyEqual(s.InnerEdgeRadius, 0);
        end

        function wallMustLeaveACavity(tc)
            % Half the size must exceed the wall plus the rounding in every
            % direction; checked where the shape is used, in any order of
            % setting
            s = phx.shape.BoxShell("Size", [1 1 0.1]);
            tc.verifyError(@() s.computeMass, "phx:BoxShell:badWallThickness");
            s.Size = [1 1 1];
            tc.verifyWarningFree(@() s.computeMass);
            s.InnerEdgeRadius = 0.45;
            tc.verifyError(@() s.computeMass, "phx:BoxShell:badWallThickness");
            tc.verifyError(@() phx.shape.BoxShell("InnerEdgeRadius", -0.1), "MATLAB:validators:mustBeNonnegative");
        end

        function massIsTheRoundedBoxLessItsCavity(tc)
            % The volume of a box grown by r is known in closed form; the
            % mass follows it with sharp and with rounded cavity edges, and a
            % cube shell turns alike about every axis
            grown = @(h, r) prod(2*h) + 8*r*(h(1)*h(2) + h(2)*h(3) + h(1)*h(3)) + 2*pi*r^2*sum(h) + 4/3*pi*r^3;
            for ri = [0 0.02]
                s = phx.shape.BoxShell("Size", [1.2 0.8 1], "WallThickness", 0.04, "InnerEdgeRadius", ri, "Density", 1000);
                core = [0.6 0.4 0.5] - 0.04 - ri;
                [m, ~, c] = s.computeMass;
                tc.verifyEqual(m, 1000*(grown(core, ri + 0.04) - grown(core, ri)), "RelTol", 1e-3);
                tc.verifyEqual(c, [0 0 0]);
            end
            [~, I] = phx.shape.BoxShell("Size", [1 1 1], "WallThickness", 0.05).computeMass;
            tc.verifyEqual(I, I(1)*[1 1 1], "RelTol", 1e-6);   % the mesh has its poles on z
        end
    end

    % --- Drawing ----------------------------------------------------------
    methods (Test, TestTags = {'Graphics'})
        function drawsTheOuterFaceAndTheInnerTurnedToTheCavity(tc)
            % Every vertex lies on one of the two faces, at the signed
            % distances of the rounding from the core; each face, sharp or
            % rounded, is closed and faces out of the wall
            for ri = [0 0.01]
                ax = tc.prepareAxes;
                s = phx.shape.BoxShell("Size", [1.2 0.8 1], "WallThickness", 0.04, "InnerEdgeRadius", ri);
                s.drawTo(ax);
                p = findobj(ax, "Type", "patch");
                tc.assertNumElements(p, 1);
                V = p.Vertices;
                F = p.Faces;
                core = [0.6 0.4 0.5] - 0.04 - ri;
                d = fromBox(V, core);
                tc.verifyTrue(all(abs(d - ri - 0.04) < 1e-6 | abs(d - ri) < 1e-6), "Every vertex lies on a face.");
                centre = (V(F(:, 1), :) + V(F(:, 2), :) + V(F(:, 3), :))/3;
                n = cross(V(F(:, 2), :) - V(F(:, 1), :), V(F(:, 3), :) - V(F(:, 1), :), 2);
                n = n./vecnorm(n, 2, 2);
                rising = fromBox(centre + 1e-6*n, core) - fromBox(centre, core);
                inner = fromBox(centre, core) < ri + 0.02;
                tc.verifyTrue(all(rising(~inner) > 0), "The outer face faces outwards.");
                tc.verifyTrue(all(rising(inner) < 0), "The inner face faces the cavity.");
                a = V(F(:, 1), :); b = V(F(:, 2), :); c = V(F(:, 3), :);
                volume = sum(dot(a, cross(b, c, 2), 2))/6;
                tc.verifyEqual(volume, s.computeMass/s.Density, "RelTol", 5e-3, "The faces enclose the wall.");
            end
        end

        function alphaIsTheOpacityOfTheWholeWall(tc)
            % A line of sight through the wall crosses both faces, so each
            % takes 1 - sqrt(1 - Alpha); the shape keeps the Alpha given
            a = 0.5;
            ax = tc.prepareAxes;
            b = phx.Body(ax, "Type", "static", "Shape", phx.shape.BoxShell("Alpha", a));
            p = findobj(b.Graphics, "Type", "patch");
            tc.verifyEqual(p.FaceAlpha, 1 - sqrt(1 - a), "AbsTol", 1e-12);
            tc.verifyEqual(tc.bodyShape(b).Alpha, a);
        end
    end

    % --- Bodies -----------------------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function tankHoldsABallAndACrateRestsOnAFloor(tc)
            % A ball thrown about in a static shell stays in its cavity; a
            % dynamic shell rests on a floor on its flat bottom and weighs
            % what computeMass gives
            tc.requireEngine;
            ax = tc.prepareAxes;
            phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], "Shape", {"Box", "Size", [6 6 0.1]});
            phx.Body(ax, "Type", "static", "Position", [0 0 1], ...
                "Shape", phx.shape.BoxShell("Size", [1.2 0.8 1], "WallThickness", 0.04, "InnerEdgeRadius", 0.01));
            ball = phx.Body(ax, "Position", [0.2 0.1 1], "Shape", {"Sphere", "Radius", 0.05});
            ball.LinearVelocity = [4 3 2];
            shell = phx.shape.BoxShell("Size", [0.5 0.5 0.4], "WallThickness", 0.02);
            crate = phx.Body(ax, "Position", [2 0 0.5], "Shape", shell);
            sim = phx.Simulation(ax);
            tc.addTeardown(@() delete(sim));
            farthest = -inf;
            for k = 1:200
                sim.step(0.01, 2, -1);
                farthest = max(farthest, max(abs(ball.Position - [0 0 1]) - [0.56 0.36 0.46]));
            end
            tc.verifyLessThan(farthest, -0.05 + 2e-3, "Farthest the ball's surface got past the cavity (m)");
            tc.verifyEqual(crate.Position(3), 0.2, "AbsTol", 2e-3, "Height of the resting crate (m)");
            tc.verifyEqual(crate.Mass, shell.computeMass, "RelTol", 1e-12);
        end
    end

end

function d = fromBox(P, half)
% Signed distances of the points P (Nx3) from the centred box of the given
% half extents
q = abs(P) - half;
d = vecnorm(max(q, 0), 2, 2) + min(max(q, [], 2), 0);
end
