classdef tTube < PhxTestCase
%tTube Tests for the phx.shape.Tube geometry.
%
%   The property and mass tests are pure - computeMass is a method on a value
%   class. The mesh tests need a graphics session because the triangulation is
%   read back off the drawn primitive, and the envelope tests need the engine.
%
%   See also phx.shape.Tube, tShapeMass, tRevolutionEnvelope

%   Copyright 2026 HUMUSOFT s.r.o.

    properties
        Ax
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.Ax = tc.prepareAxes;
        end
    end

    % --- Properties and mass --------------------------------------------
    methods (Test)
        function outerSizeFollowsTheBoreAndTheWall(tc)
            s = phx.shape.Tube("InnerDiameter", 0.8, "WallThickness", 0.1);
            tc.verifyEqual(s.Diameter, 1, "RelTol", 1e-12);
            tc.verifyEqual(s.Radius, 0.5, "RelTol", 1e-12);
            tc.verifyEqual(s.InnerRadius, 0.4, "RelTol", 1e-12);
        end

        function settingTheOuterSizeKeepsTheWall(tc)
            % The stored pair is the bore and the wall, so writing an outer
            % dimension has to come back out of the dependent getter intact.
            s = phx.shape.Tube("WallThickness", 0.1);
            s.Diameter = 2;
            tc.verifyEqual(s.Diameter, 2, "RelTol", 1e-12);
            tc.verifyEqual(s.InnerDiameter, 1.8, "RelTol", 1e-12);
            s.Radius = 3;
            tc.verifyEqual(s.Diameter, 6, "RelTol", 1e-12);
            s.InnerRadius = 1;
            tc.verifyEqual(s.InnerDiameter, 2, "RelTol", 1e-12);
            tc.verifyEqual(s.WallThickness, 0.1, "RelTol", 1e-12);
        end

        function taperIsLimitedToTheClosedRange(tc)
            tc.verifyError(@() phx.shape.Tube("Taper", 1.5), ...
                "MATLAB:validators:mustBeInRange");
            tc.verifyWarningFree(@() phx.shape.Tube("Taper", -1));
        end

        function straightTubeMatchesTheAnnulusFormulas(tc)
            % Enough segments that the faceted mesh converges on the ideal
            % annular cylinder it approximates.
            s = phx.shape.Tube("InnerDiameter", 0.8, "WallThickness", 0.1, ...
                "Height", 2, "Segments", 720, "Density", 1000);
            [m, I] = s.computeMass;
            ro = 0.5; ri = 0.4; h = 2;
            tc.verifyEqual(m, 1000*pi*(ro^2 - ri^2)*h, "RelTol", 1e-4);
            ia = m*(ro^2 + ri^2)/2;                    % about the tube axis
            io = m*(3*(ro^2 + ri^2) + h^2)/12;         % about transverse axes
            tc.verifyEqual(I, [io io ia], "RelTol", 1e-4);
        end

        function taperRemovesMaterialSymmetrically(tc)
            % The two signs are the same shape flipped end for end, and a
            % narrower bore means more wall left standing.
            mass = @(t) massOf(phx.shape.Tube("Taper", t));
            tc.verifyEqual(mass(0.5), mass(-0.5), "RelTol", 1e-12);
            tc.verifyLessThan(mass(0.5), mass(0));
            tc.verifyLessThan(mass(1), mass(0.9));
        end
    end

    % --- Mesh ------------------------------------------------------------
    methods (Test, TestTags = {'Graphics'})
        function meshIsClosedAndFacesOutwards(tc)
            for t = [-1 -0.5 0 0.5 0.9 1]
                [V, F] = tc.meshOf(phx.shape.Tube("Taper", t));
                tc.verifyTrue(isWatertight(V, F), ...
                    "Tube mesh has a hole at Taper = " + t);
                tc.verifyGreaterThan(signedVolume(V, F), 0, ...
                    "Tube mesh is wound inside out at Taper = " + t);
            end
        end

        function taperNarrowsTheEndOnThePositiveSideOfTheAxis(tc)
            [V, ~] = tc.meshOf(phx.shape.Tube("Taper", 0.5, "Height", 1));
            tc.verifyEqual(endRadius(V, -1), 0.5, "AbsTol", 1e-6);
            tc.verifyEqual(endRadius(V, +1), 0.45*(1 - 0.5) + 0.05, "AbsTol", 1e-6);

            % The negative sign narrows the opposite end
            [V, ~] = tc.meshOf(phx.shape.Tube("Taper", -0.5, "Height", 1));
            tc.verifyEqual(endRadius(V, +1), 0.5, "AbsTol", 1e-6);
        end

        function closedTaperLeavesJustTheWallAtTheTip(tc)
            % Taper 1 pinches the bore shut, so the tip is a disc as wide as
            % the wall is thick - not a degenerate point.
            s = phx.shape.Tube("Taper", 1, "WallThickness", 0.08);
            [V, ~] = tc.meshOf(s);
            tc.verifyEqual(endRadius(V, +1), 0.08, "AbsTol", 1e-6);
        end
    end

    % --- Collision envelopes ---------------------------------------------
    methods (Test, TestTags = {'Engine'})
        function everyEnvelopeBuildsABody(tc)
            tc.requireEngine;
            for env = ["concave" "convex" "cylinder"]
                b = phx.Body(tc.Ax, "Type", "static", ...
                    "Shape", phx.shape.Tube("Envelope", env));
                sim = phx.Simulation(tc.Ax);
                tc.addTeardown(@() delete(sim));
                tc.verifyGreaterThan(b.Mass, 0);
                tc.verifyWarningFree(@() sim.step(0.05, 5, -1));
                delete(sim);
                delete(b);
            end
        end

        function concaveBoreStaysOpen(tc)
            % The defining property of the shape: a body narrower than the
            % throat falls through, a wider one is caught by the taper.
            tc.requireEngine;
            zEnd = @(r) tc.dropBall(r);
            tc.verifyGreaterThan(zEnd(0.15), -0.5);  % caught, throat is 0.045
            tc.verifyLessThan(zEnd(0.02), -1);       % through and still falling
        end
    end

    methods (Access = private)
        function [V, F] = meshOf(tc, shape)
            % The primitive keeps its vertices in single precision, so
            % readings off it are only good to about 1e-6.
            % Triangulation of a shape, read back off the drawn primitive.
            cla(tc.Ax);
            shape.drawTo(tc.Ax);
            p = findobj(tc.Ax, "Type", "patch");
            tc.assertNotEmpty(p, "The shape drew no patch to read back.");
            V = p(1).Vertices;
            F = p(1).Faces;
        end

        function z = dropBall(tc, radius)
            ax = tc.prepareAxes;
            phx.Body(ax, "Type", "static", "Shape", ...
                phx.shape.Tube("Taper", -0.9, "Envelope", "concave", "Segments", 48));
            ball = phx.Body(ax, "Position", [0 0 0.8], ...
                "Shape", phx.shape.Sphere("Diameter", radius*2));
            sim = phx.Simulation(ax);
            sim.step(2, 1000, -1);
            z = ball.Position(3);
            delete(sim);
        end
    end

end

function m = massOf(shape)
m = shape.computeMass;
end

function r = endRadius(V, side)
% Largest radius about the z axis among the vertices at one end of the tube
k = V(:, 3)*side > 0.4*max(abs(V(:, 3)));
r = max(sqrt(sum(V(k, 1:2).^2, 2)));
end

function v = signedVolume(V, F)
a = V(F(:, 1), :); b = V(F(:, 2), :); c = V(F(:, 3), :);
v = sum(dot(a, cross(b, c, 2), 2))/6;
end

function ok = isWatertight(V, F)
% Every edge of a closed surface is shared by exactly two triangles
[~, ~, ic] = unique(round(V, 9), "rows");
G = ic(F);
E = sort([G(:, [1 2]); G(:, [2 3]); G(:, [3 1])], 2);
E(E(:, 1) == E(:, 2), :) = [];
[~, ~, ie] = unique(E, "rows");
ok = all(accumarray(ie, 1) == 2);
end
