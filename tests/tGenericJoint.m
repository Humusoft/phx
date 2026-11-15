classdef tGenericJoint < PhxTestCase
%tGenericJoint Integration tests of the generic 6-DOF joint (MEX).
%
%   These pin down what the limits of phx.GenericJoint mean: an axis with both
%   limits equal is locked at that value - which, with the all-zero defaults,
%   makes a bare joint rigid - an axis limited to [-Inf Inf] is free, and
%   anything in between bounds the travel. The
%   angular limits are checked in both directions, as a sign slip in the
%   chain to the engine would silently mirror every bounded rotation. The
%   limits write through to a running simulation.
%
%   Only what is specific to this joint lives here; what it shares with the
%   rest of the family - the joint frame properties inherited from
%   phx.base.Joint, the reaction feedback and holding two bodies together -
%   is in tJointContract.
%
%   See also phx.GenericJoint, phx.base.Joint, tJointContract

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        G = 9.81
        % Every axis free, i.e. the opposite of the all-locked default
        FREE = {"LinearLimits", repmat([-Inf Inf], 3, 1), "AngularLimits", repmat([-Inf Inf], 3, 1)}
    end

    methods (TestClassSetup)
        function engineIsPresent(tc)
            tc.requireEngine;
        end
    end

    methods (Test, TestTags = {'Engine'})
        function defaultLimitsLockEveryAxis(tc)
            % Every limit defaults to zero, i.e. lower == upper on all six axes
            [~, b, j, sim] = tc.scene();
            sim.step(1, 500, -1);
            tc.verifyEqual(b.Position, [0 0 0], "AbsTol", 1e-3, ...
                "A joint with the default limits must hold the body rigidly.");
            tc.verifyEqual(b.EulerAngles, [0 0 0], "AbsTol", 1e-3);
            % The joint carries the whole weight of the body
            tc.verifyEqual(norm(j.ForceA), b.Mass*tc.G, "RelTol", 0.02);
        end

        function infiniteLimitsFreeEveryAxis(tc)
            % [-Inf Inf] on every axis leaves the body unconstrained
            [~, b, ~, sim] = tc.scene(tc.FREE{:});
            sim.step(1, 500, -1);
            tc.verifyEqual(b.Position(3), -0.5*tc.G, "RelTol", 0.02);
            tc.verifyEqual(b.Position(1:2), [0 0], "AbsTol", 1e-6);
        end

        function equalLimitsLockTheAxisAtThatValue(tc)
            [~, b, ~, sim] = tc.scene(...
                "LinearLimits", [0 0; 0 0; -0.3 -0.3], ...
                "AngularLimits", [0 0; 0 0; 0.3 0.3]);
            sim.step(1, 1000, -1);
            tc.verifyEqual(b.Position(3), -0.3, "AbsTol", 1e-3);
            tc.verifyEqual(b.EulerAngles(3), 0.3, "AbsTol", 1e-3);
        end

        function linearStopBoundsTheTravel(tc)
            [~, b, ~, sim] = tc.scene(...
                "LinearLimits", [0 0; 0 0; -0.5 0]);
            sim.step(0.3, 300, -1);
            tc.verifyGreaterThan(b.Position(3), -0.5, ...
                "The body should still be travelling between the stops.");
            sim.step(1, 500, -1);
            tc.verifyEqual(b.Position(3), -0.5, "AbsTol", 0.02);
            tc.verifyEqual(b.Position(1:2), [0 0], "AbsTol", 1e-3);
        end

        function angularLimitsFollowTheRightHandedSense(tc)
            % A range of [0.2 0.5] must be reachable as a positive rotation:
            % driving the body either way parks it on one of the two stops.
            for ax = 1:3
                for s = [1 -1]
                    limits = zeros(3, 2);
                    limits(ax, :) = [0.2 0.5];
                    [~, b, ~, sim] = tc.scene(...
                        "AngularLimits", limits);
                    torque = [0 0 0];
                    torque(ax) = s*50;
                    for k = 1:100
                        b.applyTorque(torque);
                        sim.step(0.01, 10, -1);
                    end
                    expected = 0.2 + 0.3*(s > 0);
                    tc.verifyEqual(b.EulerAngles(ax), expected, "AbsTol", 0.02, ...
                        sprintf("Axis %d driven by torque %+d.", ax, s));
                    delete(sim);
                end
            end
        end

        function freeAngularAxisActsAsAHinge(tc)
            % Pivot at the origin, bob one metre away: the arm swings under
            % gravity and the joint keeps it on its circle.
            [~, b, ~, sim] = tc.pendulum(...
                "AngularLimits", [-Inf Inf; 0 0; 0 0]);
            sim.step(0.4, 400, -1);
            p = b.Position;
            tc.verifyEqual(norm(p), 1, "AbsTol", 1e-3, "The pivot must hold.");
            tc.verifyEqual(p(1), 0, "AbsTol", 1e-3, "The swing must stay in the Y-Z plane.");
            tc.verifyGreaterThan(atan2(-p(3), p(2)), 0.1, "The arm should have swung down.");
        end

        function angularStopCatchesTheSwing(tc)
            % Gravity drives the arm towards a negative rotation about +X, so
            % it comes to rest against the lower stop.
            [~, b, ~, sim] = tc.pendulum(...
                "AngularLimits", [-0.2 1.5; 0 0; 0 0]);
            sim.step(3, 3000, -1);
            p = b.Position;
            tc.verifyEqual(atan2(-p(3), p(2)), 0.2, "AbsTol", 0.02);
        end

        function limitsSetDuringTheRunTakeEffect(tc)
            % Starting rigid, the joint lets the body drop onto a stop
            % opened mid-run, moves with the stop, and lets go when freed
            [~, b, j, sim] = tc.scene();
            sim.step(0.5, 250, -1);
            tc.verifyEqual(b.Position, [0 0 0], "AbsTol", 1e-3);

            j.LinearLimits(3, 1) = -0.3;
            sim.step(1, 500, -1);
            tc.verifyEqual(b.Position(3), -0.3, "AbsTol", 2e-3, ...
                "The stop opened during the run did not take effect.");

            j.LinearLimits(3, 1) = -0.5;
            sim.step(1, 500, -1);
            tc.verifyEqual(b.Position(3), -0.5, "AbsTol", 2e-3);

            set(j, tc.FREE{:});
            sim.step(0.5, 250, -1);
            tc.verifyLessThan(b.Position(3), -1, "The freed body did not fall.");
        end

        function rotatedJointFrameRotatesTheConstrainedAxes(tc)
            % Turn the joint frame 90 deg about Y so that its X axis points
            % along world -Z; the freed axis then lets the body sink.
            [~, b, ~, sim] = tc.scene(...
                "EulerAnglesA", [0 pi/2 0], "EulerAnglesB", [0 pi/2 0], ...
                "LinearLimits", [0 0.5; 0 0; 0 0]);
            sim.step(1, 1000, -1);
            tc.verifyEqual(b.Position(3), -0.5, "AbsTol", 0.02);
            tc.verifyEqual(b.Position(1:2), [0 0], "AbsTol", 1e-3);
        end

    end

    methods (Access = private)
        function b = cube(tc, position, type)
            arguments
                tc
                position (1, 3) double
                type (1, 1) string = "dynamic"
            end
            b = tc.spawnBody(position, "Type", type, ...
                "Shape", {"Box", "Size", [0.2 0.2 0.2]});
        end

        function [a, b, j, sim] = scene(tc, varargin)
            % Static anchor and a dynamic body, both at the origin
            a = tc.cube([0 0 0], "static");
            b = tc.cube([0 0 0]);
            j = phx.GenericJoint(a, b, varargin{:});
            sim = phx.Simulation([a b], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(sim));
        end

        function [a, b, j, sim] = pendulum(tc, varargin)
            % Pivot at the world origin, bob one metre away along +Y
            a = tc.cube([0 0 0], "static");
            b = tc.cube([0 1 0]);
            j = phx.GenericJoint(a, b, "PointA", [0 0 0], "PointB", [0 -1 0], varargin{:});
            sim = phx.Simulation([a b], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(sim));
        end
    end

end
