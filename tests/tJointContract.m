classdef tJointContract < PhxTestCase
%tJointContract Behavior every phx joint class shares.
%
%   These were written once per joint class and only ever differed in the
%   constructor call, so they are run here over the whole family instead:
%
%     * the reaction feedback (ForceA/TorqueA/ForceB/TorqueB) reads NaN until
%       the joint has been initialized - it is a window into the engine, and
%       an uninitialized joint has none, so it must not report a plausible
%       zero. This is implemented on phx.base.Joint and holds for every joint,
%       including those (gear, bushing) that constrain no position at all;
%     * a joint between two unanchored bodies holds them together while they
%       fall: their anchor points stay coincident and the pair drops as one
%       rigid piece. Joints with a free translation are given an axis across
%       gravity, so that the free degree of freedom is not the one gravity
%       pulls along. The compliant bushing and the gear joint, which couples
%       rotation only, do not make this promise and are left out.
%
%   The bodies are headless ([] axes), so only the engine is needed.
%
%   See also phx.base.Joint, tGenericJoint, tCylindricalJoint, tJointFrames

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        G = 9.81
        Cube = {"Box", "Size", [0.2 0.2 0.2]}
        Span = 0.5          % distance between the two bodies
    end

    properties (TestParameter)
        % Every joint class, for the contracts that need no constraint
        Joint = tJointContract.allJoints
        % Those that really hold two bodies in place relative to each other
        RigidJoint = tJointContract.rigidJoints
    end

    methods (Test)
        function frameHelpersRoundTrip(tc, Joint)
            % PointA/PointB and the rotation helpers are windows into
            % TransformA/TransformB and must survive a round trip as 1x3 rows.
            j = tc.mkJoint(Joint, "PointA", [0.1 0.2 0.3], ...
                "EulerAnglesA", [0.1 0.2 0.3]);

            tc.verifySize(j.PointA, [1 3]);
            tc.verifySize(j.PointB, [1 3]);
            tc.verifyEqual(j.PointA, [0.1 0.2 0.3], "AbsTol", 1e-12);
            tc.verifyEqual(j.TransformA(1:3, 4)', [0.1 0.2 0.3], "AbsTol", 1e-12);
            tc.verifyEqual(j.EulerAnglesA, [0.1 0.2 0.3], "AbsTol", 1e-9);

            j.PointB = [0.4 0.5 0.6];
            tc.verifyEqual(j.PointB, [0.4 0.5 0.6], "AbsTol", 1e-12);

            j.AxisAngleB = [0 0 1 0.5];
            tc.verifyEqual(j.AxisAngleB, [0 0 1 0.5], "AbsTol", 1e-9);
            tc.verifyEqual(j.TransformB(1:3, 1:3)'*j.TransformB(1:3, 1:3), eye(3), ...
                "AbsTol", 1e-12);
        end

        function axisPropertiesReadBackExactly(tc, Joint)
            % AxisA/AxisB are a view of the third column of the joint frames,
            % so a normalized direction must come back bit-for-bit, and the
            % connecting point must survive an axis change untouched.
            j = tc.mkJoint(Joint);

            tc.verifyEqual(j.AxisA, [0 0 1], "The default axis must be Z.");
            tc.verifyEqual(j.AxisB, [0 0 1], "The default axis must be Z.");

            j.PointA = [0.1 0.2 0.3];
            j.AxisA = [0 1 0];
            tc.verifyEqual(j.AxisA, [0 1 0]);
            tc.verifyEqual(j.PointA, [0.1 0.2 0.3], ...
                "Setting the axis must not disturb the connecting point.");
            tc.verifyEqual(j.TransformA(1:3, 1:3)'*j.TransformA(1:3, 1:3), eye(3), ...
                "AbsTol", 1e-15);

            j.AxisB = [0 0 4];
            tc.verifyEqual(j.AxisB, [0 0 1], "The direction must be normalized.");

            tc.verifyError(@() setZeroAxis(j), "phx:Joint:invalidAxis");
            function setZeroAxis(j)
                j.AxisA = [0 0 0];
            end
        end

        function feedbackIsNaNBeforeInitialization(tc, Joint)
            j = tc.mkJoint(Joint);
            tc.verifyEqual(j.ForceA, [NaN NaN NaN]);
            tc.verifyEqual(j.TorqueA, [NaN NaN NaN]);
            tc.verifyEqual(j.ForceB, [NaN NaN NaN]);
            tc.verifyEqual(j.TorqueB, [NaN NaN NaN]);
        end
    end

    methods (Test, TestTags = {'Engine'})
        function jointHoldsTwoDynamicBodiesTogether(tc, RigidJoint)
            tc.requireEngine;
            a = tc.spawnBody([0 0 5], "Shape", tc.Cube);
            b = tc.spawnBody([tc.Span 0 5], "Shape", tc.Cube);
            tc.addTeardown(@() delete([a b]));

            j = feval(RigidJoint.Class, a, b, "PointA", [tc.Span 0 0], ...
                RigidJoint.Args{:});
            sim = phx.Simulation([a b], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(sim));

            sim.step(1, 1000, -1);

            tc.verifyEqual(norm(b.Position - a.Position), tc.Span, "AbsTol", 0.01, ...
                "The pair did not stay together.");
            tc.verifyEqual(5 - a.Position(3), 0.5*tc.G, "RelTol", 0.02, ...
                "The pair did not fall freely.");
            tc.verifyAnchorsCoincide(j, 0.01);
        end
    end

    methods (Access = private)
        function j = mkJoint(tc, Joint, varargin)
            % An uninitialized joint between two headless bodies, for the
            % property-level contracts. The bodies keep it alive.
            a = tc.spawnBody([0 0 0], "Type", "static");
            b = tc.spawnBody([1 0 0]);
            tc.addTeardown(@() delete([a b]));
            j = feval(Joint.Class, a, b, varargin{:});
        end
    end

    methods (Static, Access = private)
        function s = allJoints()
            s = tJointContract.rigidJoints;
            % Constrain no position, so they sit out the holding contract
            s.bushing = struct("Class", "phx.BushingJoint", "Args", {{}});
            s.gear = struct("Class", "phx.GearJoint", "Args", {{}});
        end

        function s = rigidJoints()
            acrossGravity = {"AxisA", [1 0 0], "AxisB", [1 0 0]};
            s.fixed = struct("Class", "phx.FixedJoint", "Args", {{}});
            s.revolute = struct("Class", "phx.RevoluteJoint", "Args", {{}});
            s.spherical = struct("Class", "phx.SphericalJoint", "Args", {{}});
            s.generic = struct("Class", "phx.GenericJoint", "Args", {{}});
            s.prismatic = struct("Class", "phx.PrismaticJoint", "Args", {acrossGravity});
            s.cylindrical = struct("Class", "phx.CylindricalJoint", "Args", {acrossGravity});
        end
    end

end
