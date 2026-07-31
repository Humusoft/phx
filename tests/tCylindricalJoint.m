classdef tCylindricalJoint < matlab.unittest.TestCase
%tCylindricalJoint Integration tests of the cylindrical joint (MEX).
%
%   The class is built on the generic 6-DOF constraint with the Z translation
%   and the Z rotation left free, so these tests pin down exactly that: the two
%   free axes really move, the other four stay locked, and the free axis follows
%   the joint frame when it is turned away from Z.
%
%   The last one guards what sets this joint apart from phx.PrismaticJoint - it
%   leaves the rotation about the axis free, so AxisA and AxisB alone describe
%   it and a roll mismatch between the two joint frames must not twist the
%   bodies into line.
%
%   See also phx.CylindricalJoint, tGenericJoint, tJointFrames

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        G = 9.81
    end

    methods (TestClassSetup)
        function requireEngine(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function bodySlidesAlongTheAxisAndNowhereElse(tc)
            [~, b, j, sim] = tc.scene();
            sim.step(1, 1000, -1);
            % A free fall along the joint axis, held on it sideways
            tc.verifyEqual(b.Position(3), -0.5*tc.G, "RelTol", 0.02, ...
                "The translation along the joint axis must be free.");
            tc.verifyEqual(b.Position(1:2), [0 0], "AbsTol", 1e-6);
            tc.verifyEqual(b.EulerAngles, [0 0 0], "AbsTol", 1e-6);
            % Nothing acts across the axis, so the joint carries no load
            tc.verifyLessThan(norm(j.ForceA), 0.02*b.Mass*tc.G);
        end

        function bodySpinsAboutTheAxisAndNowhereElse(tc)
            [~, b, ~, sim] = tc.scene("Gravity", [0 0 0]);
            for k = 1:100
                b.applyTorque([0 0 0.02]);
                sim.step(0.01, 10, -1);
            end
            tc.verifyGreaterThan(b.EulerAngles(3), 0.01, ...
                "The rotation about the joint axis must be free.");
            tc.verifyEqual(b.EulerAngles(1:2), [0 0], "AbsTol", 1e-6);

            [~, b, ~, sim] = tc.scene("Gravity", [0 0 0]);
            for k = 1:100
                b.applyTorque([2 2 0]);
                sim.step(0.01, 10, -1);
            end
            tc.verifyEqual(b.EulerAngles(1:2), [0 0], "AbsTol", 1e-3, ...
                "The rotations across the joint axis must stay locked.");
        end

        function jointAxisFollowsTheFrame(tc)
            % Axis along world X: gravity now pulls across the joint axis
            [~, b, ~, sim] = tc.scene("AxisA", [1 0 0], "AxisB", [1 0 0]);
            sim.step(1, 1000, -1);
            tc.verifyEqual(b.Position, [0 0 0], "AbsTol", 1e-3, ...
                "With the axis across gravity the body must not move.");
        end

        function rolledFramesAreNotTwistedIntoLine(tc)
            % The rotation about the axis is free, so a frame rolled about it
            % only shifts the zero of that rotation - unlike a prismatic joint,
            % which would twist the body by the mismatch
            [~, b, ~, sim] = tc.scene("Gravity", [0 0 0], "EulerAnglesB", [0 0 0.7]);
            sim.step(1, 1000, -1);
            tc.verifyEqual(b.EulerAngles, [0 0 0], "AbsTol", 1e-6);
            tc.verifyEqual(b.Position, [0 0 0], "AbsTol", 1e-6);
        end

        function offsetAlongTheAxisIsInert(tc)
            % A joint frame shifted along the axis only moves the zero of the
            % free travel, so the body is not pulled anywhere sideways
            [~, b, ~, sim] = tc.scene("Gravity", [0 0 0], "PointA", [0 0 0.5]);
            sim.step(1, 1000, -1);
            tc.verifyEqual(b.Position, [0 0 0], "AbsTol", 1e-6);
        end

        function jointHoldsTwoDynamicBodiesTogether(tc)
            % Neither body is anchored and the axis is across gravity, so the
            % pair must fall as one rigid piece
            a = tc.spawnBody([0 0 5]);
            b = tc.spawnBody([0.5 0 5]);
            phx.CylindricalJoint(a, b, "PointA", [0.5 0 0], "AxisA", [1 0 0], "AxisB", [1 0 0]);
            sim = phx.Simulation([a b], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(sim));
            sim.step(1, 1000, -1);
            tc.verifyEqual(norm(b.Position - a.Position), 0.5, "AbsTol", 0.01);
            tc.verifyEqual(5 - a.Position(3), 0.5*tc.G, "RelTol", 0.02);
        end
    end

    methods (Test)
        function feedbackIsNaNBeforeInitialization(tc)
            a = phx.Body([], "Type", "static");
            b = phx.Body([], "Position", [1 0 0]);
            tc.addTeardown(@() delete([a b]));
            j = phx.CylindricalJoint(a, b);
            tc.verifyEqual(j.ForceA, [NaN NaN NaN]);
            tc.verifyEqual(j.TorqueB, [NaN NaN NaN]);
        end
    end

    methods (Access = private)
        function b = spawnBody(~, position, type)
            arguments
                ~
                position (1, 3) double
                type (1, 1) string = "dynamic"
            end
            b = phx.Body([], "Position", position, "Type", type, ...
                "Shape", {"Box", "Size", [0.2 0.2 0.2]});
        end

        function [a, b, j, sim] = scene(tc, varargin)
            % Static anchor and a dynamic body, both at the origin. Options
            % named Gravity go to the simulation, the rest to the joint.
            args = struct(varargin{:});
            gravity = [0 0 -tc.G];
            if isfield(args, "Gravity")
                gravity = args.Gravity;
                args = rmfield(args, "Gravity");
            end
            a = tc.spawnBody([0 0 0], "static");
            b = tc.spawnBody([0 0 0]);
            opts = namedargs2cell(args);
            j = phx.CylindricalJoint(a, b, opts{:});
            sim = phx.Simulation([a b], "Gravity", gravity);
            tc.addTeardown(@() delete(sim));
        end
    end

end
