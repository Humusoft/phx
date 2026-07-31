classdef tJointFrames < matlab.unittest.TestCase
%tJointFrames Regression tests for the joint-frame coincidence rule.
%
%   A joint keeps its two joint frames (TransformA on body A, TransformB on
%   body B) in coincidence, except for the degrees of freedom it leaves
%   free. These tests pin down the two halves of that documented rule, which
%   is what tells a user how to choose PointA/PointB:
%     * a mismatch in a FREE direction is inert - it only shifts the zero of
%       that degree of freedom. On a prismatic joint an offset along the
%       sliding axis therefore changes nothing observable.
%     * a mismatch in a CONSTRAINED direction is not an error - the solver
%       pulls the bodies together over the following steps (gradually, not
%       as a teleport in the first substep).
%
%   The axis convention is pinned down here as well: the free axis of a joint
%   is the Z axis of its joint frames, which the engine expects on a different
%   axis for a slider than for a hinge, so a regression in that remapping would
%   silently send every prismatic joint sliding sideways.
%
%   All bodies are created without graphics ([] axes), so only the engine is
%   needed. The scene is a static column with a platform sliding along it on
%   a vertical prismatic joint (sliding axis = local Z of the joint frames,
%   which is world Z for the untouched default frames).
%
%   See also phx.base.Joint, phx.PrismaticJoint, phx.FixedJoint, tForceApplication

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant)
        Dt = 0.005          % substep size
        ZColumn = 1.5       % centre height of the static column
        ZStart = 0.5        % initial height of the platform
        Skew = 0.4          % frame mismatch used by the pull tests
    end

    methods (TestClassSetup)
        function requireEngine(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function freeAxisOffsetIsInert(tc)
            % An offset ALONG the sliding axis only shifts the zero of the
            % free translation: a consistent PointA and a plain [0 0 0] must
            % put the platform in exactly the same place.
            consistent = tc.slide([0 0 tc.ZStart - tc.ZColumn], 1);
            zeroed     = tc.slide([0 0 0], 1);
            tc.verifyEqual(zeroed, consistent, "AbsTol", 1e-6);
        end

        function freeAxisIsActuallyFree(tc)
            % Sanity check for the test above: the platform is not held at
            % all along the sliding axis, it falls freely from ZStart.
            p = tc.slide([0 0 tc.ZStart - tc.ZColumn], 1);
            expected = tc.ZStart - 0.5*9.81*1^2;
            tc.verifyEqual(p(3), expected, "AbsTol", 0.05);
            tc.verifyEqual(p(1:2), [0 0], "AbsTol", 1e-6);
        end

        function constrainedOffsetPullsBodyOver(tc)
            % An offset PERPENDICULAR to the sliding axis is enforced: the
            % platform ends up displaced by that offset in y.
            p = tc.slide([0 tc.Skew tc.ZStart - tc.ZColumn], 1);
            tc.verifyEqual(p(2), tc.Skew, "AbsTol", 0.02);
        end

        function constrainedOffsetIsPulledGradually(tc)
            % The recovery is a pull over several steps, not a teleport in
            % the first substep - a scene built with inconsistent frames
            % drifts into place rather than exploding.
            p = tc.slide([0 tc.Skew tc.ZStart - tc.ZColumn], tc.Dt);
            tc.verifyGreaterThan(p(2), 0);
            tc.verifyLessThan(p(2), tc.Skew/2);
        end

        function perpendicularDirectionStaysConstrained(tc)
            % With consistent frames the platform cannot be pushed off the
            % sliding axis: a sideways world force moves it nowhere in y.
            [column, platform] = tc.buildSlide([0 0 tc.ZStart - tc.ZColumn]);
            sim = phx.Simulation([column platform]);
            for k = 1:200
                platform.applyForce([0 500 0], [0 0 0], false, true);
                sim.step(tc.Dt, 1, -1);
            end
            p = platform.Position;
            tc.addTeardown(@() delete([column platform]));
            delete(sim);
            tc.verifyEqual(p(2), 0, "AbsTol", 1e-3);
        end

        function slidingAxisFollowsAxisAAndAxisB(tc)
            % Convention regression: setting the joint axis to world X must
            % turn the vertical slide into a horizontal one - the platform is
            % then held against gravity and travels along x under a push.
            column = phx.Body([], "Type", "static", "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [0.25 0.25 0.25]});
            platform = phx.Body([], "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [0.4 0.4 0.4], "Density", 200});
            tc.addTeardown(@() delete([column platform]));

            phx.PrismaticJoint(column, platform, "AxisA", [1 0 0], "AxisB", [1 0 0]);

            % A gentle push, and only a short travel - a platform driven far
            % from the anchor measures the softness of the solver rather than
            % the direction of the sliding axis.
            sim = phx.Simulation([column platform]);
            for k = 1:100
                platform.applyForce([20 0 0], [0 0 0], false, true);
                sim.step(tc.Dt, 1, -1);
            end
            p = platform.Position;
            delete(sim);

            tc.verifyGreaterThan(p(1), 0.05, ...
                "The platform must travel along the requested sliding axis.");
            % Free fall over the same 0.5 s would be more than a metre, so this
            % bound still tells "held by the joint" from "sliding downwards".
            tc.verifyEqual(p(2:3), [0 0], "AbsTol", 0.02, ...
                "Gravity must be carried by the joint, not leak into a slide.");
        end

        function axisPropertiesReadBackExactly(tc)
            % AxisA/AxisB are a view of the third column of the joint frames,
            % so a normalized direction must come back bit-for-bit, and the
            % connecting point must survive an axis change untouched.
            a = phx.Body([], "Type", "static");
            b = phx.Body([], "Position", [1 0 0]);
            tc.addTeardown(@() delete([a b]));

            j = phx.RevoluteJoint(a, b);
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

            tc.verifyError(@() setAxis(j), "phx:Joint:invalidAxis");
            function setAxis(j)
                j.AxisA = [0 0 0];
            end
        end

        function fixedJointLeavesNoFreeDirection(tc)
            % The counterpart: a fixed joint has no free degree of freedom,
            % so an offset in ANY direction is pulled out - here the body is
            % dragged from z = 1 down onto the frame of the static anchor.
            anchor = phx.Body([], "Type", "static", "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [0.4 0.4 0.4]});
            block = phx.Body([], "Position", [0 0 1], ...
                "Shape", {"Box", "Size", [0.4 0.4 0.4], "Density", 200});
            tc.addTeardown(@() delete([anchor block]));

            phx.FixedJoint(anchor, block, "PointA", [0 0 0], "PointB", [0 0 0]);

            sim = phx.Simulation([anchor block]);
            sim.step(1, 200, -1);
            p = block.Position;
            delete(sim);

            tc.verifyEqual(p, [0 0 0], "AbsTol", 0.02);
        end
    end

    methods (Access = private)
        function p = slide(tc, pointA, duration)
            % Build the slide with the given PointA, run it, return the
            % platform position.
            [column, platform] = tc.buildSlide(pointA);
            sim = phx.Simulation([column platform]);
            sim.step(duration, round(duration/tc.Dt), -1);
            p = platform.Position;
            delete(sim);
            delete([column platform]);
        end

        function [column, platform] = buildSlide(tc, pointA)
            column = phx.Body([], "Type", "static", "Position", [0 0 tc.ZColumn], ...
                "Shape", {"Box", "Size", [0.25 0.25 3]});
            platform = phx.Body([], "Position", [0 0 tc.ZStart], ...
                "Shape", {"Box", "Size", [1.1 1.1 0.1], "Density", 200});

            % Sliding axis = local Z of both joint frames, i.e. the default.
            phx.PrismaticJoint(column, platform, ...
                "PointA", pointA, "PointB", [0 0 0]);
        end
    end

end
