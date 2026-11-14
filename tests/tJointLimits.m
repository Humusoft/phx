classdef tJointLimits < PhxTestCase
%tJointLimits The Limits of phx.RevoluteJoint and phx.PrismaticJoint.
%
%   Both classes promise the same thing in two units: Limits = [lower upper]
%   bounds the joint coordinate (Angle in rad, Distance in m) with hard end
%   stops, equal limits lock the joint at that value and [-Inf Inf], the
%   default, leaves it free. These tests pin that down:
%
%     * the default is free, a scalar locks at that value;
%     * reversed or NaN limits are an error; angular limits are both finite
%       within (-pi, pi) or [-Inf Inf], linear ones may leave out a side;
%     * a driven joint stops at whichever limit it is driven against, also
%       for a range without the zero, in the sense Angle measures;
%     * limits change a running simulation;
%     * a stop carries the load, and its reaction is part of the feedback.
%
%   A body resting on a stop leaves it now and then for one step and is
%   caught again with twice the impulse (the engine engages a stop only at
%   or beyond it), so the reaction is averaged over several steps.
%
%   See also phx.RevoluteJoint, phx.PrismaticJoint, tJointMotors

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        G = 9.81
        Dt = 0.005
    end

    methods (Test, TestTags = {'Graphics'})
        function limitsAreFreeByDefault(tc)
            [base, arm] = tc.pair;
            tc.verifyEqual(phx.RevoluteJoint(base, arm).Limits, [-Inf Inf]);
            tc.verifyEqual(phx.PrismaticJoint(base, arm).Limits, [-Inf Inf]);
        end

        function scalarLocksAtThatValue(tc)
            [base, arm] = tc.pair;
            rev = phx.RevoluteJoint(base, arm, "Limits", 0.2);
            pri = phx.PrismaticJoint(base, arm, "Limits", -0.5);
            tc.verifyEqual(rev.Limits, [0.2 0.2]);
            tc.verifyEqual(pri.Limits, [-0.5 -0.5]);
        end

        function invalidLimitsAreRejected(tc)
            [base, arm] = tc.pair;
            rev = phx.RevoluteJoint(base, arm);
            pri = phx.PrismaticJoint(base, arm);
            for joint = {rev, pri}
                tc.verifyError(@() set(joint{1}, "Limits", [0.3 0.1]), "phx:Joint:invalidLimits", "reversed");
                tc.verifyError(@() set(joint{1}, "Limits", [NaN 0]), "phx:Joint:invalidLimits", "NaN");
            end
            % The angle is wrapped to [-pi, pi]: a stop on one side alone
            % would close the other one at the back point
            for bad = {[-Inf 0.5], [0 Inf], [0 pi], [-4 0]}
                tc.verifyError(@() set(rev, "Limits", bad{1}), "phx:Joint:invalidLimits", mat2str(bad{1}));
            end
            rev.Limits = [-3.1 3.1];
            tc.verifyEqual(rev.Limits, [-3.1 3.1]);
            % A slider may leave out a side
            pri.Limits = [-Inf 0.5];
            tc.verifyEqual(pri.Limits, [-Inf 0.5]);
        end
    end

    methods (Test, TestTags = {'Engine'})
        function revoluteStopsAtEitherLimit(tc)
            % Bounded to [0.1 0.3] rad, a range without the zero, and turned
            % by AppliedTorque: positively it stops at 0.3, negatively at 0.1
            for s = [1 -1]
                [sim, joint] = tc.spinRig("Limits", [0.1 0.3]);
                joint.AppliedTorque = 5*s;
                sim.step(1, 1/tc.Dt, -1);
                tc.verifyEqual(joint.Angle, 0.2 + 0.1*s, "AbsTol", 0.01, sprintf("Turned %+d", s));
            end
        end

        function revoluteLimitsChangeWhileRunning(tc)
            % Equal limits lock the turning joint at that angle against the
            % torque; [-Inf Inf] frees it again
            [sim, joint, arm] = tc.spinRig;
            joint.AppliedTorque = 5;
            sim.step(0.2, 0.2/tc.Dt, -1);
            joint.Limits = 0.5;
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(joint.Angle, 0.5, "AbsTol", 0.01, "Locked");
            joint.Limits = [-Inf Inf];
            sim.step(0.4, 0.4/tc.Dt, -1);
            tc.verifyEqual(arm.AngularVelocity(3), 5*0.4, "RelTol", 0.05, "Freed");
        end

        function prismaticStopCarriesTheLoad(tc)
            % A slider hanging along z, bounded to [-0.3 -0.1] m (a range
            % without the zero): it drops onto the lower stop, whose
            % reaction carries its weight
            m = 2;
            [sim, joint] = tc.hangRig(m, "Limits", [-0.3 -0.1]);
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(joint.Distance, -0.3, "AbsTol", 1e-3);
            n = 40;
            F = zeros(n, 3);
            for k = 1:n
                sim.step(tc.Dt, 1, -1);
                F(k, :) = joint.ForceB;
            end
            tc.verifyEqual(mean(F), [0 0 m*tc.G], "AbsTol", 0.02*m*tc.G);
        end

        function prismaticLimitsChangeWhileRunning(tc)
            % Locked at -0.2 the slider holds there; with no lower stop it
            % falls away
            [sim, joint] = tc.hangRig(2, "Limits", [-0.3 -0.1]);
            sim.step(0.5, 0.5/tc.Dt, -1);
            joint.Limits = -0.2;
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(joint.Distance, -0.2, "AbsTol", 1e-3, "Locked");
            joint.Limits = [-Inf 0];
            sim.step(0.5, 0.5/tc.Dt, -1);
            tc.verifyLessThan(joint.Distance, -1, "No lower stop");
        end
    end

    methods (Access = private)
        function [base, arm] = pair(tc)
            % Two headless bodies that outlive the joints built on them.
            base = tc.spawnBody([0 0 0], "Type", "static");
            arm = tc.spawnBody([0 0 1]);
            tc.addTeardown(@() delete([base arm]));
        end

        function [sim, joint, arm] = spinRig(tc, varargin)
            % Static base and a body of unit inertia hinged about z through
            % its own centre, no gravity.
            tc.requireEngine;
            base =tc.spawnBody([0 0 -1], "Type", "static");
            arm = tc.spawnBody([0 0 0], "Mass", 1, "Inertia", [1 1 1]);
            tc.addTeardown(@() delete([base arm]));
            joint = phx.RevoluteJoint(base, arm, "PointA", [0 0 1], "PointB", [0 0 0], varargin{:});
            sim = phx.Simulation([base arm], "Gravity", [0 0 0]);
            tc.addTeardown(@() delete(sim));
        end

        function [sim, joint] = hangRig(tc, m, varargin)
            % A body of mass m hanging 1 m below a static base on a slider
            % along z, under gravity.
            tc.requireEngine;
            base =tc.spawnBody([0 0 2], "Type", "static");
            slider = tc.spawnBody([0 0 1], "Mass", m, "Inertia", m*[1 1 1]/150);
            tc.addTeardown(@() delete([base slider]));
            joint = phx.PrismaticJoint(base, slider, "PointA", [0 0 -1], "PointB", [0 0 0], varargin{:});
            sim = phx.Simulation([base slider], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(sim));
        end
    end

end
