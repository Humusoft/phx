classdef tJointLimits < PhxTestCase
%tJointLimits The limits of the joints, under one set of rules.
%
%   The classes promise the same thing in two units: a pair [lower upper]
%   bounds a joint coordinate (Angle in rad, Distance in m) with hard end
%   stops, equal limits lock the joint at that value and [-Inf Inf] leaves
%   it free. phx.RevoluteJoint and phx.PrismaticJoint name it Limits,
%   phx.CylindricalJoint AngularLimits and LinearLimits, and
%   phx.GenericJoint and phx.BushingJoint hold one such pair per axis in the
%   rows of 3-by-2 AngularLimits and LinearLimits. These tests pin that down:
%
%     * the default is free, except for the rigid generic joint; a scalar
%       locks at that value;
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
%   What the six rows of a generic joint mean in motion is in tGenericJoint.
%
%   See also phx.RevoluteJoint, phx.PrismaticJoint, phx.CylindricalJoint,
%   phx.GenericJoint, phx.BushingJoint, tGenericJoint, tJointMotors

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
            cyl = phx.CylindricalJoint(base, arm);
            tc.verifyEqual(cyl.LinearLimits, [-Inf Inf]);
            tc.verifyEqual(cyl.AngularLimits, [-Inf Inf]);
        end

        function scalarLocksAtThatValue(tc)
            [base, arm] = tc.pair;
            rev = phx.RevoluteJoint(base, arm, "Limits", 0.2);
            pri = phx.PrismaticJoint(base, arm, "Limits", -0.5);
            cyl = phx.CylindricalJoint(base, arm, "LinearLimits", -0.5, "AngularLimits", 0.2);
            tc.verifyEqual(rev.Limits, [0.2 0.2]);
            tc.verifyEqual(pri.Limits, [-0.5 -0.5]);
            tc.verifyEqual(cyl.LinearLimits, [-0.5 -0.5]);
            tc.verifyEqual(cyl.AngularLimits, [0.2 0.2]);
        end

        function invalidLimitsAreRejected(tc)
            [base, arm] = tc.pair;
            rev = phx.RevoluteJoint(base, arm);
            pri = phx.PrismaticJoint(base, arm);
            cyl = phx.CylindricalJoint(base, arm);
            angular = {rev, "Limits"; cyl, "AngularLimits"};
            linear = {pri, "Limits"; cyl, "LinearLimits"};
            for k = 1:2
                for p = [angular(k, :); linear(k, :)]'
                    tc.verifyError(@() set(p{1}, p{2}, [0.3 0.1]), "phx:Joint:invalidLimits", p{2} + " reversed");
                    tc.verifyError(@() set(p{1}, p{2}, [NaN 0]), "phx:Joint:invalidLimits", p{2} + " NaN");
                end
                % The angle is wrapped to [-pi, pi]: a stop on one side alone
                % would close the other one at the back point
                [joint, name] = angular{k, :};
                for bad = {[-Inf 0.5], [0 Inf], [0 pi], [-4 0]}
                    tc.verifyError(@() set(joint, name, bad{1}), "phx:Joint:invalidLimits", name + " " + mat2str(bad{1}));
                end
                joint.(name) = [-3.1 3.1];
                tc.verifyEqual(joint.(name), [-3.1 3.1]);
                % A slide may leave out a side
                [joint, name] = linear{k, :};
                joint.(name) = [-Inf 0.5];
                tc.verifyEqual(joint.(name), [-Inf 0.5]);
            end
        end

        function sixAxisLimitsAreRowsPerAxis(tc)
            % The generic joint is rigid by default, the bushing free; a
            % scalar locks all three axes at that value
            [base, arm] = tc.pair;
            gen = phx.GenericJoint(base, arm);
            bus = phx.BushingJoint(base, arm);
            tc.verifyEqual(gen.LinearLimits, zeros(3, 2));
            tc.verifyEqual(gen.AngularLimits, zeros(3, 2));
            tc.verifyEqual(bus.LinearLimits, repmat([-Inf Inf], 3, 1));
            tc.verifyEqual(bus.AngularLimits, repmat([-Inf Inf], 3, 1));
            gen.AngularLimits = 0.2;
            tc.verifyEqual(gen.AngularLimits, repmat(0.2, 3, 2));
            % Each row on its own: free, locked, bounded, one-sided linear
            bus.LinearLimits = [-Inf Inf; 0 0; -Inf 0.5];
            bus.AngularLimits = [-Inf Inf; 0.1 0.1; -3.1 3.1];
            tc.verifyEqual(bus.LinearLimits, [-Inf Inf; 0 0; -Inf 0.5]);
            tc.verifyEqual(bus.AngularLimits, [-Inf Inf; 0.1 0.1; -3.1 3.1]);
        end

        function sixAxisLimitsRejectAnyBadRow(tc)
            % One bad row is enough, whichever axis it is on
            [base, arm] = tc.pair;
            for joint = {phx.GenericJoint(base, arm), phx.BushingJoint(base, arm)}
                for ax = 1:3
                    for name = ["LinearLimits" "AngularLimits"]
                        for bad = {[0.3 0.1], [NaN 0]}
                            L = zeros(3, 2);
                            L(ax, :) = bad{1};
                            tc.verifyError(@() set(joint{1}, name, L), "phx:Joint:invalidLimits", ...
                                sprintf("%s row %d %s", name, ax, mat2str(bad{1})));
                        end
                    end
                    L = zeros(3, 2);
                    L(ax, :) = [-Inf 0.5];
                    tc.verifyError(@() set(joint{1}, "AngularLimits", L), "phx:Joint:invalidLimits", ...
                        sprintf("One-sided angular row %d", ax));
                end
                tc.verifyError(@() set(joint{1}, "LinearLimits", [0 1]), "MATLAB:validation:IncompatibleSize", ...
                    "A single pair is not three rows");
            end
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
            [sim, joint] = tc.hangRig(m, @phx.PrismaticJoint, "Limits", [-0.3 -0.1]);
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(joint.Distance, -0.3, "AbsTol", 1e-3);
            tc.verifyEqual(tc.meanForceB(sim, joint), [0 0 m*tc.G], "AbsTol", 0.02*m*tc.G);
        end

        function prismaticLimitsChangeWhileRunning(tc)
            % Locked at -0.2 the slider holds there; with no lower stop it
            % falls away
            [sim, joint] = tc.hangRig(2, @phx.PrismaticJoint, "Limits", [-0.3 -0.1]);
            sim.step(0.5, 0.5/tc.Dt, -1);
            joint.Limits = -0.2;
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(joint.Distance, -0.2, "AbsTol", 1e-3, "Locked");
            joint.Limits = [-Inf 0];
            sim.step(0.5, 0.5/tc.Dt, -1);
            tc.verifyLessThan(joint.Distance, -1, "No lower stop");
        end

        function cylindricalStopsBothMotions(tc)
            % Hanging along z on [-0.3 -0.1] m and turned on [0.1 0.3] rad
            % (ranges without the zero): it drops onto the lower linear stop,
            % which carries its weight, and turns to whichever angular stop
            % the torque drives it against
            m = 2;
            for s = [1 -1]
                [sim, joint, body] = tc.hangRig(m, @phx.CylindricalJoint, ...
                    "LinearLimits", [-0.3 -0.1], "AngularLimits", [0.1 0.3]);
                body.AppliedTorque = [0 0 5*s];
                sim.step(1, 1/tc.Dt, -1);
                tc.verifyEqual(joint.Distance, -0.3, "AbsTol", 1e-3, sprintf("Distance, turned %+d", s));
                tc.verifyEqual(joint.Angle, 0.2 + 0.1*s, "AbsTol", 0.01, sprintf("Angle, turned %+d", s));
                F = tc.meanForceB(sim, joint);
                tc.verifyEqual(F(3), m*tc.G, "AbsTol", 0.02*m*tc.G, sprintf("Weight, turned %+d", s));
            end
        end

        function cylindricalLimitsChangeWhileRunning(tc)
            % Both motions locked by scalars hold against the weight and the
            % torque; freed, the body falls and spins up
            [sim, joint, body] = tc.hangRig(2, @phx.CylindricalJoint);
            body.AppliedTorque = [0 0 5];
            sim.step(0.2, 0.2/tc.Dt, -1);
            joint.LinearLimits = -0.5;
            joint.AngularLimits = 0.5;
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(joint.Distance, -0.5, "AbsTol", 1e-3, "Linear locked");
            tc.verifyEqual(joint.Angle, 0.5, "AbsTol", 0.01, "Angular locked");
            joint.LinearLimits = [-Inf Inf];
            joint.AngularLimits = [-Inf Inf];
            sim.step(0.4, 0.4/tc.Dt, -1);
            tc.verifyEqual(body.LinearVelocity(3), -tc.G*0.4, "RelTol", 0.05, "Linear freed");
            tc.verifyEqual(body.AngularVelocity(3), 5*0.4, "RelTol", 0.05, "Angular freed");
        end

        function bushingStopBoundsTheSpringTravel(tc)
            % A bushing with no spring along z, bounded there to [-0.2 Inf]
            % m by the third row: the body drops onto the stop, which then
            % carries its weight; with the row freed it falls away
            m = 2;
            [sim, joint, body] = tc.hangRig(m, @phx.BushingJoint, ...
                "LinearStiffness", [1e4 1e4 0], "LinearDamping", [1e3 1e3 0], ...
                "LinearLimits", [-Inf Inf; -Inf Inf; -0.2 Inf]);
            sim.step(1, 1/tc.Dt, -1);
            tc.verifyEqual(body.Position, [0 0 0.8], "AbsTol", 2e-3, "On the stop");
            F = tc.meanForceB(sim, joint);
            tc.verifyEqual(F(3), m*tc.G, "AbsTol", 0.02*m*tc.G, "The stop carries the weight");
            joint.LinearLimits(3, :) = [-Inf Inf];
            sim.step(0.5, 0.5/tc.Dt, -1);
            tc.verifyLessThan(body.Position(3), 0, "Freed");
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
            base = tc.spawnBody([0 0 -1], "Type", "static");
            arm = tc.spawnBody([0 0 0], "Mass", 1, "Inertia", [1 1 1]);
            tc.addTeardown(@() delete([base arm]));
            joint = phx.RevoluteJoint(base, arm, "PointA", [0 0 1], "PointB", [0 0 0], varargin{:});
            sim = phx.Simulation([base arm], "Gravity", [0 0 0]);
            tc.addTeardown(@() delete(sim));
        end

        function [sim, joint, body] = hangRig(tc, m, makeJoint, varargin)
            % A body of mass m and unit inertia hanging 1 m below a static
            % base on a joint along z made by makeJoint, under gravity.
            tc.requireEngine;
            base = tc.spawnBody([0 0 2], "Type", "static");
            body = tc.spawnBody([0 0 1], "Mass", m, "Inertia", [1 1 1]);
            tc.addTeardown(@() delete([base body]));
            joint = makeJoint(base, body, "PointA", [0 0 -1], "PointB", [0 0 0], varargin{:});
            sim = phx.Simulation([base body], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(sim));
        end

        function F = meanForceB(tc, sim, joint)
            % The reaction on body B averaged over 40 steps.
            n = 40;
            F = zeros(n, 3);
            for k = 1:n
                sim.step(tc.Dt, 1, -1);
                F(k, :) = joint.ForceB;
            end
            F = mean(F);
        end
    end

end
