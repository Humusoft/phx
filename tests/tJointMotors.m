classdef tJointMotors < PhxTestCase
%tJointMotors The motor of phx.RevoluteJoint and phx.PrismaticJoint.
%
%   Both classes promise the same thing in two units: the motor regulates the
%   free degree of freedom to TargetVelocity as long as it needs less than
%   MaxTorque (MaxForce), which is zero - no motor - by default. Everything a
%   user can do with it follows from that one sentence, so that is what these
%   tests pin down:
%
%     * a reachable target is reached and held;
%     * a deliberately unreachable target turns the motor into a pure torque
%       (force) source, so the driven body accelerates at MaxTorque/inertia
%       (MaxForce/mass) - this is the documented way to command effort;
%     * TargetVelocity = 0 is a brake: it holds the joint against a load
%       smaller than the limit and gives way to a larger one;
%     * a zero limit is off, not a zero-effort motor;
%     * both properties are write-through, so a scene can steer a motor from
%       inside the frame loop;
%     * the effort the motor spends is part of the reaction feedback, which is
%       what lets a joint be the seam for an external actuator model.
%
%   The bodies are headless ([] axes) and the motor is isolated from gravity
%   wherever the test is about the motor rather than about a load. Mass and
%   inertia are set explicitly because the analytic expectations need them.
%
%   See also phx.RevoluteJoint, phx.PrismaticJoint, tJointContract

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        G = 9.81
        NoGravity = {"Gravity", [0 0 0]}
        Unreachable = 1e4   % a target no real motor can reach, in rad/s or m/s
    end

    methods (Test, TestTags = {'Graphics'})
        function motorIsOffByDefault(tc)
            % A joint built without motor options must not drive anything.
            [base, arm] = tc.pair;
            rev = phx.RevoluteJoint(base, arm, "PointA", [0 0 0], "PointB", [0 0 0]);
            pri = phx.PrismaticJoint(base, arm, "PointA", [0 0 0], "PointB", [0 0 0]);

            tc.verifyEqual(rev.TargetVelocity, 0);
            tc.verifyEqual(rev.MaxTorque, 0);
            tc.verifyEqual(pri.TargetVelocity, 0);
            tc.verifyEqual(pri.MaxForce, 0);
        end

        function negativeLimitIsRejected(tc)
            % The limit is a magnitude - a negative one has no meaning that
            % could be given consistently, so it must not be accepted.
            [base, arm] = tc.pair;
            rev = phx.RevoluteJoint(base, arm);
            pri = phx.PrismaticJoint(base, arm);

            tc.verifyError(@() set(rev, "MaxTorque", -1), ?MException);
            tc.verifyError(@() set(pri, "MaxForce", -1), ?MException);
            % A negative target is fine - it is a direction.
            rev.TargetVelocity = -2;
            tc.verifyEqual(rev.TargetVelocity, -2);
        end
    end

    methods (Test, TestTags = {'Engine'})
        function revoluteMotorReachesAndHoldsTheTarget(tc)
            % With torque to spare, the spin settles at TargetVelocity.
            tc.requireEngine;
            [~, arm, sim] = tc.spinRig("TargetVelocity", 2, "MaxTorque", 50);

            sim.step(1, 200, -1);
            tc.verifyEqual(arm.AngularVelocity(3), 2, "RelTol", 0.02);

            sim.step(1, 200, -1);
            tc.verifyEqual(arm.AngularVelocity(3), 2, "RelTol", 0.02, ...
                "The motor did not hold the target.");
        end

        function revoluteMotorSaturatesIntoATorqueSource(tc)
            % An unreachable target leaves the motor delivering MaxTorque, so
            % the arm accelerates at MaxTorque/I about the joint axis. The
            % joint sits at the arm's own centre, so I is the plain inertia.
            tc.requireEngine;
            I = 2;
            T = 4;
            [~, arm, sim] = tc.spinRig("TargetVelocity", tc.Unreachable, ...
                "MaxTorque", T, "Inertia", [1 1 I]);

            sim.step(1, 200, -1);

            tc.verifyEqual(arm.AngularVelocity(3), T/I, "RelTol", 0.02, ...
                "The saturated motor did not deliver MaxTorque.");
        end

        function zeroMaxTorqueIsNoMotorAtAll(tc)
            tc.requireEngine;
            [~, arm, sim] = tc.spinRig("TargetVelocity", 5, "MaxTorque", 0);

            sim.step(1, 200, -1);

            tc.verifyEqual(arm.AngularVelocity(3), 0, "AbsTol", 1e-9);
        end

        function zeroTargetIsABrakeThatHoldsUpToItsLimit(tc)
            % TargetVelocity = 0 holds the arm level against a load it has the
            % torque for, and yields to a bigger one - while still resisting
            % it, which is what tells a brake from an absent motor. The load
            % is the arm's own weight, m*g*d = 9.81 N*m, and the three rigs
            % differ in nothing but MaxTorque.
            tc.requireEngine;
            load = tc.G;                    % m = 1 kg, d = 1 m
            free = tc.armRig("MaxTorque", 0);
            half = tc.armRig("MaxTorque", 0.5*load);
            held = tc.armRig("MaxTorque", 2*load);

            for rig = [free half held]
                rig.Sim.step(0.5, 400, -1);
            end

            tc.assumeLessThan(free.Arm.Position(3), -0.8, ...
                "Premise: without a motor the arm swings down.");
            tc.verifyEqual(held.Arm.Position(3), 0, "AbsTol", 0.01, ...
                "The brake did not hold a load it had the torque for.");
            tc.verifyLessThan(half.Arm.Position(3), -0.3, ...
                "The brake held a load beyond its torque.");
            tc.verifyGreaterThan(half.Arm.Position(3), free.Arm.Position(3) + 0.2, ...
                "The half-strength brake did not resist the load at all.");
        end

        function motorPropertiesAreLiveDuringTheSimulation(tc)
            % Both setters write through to a running world, which is what
            % makes steering a motor from the frame loop possible.
            tc.requireEngine;
            [~, arm, sim, joint] = tc.spinRig("TargetVelocity", 3, "MaxTorque", 0);

            sim.step(0.5, 100, -1);
            tc.assertEqual(arm.AngularVelocity(3), 0, "AbsTol", 1e-9);

            joint.MaxTorque = 50;                 % switch it on mid-run
            sim.step(0.5, 100, -1);
            tc.verifyEqual(arm.AngularVelocity(3), 3, "RelTol", 0.02);

            joint.TargetVelocity = -1;            % and reverse it
            sim.step(1, 200, -1);
            tc.verifyEqual(arm.AngularVelocity(3), -1, "RelTol", 0.02);
        end

        function motorEffortIsPartOfTheReactionFeedback(tc)
            % What the motor spends shows up in the joint feedback, with the
            % action/reaction split - this is what makes a joint the seam an
            % external actuator model can be attached to.
            tc.requireEngine;
            T = 4;
            [~, ~, sim, joint] = tc.spinRig("TargetVelocity", tc.Unreachable, ...
                "MaxTorque", T);

            sim.step(0.5, 100, -1);

            tc.verifyEqual(joint.TorqueA(3), -T, "AbsTol", 1e-3);
            tc.verifyEqual(joint.TorqueB(3), T, "AbsTol", 1e-3);
        end

        function prismaticMotorReachesTheTarget(tc)
            tc.requireEngine;
            [~, slider, sim] = tc.slideRig("TargetVelocity", 0.5, "MaxForce", 50);

            sim.step(1, 200, -1);

            tc.verifyEqual(slider.LinearVelocity(3), 0.5, "RelTol", 0.02);
        end

        function prismaticMotorSaturatesIntoAForceSource(tc)
            % Same contract in linear units: a = MaxForce/m.
            tc.requireEngine;
            m = 2;
            F = 6;
            [~, slider, sim] = tc.slideRig("TargetVelocity", tc.Unreachable, ...
                "MaxForce", F, "Mass", m);

            sim.step(1, 200, -1);

            tc.verifyEqual(slider.LinearVelocity(3), F/m, "RelTol", 0.02);
        end

        function zeroMaxForceIsNoMotorAtAll(tc)
            tc.requireEngine;
            [~, slider, sim] = tc.slideRig("TargetVelocity", 1, "MaxForce", 0);

            sim.step(1, 200, -1);

            tc.verifyEqual(slider.LinearVelocity(3), 0, "AbsTol", 1e-9);
        end
    end

    methods (Access = private)
        function [base, arm] = pair(tc)
            % Two headless bodies that outlive the joint built on them.
            base = tc.spawnBody([0 0 0], "Type", "static");
            arm = tc.spawnBody([0 0 1]);
            tc.addTeardown(@() delete([base arm]));
        end

        function [base, arm, sim, joint] = spinRig(tc, Options)
            % Static base and an arm hinged about z through its own centre,
            % in a world without gravity: nothing but the motor acts on it.
            arguments
                tc
                Options.TargetVelocity (1, 1) double = 0
                Options.MaxTorque (1, 1) double = 0
                Options.Inertia (1, 3) double = [1 1 1]
            end
            base = tc.spawnBody([0 0 -1], "Type", "static");
            arm = tc.spawnBody([0 0 0], "Mass", 1, "Inertia", Options.Inertia);
            tc.addTeardown(@() delete([base arm]));

            joint = phx.RevoluteJoint(base, arm, "PointA", [0 0 1], "PointB", [0 0 0], ...
                "TargetVelocity", Options.TargetVelocity, "MaxTorque", Options.MaxTorque);
            sim = phx.Simulation([base arm], tc.NoGravity{:});
            tc.addTeardown(@() delete(sim));
        end

        function rig = armRig(tc, Options)
            % An arm reaching out along x, hinged about y, under gravity: the
            % load on the motor is its own weight, m*g*d. The inertia has to
            % be given explicitly - setting Mass alone leaves the one the
            % shape derived from its density, which here is ~150x larger and
            % makes the arm barely move whatever the motor does.
            arguments
                tc
                Options.TargetVelocity (1, 1) double = 0
                Options.MaxTorque (1, 1) double = 0
            end
            base = tc.spawnBody([0 0 0], "Type", "static");
            arm = tc.spawnBody([1 0 0], "Mass", 1, "Inertia", [0.1 0.1 0.1]);
            tc.addTeardown(@() delete([base arm]));

            phx.RevoluteJoint(base, arm, "PointA", [0 0 0], "PointB", [-1 0 0], ...
                "AxisA", [0 1 0], "AxisB", [0 1 0], ...
                "TargetVelocity", Options.TargetVelocity, "MaxTorque", Options.MaxTorque);
            rig.Arm = arm;
            rig.Sim = phx.Simulation([base arm], "Gravity", [0 0 -tc.G]);
            tc.addTeardown(@() delete(rig.Sim));
        end

        function [base, slider, sim] = slideRig(tc, Options)
            % Static base and a slider free along z, no gravity.
            arguments
                tc
                Options.TargetVelocity (1, 1) double = 0
                Options.MaxForce (1, 1) double = 0
                Options.Mass (1, 1) double = 1
            end
            base = tc.spawnBody([0 0 0], "Type", "static");
            slider = tc.spawnBody([0 0 2], "Mass", Options.Mass);
            tc.addTeardown(@() delete([base slider]));

            phx.PrismaticJoint(base, slider, "PointA", [0 0 0], "PointB", [0 0 0], ...
                "TargetVelocity", Options.TargetVelocity, "MaxForce", Options.MaxForce);
            sim = phx.Simulation([base slider], tc.NoGravity{:});
            tc.addTeardown(@() delete(sim));
        end
    end

end
