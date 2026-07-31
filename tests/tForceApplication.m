classdef tForceApplication < matlab.unittest.TestCase
%tForceApplication Regression tests for local/world force and torque application.
%
%   These guard two engine-wrapper bugs in phx.Body.applyForce/applyTorque
%   that were fixed 2026-07-17:
%     * a LOCAL force (or torque) must be rotated into world axes only - the
%       body's world position must NOT be added to the vector. The old
%       wrapper transformed the force vector as if it were a point, so a
%       constant local force grew with the body's distance from the origin.
%     * a GLOBAL point of application is given in ABSOLUTE world coordinates
%       and must be converted to a centre-of-mass-relative offset before it
%       reaches the engine. The old wrapper passed it straight through, so
%       the moment was taken about the world origin instead of the body.
%
%   Each test applies a single known force/torque to a 1 kg cube for one
%   short step with gravity off, then checks the resulting velocity against
%   the analytic impulse dv = F*dt/m (and dw = tau*dt/I). The expected values
%   are derived from the body's own Mass/Inertia so the test targets the
%   application logic, not the mass/inertia computation (see tShapeMass).
%   The body is deliberately placed far from the origin and often rotated,
%   which is exactly where both bugs used to show up.
%
%   See also phx.Body, tSimulation, tShapeMass

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant)
        Dt = 0.01           % single-substep interval
        Far = [100 0 0]     % body placed far from the origin
        Spin = [0 0 pi/2]   % +90 deg about z: local x -> world y
    end

    methods (TestClassSetup)
        function requireEngine(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function localForceIsNotTranslatedByPosition(tc)
            % BUG #1: local force far from origin, no rotation. Body position
            % must not leak into the force -> pure dv = F/m*dt along x.
            [lin, ~, m] = tc.applyForce(tc.Far, [0 0 0], [10 0 0], [0 0 0], true, true);
            tc.verifyEqual(lin, [10/m*tc.Dt 0 0], "AbsTol", 1e-4);
        end

        function localForceRotatesButDoesNotTranslate(tc)
            % BUG #1: rotated body far from origin. The local +x force must
            % rotate into world +y (rotation IS applied) with no position.
            [lin, ~, m] = tc.applyForce(tc.Far, tc.Spin, [10 0 0], [0 0 0], true, true);
            tc.verifyEqual(lin, [0 10/m*tc.Dt 0], "AbsTol", 1e-4);
        end

        function worldForceIsIndependentOfPosition(tc)
            % Control: world-frame force was always correct, stays correct.
            [lin, ~, m] = tc.applyForce(tc.Far, tc.Spin, [0 10 0], [0 0 0], false, true);
            tc.verifyEqual(lin, [0 10/m*tc.Dt 0], "AbsTol", 1e-4);
        end

        function localForceAtOriginIsCorrect(tc)
            % Control: at the origin the (buggy) point-transform coincided
            % with the correct vector-transform, so this always passed.
            [lin, ~, m] = tc.applyForce([0 0 0], [0 0 0], [10 0 0], [0 0 0], true, true);
            tc.verifyEqual(lin, [10/m*tc.Dt 0 0], "AbsTol", 1e-4);
        end

        function globalPointIsAbsoluteWorldCoordinate(tc)
            % BUG #2: a global point is absolute world coordinates. A point
            % 0.5 m in +x from the centre of a body at x=100 is [100.5 0 0];
            % the moment must be taken about the centre (offset 0.5), giving
            % tau = [0.5 0 0] x [0 0 10] = [0 -5 0], NOT about the origin.
            [~, ang, ~, I] = tc.applyForce(tc.Far, [0 0 0], [0 0 10], [100.5 0 0], false, false);
            tc.verifyEqual(ang, [0 -5/I(2)*tc.Dt 0], "AbsTol", 1e-3);
        end

        function globalPointAtCentreGivesNoTorque(tc)
            % BUG #2: a global point exactly at the body centre is a zero
            % offset -> no moment. (Was a large spurious torque before.)
            [~, ang] = tc.applyForce(tc.Far, [0 0 0], [0 0 10], tc.Far, false, false);
            tc.verifyEqual(ang, [0 0 0], "AbsTol", 1e-5);
        end

        function localAndGlobalPointDescribeSameForce(tc)
            % The same physical application point, given as a local offset or
            % as an absolute world coordinate, must produce the same torque.
            [~, angLocal]  = tc.applyForce(tc.Far, [0 0 0], [0 0 10], [0.5 0 0],   false, true);
            [~, angGlobal] = tc.applyForce(tc.Far, [0 0 0], [0 0 10], [100.5 0 0], false, false);
            tc.verifyEqual(angGlobal, angLocal, "AbsTol", 1e-3);
        end

        function localTorqueIsNotTranslatedByPosition(tc)
            % Torque shares the local->world vector path. A local torque about
            % z on a body spun +90 about z stays about world z (invariant);
            % the position must not leak in -> dw = tau/I*dt about z only.
            [ang, I] = tc.applyTorque(tc.Far, tc.Spin, [0 0 5], true);
            tc.verifyEqual(ang, [0 0 5/I(3)*tc.Dt], "AbsTol", 1e-3);
        end

        function localTorqueRotatesButDoesNotTranslate(tc)
            % A local torque about +x on a body spun +90 about z becomes a
            % world torque about +y (rotation applied, no position added).
            [ang, I] = tc.applyTorque(tc.Far, tc.Spin, [5 0 0], true);
            tc.verifyEqual(ang(1:2), [0 5/I(2)*tc.Dt], "AbsTol", 1e-3);
        end
    end

    methods (Access = private)
        function [lin, ang, m, I] = applyForce(tc, pos, eul, F, P, localForce, localPoint)
            b = tc.spawnCube(pos, eul);
            m = b.Mass; I = b.Inertia;
            sim = phx.Simulation(b, "Gravity", [0 0 0]);
            b.applyForce(F, P, localForce, localPoint);
            sim.step(tc.Dt, 1, -1);
            lin = b.LinearVelocity; ang = b.AngularVelocity;
            delete(sim);
        end

        function [ang, I] = applyTorque(tc, pos, eul, T, localTorque)
            b = tc.spawnCube(pos, eul);
            I = b.Inertia;
            sim = phx.Simulation(b, "Gravity", [0 0 0]);
            b.applyTorque(T, localTorque);
            sim.step(tc.Dt, 1, -1);
            ang = b.AngularVelocity;
            delete(sim);
        end

        function b = spawnCube(tc, pos, eul)
            % A 1 kg cube (0.2 m side, density 125) with isotropic inertia.
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            b = phx.Body(axes(f), "Position", pos, "EulerAngles", eul, ...
                "Shape", {"Box", "Size", [0.2 0.2 0.2], "Density", 125});
        end
    end

end
