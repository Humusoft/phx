classdef tForceApplication < PhxTestCase
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
%   Every case applies a single known force/torque to a 1 kg cube for one
%   short step with gravity off, then checks the resulting velocity against
%   the analytic impulse dv = F*dt/m (and dw = tau*dt/I). The expected values
%   are derived from the body's own Mass/Inertia so the test targets the
%   application logic, not the mass/inertia computation (see tShapeMass).
%   The body is deliberately placed far from the origin and often rotated,
%   which is exactly where both bugs used to show up.
%
%   The cases are a table rather than one test each, so that the matrix they
%   span - local/world vector against local/global point, at the origin and
%   far from it, upright and rotated - can be read in one place. The name of
%   each row becomes the name of its test.
%
%   See also phx.Body, tSimulation, tShapeMass

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant)
        Dt = 0.01           % single-substep interval
        Far = [100 0 0]     % body placed far from the origin
        Spin = [0 0 pi/2]   % +90 deg about z: local x -> world y
    end

    properties (TestParameter)
        Case = tForceApplication.cases
    end

    methods (TestClassSetup)
        function engineIsPresent(tc)
            tc.requireEngine;
        end
    end

    methods (Test, TestTags = {'Engine'})
        function impulseMatchesTheAnalyticValue(tc, Case)
            b = tc.spawnBody(Case.Position, "EulerAngles", Case.EulerAngles, ...
                "Shape", {"Box", "Size", [0.2 0.2 0.2], "Density", 125});
            m = b.Mass;
            I = b.Inertia;
            sim = phx.Simulation(b, "Gravity", [0 0 0]);
            tc.addTeardown(@() delete(sim));

            if isempty(Case.Torque)
                b.applyForce(Case.Force, Case.Point, Case.LocalVector, Case.LocalPoint);
            else
                b.applyTorque(Case.Torque, Case.LocalVector);
            end
            sim.step(tc.Dt, 1, -1);

            if ~isempty(Case.LinearVelocity)
                tc.verifyEqual(b.LinearVelocity, Case.LinearVelocity(m, I), ...
                    "AbsTol", Case.Tolerance);
            end
            if ~isempty(Case.AngularVelocity)
                tc.verifyEqual(b.AngularVelocity, Case.AngularVelocity(m, I), ...
                    "AbsTol", Case.Tolerance);
            end
        end
    end

    methods (Static, Access = private)
        function s = cases()
            % One row per application case. LinearVelocity/AngularVelocity are
            % the analytic result as a function of the body's own mass and
            % inertia; an empty one is not checked.
            dt = tForceApplication.Dt;
            far = tForceApplication.Far;
            spin = tForceApplication.Spin;
            row = @tForceApplication.makeCase;

            % BUG #1: local force far from origin, no rotation. Body position
            % must not leak into the force -> pure dv = F/m*dt along x.
            s.localForceFarFromOrigin = row("Position", far, ...
                "Force", [10 0 0], "LocalVector", true, ...
                "LinearVelocity", @(m, I) [10/m*dt 0 0]);

            % BUG #1: rotated body far from origin. The local +x force must
            % rotate into world +y (rotation IS applied) with no position.
            s.localForceOnARotatedBody = row("Position", far, "EulerAngles", spin, ...
                "Force", [10 0 0], "LocalVector", true, ...
                "LinearVelocity", @(m, I) [0 10/m*dt 0]);

            % Control: world-frame force was always correct, stays correct.
            s.worldForceIsIndependentOfPosition = row("Position", far, "EulerAngles", spin, ...
                "Force", [0 10 0], "LocalVector", false, ...
                "LinearVelocity", @(m, I) [0 10/m*dt 0]);

            % Control: at the origin the (buggy) point-transform coincided
            % with the correct vector-transform, so this always passed.
            s.localForceAtTheOrigin = row("Position", [0 0 0], ...
                "Force", [10 0 0], "LocalVector", true, ...
                "LinearVelocity", @(m, I) [10/m*dt 0 0]);

            % BUG #2: a global point is absolute world coordinates. A point
            % 0.5 m in +x from the centre of a body at x=100 is [100.5 0 0];
            % the moment must be taken about the centre (offset 0.5), giving
            % tau = [0.5 0 0] x [0 0 10] = [0 -5 0], NOT about the origin.
            s.globalPointOffTheCentre = row("Position", far, ...
                "Force", [0 0 10], "Point", [100.5 0 0], "LocalPoint", false, ...
                "AngularVelocity", @(m, I) [0 -5/I(2)*dt 0], "Tolerance", 1e-3);

            % The same physical point given as a local offset must produce the
            % same moment - checked against the same analytic value.
            s.localPointOffTheCentre = row("Position", far, ...
                "Force", [0 0 10], "Point", [0.5 0 0], "LocalPoint", true, ...
                "AngularVelocity", @(m, I) [0 -5/I(2)*dt 0], "Tolerance", 1e-3);

            % BUG #2: a global point exactly at the body centre is a zero
            % offset -> no moment. (Was a large spurious torque before.)
            s.globalPointAtTheCentre = row("Position", far, ...
                "Force", [0 0 10], "Point", far, "LocalPoint", false, ...
                "AngularVelocity", @(m, I) [0 0 0], "Tolerance", 1e-5);

            % Torque shares the local->world vector path. A local torque about
            % z on a body spun +90 about z stays about world z (invariant);
            % the position must not leak in -> dw = tau/I*dt about z only.
            s.localTorqueAboutTheSpinAxis = row("Position", far, "EulerAngles", spin, ...
                "Torque", [0 0 5], "LocalVector", true, ...
                "AngularVelocity", @(m, I) [0 0 5/I(3)*dt], "Tolerance", 1e-3);

            % A local torque about +x on a body spun +90 about z becomes a
            % world torque about +y (rotation applied, no position added).
            s.localTorqueAcrossTheSpinAxis = row("Position", far, "EulerAngles", spin, ...
                "Torque", [5 0 0], "LocalVector", true, ...
                "AngularVelocity", @(m, I) [0 5/I(2)*dt 0], "Tolerance", 1e-3);
        end

        function c = makeCase(Options)
            arguments
                Options.Position (1, 3) double = [0 0 0]
                Options.EulerAngles (1, 3) double = [0 0 0]
                Options.Force (1, 3) double = [0 0 0]
                Options.Torque double = []
                Options.Point (1, 3) double = [0 0 0]
                Options.LocalVector (1, 1) logical = false
                Options.LocalPoint (1, 1) logical = true
                Options.LinearVelocity = []
                Options.AngularVelocity = []
                Options.Tolerance (1, 1) double = 1e-4
            end
            c = Options;
        end
    end

end
