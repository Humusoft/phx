classdef tEngineProtocol < PhxTestCase
%tEngineProtocol Contract tests of the phx.engine.io protocol 1.
%
%   These tests call phx.engine.io directly, without any phx.* object, and
%   check what PHX relies on: the verbs, the shapes of their inputs and
%   outputs, and the physical meaning of what they set. Only the parts PHX
%   uses are tested, and nothing that the protocol marks for cleanup is
%   locked in beyond what PHX needs today.
%
%   The engine under test is the phx.engine.io on the path. Another
%   implementation of the protocol is tested by passing its folder as the
%   external parameter MexFolder, which is put ahead of the path for the
%   class:
%
%       import matlab.unittest.parameters.Parameter
%       p = Parameter.fromData("MexFolder", {"<folder holding +phx/+engine/io>"});
%       runtests("tEngineProtocol", "ExternalParameters", p)
%
%   Tolerances are those of single precision where an engine may compute in
%   it; contact cases allow what a contact solver needs to settle.
%
%   See also tForceApplication, tJointContract, phx.engine.Settings

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant)
        G = 9.81    % gravity used by the tests
        Dt = 0.005  % step of the settling and joint cases
    end

    properties (ClassSetupParameter)
        % Folder holding +phx/+engine/io of the engine under test; empty
        % tests the one already on the path
        MexFolder = struct("onPath", "")
    end

    properties (TestParameter)
        Shape = tEngineProtocol.shapeCases
    end

    methods (TestClassSetup)
        function selectEngine(tc, MexFolder)
            if MexFolder ~= ""
                tc.assumeTrue(isfolder(MexFolder), "Engine folder " + MexFolder + " does not exist.");
                oldPath = addpath(MexFolder, "-begin");
                clear("mex"); %#ok<CLMEX> unload the engine loaded before
                tc.addTeardown(@() tEngineProtocol.restorePath(oldPath));
                tc.assertTrue(startsWith(which("phx.engine.io"), MexFolder), ...
                    "phx.engine.io does not resolve to " + MexFolder + ".");
            end
            tc.requireEngine;
        end
    end

    % Worlds and stepping
    methods (Test, TestTags = {'Engine'})
        function setupGivesDistinctWorldHandles(tc)
            w1 = tc.newWorld;
            w2 = tc.newWorld;
            tc.verifyClass(w1, "uint64");
            tc.verifySize(w1, [1 1]);
            tc.verifyNotEqual(w1, uint64(0));
            tc.verifyNotEqual(w1, w2);
        end

        function stepWithoutMovingBodiesReturnsFalse(tc)
            w = tc.newWorld;
            tc.addBox(w, 0, [0 0 0], [1 1 1]);
            [ids, T] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(ids, false);
            tc.verifyEqual(T, false);
        end

        function stepReportsMovedBodiesWithTheirPoses(tc)
            w = tc.newWorld;
            tc.addBox(w, 0, [0 0 -10], [1 1 1]);
            h1 = tc.addBox(w, 2, [1 0 5], 0.2*[1 1 1]);
            h2 = tc.addBox(w, 2, [-1 0 5], 0.2*[1 1 1]);
            [ids, T] = phx.engine.io('step', w, tc.Dt);
            tc.verifyClass(ids, "uint64");
            tc.verifySize(ids, [1 2]);
            tc.verifyEqual(sort(ids), sort([h1 h2]));
            tc.verifySize(T, [4 4 2]);
            x = squeeze(T(1, 4, :))';
            tc.verifyEqual(x(ids == h1), 1, "AbsTol", 1e-6);
            tc.verifyEqual(x(ids == h2), -1, "AbsTol", 1e-6);
            tc.verifyEqual(T(4, :, 1), [0 0 0 1]);
        end

        function freeFallFollowsSemiImplicitEuler(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 10], 0.2*[1 1 1]);
            n = 20;
            for k = 1:n
                [~, T] = phx.engine.io('step', w, tc.Dt);
            end
            % v(k) = -g*k*dt, z(n) = z0 + dt*sum(v(1..n))
            tc.verifyEqual(T(3, 4), 10 - tc.G*tc.Dt^2*n*(n + 1)/2, "AbsTol", 1e-5);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0; 0; -tc.G*n*tc.Dt], "AbsTol", 1e-5);
        end

        function gravityIsTheWorldAcceleration(tc)
            w = tc.newWorld([1 -2 0.5]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            [~] = phx.engine.io('step', w, 0.01);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), 0.01*[1; -2; 0.5], "AbsTol", 1e-6);
        end

        function worldsAreIndependent(tc)
            wA = tc.newWorld;
            wB = tc.newWorld;
            hA = tc.addBox(wA, 2, [0 0 0], 0.2*[1 1 1]);
            hB = tc.addBox(wB, 2, [0 0 0], 0.2*[1 1 1]);
            tc.verifyNotEqual(hA, hB, "Body handles are unique across worlds.");
            for k = 1:10
                [~] = phx.engine.io('step', wA, tc.Dt);
            end
            tc.verifyEqual(phx.engine.io('get', wB, hB, 'linvel'), [0; 0; 0]);
            tc.verifyLessThan(phx.engine.io('get', wA, hA, 'linvel'), [1; 1; 0]);
        end
    end

    % Shapes
    methods (Test, TestTags = {'Engine'})
        function shapeRestsOnItsGeometry(tc, Shape)
            % Dropped on a floor whose top is at z = 0, every shape comes to
            % rest with its origin at the height its geometry gives. The
            % margin is zeroed so that the shapes collide as they are.
            w = tc.newWorld;
            phx.engine.io('set', w, uint64(0), 'margin', 0);
            tc.addFloor(w);
            h = Shape.Add(w, tEngineProtocol.pose([0 0 1.5]));
            tc.verifyClass(h, "uint64");
            tc.verifyNotEqual(h, uint64(0));
            T = tc.settle(w, 2);
            tc.verifyEqual(T(3, 4), Shape.RestHeight, "AbsTol", Shape.Tolerance);
        end

        function concaveCubeComesToRest(tc)
            % A closed concave cube dropped on a box floor settles flat and
            % stays still: the triangles of a closed mesh give no contacts
            % from behind. (The Jolt gateway with two-sided triangles rocked
            % it by up to 26 deg and hopped it 0.13 m for ever.)
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            w = tc.newWorld;
            phx.engine.io('set', w, uint64(0), 'margin', 0);
            tc.addFloor(w);
            h = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0 0 1]));
            T = tc.lastPose(w, h, 3);
            speed = 0;
            for k = 1:200
                [ids, P] = phx.engine.io('step', w, tc.Dt);
                if ~islogical(ids) && any(ids == h)
                    T = P(:, :, ids == h);
                end
                speed = max([speed, norm(phx.engine.io('get', w, h, 'linvel')), ...
                    0.2*norm(phx.engine.io('get', w, h, 'angvel'))]);
            end
            tc.verifyEqual(T(3, 4), 0.2, "AbsTol", 2e-3, "Height of the resting cube (m)");
            tc.verifyLessThan(acosd(min(1, T(3, 3))), 1, "Tilt of the resting cube (deg)");
            tc.verifyLessThan(speed, 0.01, "Largest speed of the resting cube in its last second (m/s)");
        end

        function fanMeshCarriesTheFrictionWhereItActs(tc)
            % A 2 kg box sliding along y across a static disc whose top is a
            % fan of 64 triangles meeting at the centre, 5 cm off that
            % centre: the friction acts where the box slides, so its moment
            % about the disc's vertical axis is 0.05 m times the friction.
            % (With the contacts of every triangle kept apart, the many
            % slivers around the fan's centre took the load and the friction
            % there, and the moment was about 0.)
            tc.requireProtocol(1.1);
            n = 64;
            a = 2*pi*(0:n - 1)/n;
            V = [0 0 0; cos(a') sin(a') zeros(n, 1)]';   % centre, then the rim
            F = int32([zeros(1, n); 1:n; [2:n 1]]);
            w = tc.newWorld;
            phx.engine.io('set', w, uint64(0), 'margin', 0);
            disc = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0 0 0]), 0);
            box = tc.addBox(w, 2, [0.05 -0.3 0.1], [0.2 0.2 0.2], 2);
            tc.settle(w, 0.2);
            phx.engine.io('set', w, box, 'linvel', [0 2 0]);
            arm = zeros(1, 40);
            for k = 1:40
                [~] = phx.engine.io('step', w, tc.Dt);
                f = phx.engine.io('get', w, disc, 'totalforce');
                t = phx.engine.io('get', w, disc, 'totaltorque');
                arm(k) = t(3)/f(2);   % torque about z of a force along y at x
            end
            tc.verifyEqual(median(arm), 0.05, "AbsTol", 0.005, "Arm of the friction about the disc's axis (m)");
        end

        function concaveBodiesCollideWithEachOther(tc)
            % A concave cube dropped on a concave plate rests on it, and a
            % moving concave cube pushes another one instead of passing it.
            % Concave envelopes keep an engine-chosen cushion, so only a
            % coarse height is part of the contract.
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            w = tc.newWorld;
            plate = tEngineProtocol.addTrimesh(w, V.*[10; 10; 0.5], F, tEngineProtocol.pose([0 0 -0.1]), 0);
            tc.verifyClass(plate, "uint64");
            h = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0 0 1]));
            T = tc.lastPose(w, h, 2);
            tc.verifyEqual(T(3, 4), 0.2, "AbsTol", 0.06, "The cube rests on the plate.");

            w = tc.newWorld([0 0 0]);
            a = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0 0 0]));
            b = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([1 0 0]));
            phx.engine.io('set', w, a, 'linvel', [1 0 0]);
            tc.settle(w, 2);
            vb = phx.engine.io('get', w, b, 'linvel');
            tc.verifyGreaterThan(vb(1), 0.3, "The struck cube is pushed along.");
            Ta = tc.lastPose(w, a, tc.Dt);
            Tb = tc.lastPose(w, b, tc.Dt);
            tc.verifyLessThan(Ta(1, 4), Tb(1, 4) - 0.3, "The cubes do not pass through each other.");
        end

        function terrainIsCentredBetweenItsLowestAndHighestPoint(tc)
            % The heightfield origin lies midway between the min and max
            % height passed in, so a surface at height 0.2 with a 1.0 peak
            % elsewhere lies 0.4 below the terrain body's origin.
            w = tc.newWorld;
            H = 0.2*ones(11);
            H(end, end) = 1;
            phx.engine.io('add', w, 'terrain', 0, H(:), 11, 11, 0.2, 1.0, 'z', [1 1 1], eye(4), 0, [0 0 0]);
            phx.engine.io('add', w, 'sphere', 2, 0.25, tEngineProtocol.pose([-2 -2 2]), 1, [0.1 0.1 0.1]);
            T = tc.settle(w, 2);
            tc.verifyEqual(T(3, 4), 0.2 - 0.6 + 0.25, "AbsTol", 2e-3);
        end
    end

    % Body fields
    methods (Test, TestTags = {'Engine'})
        function transformTeleportsTheBody(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            P = makehgtform("translate", [3 -2 1], "zrotate", 0.7, "xrotate", -0.3);
            phx.engine.io('set', w, h, 'transform', P);
            [~, T] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(T, P, "AbsTol", 1e-6);
        end

        function typeSwitchesBetweenStaticKinematicAndDynamic(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 0, [0 0 5], 0.2*[1 1 1], 1);
            [~] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0; 0; 0], "A static body does not fall.");

            phx.engine.io('set', w, h, 'state', 2);
            [ids, T] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(ids, h);
            tc.verifyLessThan(T(3, 4), 5, "A body switched to dynamic falls.");

            phx.engine.io('set', w, h, 'state', 1);
            phx.engine.io('set', w, h, 'linvel', [0 0 0]);
            for k = 1:10
                [~] = phx.engine.io('step', w, tc.Dt);
            end
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0; 0; 0], "AbsTol", 1e-9, ...
                "A kinematic body does not fall.");
        end

        function massAndInertiaScaleTheResponse(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phx.engine.io('set', w, h, 'mass', 2);
            phx.engine.io('set', w, h, 'inertia', [1 2 4]);
            % Bullet takes a changed inertia into its world tensor only in
            % the next step (see PROTOCOL.md)
            [~] = phx.engine.io('step', w, 0.01);
            [~] = phx.engine.io('apply', w, h, 'centralforce', [4 0 0], false);
            [~] = phx.engine.io('apply', w, h, 'torque', [1 1 1], false);
            [~] = phx.engine.io('step', w, 0.01);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0.02; 0; 0], "AbsTol", 1e-6);
            tc.verifyEqual(phx.engine.io('get', w, h, 'angvel'), 0.01*[1; 1/2; 1/4], "AbsTol", 1e-5);
        end

        function freeBodyTumblesAboutItsMiddleAxis(tc)
            % The gyroscopic effect: a box spun close to its intermediate
            % principal axis turns over (Euler's equations put the axis
            % upside down from about 1.55 s to 5.93 s), where an engine
            % without it would keep the box spinning about that axis
            s = [0.1 0.3 0.6];
            I = (sum(s.^2) - s.^2)/12;
            w = tc.newWorld([0 0 0]);
            b = tc.addBox(w, 2, [0 0 0], s, 1, I);
            phx.engine.io('set', w, b, 'angvel', [0.2 3 0.2]);
            T = tc.lastPose(w, b, 3);
            tc.verifyLessThan(T(2, 2), -0.9, "The middle axis has turned over.");
        end

        function zeroMassFreezesTheBody(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 0, [1 1 1]);
            [~] = phx.engine.io('apply', w, h, 'centralforce', [1 0 0], false);
            [~] = phx.engine.io('apply', w, h, 'torque', [1 1 1], false);
            [~] = phx.engine.io('step', w, 0.01);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0; 0; 0], "AbsTol", 1e-9);
            tc.verifyEqual(phx.engine.io('get', w, h, 'angvel'), [0; 0; 0], "AbsTol", 1e-9);
        end

        function zeroInertiaFreezesThatRotation(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [0 0 1]);
            [~] = phx.engine.io('apply', w, h, 'torque', [1 1 1], false);
            [~] = phx.engine.io('step', w, 0.01);
            tc.verifyEqual(phx.engine.io('get', w, h, 'angvel'), [0; 0; 0.01], "AbsTol", 1e-6);
        end

        function velocitiesRoundTrip(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phx.engine.io('set', w, h, 'linvel', [1 -2 3]);
            phx.engine.io('set', w, h, 'angvel', [-0.5 0.25 2]);
            v = phx.engine.io('get', w, h, 'linvel');
            omega = phx.engine.io('get', w, h, 'angvel');
            tc.verifySize(v, [3 1]);
            tc.verifySize(omega, [3 1]);
            tc.verifyEqual(v, [1; -2; 3], "AbsTol", 1e-6);
            tc.verifyEqual(omega, [-0.5; 0.25; 2], "AbsTol", 1e-6);
        end

        function frictionOfTheTwoBodiesMultiplies(tc)
            % A box sliding on a floor decelerates by mu_box*mu_floor*g
            w = tc.newWorld;
            phx.engine.io('set', w, tc.addFloor(w), 'friction', 0.8);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phx.engine.io('set', w, h, 'friction', 0.5);
            tc.settle(w, 0.1);
            phx.engine.io('set', w, h, 'linvel', [3 0 0]);
            v0 = phx.engine.io('get', w, h, 'linvel');
            tc.settle(w, 0.2);
            v1 = phx.engine.io('get', w, h, 'linvel');
            tc.verifyEqual((v0(1) - v1(1))/0.2, 0.5*0.8*tc.G, "RelTol", 0.03);
        end

        function restitutionSetsTheBounce(tc)
            peak = zeros(1, 2);
            for i = 1:2
                r = i - 1;
                w = tc.newWorld;
                phx.engine.io('set', w, tc.addFloor(w), 'restitution', 1);
                h = phx.engine.io('add', w, 'sphere', 2, 0.2, tEngineProtocol.pose([0 0 1.2]), 1, [0.016 0.016 0.016]);
                phx.engine.io('set', w, h, 'restitution', r);
                bounced = false;
                for k = 1:250
                    [~, T] = phx.engine.io('step', w, tc.Dt);
                    v = phx.engine.io('get', w, h, 'linvel');
                    bounced = bounced || v(3) > 0;
                    if bounced && ~islogical(T)
                        peak(i) = max(peak(i), T(3, 4));
                    end
                end
            end
            tc.verifyLessThan(peak(1), 0.22, "Restitution 0 does not bounce.");
            tc.verifyGreaterThan(peak(2), 0.9*1.2, "Restitution 1 bounces back to about the drop height.");
        end

        function slidingSpinningPlateStopsBothTogether(tc)
            % A flat square plate launched at 1 m/s and 20 rad/s on a floor
            % slides and spins down together, as Coulomb friction at its
            % four corners couples the two (both stop at 0.33 s; the
            % sliding alone would stop at 0.20 s). (Jolt 5.6.0 limited the
            % linear and the twist friction independently: the sliding
            % stopped at 0.205 s whatever the spin; Bullet 0.365 s.)
            s = [0.2 0.2 0.02];
            w = tc.newWorld;
            phx.engine.io('set', w, tc.addFloor(w), 'friction', 1);
            h = tc.addBox(w, 2, [0 0 0.01], s);
            phx.engine.io('set', w, h, 'friction', 0.5);
            phx.engine.io('set', w, h, 'linvel', [1 0 0]);
            phx.engine.io('set', w, h, 'angvel', [0 0 20]);
            slideStops = NaN;
            spinStops = NaN;
            for k = 1:round(1/tc.Dt)
                [~] = phx.engine.io('step', w, tc.Dt);
                v = phx.engine.io('get', w, h, 'linvel');
                om = phx.engine.io('get', w, h, 'angvel');
                if isnan(slideStops) && norm(v(1:2)) < 1e-3
                    slideStops = k*tc.Dt;
                end
                if isnan(spinStops) && abs(om(3)) < 1e-2
                    spinStops = k*tc.Dt;
                end
            end
            tc.verifyEqual(slideStops, 0.329, "RelTol", 0.15, "Time the sliding stops (s)");
            tc.verifyEqual(spinStops, slideStops, "AbsTol", 0.04, "Time the spinning stops (s)");
        end

        function bounceRisesToRestitutionSquared(tc)
            % A ball dropped from 1 m with restitution 0.8 (the floor 1)
            % bounces back to e^2 = 0.64 m. (Jolt 5.6.0 reached 0.115 m at
            % this step, as its speculative contact bounced the ball from
            % where it was; Bullet 0.704 m.)
            r = 0.1;
            w = tc.newWorld;
            phx.engine.io('set', w, tc.addFloor(w), 'restitution', 1);
            h = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 1 + r]), 1, 0.4*r^2*[1 1 1]);
            phx.engine.io('set', w, h, 'restitution', 0.8);
            z = nan(1, round(2/tc.Dt));
            for k = 1:numel(z)
                [ids, T] = phx.engine.io('step', w, tc.Dt);
                if ~islogical(ids) && any(ids == h)
                    z(k) = T(3, 4, ids == h) - r;
                end
            end
            [~, touch] = min(z);
            tc.verifyEqual(max(z(touch:end)), 0.64, "RelTol", 0.15, "Height of the first bounce");
        end

        function ballCrossesASeamWithoutHopping(tc)
            % A ball rolling fast over the seam of two coplanar static boxes
            % stays on them. (Jolt 5.6.0 bounced it off the tilted normal of
            % the second box's edge, which it predicted to hit: 121 mm with
            % restitution 0.6, Bullet 12 mm.)
            r = 0.1;
            dt = 0.002;
            w = tc.newWorld;
            for x = [-2 2]
                lane = tc.addBox(w, 0, [x 0 -0.05], [4 1 0.1], 0, [0 0 0]);
                phx.engine.io('set', w, lane, 'restitution', 0.1);
            end
            h = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([-3.5 0 r]), 1, 0.4*r^2*[1 1 1]);
            phx.engine.io('set', w, h, 'restitution', 0.6);
            phx.engine.io('set', w, h, 'linvel', [8 0 0]);
            phx.engine.io('set', w, h, 'angvel', [0 8/r 0]);
            lift = 0;
            for k = 1:round(0.8/dt)
                [ids, T] = phx.engine.io('step', w, dt);
                if ~islogical(ids) && any(ids == h)
                    lift = max(lift, T(3, 4, ids == h) - r);
                end
            end
            tc.verifyLessThan(lift, 0.025, "Highest lift of the ball (m)");
        end

        function ballCrossesASeamAtLargeSteps(tc)
            % The ball of ballCrossesASeamWithoutHopping at 5 and 10 ms
            % steps stays within 5 mm of the boxes. (It predicted a hit
            % with the second box's edge, which it only passes at zero
            % distance: 34 and 192 mm before item 21.)
            % Protocol 1.1: the Bullet MEX lifts it 38 mm at 5 ms.
            tc.requireProtocol(1.1);
            r = 0.1;
            for dt = [0.005 0.01]
                w = tc.newWorld;
                for x = [-2 2]
                    lane = tc.addBox(w, 0, [x 0 -0.05], [4 1 0.1], 0, [0 0 0]);
                    phx.engine.io('set', w, lane, 'restitution', 0.1);
                end
                h = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([-3.5 0 r]), 1, 0.4*r^2*[1 1 1]);
                phx.engine.io('set', w, h, 'restitution', 0.6);
                phx.engine.io('set', w, h, 'linvel', [8 0 0]);
                phx.engine.io('set', w, h, 'angvel', [0 8/r 0]);
                lift = 0;
                for k = 1:round(0.8/dt)
                    [ids, T] = phx.engine.io('step', w, dt);
                    if ~islogical(ids) && any(ids == h)
                        lift = max(lift, T(3, 4, ids == h) - r);
                    end
                end
                tc.verifyLessThan(lift, 0.005, "Highest lift of the ball (m), dt " + dt);
            end
        end

        function spheresMeetingObliquelyBounceApart(tc)
            % Two equal spheres, one moving at 3 m/s, meet half a radius off
            % centre (restitution 1, no friction): the normal velocities are
            % exchanged, within 0.1 m/s at 5 ms steps. (The speculative
            % contact stopped them short of touching and they parted without
            % a bounce: 1.5 m/s off before item 21.)
            % Protocol 1.1: the Bullet MEX is 0.26 m/s off.
            tc.requireProtocol(1.1);
            r = 0.1;
            b = 0.5*r;
            w = tc.newWorld([0 0 0]);
            h1 = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([-0.3 0 0]), 1, 0.4*r^2*[1 1 1]);
            h2 = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 b 0]), 1, 0.4*r^2*[1 1 1]);
            for h = [h1 h2]
                phx.engine.io('set', w, h, 'restitution', 1);
                phx.engine.io('set', w, h, 'friction', 0);
            end
            phx.engine.io('set', w, h1, 'linvel', [3 0 0]);
            for k = 1:round(0.2/tc.Dt)
                phx.engine.io('step', w, tc.Dt);
            end
            n = [sqrt(4*r^2 - b^2); b; 0]/(2*r);
            vn = 3*n(1);
            tc.verifyEqual(phx.engine.io('get', w, h1, 'linvel'), [3; 0; 0] - vn*n, "AbsTol", 0.1, "Velocity of the hitting sphere (m/s)");
            tc.verifyEqual(phx.engine.io('get', w, h2, 'linvel'), vn*n, "AbsTol", 0.1, "Velocity of the hit sphere (m/s)");
        end

        function slowlyRollingBallKeepsItsSpeed(tc)
            % A ball rolling slowly without slip on a flat floor keeps its
            % speed and rests on the surface. (Jolt reusing the contacts of
            % the last step, glued to the turning ball, sped it up by a
            % third in 10 s and let it sink 1 mm.)
            r = 0.1;
            v0 = 0.15;
            w = tc.newWorld;
            tc.addFloor(w);
            h = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([-4 0 r]), 1, 0.4*r^2*[1 1 1]);
            phx.engine.io('set', w, h, 'linvel', [v0 0 0]);
            phx.engine.io('set', w, h, 'angvel', [0 v0/r 0]);
            lowest = r;
            for k = 1:round(10/tc.Dt)
                [ids, T] = phx.engine.io('step', w, tc.Dt);
                if ~islogical(ids) && any(ids == h)
                    lowest = min(lowest, T(3, 4, ids == h));
                end
            end
            v = phx.engine.io('get', w, h, 'linvel');
            tc.verifyEqual(v(1), v0, "RelTol", 0.01, "Speed after 10 s");
            tc.verifyGreaterThan(lowest, r - 2e-4, "Lowest height of the centre");
        end

        function tiltedDiskLandsOnItsRimWithoutSinking(tc)
            % A thin disk set down tilted on the lowest point of its rim
            % falls flat; the rim point it stands on keeps carrying it.
            % (Jolt 5.6.0 pruned that point in favour of rim points still
            % apart from the floor, and the rim sank 1.2-3.2 mm.)
            r = 0.3;
            hh = 0.01;
            rim = [r*cospi(0:0.005:2); r*sinpi(0:0.005:2); -hh*ones(1, 401)];
            for tilt = [1 2 4 8]
                w = tc.newWorld;
                tc.addFloor(w);
                R = [1 0 0; 0 cosd(tilt) -sind(tilt); 0 sind(tilt) cosd(tilt)];
                lowest = R*[0; -r; -hh];
                h = phx.engine.io('add', w, 'cylinder', 2, [r r hh], 'z', [R [0; 0; -lowest(3)]; 0 0 0 1], 1, [0.02 0.02 0.045]);
                depth = 0;
                for k = 1:20
                    [ids, T] = phx.engine.io('step', w, tc.Dt);
                    if ~islogical(ids) && any(ids == h)
                        M = T(:, :, ids == h);
                        depth = max(depth, -min(M(3, 1:3)*rim + M(3, 4)));
                    end
                end
                tc.verifyLessThan(depth, 1e-3, "Rim depth in the floor, tilt " + tilt + " deg");
            end
        end

        function bodyWithoutCollisionsFallsThrough(tc)
            w = tc.newWorld;
            tc.addFloor(w);
            h = tc.addBox(w, 2, [0 0 0.5], 0.2*[1 1 1]);
            phx.engine.io('set', w, h, 'collisions', false);
            T = tc.settle(w, 1);
            tc.verifyLessThan(T(3, 4), -1);
        end

        function kinematicBodyMovedByPoseCarriesWhatRestsOnIt(tc)
            % A kinematic plate driven by writing its pose every step moves
            % with the velocity of those poses, so friction carries a box
            w = tc.newWorld;
            plate = tc.addBox(w, 1, [0 0 -0.1], [4 4 0.2]);
            phx.engine.io('set', w, plate, 'friction', 1);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phx.engine.io('set', w, h, 'friction', 1);
            tc.settle(w, 0.1);
            v = 0.5;
            n = round(1/tc.Dt);
            for k = 1:n
                phx.engine.io('set', w, plate, 'transform', tEngineProtocol.pose([v*k*tc.Dt 0 -0.1]));
                [ids, T] = phx.engine.io('step', w, tc.Dt);
            end
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [v; 0; 0], "AbsTol", 0.02);
            tc.verifyEqual(T(1, 4, ids == h), v*n*tc.Dt, "AbsTol", 0.02, "The box rides along.");
        end

        function tiltedStackDoesNotTwist(tc)
            % A stack of six wide, light boxes (3 x 3 x 1 m, 1 kg, inertia
            % 0.1, as in phxex_stack) on a kinematic pallet tilted to 8 deg
            % stays put: nothing turns it about the vertical. (A friction
            % model letting the twist friction reverse the twist made such
            % a stack twitch at 2 rad/s and topple at 15 instead of 23 deg;
            % Bullet twitches at 0.3 rad/s.)
            w = tc.newWorld;
            pallet = tc.addBox(w, 1, [0 0 0], [8 8 0.5]);
            phx.engine.io('set', w, pallet, 'friction', 0.5);
            offsets = [0.02 -0.03; -0.04 0.01; 0.03 0.04; -0.01 -0.02; 0.04 0.02; -0.03 0.03];
            boxes = zeros(1, 6, "uint64");
            for i = 1:6
                boxes(i) = tc.addBox(w, 2, [offsets(i, :) 0.25 + (i - 0.5) + 0.02*i], [3 3 1], 1, [0.1 0.1 0.1]);
                phx.engine.io('set', w, boxes(i), 'friction', 0.5);
            end
            tc.settle(w, 0.5);
            n = round(1.6/tc.Dt);
            twist = 0;
            for k = 1:n
                tilt = deg2rad(8)*k/n;
                phx.engine.io('set', w, pallet, 'transform', makehgtform("xrotate", tilt));
                [~] = phx.engine.io('step', w, tc.Dt);
                for i = 1:6
                    om = phx.engine.io('get', w, boxes(i), 'angvel');
                    up = makehgtform("xrotate", tilt)*[0; 0; 1; 0];
                    twist = max(twist, abs(om'*up(1:3)));
                end
            end
            tc.verifyLessThan(twist, 0.5, "Largest turning rate of a box about the pallet normal (rad/s)");
        end

        function kinematicBodyTiltedByPoseHoldsWhatRestsOnIt(tc)
            % Tilting a kinematic plate by its pose turns a box resting on
            % it along, held by static friction (tan 0.15 < 0.8*0.8)
            w = tc.newWorld;
            plate = tc.addBox(w, 1, [0 0 -0.1], [4 4 0.2]);
            phx.engine.io('set', w, plate, 'friction', 0.8);
            h = tc.addBox(w, 2, [1 0 0.2], 0.4*[1 1 1]);
            phx.engine.io('set', w, h, 'friction', 0.8);
            tc.settle(w, 0.1);
            n = round(1/tc.Dt);
            for k = 1:n
                tilt = 0.15*k/n;
                phx.engine.io('set', w, plate, 'transform', makehgtform("yrotate", -tilt, "translate", [0 0 -0.1]));
                [ids, T] = phx.engine.io('step', w, tc.Dt);
            end
            R = T(1:3, 1:3, ids == h);
            tc.verifyEqual(atan2(R(3, 1), R(1, 1)), 0.15, "AbsTol", 0.01, "The box tilts with the plate.");
            % Where the box would be had it stayed fixed to the plate
            expected = makehgtform("yrotate", -0.15)*[1; 0; 0.2; 1];
            tc.verifyEqual(T(1:3, 4, ids == h), expected(1:3), "AbsTol", 0.01, "The box does not slide.");
        end

        function coneTurnsAboutItsCentreOfMass(tc)
            % Protocol 1.1: the cone's centre of mass lies a quarter of the
            % height below its origin; half a turn about a transverse axis
            % takes the origin from above the centre of mass to below it
            tc.requireProtocol(1.1);
            r = 0.3;
            h = 0.8;
            I = [3*h^2/80 + 3*r^2/20, 3*h^2/80 + 3*r^2/20, 3*r^2/10];
            w = tc.newWorld([0 0 0]);
            c = phx.engine.io('add', w, 'cone', 2, r, h, 'z', tEngineProtocol.pose([0 0 0]), 1, I);
            phx.engine.io('set', w, c, 'angvel', [pi 0 0]);
            T = tc.lastPose(w, c, 1);
            tc.verifyEqual(T(1:3, 4), [0; 0; -h/2], "AbsTol", 1e-3);
            tc.verifyEqual(T(1:3, 1:3)*[0; 0; 1], [0; 0; -1], "AbsTol", 1e-3, "The apex points down.");
        end

        function kinematicBodyFollowsItsVelocity(tc)
            % Protocol 1.1: a kinematic body moves with the velocity set to it
            tc.requireProtocol(1.1);
            w = tc.newWorld;
            k = tc.addBox(w, 1, [0 0 0], [1 1 0.2]);
            phx.engine.io('set', w, k, 'linvel', [1 0 0]);
            phx.engine.io('set', w, k, 'angvel', [0 0 0.5]);
            T = tc.lastPose(w, k, 1);
            tc.verifyEqual(T(1:3, 4), [1; 0; 0], "AbsTol", 1e-4);
            tc.verifyEqual(atan2(T(2, 1), T(1, 1)), 0.5, "AbsTol", 1e-4);
            tc.verifyEqual(phx.engine.io('get', w, k, 'linvel'), [1; 0; 0], "AbsTol", 1e-6);
        end

        function latestKinematicCommandHolds(tc)
            % Protocol 1.1: a pose is reached in the next step, after which
            % the body stands still; a velocity holds until the next command;
            % a velocity set after a pose applies once the pose is reached
            tc.requireProtocol(1.1);
            w = tc.newWorld([0 0 0]);
            k = tc.addBox(w, 1, [0 0 0], [1 1 0.2]);
            dt = tc.Dt;

            % A velocity, then a pose before the step: the pose wins
            phx.engine.io('set', w, k, 'linvel', [0 1 0]);
            phx.engine.io('set', w, k, 'transform', tEngineProtocol.pose([1 0 0]));
            T = tc.lastPose(w, k, 0.5);
            tc.verifyEqual(T(1:3, 4), [1; 0; 0], "AbsTol", 1e-4, "The body stands at the pose.");
            tc.verifyEqual(phx.engine.io('get', w, k, 'linvel'), [0; 0; 0], "AbsTol", 1e-9);

            % A pose, then a velocity before the step: the pose, then the velocity
            phx.engine.io('set', w, k, 'transform', tEngineProtocol.pose([2 0 0]));
            phx.engine.io('set', w, k, 'linvel', [0 1 0]);
            T = tc.lastPose(w, k, 0.5 + dt);
            tc.verifyEqual(T(1:3, 4), [2; 0.5; 0], "AbsTol", 1e-4, "The body moves on from the pose.");

            % A velocity after a pose step: only the new velocity, nothing
            % left of the motion towards the pose
            phx.engine.io('set', w, k, 'transform', tEngineProtocol.pose([3 0 0]));
            tc.lastPose(w, k, dt);
            phx.engine.io('set', w, k, 'linvel', [0 0 1]);
            T = tc.lastPose(w, k, 0.5);
            tc.verifyEqual(T(1:3, 4), [3; 0; 0.5], "AbsTol", 1e-4);
        end

        function surfaceVelocityIsGivenInTheBodyFrame(tc)
            % A static belt turned by 90 deg about z drives along world y
            w = tc.newWorld;
            belt = phx.engine.io('add', w, 'box', 0, [5 5 0.5], ...
                makehgtform("translate", [0 0 -0.5], "zrotate", pi/2), 0, [0 0 0]);
            phx.engine.io('set', w, belt, 'friction', 1);
            phx.engine.io('set', w, belt, 'surfacevelocity', [1 0 0]);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phx.engine.io('set', w, h, 'friction', 1);
            tc.settle(w, 1);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0; 1; 0], "AbsTol", 0.05);
        end

        function rollingAndSpinningFrictionAreAccepted(tc)
            % Their meaning is engine-specific (see PROTOCOL.md); the
            % contract is only that PHX can set them.
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phx.engine.io('set', w, h, 'rollfriction', 0.1);
            phx.engine.io('set', w, h, 'spinfriction', 0.1);
            [~] = phx.engine.io('step', w, tc.Dt);
        end

        function totalForceIsTheResultantOfAllForces(tc)
            m = 2;
            % Falling freely: the weight plus what was applied
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 5], 0.2*[1 1 1], m);
            [~] = phx.engine.io('apply', w, h, 'centralforce', [3 0 0], false);
            [~] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(phx.engine.io('get', w, h, 'totalforce'), [3; 0; -m*tc.G], "AbsTol", 1e-4);

            % At rest on a floor: the contact cancels the weight
            w = tc.newWorld;
            tc.addFloor(w);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1], m);
            tc.settle(w, 0.5);
            tc.verifyEqual(phx.engine.io('get', w, h, 'totalforce'), [0; 0; 0], "AbsTol", 0.02*m*tc.G);
        end

        function totalTorqueIsTheResultantTorque(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            [~] = phx.engine.io('apply', w, h, 'torque', [0.1 -0.2 0.3], false);
            [~] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(phx.engine.io('get', w, h, 'totaltorque'), [0.1; -0.2; 0.3], "AbsTol", 1e-6);
        end

        function totalTorqueOfAFreeTumblingBodyIsZero(tc)
            % The gyroscopic turning of a free asymmetric body (here about
            % 0.02 N m worth of w x Iw) is no torque
            s = [0.1 0.3 0.6];
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], s, 1, (sum(s.^2) - s.^2)/12);
            phx.engine.io('set', w, h, 'angvel', [0.2 3 0.2]);
            for k = 1:20
                [~] = phx.engine.io('step', w, tc.Dt);
                tc.verifyEqual(phx.engine.io('get', w, h, 'totaltorque'), [0; 0; 0], "AbsTol", 1e-5);
            end
        end
    end

    % Forces and queries
    methods (Test, TestTags = {'Engine'})
        function kinematicSupportReadsTheWeight(tc)
            % A 2 kg box resting 0.3 m off the centre of a kinematic plate:
            % the plate's resultant is the box's weight, pressing down, and
            % its moment about the plate's centre of mass
            w = tc.newWorld;
            support = tc.addBox(w, 1, [0 0 0], [2 2 0.2]);
            tc.addBox(w, 2, [0.3 0 0.2], [0.2 0.2 0.2], 2);
            tc.settle(w, 1);
            W = 2*tEngineProtocol.G;
            tc.verifyEqual(phx.engine.io('get', w, support, 'totalforce'), [0; 0; -W], "AbsTol", 0.01*W);
            tc.verifyEqual(phx.engine.io('get', w, support, 'totaltorque'), [0; 0.3*W; 0], "AbsTol", 0.01*W);
        end

        function staticFloorReadsTheWeight(tc)
            % The same on a static plate (protocol 1.1: the Bullet MEX
            % divides the force on a static body by the step once more)
            tc.requireProtocol(1.1);
            w = tc.newWorld;
            floor = tc.addBox(w, 0, [0 0 0], [2 2 0.2]);
            tc.addBox(w, 2, [0.3 0 0.2], [0.2 0.2 0.2], 2);
            tc.settle(w, 1);
            W = 2*tEngineProtocol.G;
            tc.verifyEqual(phx.engine.io('get', w, floor, 'totalforce'), [0; 0; -W], "AbsTol", 0.01*W);
            tc.verifyEqual(phx.engine.io('get', w, floor, 'totaltorque'), [0; 0.3*W; 0], "AbsTol", 0.01*W);
        end

        function staticAnchorReadsTheHangingWeight(tc)
            % A 3 kg box hanging still from a static anchor by a point joint
            % 0.5 m above it: the anchor carries its weight
            tc.requireProtocol(1.1);
            w = tc.newWorld;
            anchor = tc.addBox(w, 0, [0 0 1], 0.1*[1 1 1]);
            box = tc.addBox(w, 2, [0 0 0.5], 0.2*[1 1 1], 3);
            phx.engine.io('add', w, 'pointtopointconstraint', anchor, box, [0 0 0], [0 0 0.5], true);
            tc.settle(w, 1);
            W = 3*tEngineProtocol.G;
            tc.verifyEqual(phx.engine.io('get', w, anchor, 'totalforce'), [0; 0; -W], "AbsTol", 0.01*W);
        end

        function appliedForcesActForOneStep(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            [~] = phx.engine.io('apply', w, h, 'centralforce', [1 0 0], false);
            [~] = phx.engine.io('step', w, 0.01);
            [~] = phx.engine.io('step', w, 0.01);
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), [0.01; 0; 0], "AbsTol", 1e-6);
        end

        function singleBodyForcesUseTheirFrames(tc)
            % Body turned +90 deg about z (local x = world y), unit mass and
            % inertia, far from the origin
            w = tc.newWorld([0 0 0]);
            P = makehgtform("translate", [10 0 0], "zrotate", pi/2);
            h = phx.engine.io('add', w, 'box', 2, 0.1*[1 1 1], P, 1, [1 1 1]);
            io = @(varargin) phx.engine.io('apply', w, h, varargin{:});

            [~] = io('centralforce', [1 0 0], true);          % local x = world y
            [~] = io('force', [0 0 1], [1 0 0], false, true); % at local x: lever along world y
            [~] = io('force', [0 0 1], [10 -1 0], false, false); % at world point, lever along -y
            [~] = io('torque', [1 0 0], true);                % about local x = world y
            [~] = phx.engine.io('step', w, 0.01);
            % Linear: (0,1,0) + (0,0,1) + (0,0,1); angular: (1,0,0) + (-1,0,0) + (0,1,0)
            tc.verifyEqual(phx.engine.io('get', w, h, 'linvel'), 0.01*[0; 1; 2], "AbsTol", 1e-6);
            tc.verifyEqual(phx.engine.io('get', w, h, 'angvel'), 0.01*[0; 1; 0], "AbsTol", 1e-6);
        end

        function batchVerbsApplyPerBody(tc)
            w = tc.newWorld([0 0 0]);
            s = tc.addBox(w, 0, [0 5 0], 0.1*[1 1 1]);
            h1 = phx.engine.io('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([0 0 0]), 1, [1 1 1]);
            h2 = phx.engine.io('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([2 0 0]), 1, [1 1 1]);
            hs = [h1 s h2];  % a static body in a batch is harmless
            [~] = phx.engine.io('apply', w, hs, 'centralforces', [1 0 0, 5 5 5, 0 2 0], false);
            [~] = phx.engine.io('apply', w, hs, 'forces', [0 0 1, 5 5 5, 0 0 1], [1 0 0, 0 0 0, 0 1 0], false, true);
            [~] = phx.engine.io('apply', w, hs, 'torques', [0 0 1, 5 5 5, 0 0 -1], false);
            [~] = phx.engine.io('step', w, 0.01);
            tc.verifyEqual(phx.engine.io('get', w, h1, 'linvel'), 0.01*[1; 0; 1], "AbsTol", 1e-6);
            tc.verifyEqual(phx.engine.io('get', w, h2, 'linvel'), 0.01*[0; 2; 1], "AbsTol", 1e-6);
            % h1: lever (1,0,0) x (0,0,1) = (0,-1,0); h2: (0,1,0) x (0,0,1) = (1,0,0)
            tc.verifyEqual(phx.engine.io('get', w, h1, 'angvel'), 0.01*[0; -1; 1], "AbsTol", 1e-6);
            tc.verifyEqual(phx.engine.io('get', w, h2, 'angvel'), 0.01*[1; 0; -1], "AbsTol", 1e-6);
        end

        function raycastReportsTheClosestHit(tc)
            w = tc.newWorld([0 0 0]);
            near = tc.addBox(w, 0, [0 0 0], [1 1 1]);
            tc.addBox(w, 0, [0 0 -3], [1 1 1]);
            O = [0 0 5; 3 0 5]';
            E = [0 0 -5; 3 0 -5]';
            [p, n, d, ids] = phx.engine.io('apply', w, uint64(0), 'raycast', O, E);
            tc.verifySize(p, [3 2]);
            tc.verifySize(n, [3 2]);
            tc.verifySize(d, [1 2]);
            tc.verifyClass(ids, "uint64");
            tc.verifySize(ids, [1 2]);
            tc.verifyEqual(ids(1), near);
            tc.verifyEqual(p(:, 1), [0; 0; 0.5], "AbsTol", 1e-5);
            tc.verifyEqual(n(:, 1), [0; 0; 1], "AbsTol", 1e-5);
            tc.verifyEqual(d(1), 4.5, "AbsTol", 1e-5);
            tc.verifyEqual(ids(2), uint64(0), "A miss reports handle 0.");
            tc.verifyEqual([p(:, 2); n(:, 2); d(2)], zeros(7, 1), "A miss is zero-filled.");
        end
    end

    % Joints
    methods (Test, TestTags = {'Engine'})
        function hingeHoldsItsAnchorAndReportsReactions(tc)
            m = 2;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1], m);
            j = phx.engine.io('add', w, 'hingeconstraint', a, b, [0 0 0], [0 0 1], [1 0 0], [1 0 0], true, true);
            tc.verifyClass(j, "uint64");
            T = tc.settle(w, 1);
            tc.verifyEqual(T(1:3, 4), [0; 0; 1], "AbsTol", 1e-3);
            [fA, tA, fB, tB] = phx.engine.io('get', w, j, 'feedback');
            tc.verifySize(fA, [3 1]);
            tc.verifySize(tA, [3 1]);
            tc.verifySize(tB, [3 1]);
            tc.verifyEqual(fB, [0; 0; m*tc.G], "AbsTol", 0.01*m*tc.G, "The joint holds body B up.");
            tc.verifyEqual(fA, -fB, "AbsTol", 0.01*m*tc.G, "Body A feels the opposite reaction.");
        end

        function hingeMotorDrivesTheAxis(tc)
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            j = phx.engine.io('add', w, 'hingeconstraint', a, b, [0 0 1], [0 0 0], [1 0 0], [1 0 0], true, true);
            phx.engine.io('set', w, j, 'motor', true, 2, 100);
            tc.settle(w, 0.5);
            tc.verifyEqual(phx.engine.io('get', w, b, 'angvel'), [2; 0; 0], "AbsTol", 1e-3);
        end

        function lowDropBouncesToRestitutionSquared(tc)
            % A ball of restitution 0.8 dropped 2 cm onto a floor of
            % restitution 1 hits it at 0.63 m/s and bounces to e^2 h.
            % (Jolt bounces only impacts faster than 1 m/s by default.)
            % Protocol 1.1: the Bullet MEX bounces this ball to half the
            % height (6.1 mm).
            tc.requireProtocol(1.1);
            w = tc.newWorld;
            phx.engine.io('set', w, tc.addFloor(w), 'restitution', 1);
            r = 0.05;
            h = phx.engine.io('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 r + 0.02]), 1, 0.4*r^2*[1 1 1]);
            phx.engine.io('set', w, h, 'restitution', 0.8);
            top = -inf;
            for k = 1:round(0.2/tc.Dt)   % the first bounce peaks at about 0.13 s
                [ids, P] = phx.engine.io('step', w, tc.Dt);
                if k*tc.Dt > 0.08 && ~islogical(ids) && any(ids == h)
                    top = max(top, P(3, 4, ids == h) - r);
                end
            end
            tc.verifyEqual(top, 0.64*0.02, "RelTol", 0.2, "Height of the first bounce (m)");
        end

        function bouncyStackRests(tc)
            % Five boxes of restitution 0.9 stacked on a floor of restitution
            % 0.9 rest still: a body lying on its support approaches it by
            % g dt every step, which is no impact to bounce from.
            w = tc.newWorld;
            phx.engine.io('set', w, tc.addFloor(w), 'restitution', 0.9);
            boxes = zeros(1, 5, "uint64");
            for i = 1:5
                boxes(i) = tc.addBox(w, 2, [0 0 0.2*(i - 0.5)], 0.2*[1 1 1]);
                phx.engine.io('set', w, boxes(i), 'restitution', 0.9);
            end
            tc.settle(w, 1);
            speed = 0;
            for k = 1:round(1/tc.Dt)
                [~] = phx.engine.io('step', w, tc.Dt);
                for i = 1:5
                    speed = max(speed, norm(phx.engine.io('get', w, boxes(i), 'linvel')));
                end
            end
            tc.verifyLessThan(speed, 0.01, "Largest speed of the resting boxes (m/s)");
        end

        function tiltedRotorKeepsItsSpeed(tc)
            % A box rotor on a hinge, without gravity, its principal axes
            % tilted 10 deg to the hinge (dynamic imbalance), spinning at
            % 20 rad/s (w dt 0.1 at 5 ms): nothing acts about the axis, so
            % the speed stays within 1 % for 2 s. (With the gyroscopic step
            % applied before the joints' solve, which then projected the
            % angular velocity back onto the axis, it lost 2.6 %.)
            % Protocol 1.1: the Bullet MEX slows it to 19.2 rad/s.
            tc.requireProtocol(1.1);
            w = tc.newWorld([0 0 0]);
            s = [0.1 0.06 0.04];
            R0 = [cosd(10) 0 sind(10); 0 1 0; -sind(10) 0 cosd(10)];
            anchor = tc.addBox(w, 0, [0 0 0], 0.01*[1 1 1]);
            h = phx.engine.io('add', w, 'box', 2, s/2, [R0 [0; 0; 0]; 0 0 0 1], 1, (sum(s.^2) - s.^2)/12);
            phx.engine.io('add', w, 'hingeconstraint', anchor, h, [0 0 0], [0 0 0], [0 0 1], (R0'*[0; 0; 1])', true, true);
            phx.engine.io('set', w, h, 'angvel', [0 0 20]);
            tc.settle(w, 2);
            tc.verifyEqual(phx.engine.io('get', w, h, 'angvel'), [0; 0; 20], "AbsTol", 0.2, "Angular velocity after 2 s (rad/s)");
        end

        function topKeepsItsPrecession(tc)
            % A spinning disc held by a point joint below its centre of
            % mass, its axis tilted 30 deg and started in steady precession:
            % the tilt stays within 0.5 deg for 5 s at 5 ms. (With the
            % gyroscopic step applied before the joint's solve the top sank
            % 17 deg.)
            % Protocol 1.1: the Bullet MEX tilts it by 28 deg.
            tc.requireProtocol(1.1);
            g = tEngineProtocol.G;
            m = 1; rad = 0.1; hh = 0.01; l = 0.02; w3 = 40; th = 30;
            Ic = m*[(3*rad^2 + (2*hh)^2)/12*[1 1] rad^2/2];
            I1 = Ic(1) + m*l^2;
            I3 = Ic(3);
            W = (I3*w3 - sqrt((I3*w3)^2 - 4*I1*cosd(th)*m*g*l))/(2*I1*cosd(th));
            w = tc.newWorld;
            anchor = tc.addBox(w, 0, [0 0 0], 0.01*[1 1 1]);
            R0 = [1 0 0; 0 cosd(th) -sind(th); 0 sind(th) cosd(th)];
            e3 = R0(:, 3);
            h = phx.engine.io('add', w, 'cylinder', 2, [rad rad hh], 'z', [R0 l*e3; 0 0 0 1], m, Ic);
            phx.engine.io('add', w, 'pointtopointconstraint', anchor, h, [0 0 0], [0 0 -l], true);
            om0 = W*[0; 0; 1] + (w3 - W*cosd(th))*e3;
            phx.engine.io('set', w, h, 'angvel', om0);
            phx.engine.io('set', w, h, 'linvel', cross(om0, l*e3));
            tilt = 0;
            for k = 1:round(5/tc.Dt)
                [ids, P] = phx.engine.io('step', w, tc.Dt);
                if ~islogical(ids) && any(ids == h)
                    tilt = max(tilt, abs(acosd(P(3, 3, ids == h)) - th));
                end
            end
            tc.verifyLessThan(tilt, 0.5, "Largest change of the tilt (deg)");
        end

        function hingeCarriesItsMomentSmoothly(tc)
            % A 10 kg box held out 0.5 m by a hinge with a vertical axis,
            % turning about it at 2 rad/s, at 10 velocity iterations: the
            % anchor carries the box's weight moment, which changes only by
            % its slow turn from one step to the next. (Jolt's hinge
            % expressed its warm start in axes that jump by 90 deg when the
            % axis lies near a coordinate axis, and the reaction lost about
            % half its value for single steps.)
            w = tc.newWorld;
            try
                % Jolt only: the iterations of jointed islands
                phx.engine.io('set', w, uint64(0), 'jointiterations', 10);
            catch
            end
            anchor = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            box = tc.addBox(w, 2, [0.5 0 0], [0.2 0.2 0.2], 10);
            j = phx.engine.io('add', w, 'hingeconstraint', anchor, box, [0 0 0], [-0.5 0 0], [0 0 1], [0 0 1], true, true);
            phx.engine.io('set', w, box, 'angvel', [0 0 2]);
            phx.engine.io('set', w, box, 'linvel', [0 1 0]);
            n = 400;
            T = zeros(3, n);
            for k = 1:n
                [~] = phx.engine.io('step', w, tc.Dt);
                [~, T(:, k)] = phx.engine.io('get', w, j, 'feedback');
            end
            M = 10*tEngineProtocol.G*0.5;   % weight moment about the hinge
            jump = max(vecnorm(diff(T(:, 101:end), 1, 2)));
            tc.verifyLessThan(jump, 0.05*M, "Largest change of the hinge reaction torque in one step (N m)");
        end

        function servoedArmDoesNotRing(tc)
            % A three-link arm held out against gravity, every joint servoed
            % by its motor (target velocity 200*(reference - angle), set
            % every 1 ms step) along a slow sine, at the default iterations:
            % the tracking error follows the reference smoothly. (Jolt at 10
            % velocity iterations, warm starting each motor with the impulse
            % of its last target, rang by 5.6 mrad; Bullet 0.01 mrad.)
            dt = 0.001;
            Kp = 200;
            w = tc.newWorld;
            L = 0.3;
            s = [L 0.05 0.05];
            n = 3;
            previous = tc.addBox(w, 0, [0 0 0], [0.1 0.1 0.1]);
            links = zeros(1, n, "uint64");
            joints = zeros(1, n, "uint64");
            for i = 1:n
                links(i) = tc.addBox(w, 2, [L*(i - 0.5) 0 0], s);
                pivotA = [L/2 0 0]*(i > 1);
                joints(i) = phx.engine.io('add', w, 'hingeconstraint', previous, links(i), pivotA, [-L/2 0 0], ...
                    [0 1 0], [0 1 0], true, true);
                previous = links(i);
            end
            R = repmat(eye(3), 1, 1, n + 1);
            steps = round(3/dt);
            e = zeros(steps, n);
            for k = 1:steps
                reference = 0.3*sin(pi*(k - 1)*dt);
                for i = 1:n
                    relative = R(:, :, i)'*R(:, :, i + 1);
                    e(k, i) = reference - atan2(relative(1, 3), relative(1, 1));
                    phx.engine.io('set', w, joints(i), 'motor', true, Kp*e(k, i), 1000);
                end
                [ids, P] = phx.engine.io('step', w, dt);
                for i = 1:n
                    R(:, :, i + 1) = P(1:3, 1:3, ids == links(i));
                end
            end
            last = e(1001:end, :);
            ringing = rms(last - movmean(last, 101), "all");
            tc.verifyLessThan(ringing, 1e-4, "Ringing of the joints (rad) about their smooth tracking error");
            tc.verifyLessThan(max(abs(last), [], "all"), 0.01, "Tracking error (rad)");
        end

        function sliderMovesAlongTheXAxisOfItsFrames(tc)
            % Frames turned so that their X is world -z: the body slides
            % down under gravity and nowhere else; the motor drives along X
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0.5 0 2], 0.2*[1 1 1]);
            R = [0 0 1; 0 1 0; -1 0 0];
            j = phx.engine.io('add', w, 'sliderconstraint', a, b, [R [0.5; 0; 0]; 0 0 0 1], [R [0; 0; 0]; 0 0 0 1], true, true);
            T = tc.settle(w, 0.3);
            tc.verifyEqual(T(1:2, 4), [0.5; 0], "AbsTol", 1e-3);
            tc.verifyLessThan(T(3, 4), 1.8);
            phx.engine.io('set', w, j, 'linearmotor', true, 0.5, 1000);
            tc.settle(w, 0.3);
            tc.verifyEqual(phx.engine.io('get', w, b, 'linvel'), [0; 0; -0.5], "AbsTol", 1e-3);
        end

        function fixedJointHoldsThePose(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            phx.engine.io('add', w, 'fixedconstraint', a, b, tEngineProtocol.pose([0.5 0 0]), tEngineProtocol.pose([-0.5 0 0]), true);
            T = tc.settle(w, 1);
            % A constraint solver lets a 1 m cantilever sag slightly
            tc.verifyEqual(T(1:3, 4), [1; 0; 2], "AbsTol", 0.02);
        end

        function hingePendulumKeepsItsEnergy(tc)
            % Nothing dissipates in a frictionless pendulum: over 10 s its
            % mean energy stays (Jolt 5.6.0 lost 11 % at this step, as the
            % joint projected the velocity on the start-of-step tangent)
            m = 2.5;
            g = tc.G;
            I = m*0.2^2/6*[1 1 1];
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 sind(60) 2 - cosd(60)], 0.2*[1 1 1], m, I);
            phx.engine.io('add', w, 'hingeconstraint', a, b, [0 0 0], [0 -sind(60) cosd(60)], [1 0 0], [1 0 0], true, true);
            n = round(10/tc.Dt);
            E = zeros(1, n);
            for k = 1:n
                [ids, P] = phx.engine.io('step', w, tc.Dt);
                T = P(:, :, ids == b);
                v = phx.engine.io('get', w, b, 'linvel');
                om = phx.engine.io('get', w, b, 'angvel');
                E(k) = 0.5*m*(v'*v) + 0.5*om'*diag(I)*om + m*g*T(3, 4);
            end
            window = round(3/tc.Dt);
            tc.verifyEqual(mean(E(end-window+1:end)), mean(E(1:window)), "RelTol", 0.01);
        end

        function pointJointHoldsItsAnchor(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            phx.engine.io('add', w, 'pointtopointconstraint', a, b, [0 0 0], [-1 0 0], true);
            lowest = 2;
            for k = 1:200
                [~, T] = phx.engine.io('step', w, tc.Dt);
                % The anchor drifts by about a centimetre at the bottom of
                % the swing, where body B passes at over 4 m/s
                anchor = T(1:3, 1:3)*[-1; 0; 0] + T(1:3, 4);
                tc.assertEqual(anchor, [0; 0; 2], "AbsTol", 0.02);
                lowest = min(lowest, T(3, 4));
            end
            tc.verifyEqual(lowest, 1, "AbsTol", 0.02, "Body B swings down to hang below the anchor.");
        end

        function genericJointLimitsFreeOrLockAxes(tc)
            % lower > upper frees the z translation, lower == upper locks
            % the rest
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0.3 0 1], 0.2*[1 1 1]);
            j = phx.engine.io('add', w, 'generic6dofconstraint', a, b, tEngineProtocol.pose([0.3 0 -1]), eye(4), 'xyz', true);
            phx.engine.io('set', w, j, 'linlimits', [0 0 1], [0 0 -1]);
            phx.engine.io('set', w, j, 'anglimits', [0 0 0], [0 0 0]);
            phx.engine.io('set', w, b, 'linvel', [1 1 0]);
            phx.engine.io('set', w, b, 'angvel', [1 1 1]);
            T = tc.settle(w, 0.5);
            % The locked axes absorb the initial kick with a small error
            tc.verifyEqual(T(1:2, 4), [0.3; 0], "AbsTol", 5e-3);
            tc.verifyLessThan(T(3, 4), 0, "Body B falls along the free axis.");
            tc.verifyEqual(T(1:3, 1:3), eye(3), "AbsTol", 5e-3);
        end

        function springJointDeflectsByWeightOverStiffness(tc)
            m = 1;
            k = 1000;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1], m);
            j = phx.engine.io('add', w, 'generic6dofspringconstraint', a, b, tEngineProtocol.pose([0 0 -1]), eye(4), 'xyz', true);
            phx.engine.io('set', w, j, 'springinfo', [0 0 1], [0 0 0], [0 0 20], [0 0 0], [0 0 k], [0 0 0]);
            phx.engine.io('set', w, j, 'linlimits', [0 0 1], [0 0 -1]);
            phx.engine.io('set', w, j, 'anglimits', [0 0 0], [0 0 0]);
            T = tc.settle(w, 3);
            tc.verifyEqual(T(3, 4), 1 - m*tc.G/k, "AbsTol", 0.05*m*tc.G/k);
        end

        function gearCouplesTheAngularVelocities(tc)
            % omegaA.axisA + ratio*omegaB.axisB = 0
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1]);
            phx.engine.io('add', w, 'gearconstraint', a, b, [0 0 1], [0 0 1], 2, true);
            phx.engine.io('set', w, a, 'angvel', [0 0 1]);
            tc.settle(w, 0.25);
            wa = phx.engine.io('get', w, a, 'angvel');
            wb = phx.engine.io('get', w, b, 'angvel');
            tc.verifyGreaterThan(abs(wa(3)), 0.1);
            tc.verifyEqual(wa(3) + 2*wb(3), 0, "AbsTol", 1e-3);
        end

        function gearTransmitsTorqueByTheRatio(tc)
            % A torque on body A turns the pair as one inertia,
            % IA + IB/ratio^2, the right way round for a negative ratio too
            torque = 1;
            IA = 0.1;
            IB = 0.2;
            n = 50;
            for ratio = [-3 0.5 2]
                w = tc.newWorld([0 0 0]);
                a = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [1 1 IA]);
                b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1], 1, [1 1 IB]);
                phx.engine.io('add', w, 'gearconstraint', a, b, [0 0 1], [0 0 1], ratio, true);
                for k = 1:n
                    [~] = phx.engine.io('apply', w, a, 'torque', [0 0 torque], false);
                    [~] = phx.engine.io('step', w, tc.Dt);
                end
                wa = torque/(IA + IB/ratio^2)*n*tc.Dt;
                tc.verifyEqual(phx.engine.io('get', w, a, 'angvel'), [0; 0; wa], "AbsTol", 0.01*wa, ...
                    "Body A, ratio " + ratio);
                tc.verifyEqual(phx.engine.io('get', w, b, 'angvel'), [0; 0; -wa/ratio], "AbsTol", 0.01*wa, ...
                    "Body B, ratio " + ratio);
            end
        end

        function mutualCollisionsFollowTheLastArgument(tc)
            % Two overlapping boxes on a point joint: with collisions off
            % (true) they stay put, with them on (false) the contact pushes
            % them apart
            d = zeros(1, 2);
            flags = [true false];
            for i = 1:2
                w = tc.newWorld([0 0 0]);
                a = tc.addBox(w, 2, [0 0 0], 0.4*[1 1 1]);
                b = tc.addBox(w, 2, [0.1 0 0], 0.4*[1 1 1]);
                phx.engine.io('add', w, 'pointtopointconstraint', a, b, [0.05 0 0], [-0.05 0 0], flags(i));
                tc.settle(w, 0.25);
                pa = tEngineProtocol.positionOf(w, a);
                pb = tEngineProtocol.positionOf(w, b);
                d(i) = norm(pa - pb);
            end
            tc.verifyEqual(d(1), 0.1, "AbsTol", 1e-3);
            tc.verifyGreaterThan(d(2), 0.2);
        end
    end

    % Soft bodies (protocol 1.2; the Bullet gateway has none)
    methods (Test, TestTags = {'Engine'})
        function softBodyRestsOnTheFloorKeepingItsShape(tc)
            % A closed cube mesh dropped as a soft body comes to rest on the
            % floor, its lowest vertex at z = 0 and its sides upright. Its
            % vertices are read back in world coordinates in the order given;
            % the step output reports rigid bodies only, and a soft body is
            % removed as a body is.
            tc.requireProtocol(1.2);
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            w = tc.newWorld;
            tc.addFloor(w);
            h = phx.engine.io('add', w, 'softbody', V, F, tEngineProtocol.pose([0 0 1]), 1, 0);
            tc.verifyClass(h, "uint64");
            tc.verifyEqual(phx.engine.io('get', w, h, 'vertices'), V + [0; 0; 1], "AbsTol", 1e-6);
            for k = 1:round(2/tc.Dt)
                ids = phx.engine.io('step', w, tc.Dt);
            end
            tc.verifyEqual(ids, false, "The soft body is not among the moved bodies.");
            X = phx.engine.io('get', w, h, 'vertices');
            tc.verifySize(X, [3 8]);
            tc.verifyEqual(min(X(3, :)), 0, "AbsTol", 2e-3, "The cube rests on the floor.");
            tc.verifyEqual(X(:, 5:8) - X(:, 1:4), repmat([0; 0; 0.4], 1, 4), "AbsTol", 2e-3, ...
                "The cube keeps its shape.");
            phx.engine.io('remove', w, h);
            tc.verifyError(@() phx.engine.io('get', w, h, 'vertices'), ?MException);
        end
    end

    % Particle groups (protocol 1.3; the Bullet gateway has none)
    methods (Test, TestTags = {'Engine'})
        function particlesRestOnTheFloorUnreportedByStep(tc)
            % Particles dropped on the floor are read back as 3xN in the
            % order given and come to rest touching it; the step reports
            % only the rigid body beside them.
            tc.requireProtocol(1.3);
            r = 0.05;
            P = [(-2:2)*0.3; zeros(1, 5); 0.5*ones(1, 5)];
            w = tc.newWorld;
            tc.addFloor(w);
            b = tc.addBox(w, 2, [0 1 0.5], 0.2*[1 1 1]);
            g = phx.engine.io('add', w, 'particles', P, r, 0.1, false);
            tc.verifyClass(g, "uint64");
            tc.verifyEqual(phx.engine.io('get', w, g, 'count'), 5);
            tc.verifyEqual(phx.engine.io('get', w, g, 'positions'), P, "AbsTol", 1e-6);
            for k = 1:round(1/tc.Dt)
                ids = phx.engine.io('step', w, tc.Dt);
                tc.assertEqual(ids, b, "Only the rigid body is reported.");
            end
            X = phx.engine.io('get', w, g, 'positions');
            tc.verifySize(X, [3 5]);
            tc.verifyEqual(X(1:2, :), P(1:2, :), "AbsTol", 1e-3);
            tc.verifyEqual(X(3, :), r*ones(1, 5), "AbsTol", 2e-3, "The particles rest on the floor.");
            tc.verifyEqual(phx.engine.io('get', w, g, 'velocities'), zeros(3, 5), "AbsTol", 1e-2);
            phx.engine.io('remove', w, g);
            tc.verifyError(@() phx.engine.io('get', w, g, 'positions'), "phx:engine:badHandle");
        end

        function particlesKeepTheirOrder(tc)
            % Removing particles keeps the order of the rest, appended ones
            % come last, and a subset is set by zero-based indices.
            tc.requireProtocol(1.3);
            w = tc.newWorld([0 0 0]);
            P = [1; 0; 0] * (1:5);
            g = phx.engine.io('add', w, 'particles', P, 0.1, 1, false);
            phx.engine.io('remove', w, g, int32([1 3]));
            tc.verifyEqual(phx.engine.io('get', w, g, 'positions'), P(:, [1 3 5]), "AbsTol", 1e-6);
            n = phx.engine.io('add', w, g, [0 0; 2 3; 0 0], [0 0; 0 0; 1 2]);
            tc.verifyEqual(n, 5);
            tc.verifyEqual(phx.engine.io('get', w, g, 'velocities'), [zeros(3, 3), [0; 0; 1], [0; 0; 2]], "AbsTol", 1e-6);
            phx.engine.io('set', w, g, 'positions', [0; 0; 7], int32(4));
            X = phx.engine.io('get', w, g, 'positions');
            tc.verifyEqual(X(:, [1 5]), [1 0; 0 0; 0 7], "AbsTol", 1e-6);
            tc.verifyError(@() phx.engine.io('remove', w, g, int32(5)), "phx:engine:badArgument");
        end

        function particleForcesActForOneStep(tc)
            % A force for all (3x1) or one per particle (3xN) changes the
            % velocity by F/m dt in the next step only.
            tc.requireProtocol(1.3);
            w = tc.newWorld([0 0 0]);
            m = 0.5;
            g = phx.engine.io('add', w, 'particles', [0 1; 0 0; 0 0], 0.1, m, false);
            phx.engine.io('apply', w, g, 'centralforces', [2; 0; 0], false);
            phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(phx.engine.io('get', w, g, 'velocities'), [1 1; 0 0; 0 0]*2/m*tc.Dt, "AbsTol", 1e-6);
            phx.engine.io('apply', w, g, 'centralforces', [0 0; 1 -1; 0 0], false);
            phx.engine.io('step', w, tc.Dt);
            phx.engine.io('step', w, tc.Dt);
            V = phx.engine.io('get', w, g, 'velocities');
            tc.verifyEqual(V(2, :), [1 -1]/m*tc.Dt, "AbsTol", 1e-6, "The force acts in one step only.");
        end

        function particleSelfCollisionsCanBeSwitchedOff(tc)
            % Two particles dropped one above the other stack; without
            % self-collisions the upper one passes through to the floor.
            tc.requireProtocol(1.3);
            r = 0.05;
            heights = zeros(2, 2);
            for on = [true false]
                w = tc.newWorld;
                tc.addFloor(w);
                g = phx.engine.io('add', w, 'particles', [0 0; 0 0; r 4*r], r, 0.1, false);
                phx.engine.io('set', w, g, 'selfcollisions', on);
                for k = 1:round(1/tc.Dt)
                    phx.engine.io('step', w, tc.Dt);
                end
                X = phx.engine.io('get', w, g, 'positions');
                heights(:, 2 - on) = X(3, :)';
            end
            tc.verifyEqual(heights(:, 1), [r; 3*r], "AbsTol", 3e-3, "With self-collisions they stack.");
            tc.verifyEqual(heights(:, 2), [r; r], "AbsTol", 3e-3, "Without, both rest on the floor.");
        end

        function particlesRollOnlyWithRotation(tc)
            % A particle sliding over the floor starts rolling (w = v/r about
            % y) only when its group rotates.
            tc.requireProtocol(1.3);
            r = 0.05;
            w = tc.newWorld;
            tc.addFloor(w);
            sliding = phx.engine.io('add', w, 'particles', [0; 0; r], r, 0.1, false);
            rolling = phx.engine.io('add', w, 'particles', [0; 1; r], r, 0.1, true);
            phx.engine.io('set', w, sliding, 'velocities', [1; 0; 0]);
            phx.engine.io('set', w, rolling, 'velocities', [1; 0; 0]);
            for k = 1:round(0.5/tc.Dt)
                phx.engine.io('step', w, tc.Dt);
            end
            tc.verifyEqual(phx.engine.io('get', w, sliding, 'angvel'), [0; 0; 0]);
            v = phx.engine.io('get', w, rolling, 'velocities');
            tc.verifyEqual(phx.engine.io('get', w, rolling, 'angvel'), [0; v(1)/r; 0], "AbsTol", 0.05*v(1)/r);
        end

        function particlesSleepByTheirOwnSetting(tc)
            % In a world that keeps its bodies awake, a group allowed to sleep
            % falls asleep once at rest; forbidding it wakes the particles.
            tc.requireProtocol(1.3);
            w = tc.newWorld;
            tc.addFloor(w);
            g = phx.engine.io('add', w, 'particles', [0 0.3; 0 0; 0.05 0.05], 0.05, 0.1, false);
            phx.engine.io('set', w, g, 'sleeping', true);
            for k = 1:round(2/tc.Dt)
                phx.engine.io('step', w, tc.Dt);
            end
            tc.verifyEqual(phx.engine.io('get', w, g, 'active'), [false false]);
            phx.engine.io('set', w, g, 'sleeping', false);
            tc.verifyEqual(phx.engine.io('get', w, g, 'active'), [true true]);
        end
    end

    % Spring constraint (protocol 1.5; the Bullet gateway has none)
    methods (Test, TestTags = {'Engine'})
        function springHoldsAMassAtItsStaticDeflection(tc)
            % 2 kg hanging from a static anchor on a spring of free length
            % 0.5 m, k = 400 N/m, damped to rest: stretched by m*g/k
            tc.requireProtocol(1.5);
            m = 2;
            k = 400;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1.5], 0.2*[1 1 1], m);
            s = phx.engine.io('add', w, 'springconstraint', a, b, [0 0 0], [0 0 0], 0.5, k, 40, true);
            tc.verifyClass(s, "uint64");
            tc.settle(w, 3);
            [f, L] = phx.engine.io('get', w, s, 'spring');
            W = m*tc.G;
            tc.verifyEqual(L, 0.5 + W/k, "AbsTol", 1e-4);
            tc.verifyEqual(f, W, "RelTol", 1e-3, "A stretched spring reads a positive tension.");
            [fA, ~, fB] = phx.engine.io('get', w, s, 'feedback');
            tc.verifyEqual(fB, [0; 0; W], "AbsTol", 1e-3*W);
            tc.verifyEqual(fA, -fB, "AbsTol", 1e-3*W);

            % New free length and spring at run time
            phx.engine.io('set', w, s, 'lengths', 0.7);
            tc.settle(w, 3);
            [~, L] = phx.engine.io('get', w, s, 'spring');
            tc.verifyEqual(L, 0.7 + W/k, "AbsTol", 1e-4);
            phx.engine.io('set', w, s, 'spring', 2*k, 40);
            tc.settle(w, 3);
            [~, L] = phx.engine.io('get', w, s, 'spring');
            tc.verifyEqual(L, 0.7 + W/(2*k), "AbsTol", 1e-4);
        end

        function springOscillatesAtItsNaturalFrequency(tc)
            % 1 kg on k = 100 N/m without damping, released 0.1 m out:
            % period 2*pi/10 s, and the motion neither grows nor decays
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1.1 0 0], 0.2*[1 1 1]);
            s = phx.engine.io('add', w, 'springconstraint', a, b, [0 0 0], [0 0 0], 1, 100, 0, true);
            n = round(2/tc.Dt);
            x = zeros(1, n);
            for i = 1:n
                [~] = phx.engine.io('step', w, tc.Dt);
                [~, L] = phx.engine.io('get', w, s, 'spring');
                x(i) = L - 1;
            end
            t = (1:n)*tc.Dt;
            down = find(x(1:end-1) > 0 & x(2:end) <= 0);
            crossings = t(down) - x(down)*tc.Dt./(x(down + 1) - x(down));
            tc.verifyEqual(mean(diff(crossings)), 2*pi/10, "RelTol", 0.02);
            lastPeriod = x(t > t(end) - 2*pi/10);
            tc.verifyEqual(max(abs(lastPeriod)), 0.1, "RelTol", 0.01);
        end

        function springIsSlackInsideItsRange(tc)
            % Lengths [0.5 1.5]: no force inside, a pull beyond the upper
            % length, a push below the lower one
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            lengths = [1 2 0.3];
            for i = 3:-1:1
                b = tc.addBox(w, 2, [0 3*i lengths(i)], 0.1*[1 1 1]);
                s(i) = phx.engine.io('add', w, 'springconstraint', a, b, [0 3*i 0], [0 0 0], [0.5 1.5], 100, 0, true);
            end
            [~] = phx.engine.io('step', w, tc.Dt);
            f = arrayfun(@(h) phx.engine.io('get', w, h, 'spring'), s);
            tc.verifyEqual(f(1), 0, "Inside the range the spring is slack.");
            tc.verifyEqual(f(2), 100*0.5, "RelTol", 0.02, "Beyond the range it pulls.");
            tc.verifyEqual(f(3), -100*0.2, "RelTol", 0.02, "Below the range it pushes.");
        end

        function inerterAddsItsInertanceToTheMass(tc)
            % 1 kg on k = 100 N/m with an inerter of 3 kg to a static anchor,
            % released 0.1 m out: period 2*pi*sqrt((m + b)/k), amplitude kept
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1.1 0 0], 0.2*[1 1 1]);
            s = phx.engine.io('add', w, 'springconstraint', a, b, [0 0 0], [0 0 0], 1, 100, 0, true);
            phx.engine.io('set', w, s, 'inertance', 3);
            n = round(4/tc.Dt);
            x = zeros(1, n);
            for i = 1:n
                [~] = phx.engine.io('step', w, tc.Dt);
                [~, L] = phx.engine.io('get', w, s, 'spring');
                x(i) = L - 1;
            end
            t = (1:n)*tc.Dt;
            down = find(x(1:end-1) > 0 & x(2:end) <= 0);
            crossings = t(down) - x(down)*tc.Dt./(x(down + 1) - x(down));
            T = 2*pi*sqrt(4/100);
            tc.verifyEqual(mean(diff(crossings)), T, "RelTol", 0.01);
            tc.verifyEqual(max(abs(x(t > t(end) - T))), 0.1, "RelTol", 0.01);
        end

        function inerterSharesAForceByTheInertances(tc)
            % 10 N on a 1 kg body joined to a 3 kg one by an inerter of
            % 2 kg alone: the inerter passes f = b*F/(mA + b + b*mA/mB)
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            hA = phx.engine.io('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([0 0 0]), 1, [1 1 1]);
            hB = phx.engine.io('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([1 0 0]), 3, [1 1 1]);
            s = phx.engine.io('add', w, 'springconstraint', hA, hB, [0 0 0], [0 0 0], 1, 0, 0, true);
            phx.engine.io('set', w, s, 'inertance', 2);
            [~] = phx.engine.io('apply', w, hA, 'centralforce', [-10 0 0], false);
            [~] = phx.engine.io('step', w, tc.Dt);
            f = 2*10/(1 + 2 + 2/3);
            tc.verifyEqual(phx.engine.io('get', w, hA, 'linvel'), [-(10 - f)*tc.Dt; 0; 0], "RelTol", 1e-4);
            tc.verifyEqual(phx.engine.io('get', w, hB, 'linvel'), [-f/3*tc.Dt; 0; 0], "RelTol", 1e-4);
        end

        function inerterFarAboveTheMassIsStable(tc)
            % 1 kg on k = 1e4 N/m with 1e4 kg of inertance, released 0.1 m
            % out: period 2*pi*sqrt(1.0001) s, the motion neither grows nor
            % decays
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1.1 0 0], 0.2*[1 1 1]);
            s = phx.engine.io('add', w, 'springconstraint', a, b, [0 0 0], [0 0 0], 1, 1e4, 0, true);
            phx.engine.io('set', w, s, 'inertance', 1e4);
            T = 2*pi*sqrt(1.0001);
            tc.settle(w, T/4);
            [~, L] = phx.engine.io('get', w, s, 'spring');
            tc.verifyEqual(L - 1, 0, "AbsTol", 2e-3, "A quarter period passes zero.");
            tc.settle(w, T/4);
            [~, L] = phx.engine.io('get', w, s, 'spring');
            tc.verifyEqual(L - 1, -0.1, "RelTol", 0.01, "Half a period swings to the other side.");
            tc.verifyError(@() phx.engine.io('set', w, s, 'inertance', -1), "phx:engine:badArgument");
        end

        function torsionSpringTwistsByTorqueOverStiffness(tc)
            % 2 N*m about z on a body held by k = 10 N*m/rad (damped): it
            % turns by M/k; the spring holds it with -M, its base feels +M
            tc.requireProtocol(1.5);
            [w, a, b, s] = tc.torsionPair(1, 0, 10, 5);
            for i = 1:round(5/tc.Dt)
                [~] = phx.engine.io('apply', w, b, 'torque', [0 0 2], false);
                [~] = phx.engine.io('step', w, tc.Dt);
            end
            [M, angle] = phx.engine.io('get', w, s, 'spring');
            tc.verifyEqual(angle, 0.2, "AbsTol", 1e-4);
            tc.verifyEqual(M, -2, "RelTol", 1e-3);
            [~, tA, ~, tB] = phx.engine.io('get', w, s, 'feedback');
            tc.verifyEqual(tB, [0; 0; -2], "AbsTol", 2e-3);
            tc.verifyEqual(tA, [0; 0; 2], "AbsTol", 2e-3);
            tc.verifyEqual(phx.engine.io('get', w, a, 'totaltorque'), [0; 0; 2], "AbsTol", 2e-3);
        end

        function torsionSpringOscillatesAtSqrtKOverI(tc)
            % I = 0.5 kg*m^2 on k = 2 N*m/rad, rest angle 0.3 rad, released
            % at 0: period pi, swinging between 0 and 0.6 rad; with an
            % inertance of 1.5 kg*m^2 the period doubles
            tc.requireProtocol(1.5);
            for inertance = [0 1.5]
                [w, ~, ~, s] = tc.torsionPair(0.5, 0.3, 2, 0);
                phx.engine.io('set', w, s, 'inertance', inertance);
                T = pi*sqrt((0.5 + inertance)/0.5);
                n = round(3*T/tc.Dt);
                x = zeros(1, n);
                for i = 1:n
                    [~] = phx.engine.io('step', w, tc.Dt);
                    [~, x(i)] = phx.engine.io('get', w, s, 'spring');
                end
                x = x - 0.3;
                t = (1:n)*tc.Dt;
                up = find(x(1:end-1) < 0 & x(2:end) >= 0);
                crossings = t(up) - x(up)*tc.Dt./(x(up + 1) - x(up));
                tc.verifyEqual(mean(diff(crossings)), T, "RelTol", 0.01);
                tc.verifyEqual(max(abs(x(t > t(end) - T))), 0.3, "RelTol", 0.01);
            end
        end

        function torsionSpringWindsOverTurns(tc)
            % A body spun at 10 rad/s against k = 0.1 N*m/rad (I = 1) winds
            % it 10/sqrt(0.1) = 31.6 rad, five turns, before it turns back
            tc.requireProtocol(1.5);
            [w, ~, b, s] = tc.torsionPair(1, 0, 0.1, 0);
            phx.engine.io('set', w, b, 'angvel', [0 0 10]);
            top = 0;
            for i = 1:round(6/tc.Dt)
                [~] = phx.engine.io('step', w, tc.Dt);
                [~, angle] = phx.engine.io('get', w, s, 'spring');
                top = max(top, angle);
            end
            tc.verifyEqual(top, 10/sqrt(0.1), "RelTol", 0.01);
        end

        function torsionSpringRejectsBadSettings(tc)
            tc.requireProtocol(1.5);
            [w, a, b, s] = tc.torsionPair(1, 0, 1, 0);
            tc.verifyError(@() phx.engine.io('add', w, 'torsionspringconstraint', a, b, eye(4), eye(4), NaN, 1, 0), ...
                "phx:engine:badArgument");
            tc.verifyError(@() phx.engine.io('add', w, 'torsionspringconstraint', a, b, eye(4), eye(4), 0, -1, 0), ...
                "phx:engine:badArgument");
            tc.verifyError(@() phx.engine.io('set', w, s, 'inertance', -1), "phx:engine:badArgument");
            tc.verifyError(@() phx.engine.io('set', w, s, 'angle', Inf), "phx:engine:badArgument");
        end

        function ropeNeverPushes(tc)
            % Lengths [-Inf 1]: no force shorter than 1 m, even where the
            % points meet, a pull beyond it
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            lengths = [0 0.3 1.5];
            for i = 3:-1:1
                b = tc.addBox(w, 2, [0 3*i lengths(i)], 0.1*[1 1 1]);
                phx.engine.io('set', w, b, 'collisions', false);
                s(i) = phx.engine.io('add', w, 'springconstraint', a, b, [0 3*i 0], [0 0 0], [-Inf 1], 100, 0, true);
            end
            [~] = phx.engine.io('step', w, tc.Dt);
            f = arrayfun(@(h) phx.engine.io('get', w, h, 'spring'), s);
            tc.verifyEqual(f(1:2), [0 0], "A rope does not push.");
            tc.verifyEqual(f(3), 100*0.5, "RelTol", 0.02, "Beyond its length it pulls.");
        end

        function ropeTackleHoldsHalfTheLoad(tc)
            % A 2 kg block hangs in the bight of a rope: from the static beam
            % down under the block (two points 0.4 m apart, the rope wraps
            % it), up over the beam and across to a static drum. Two
            % vertical runs carry it, T = m*g/2; the block sinks by the
            % stretch over two runs, T/k/2; the drum feels T toward the beam
            tc.requireProtocol(1.5);
            [w, h] = tc.tackle(4.2);
            m = 2;
            k = 1e4;
            tc.settle(w, 3);
            [T, L] = phx.engine.io('get', w, h.rope, 'spring');
            tc.verifyEqual(T, m*tc.G/2, "RelTol", 1e-3);
            tc.verifyEqual(L, 4.2 + T/k, "AbsTol", 1e-5);
            p = tEngineProtocol.positionOf(w, h.block);
            tc.verifyEqual(p(3), 2 - T/k/2, "AbsTol", 1e-5);
            tc.verifyEqual(phx.engine.io('get', w, h.drum, 'totalforce'), [-T; 0; 0], "AbsTol", 1e-3*T);
        end

        function ropeIsSlackBelowItsLength(tc)
            % The same tackle with 0.3 m of rope to spare: the block falls
            % freely and the rope reads no tension
            tc.requireProtocol(1.5);
            [w, h] = tc.tackle(4.5);
            [~] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(phx.engine.io('get', w, h.rope, 'spring'), 0);
            tc.verifyEqual(phx.engine.io('get', w, h.block, 'linvel'), [0; 0; -tc.G*tc.Dt], "AbsTol", 1e-6);
        end

        function ropeGoesWithAnyOfItsBodies(tc)
            tc.requireProtocol(1.5);
            [w, h] = tc.tackle(4.2);
            tc.settle(w, 0.1);
            phx.engine.io('remove', w, h.block);
            [~] = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(phx.engine.io('get', w, h.rope, 'spring'), 0);
            tc.verifyError(@() phx.engine.io('set', w, h.rope, 'lengths', 4), "phx:engine:badHandle");
        end

        function ropeRejectsBadSettings(tc)
            tc.requireProtocol(1.5);
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.1*[1 1 1]);
            add = @(hs, L) phx.engine.io('add', w, 'ropeconstraint', hs, zeros(3, numel(hs)), L, 100, 0);
            tc.verifyError(@() add(a, 1), "phx:engine:badArgument");
            tc.verifyError(@() add([a a], 1), "phx:engine:badArgument");
            tc.verifyError(@() add([a b], -1), "phx:engine:badArgument");
            tc.verifyClass(add([a b a], 1), "uint64");
        end

        function damperAloneIsNotRigid(tc)
            % No stiffness, c = 2 N*s/m on 1 kg moving apart at 1 m/s: the
            % speed decays as exp(-2*t) and the bodies drift apart
            tc.requireProtocol(1.5);
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1]);
            s = phx.engine.io('add', w, 'springconstraint', a, b, [0 0 0], [0 0 0], 1, 0, 2, true);
            phx.engine.io('set', w, b, 'linvel', [1 0 0]);
            tc.settle(w, 0.5);
            tc.verifyEqual(phx.engine.io('get', w, b, 'linvel'), [exp(-1); 0; 0], "RelTol", 0.02);
            [~, L] = phx.engine.io('get', w, s, 'spring');
            tc.verifyEqual(L, 1 + (1 - exp(-1))/2, "RelTol", 0.01);

            % Neither stiffness nor damping: no force at all
            phx.engine.io('set', w, s, 'spring', 0, 0);
            v = phx.engine.io('get', w, b, 'linvel');
            tc.settle(w, 0.5);
            tc.verifyEqual(phx.engine.io('get', w, b, 'linvel'), v, "AbsTol", 1e-9);
            tc.verifyEqual(phx.engine.io('get', w, s, 'spring'), 0);
        end

        function springRejectsBadSettings(tc)
            tc.requireProtocol(1.5);
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.1*[1 1 1]);
            add = @(lengths, k, c) phx.engine.io('add', w, 'springconstraint', a, b, [0 0 0], [0 0 0], lengths, k, c, true);
            tc.verifyError(@() add([1 0.5], 1, 0), "phx:engine:badArgument");
            tc.verifyError(@() add(-1, 1, 0), "phx:engine:badArgument");
            tc.verifyError(@() add(-Inf, 1, 0), "phx:engine:badArgument");
            tc.verifyError(@() add([-Inf -1], 1, 0), "phx:engine:badArgument");
            tc.verifyError(@() add(1, -1, 0), "phx:engine:badArgument");
            tc.verifyError(@() add(1, 1, NaN), "phx:engine:badArgument");
        end
    end

    % Removing
    methods (Test, TestTags = {'Engine'})
        function removedBodyLeavesTheWorld(tc)
            w = tc.newWorld;
            h1 = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            h2 = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1]);
            phx.engine.io('remove', w, h1);
            ids = phx.engine.io('step', w, tc.Dt);
            tc.verifyEqual(ids, h2);
        end

        function removedJointReleasesTheBody(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            j = phx.engine.io('add', w, 'fixedconstraint', a, b, tEngineProtocol.pose([0.5 0 0]), tEngineProtocol.pose([-0.5 0 0]), true);
            tc.settle(w, 0.1);
            phx.engine.io('remove', w, j);
            T = tc.settle(w, 0.5);
            tc.verifyLessThan(T(3, 4), 1);
        end
    end

    methods (Access = private)
        function w = newWorld(tc, gravity)
            % A world with the given gravity and no sleeping, cleared at
            % teardown
            arguments
                tc
                gravity (1, 3) double = [0 0 -tEngineProtocol.G]
            end
            w = phx.engine.io('setup', 2);
            tc.addTeardown(@() phx.engine.io('clear', w));
            phx.engine.io('set', w, uint64(0), 'gravity', gravity);
            phx.engine.io('set', w, uint64(0), 'autoactivated', false);
        end

        function h = addBox(~, w, type, position, size, mass, inertia)
            % Box of the given full size; mass and inertia default to a
            % unit mass and its box inertia
            arguments
                ~
                w
                type
                position (1, 3) double
                size (1, 3) double
                mass (1, 1) double = 1
                inertia (1, 3) double = mass*(sum(size.^2) - size.^2)/12
            end
            h = phx.engine.io('add', w, 'box', type, size/2, tEngineProtocol.pose(position), mass, inertia);
        end

        function [w, a, b, s] = torsionPair(tc, inertia, restAngle, k, c)
            % A torsion spring about z from a static body to a free one of
            % the given (isotropic) inertia, in zero gravity, apart enough
            % not to touch
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = phx.engine.io('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([1 0 0]), 1, inertia*[1 1 1]);
            s = phx.engine.io('add', w, 'torsionspringconstraint', a, b, eye(4), eye(4), restAngle, k, c);
        end

        function [w, h] = tackle(tc, freeLength)
            % The tackle of the rope cases: static beam at z = 3, a 2 kg
            % block at z = 2 wrapped by the rope 0.4 m wide, a static drum
            % 2 m to the side at the beam's height; k = 1e4, c = 200
            w = tc.newWorld;
            h.beam = tc.addBox(w, 0, [0 0 3], [0.6 0.1 0.1]);
            h.block = tc.addBox(w, 2, [0 0 2], [0.4 0.4 0.4], 2);
            h.drum = tc.addBox(w, 0, [2 0 3], 0.1*[1 1 1]);
            hs = [h.beam h.block h.block h.beam h.drum];
            points = [-0.2 0 0; -0.2 0 0; 0.2 0 0; 0.2 0 0; 0 0 0]';
            h.rope = phx.engine.io('add', w, 'ropeconstraint', hs, points, freeLength, 1e4, 200);
        end

        function h = addFloor(tc, w)
            % Static 10 x 10 m floor with its top at z = 0
            h = tc.addBox(w, 0, [0 0 -0.5], [10 10 1], 0, [0 0 0]);
        end

        function requireProtocol(tc, version)
            % Assume away the cases of a later protocol than the engine
            % implements. The Bullet MEX predates the 'version' verb, which
            % thus marks protocol 1.
            try
                v = phx.engine.io('version');
            catch
                v = 1;
            end
            tc.assumeGreaterThanOrEqual(v, version, ...
                "The engine implements protocol " + v + ", the case needs " + version + ".");
        end

        function T = lastPose(tc, w, h, duration)
            % Steps for the given time; returns the last pose reported for
            % the body (a body standing still is not reported)
            T = [];
            for k = 1:round(duration/tc.Dt)
                [ids, P] = phx.engine.io('step', w, tc.Dt);
                if ~islogical(ids) && any(ids == h)
                    T = P(:, :, ids == h);
                end
            end
            tc.assertNotEmpty(T, "The body was never reported as moving.");
        end

        function T = settle(tc, w, duration)
            % Steps for the given time; returns the poses of the last step
            % (4x4xN, or false when nothing moved)
            for k = 1:round(duration/tc.Dt)
                [~, T] = phx.engine.io('step', w, tc.Dt);
            end
        end
    end

    methods (Static, Access = private)
        function P = pose(position)
            P = [eye(3) position(:); 0 0 0 1];
        end

        function p = positionOf(w, h)
            % Position of a moving body, read off the poses of one more
            % (negligibly short) step, as PHX reads it
            [ids, T] = phx.engine.io('step', w, 1e-6);
            p = T(1:3, 4, ids == h);
        end

        function restorePath(oldPath)
            path(oldPath);
            clear("mex"); %#ok<CLMEX> the next class loads the engine on the path
        end

        function s = shapeCases()
            % One row per shape: how to add it 1.5 m above the floor, and the
            % height of its origin at rest
            I = [0.1 0.1 0.1];
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            row = @tEngineProtocol.shapeRow;
            s.box = row(@(w, P) phx.engine.io('add', w, 'box', 2, [0.2 0.3 0.4], P, 1, I), 0.4);
            s.sphere = row(@(w, P) phx.engine.io('add', w, 'sphere', 2, 0.3, P, 1, I), 0.3);
            s.cylinderLying = row(@(w, P) phx.engine.io('add', w, 'cylinder', 2, [0.5 0.2 0.2], 'x', P, 1, I), 0.2);
            s.cylinderStanding = row(@(w, P) phx.engine.io('add', w, 'cylinder', 2, [0.4 0.4 0.1], 'z', P, 1, I), 0.1);
            s.cone = row(@(w, P) phx.engine.io('add', w, 'cone', 2, 0.3, 0.8, 'z', P, 1, I), 0.4);
            s.capsule = row(@(w, P) phx.engine.io('add', w, 'capsule', 2, 0.2, 0.6, 'z', P, 1, I), 0.5);
            s.convexHull = row(@(w, P) phx.engine.io('add', w, 'convexhull', 2, V(:), 8, false, P, 1, I), 0.2);
            % The concave mesh keeps an engine-chosen collision envelope
            % whatever the margin, so only a coarse height is part of the
            % contract
            s.concaveMesh = row(@(w, P) tEngineProtocol.addTrimesh(w, V, F, P), 0.2, 0.05);
        end

        function r = shapeRow(add, restHeight, tolerance)
            arguments
                add
                restHeight
                tolerance = 2e-3
            end
            r = struct("Add", add, "RestHeight", restHeight, "Tolerance", tolerance);
        end

        function h = addTrimesh(w, V, F, P, type)
            % Concave body of the given triangle mesh, dynamic by default
            arguments
                w
                V
                F
                P
                type = 2
            end
            sh = phx.engine.io('prepare', w, uint64(0), 'concaveshape', V(:), size(V, 2), F(:), size(F, 2));
            phx.engine.io('prepare', w, sh, 'dynamictrimesh');
            phx.engine.io('prepare', w, sh, 'validation');
            h = phx.engine.io('add', w, 'rigidbody', type, sh, P, 1, [0.1 0.1 0.1]);
        end

        function [V, F] = cubeMesh(half)
            % Closed cube of the given half size: vertices 3x8, outward
            % zero-based int32 faces 3x12
            V = half*[-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1]';
            F = int32([0 2 1; 0 3 2; 4 5 6; 4 6 7; 0 1 5; 0 5 4; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 0 4; 3 4 7]');
        end
    end

end
