classdef tEngineProtocol < PhxTestCase
%tEngineProtocol Contract tests of the phxEngine protocol 2.
%
%   These tests call phxEngine directly, without any phx.* object, and
%   check what PHX relies on: the verbs, the shapes of their inputs and
%   outputs, and the physical meaning of what they set. Only the parts PHX
%   uses are tested, and nothing that the protocol marks for cleanup is
%   locked in beyond what PHX needs today.
%
%   The engine under test is the phxEngine on the path. Another
%   implementation of the protocol is tested by passing its folder as the
%   external parameter MexFolder, which is put ahead of the path for the
%   class:
%
%       import matlab.unittest.parameters.Parameter
%       p = Parameter.fromData("MexFolder", {"<folder holding phxEngine>"});
%       run(matlab.unittest.TestSuite.fromFile("tEngineProtocol.m", "ExternalParameters", p))
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
        % Folder holding the phxEngine of the engine under test; empty
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
                tc.assertTrue(startsWith(which("phxEngine"), MexFolder), ...
                    "phxEngine does not resolve to " + MexFolder + ".");
            end
            tc.requireEngine;
            % These are the contracts of protocol 2; the protocol 1 engine
            % (Bullet) predates the 'version' verb
            try
                v = phxEngine('version');
            catch
                v = 1;
            end
            tc.assumeGreaterThanOrEqual(v, 2, "The engine implements protocol " + v + "; these tests need protocol 2.");
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

        function failuresAreErrorsWithEngineIds(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 0], [1 1 1]);
            tc.verifyError(@() phxEngine('setup', 2), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('nosuchverb', w), "phx:engine:unsupported");
            tc.verifyError(@() phxEngine('set', w, h, 'nosuchfield', 1), "phx:engine:unsupported");
            tc.verifyError(@() phxEngine('get', w, h, 'nosuchfield'), "phx:engine:unsupported");
            tc.verifyError(@() phxEngine('set', w, uint64(12345), 'linvel', [0 0 0]), "phx:engine:badHandle");
        end

        function actingVerbsReturnNothing(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 0], [1 1 1]);
            tc.verifyError(@() tEngineProtocol.oneOutput(@() phxEngine('set', w, h, 'linvel', [1 0 0])), ...
                "MATLAB:unassignedOutputs");
            tc.verifyError(@() tEngineProtocol.oneOutput(@() phxEngine('apply', w, h, 'centralforce', [1 0 0], false)), ...
                "MATLAB:unassignedOutputs");
            tc.verifySize(phxEngine('get', w, h, 'linvel'), [1 3], "A vector comes out as a row.");
        end

        function stepWithoutMovingBodiesReturnsEmpties(tc)
            w = tc.newWorld;
            tc.addBox(w, 0, [0 0 0], [1 1 1]);
            [ids, T, slots] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(ids, zeros(1, 0, "uint64"));
            tc.verifyEqual(T, zeros(4, 4, 0));
            tc.verifyEqual(slots, zeros(1, 0));
        end

        function stepReportsTheSlotsOfMovedBodies(tc)
            % A body gets the slot of its place in the 'slots' row (a zero
            % leaves a place empty); step reports each moved body's slot
            % after its handle and pose, 0 for a body without one. Setting
            % the slots replaces the table, and a removed body takes its
            % slot with it, also when a new body reuses its place.
            w = tc.newWorld([0 0 0]);
            h1 = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            h2 = tc.addBox(w, 2, [2 0 0], 0.2*[1 1 1]);
            h3 = tc.addBox(w, 2, [4 0 0], 0.2*[1 1 1]);
            phxEngine('set', w, uint64(0), 'slots', [uint64(0) h2 h1]);
            [ids, ~, slots] = phxEngine('step', w, tc.Dt);
            tc.verifyClass(slots, "double");
            tc.verifySize(slots, [1 3]);
            tc.verifyEqual([slots(ids == h1) slots(ids == h2) slots(ids == h3)], [3 2 0]);

            phxEngine('set', w, uint64(0), 'slots', h3);
            [ids, ~, slots] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual([slots(ids == h1) slots(ids == h2) slots(ids == h3)], [0 0 1]);

            phxEngine('remove', w, h3);
            h4 = tc.addBox(w, 2, [6 0 0], 0.2*[1 1 1]);
            [ids, ~, slots] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(slots(ids == h4), 0, "A new body has no slot.");

            tc.verifyError(@() phxEngine('set', w, uint64(0), 'slots', [h1; h2]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, uint64(0), 'slots', h3), "phx:engine:badHandle");
        end

        function stepReportsMovedBodiesWithTheirPoses(tc)
            w = tc.newWorld;
            tc.addBox(w, 0, [0 0 -10], [1 1 1]);
            h1 = tc.addBox(w, 2, [1 0 5], 0.2*[1 1 1]);
            h2 = tc.addBox(w, 2, [-1 0 5], 0.2*[1 1 1]);
            [ids, T] = phxEngine('step', w, tc.Dt);
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
                [~, T] = phxEngine('step', w, tc.Dt);
            end
            % v(k) = -g*k*dt, z(n) = z0 + dt*sum(v(1..n))
            tc.verifyEqual(T(3, 4), 10 - tc.G*tc.Dt^2*n*(n + 1)/2, "AbsTol", 1e-5);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 0, -tc.G*n*tc.Dt], "AbsTol", 1e-5);
        end

        function gravityIsTheWorldAcceleration(tc)
            w = tc.newWorld([1 -2 0.5]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            [~] = phxEngine('step', w, 0.01);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), 0.01*[1, -2, 0.5], "AbsTol", 1e-6);
        end

        function worldsAreIndependent(tc)
            wA = tc.newWorld;
            wB = tc.newWorld;
            hA = tc.addBox(wA, 2, [0 0 0], 0.2*[1 1 1]);
            hB = tc.addBox(wB, 2, [0 0 0], 0.2*[1 1 1]);
            tc.verifyNotEqual(hA, hB, "Body handles are unique across worlds.");
            for k = 1:10
                [~] = phxEngine('step', wA, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', wB, hB, 'linvel'), [0, 0, 0]);
            tc.verifyLessThan(phxEngine('get', wA, hA, 'linvel'), [1, 1, 0]);
        end
    end

    % Shapes
    methods (Test, TestTags = {'Engine'})
        function shapeRestsOnItsGeometry(tc, Shape)
            % Dropped on a floor whose top is at z = 0, every shape comes to
            % rest with its origin at the height its geometry gives. The
            % margin is zeroed so that the shapes collide as they are.
            w = tc.newWorld;
            phxEngine('set', w, uint64(0), 'settings', struct('margin', 0));
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
            phxEngine('set', w, uint64(0), 'settings', struct('margin', 0));
            tc.addFloor(w);
            h = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0 0 1]));
            T = tc.lastPose(w, h, 3);
            speed = 0;
            for k = 1:200
                [ids, P] = phxEngine('step', w, tc.Dt);
                if any(ids == h)
                    T = P(:, :, ids == h);
                end
                speed = max([speed, norm(phxEngine('get', w, h, 'linvel')), ...
                    0.2*norm(phxEngine('get', w, h, 'angvel'))]);
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
            n = 64;
            a = 2*pi*(0:n - 1)/n;
            V = [0 0 0; cos(a') sin(a') zeros(n, 1)]';   % centre, then the rim
            F = [ones(1, n); 2:n + 1; [3:n + 1 2]];
            w = tc.newWorld;
            phxEngine('set', w, uint64(0), 'settings', struct('margin', 0));
            disc = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0 0 0]), 0);
            box = tc.addBox(w, 2, [0.05 -0.3 0.1], [0.2 0.2 0.2], 2);
            tc.settle(w, 0.2);
            phxEngine('set', w, box, 'linvel', [0 2 0]);
            arm = zeros(1, 40);
            for k = 1:40
                [~] = phxEngine('step', w, tc.Dt);
                f = phxEngine('get', w, disc, 'totalforce');
                t = phxEngine('get', w, disc, 'totaltorque');
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
            phxEngine('set', w, a, 'linvel', [1 0 0]);
            tc.settle(w, 2);
            vb = phxEngine('get', w, b, 'linvel');
            tc.verifyGreaterThan(vb(1), 0.3, "The struck cube is pushed along.");
            Ta = tc.lastPose(w, a, tc.Dt);
            Tb = tc.lastPose(w, b, tc.Dt);
            tc.verifyLessThan(Ta(1, 4), Tb(1, 4) - 0.3, "The cubes do not pass through each other.");
        end

        function terrainHasHeightZeroAtItsOrigin(tc)
            % The heightfield lies with height 0 of its data at the body
            % origin, whatever its lowest and highest point, and centred on
            % it across: a ball comes to rest on a surface at height 0.2
            % with a 1.0 peak elsewhere, 0.3 m above a terrain body at 0.1.
            w = tc.newWorld;
            H = 0.2*ones(11, 6);
            H(end, end) = 1;
            phxEngine('add', w, 'terrain', 0, H, [1 2], tEngineProtocol.pose([0 0 0.1]), 0, [0 0 0]);
            h = phxEngine('add', w, 'sphere', 2, 0.25, tEngineProtocol.pose([-4 -4 2]), 1, [0.1 0.1 0.1]);
            T = tc.lastPose(w, h, 2);
            tc.verifyEqual(T(3, 4), 0.1 + 0.2 + 0.25, "AbsTol", 2e-3);
            % Off the 10 x 10 m grid it falls
            h = phxEngine('add', w, 'sphere', 2, 0.25, tEngineProtocol.pose([5.5 0 2]), 1, [0.1 0.1 0.1]);
            T = tc.lastPose(w, h, 1);
            tc.verifyLessThan(T(3, 4), 0, "A ball beside the grid falls past it.");
        end

        function meshArgumentsAreChecked(tc)
            % Vertices and faces are matrices of 3 rows; the faces hold
            % one-based indices of existing vertices. Shapes are no longer
            % prepared apart from their bodies.
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            w = tc.newWorld;
            P = tEngineProtocol.pose([0 0 0]);
            bad = F;
            bad(1) = 0;
            tc.verifyError(@() phxEngine('add', w, 'trimesh', 2, V, bad, P, 1, [1 1 1]), "phx:engine:badArgument");
            bad(1) = 9;
            tc.verifyError(@() phxEngine('add', w, 'trimesh', 2, V, bad, P, 1, [1 1 1]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'trimesh', 2, V(:), F, P, 1, [1 1 1]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'convexhull', 2, V(:, 1:3), P, 1, [1 1 1]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('prepare', w, uint64(0), 'concaveshape', V(:), 8, int32(F(:) - 1), 12), ...
                "phx:engine:unsupported");
        end
    end

    % Body fields
    methods (Test, TestTags = {'Engine'})
        function transformTeleportsTheBody(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            P = makehgtform("translate", [3 -2 1], "zrotate", 0.7, "xrotate", -0.3);
            phxEngine('set', w, h, 'transform', P);
            [~, T] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(T, P, "AbsTol", 1e-6);
        end

        function typeSwitchesBetweenStaticKinematicAndDynamic(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 0, [0 0 5], 0.2*[1 1 1], 1);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 0, 0], "A static body does not fall.");

            phxEngine('set', w, h, 'type', 2);
            [ids, T] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(ids, h);
            tc.verifyLessThan(T(3, 4), 5, "A body switched to dynamic falls.");

            phxEngine('set', w, h, 'type', 1);
            phxEngine('set', w, h, 'linvel', [0 0 0]);
            for k = 1:10
                [~] = phxEngine('step', w, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 0, 0], "AbsTol", 1e-9, ...
                "A kinematic body does not fall.");
        end

        function massAndInertiaScaleTheResponse(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            % Mass and inertia act from the next step on
            phxEngine('set', w, h, 'massprops', 2, [1 2 4]);
            phxEngine('apply', w, h, 'centralforce', [4 0 0], false);
            phxEngine('apply', w, h, 'torque', [1 1 1], false);
            [~] = phxEngine('step', w, 0.01);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0.02, 0, 0], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, h, 'angvel'), 0.01*[1, 1/2, 1/4], "AbsTol", 1e-5);
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
            phxEngine('set', w, b, 'angvel', [0.2 3 0.2]);
            T = tc.lastPose(w, b, 3);
            tc.verifyLessThan(T(2, 2), -0.9, "The middle axis has turned over.");
        end

        function zeroMassFreezesTheBody(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 0, [1 1 1]);
            phxEngine('apply', w, h, 'centralforce', [1 0 0], false);
            phxEngine('apply', w, h, 'torque', [1 1 1], false);
            [~] = phxEngine('step', w, 0.01);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 0, 0], "AbsTol", 1e-9);
            tc.verifyEqual(phxEngine('get', w, h, 'angvel'), [0, 0, 0], "AbsTol", 1e-9);
        end

        function zeroInertiaFreezesThatRotation(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [0 0 1]);
            phxEngine('apply', w, h, 'torque', [1 1 1], false);
            [~] = phxEngine('step', w, 0.01);
            tc.verifyEqual(phxEngine('get', w, h, 'angvel'), [0, 0, 0.01], "AbsTol", 1e-6);
        end

        function velocitiesRoundTrip(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phxEngine('set', w, h, 'linvel', [1 -2 3]);
            phxEngine('set', w, h, 'angvel', [-0.5 0.25 2]);
            v = phxEngine('get', w, h, 'linvel');
            omega = phxEngine('get', w, h, 'angvel');
            tc.verifySize(v, [1 3]);
            tc.verifySize(omega, [1 3]);
            tc.verifyEqual(v, [1, -2, 3], "AbsTol", 1e-6);
            tc.verifyEqual(omega, [-0.5, 0.25, 2], "AbsTol", 1e-6);
        end

        function posesComeInBulkAsStepReportsThem(tc)
            w = tc.newWorld;
            ground = tc.addBox(w, 0, [0 0 -10], [1 1 1]);
            hs = zeros(1, 3, 'uint64');
            for i = 1:3
                hs(i) = tc.addBox(w, 2, [i 0 5], 0.2*[1 1 1]);
                phxEngine('set', w, hs(i), 'angvel', [i 1 -0.5]);
            end
            for k = 1:5
                [ids, T] = phxEngine('step', w, tc.Dt);
            end
            order = [hs(3) ground hs(1) hs(2)];
            [~, at] = ismember(hs([3 1 2]), ids);
            M = phxEngine('get', w, order, 'transform');
            P = phxEngine('get', w, order, 'position');
            tc.verifySize(M, [4 4 4]);
            tc.verifySize(P, [3 4]);
            tc.verifyEqual(M(:, :, [1 3 4]), T(:, :, at), "The moved bodies read what step reported, bit for bit.");
            tc.verifyEqual(M(:, :, 2), tEngineProtocol.pose([0 0 -10]), "AbsTol", 1e-12, "A static body reads its pose.");
            tc.verifyEqual(P, reshape(M(1:3, 4, :), 3, []), "The positions are the origins of the transforms.");
        end

        function poseReadsTakeAnyNumberOfBodies(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [1 2 3], 0.2*[1 1 1]);
            tc.verifyEqual(phxEngine('get', w, h, 'position'), [1; 2; 3], "AbsTol", 1e-12);
            tc.verifySize(phxEngine('get', w, h, 'transform'), [4 4]);
            none = zeros(1, 0, 'uint64');
            tc.verifySize(phxEngine('get', w, none, 'position'), [3 0]);
            tc.verifySize(phxEngine('get', w, none, 'transform'), [4 4 0]);
        end

        function poseReadsRejectWhatIsNoBody(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1]);
            j = phxEngine('add', w, 'fixed', [a b], eye(4), eye(4), false);
            tc.verifyError(@() phxEngine('get', w, [a uint64(12345)], 'position'), "phx:engine:badHandle");
            tc.verifyError(@() phxEngine('get', w, [a j], 'transform'), "phx:engine:badHandle");
            tc.verifyError(@() phxEngine('get', w, double(a), 'position'), "phx:engine:badHandle");
            tc.verifyError(@() phxEngine('get', w, [a; b], 'position'), "phx:engine:badArgument");
        end

        function kinematicBodyReadsItsPoseAfterTheStep(tc)
            % A pose set on a kinematic body is reached by the end of the
            % next step; until then the body reads where it is
            w = tc.newWorld;
            h = tc.addBox(w, 1, [0 0 0], 0.2*[1 1 1]);
            phxEngine('set', w, h, 'transform', tEngineProtocol.pose([1 0 0]));
            tc.verifyEqual(phxEngine('get', w, h, 'position'), [0; 0; 0], "AbsTol", 1e-12);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h, 'position'), [1; 0; 0], "AbsTol", 1e-6);
        end

        function frictionOfTheTwoBodiesMultiplies(tc)
            % A box sliding on a floor decelerates by mu_box*mu_floor*g
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'friction', 0.8);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phxEngine('set', w, h, 'friction', 0.5);
            tc.settle(w, 0.1);
            phxEngine('set', w, h, 'linvel', [3 0 0]);
            v0 = phxEngine('get', w, h, 'linvel');
            tc.settle(w, 0.2);
            v1 = phxEngine('get', w, h, 'linvel');
            tc.verifyEqual((v0(1) - v1(1))/0.2, 0.5*0.8*tc.G, "RelTol", 0.03);
        end

        function restitutionSetsTheBounce(tc)
            peak = zeros(1, 2);
            for i = 1:2
                r = i - 1;
                w = tc.newWorld;
                phxEngine('set', w, tc.addFloor(w), 'restitution', 1);
                h = phxEngine('add', w, 'sphere', 2, 0.2, tEngineProtocol.pose([0 0 1.2]), 1, [0.016 0.016 0.016]);
                phxEngine('set', w, h, 'restitution', r);
                bounced = false;
                for k = 1:250
                    [~, T] = phxEngine('step', w, tc.Dt);
                    v = phxEngine('get', w, h, 'linvel');
                    bounced = bounced || v(3) > 0;
                    if bounced && ~isempty(T)
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
            % stopped at 0.205 s whatever the spin.)
            s = [0.2 0.2 0.02];
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'friction', 1);
            h = tc.addBox(w, 2, [0 0 0.01], s);
            phxEngine('set', w, h, 'friction', 0.5);
            phxEngine('set', w, h, 'linvel', [1 0 0]);
            phxEngine('set', w, h, 'angvel', [0 0 20]);
            slideStops = NaN;
            spinStops = NaN;
            for k = 1:round(1/tc.Dt)
                [~] = phxEngine('step', w, tc.Dt);
                v = phxEngine('get', w, h, 'linvel');
                om = phxEngine('get', w, h, 'angvel');
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
            % where it was.)
            r = 0.1;
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'restitution', 1);
            h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 1 + r]), 1, 0.4*r^2*[1 1 1]);
            phxEngine('set', w, h, 'restitution', 0.8);
            z = nan(1, round(2/tc.Dt));
            for k = 1:numel(z)
                [ids, T] = phxEngine('step', w, tc.Dt);
                if any(ids == h)
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
            % restitution 0.6.)
            r = 0.1;
            dt = 0.002;
            w = tc.newWorld;
            for x = [-2 2]
                lane = tc.addBox(w, 0, [x 0 -0.05], [4 1 0.1], 0, [0 0 0]);
                phxEngine('set', w, lane, 'restitution', 0.1);
            end
            h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([-3.5 0 r]), 1, 0.4*r^2*[1 1 1]);
            phxEngine('set', w, h, 'restitution', 0.6);
            phxEngine('set', w, h, 'linvel', [8 0 0]);
            phxEngine('set', w, h, 'angvel', [0 8/r 0]);
            lift = 0;
            for k = 1:round(0.8/dt)
                [ids, T] = phxEngine('step', w, dt);
                if any(ids == h)
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
            r = 0.1;
            for dt = [0.005 0.01]
                w = tc.newWorld;
                for x = [-2 2]
                    lane = tc.addBox(w, 0, [x 0 -0.05], [4 1 0.1], 0, [0 0 0]);
                    phxEngine('set', w, lane, 'restitution', 0.1);
                end
                h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([-3.5 0 r]), 1, 0.4*r^2*[1 1 1]);
                phxEngine('set', w, h, 'restitution', 0.6);
                phxEngine('set', w, h, 'linvel', [8 0 0]);
                phxEngine('set', w, h, 'angvel', [0 8/r 0]);
                lift = 0;
                for k = 1:round(0.8/dt)
                    [ids, T] = phxEngine('step', w, dt);
                    if any(ids == h)
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
            r = 0.1;
            b = 0.5*r;
            w = tc.newWorld([0 0 0]);
            h1 = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([-0.3 0 0]), 1, 0.4*r^2*[1 1 1]);
            h2 = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 b 0]), 1, 0.4*r^2*[1 1 1]);
            for h = [h1 h2]
                phxEngine('set', w, h, 'restitution', 1);
                phxEngine('set', w, h, 'friction', 0);
            end
            phxEngine('set', w, h1, 'linvel', [3 0 0]);
            for k = 1:round(0.2/tc.Dt)
                phxEngine('step', w, tc.Dt);
            end
            n = [sqrt(4*r^2 - b^2); b; 0]/(2*r);
            vn = 3*n(1);
            tc.verifyEqual(phxEngine('get', w, h1, 'linvel'), [3, 0, 0] - vn*n', "AbsTol", 0.1, "Velocity of the hitting sphere (m/s)");
            tc.verifyEqual(phxEngine('get', w, h2, 'linvel'), vn*n', "AbsTol", 0.1, "Velocity of the hit sphere (m/s)");
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
            h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([-4 0 r]), 1, 0.4*r^2*[1 1 1]);
            phxEngine('set', w, h, 'linvel', [v0 0 0]);
            phxEngine('set', w, h, 'angvel', [0 v0/r 0]);
            lowest = r;
            for k = 1:round(10/tc.Dt)
                [ids, T] = phxEngine('step', w, tc.Dt);
                if any(ids == h)
                    lowest = min(lowest, T(3, 4, ids == h));
                end
            end
            v = phxEngine('get', w, h, 'linvel');
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
                h = phxEngine('add', w, 'cylinder', 2, r, 2*hh, 'z', [R [0; 0; -lowest(3)]; 0 0 0 1], 1, [0.02 0.02 0.045]);
                depth = 0;
                for k = 1:20
                    [ids, T] = phxEngine('step', w, tc.Dt);
                    if any(ids == h)
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
            phxEngine('set', w, h, 'collisions', false);
            T = tc.settle(w, 1);
            tc.verifyLessThan(T(3, 4), -1);
        end

        function kinematicBodyMovedByPoseCarriesWhatRestsOnIt(tc)
            % A kinematic plate driven by writing its pose every step moves
            % with the velocity of those poses, so friction carries a box
            w = tc.newWorld;
            plate = tc.addBox(w, 1, [0 0 -0.1], [4 4 0.2]);
            phxEngine('set', w, plate, 'friction', 1);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phxEngine('set', w, h, 'friction', 1);
            tc.settle(w, 0.1);
            v = 0.5;
            n = round(1/tc.Dt);
            for k = 1:n
                phxEngine('set', w, plate, 'transform', tEngineProtocol.pose([v*k*tc.Dt 0 -0.1]));
                [ids, T] = phxEngine('step', w, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [v, 0, 0], "AbsTol", 0.02);
            tc.verifyEqual(T(1, 4, ids == h), v*n*tc.Dt, "AbsTol", 0.02, "The box rides along.");
        end

        function tiltedStackDoesNotTwist(tc)
            % A stack of six wide, light boxes (3 x 3 x 1 m, 1 kg, inertia
            % 0.1, as in phxex_stack) on a kinematic pallet tilted to 8 deg
            % stays put: nothing turns it about the vertical. (A friction
            % model letting the twist friction reverse the twist made such
            % a stack twitch at 2 rad/s and topple at 15 instead of 23 deg.)
            w = tc.newWorld;
            pallet = tc.addBox(w, 1, [0 0 0], [8 8 0.5]);
            phxEngine('set', w, pallet, 'friction', 0.5);
            offsets = [0.02 -0.03; -0.04 0.01; 0.03 0.04; -0.01 -0.02; 0.04 0.02; -0.03 0.03];
            boxes = zeros(1, 6, "uint64");
            for i = 1:6
                boxes(i) = tc.addBox(w, 2, [offsets(i, :) 0.25 + (i - 0.5) + 0.02*i], [3 3 1], 1, [0.1 0.1 0.1]);
                phxEngine('set', w, boxes(i), 'friction', 0.5);
            end
            tc.settle(w, 0.5);
            n = round(1.6/tc.Dt);
            twist = 0;
            for k = 1:n
                tilt = deg2rad(8)*k/n;
                phxEngine('set', w, pallet, 'transform', makehgtform("xrotate", tilt));
                [~] = phxEngine('step', w, tc.Dt);
                for i = 1:6
                    om = phxEngine('get', w, boxes(i), 'angvel');
                    up = makehgtform("xrotate", tilt)*[0; 0; 1; 0];
                    twist = max(twist, abs(om*up(1:3)));
                end
            end
            tc.verifyLessThan(twist, 0.5, "Largest turning rate of a box about the pallet normal (rad/s)");
        end

        function kinematicBodyTiltedByPoseHoldsWhatRestsOnIt(tc)
            % Tilting a kinematic plate by its pose turns a box resting on
            % it along, held by static friction (tan 0.15 < 0.8*0.8)
            w = tc.newWorld;
            plate = tc.addBox(w, 1, [0 0 -0.1], [4 4 0.2]);
            phxEngine('set', w, plate, 'friction', 0.8);
            h = tc.addBox(w, 2, [1 0 0.2], 0.4*[1 1 1]);
            phxEngine('set', w, h, 'friction', 0.8);
            tc.settle(w, 0.1);
            n = round(1/tc.Dt);
            for k = 1:n
                tilt = 0.15*k/n;
                phxEngine('set', w, plate, 'transform', makehgtform("yrotate", -tilt, "translate", [0 0 -0.1]));
                [ids, T] = phxEngine('step', w, tc.Dt);
            end
            R = T(1:3, 1:3, ids == h);
            tc.verifyEqual(atan2(R(3, 1), R(1, 1)), 0.15, "AbsTol", 0.01, "The box tilts with the plate.");
            % Where the box would be had it stayed fixed to the plate
            expected = makehgtform("yrotate", -0.15)*[1; 0; 0.2; 1];
            tc.verifyEqual(T(1:3, 4, ids == h), expected(1:3), "AbsTol", 0.01, "The box does not slide.");
        end

        function coneTurnsAboutItsCentreOfMass(tc)
            % The cone's centre of mass lies a quarter of the
            % height below its origin; half a turn about a transverse axis
            % takes the origin from above the centre of mass to below it
            r = 0.3;
            h = 0.8;
            I = [3*h^2/80 + 3*r^2/20, 3*h^2/80 + 3*r^2/20, 3*r^2/10];
            w = tc.newWorld([0 0 0]);
            c = phxEngine('add', w, 'cone', 2, r, h, 'z', tEngineProtocol.pose([0 0 0]), 1, I);
            phxEngine('set', w, c, 'angvel', [pi 0 0]);
            T = tc.lastPose(w, c, 1);
            tc.verifyEqual(T(1:3, 4), [0; 0; -h/2], "AbsTol", 1e-3);
            tc.verifyEqual(T(1:3, 1:3)*[0; 0; 1], [0; 0; -1], "AbsTol", 1e-3, "The apex points down.");
        end

        function coneWithTwoRadiiIsTruncated(tc)
            % A cone of radius [0.4 0.2] (negative and positive end of its
            % axis) and height 0.6: a ray down at x = 0.3 meets the side
            % halfway up, one at x = 0.1 the flat top. Its centre of mass
            % lies h (R^2 + 2Rr + 3r^2)/(4 (R^2 + Rr + r^2)) above the wide
            % end, so half a turn takes the origin twice that far below.
            R = 0.4;
            r = 0.2;
            h = 0.6;
            w = tc.newWorld([0 0 0]);
            phxEngine('add', w, 'cone', 0, [R r], h, 'z', tEngineProtocol.pose([0 0 0]), 1, [1 1 1]);
            P = phxEngine('raycast', w, [0.3 0.1; 0 0; 1 1], [0.3 0.1; 0 0; -1 -1]);
            tc.verifyEqual(P(3, :), [0, h/2], "AbsTol", 1e-4, "Heights of the side and the top");

            zc = h*(R^2 + 2*R*r + 3*r^2)/(4*(R^2 + R*r + r^2));
            c = phxEngine('add', w, 'cone', 2, [R r], h, 'z', tEngineProtocol.pose([5 0 0]), 1, 0.01*[1 1 1]);
            phxEngine('set', w, c, 'angvel', [pi 0 0]);
            T = tc.lastPose(w, c, 1);
            tc.verifyEqual(T(1:3, 4), [5; 0; -2*(h/2 - zc)], "AbsTol", 1e-3);
        end

        function revolutionIsTheRevolvedProfile(tc)
            % The profile [z; r] runs from an end off the axis through a
            % groove, a step and a sharp tip. Rays towards the axis meet the
            % solid at the radius of the profile at their height, whatever
            % their azimuth (it is round, not faceted), for every axis; rays
            % along the axis meet the cone of the tip and the flat bottom
            % end, which the end cap closes.
            profile = [-0.3 -0.1 0 0.1 0.1 0.2 0.3; 0.2 0.2 0.1 0.2 0.25 0.25 0];
            heights = [-0.2 -0.05 0.02 0.05 0.15 0.25];   % off the seams, where the ray's GJK tolerance shows
            radii = [0.2 0.15 0.12 0.15 0.25 0.125];
            azimuths = deg2rad([0 37 90 200]);
            w = tc.newWorld([0 0 0]);
            for axis = 'xyz'
                % Body axes: the axis and two radial directions
                k = axis - 'w';
                e = eye(3);
                e = e(:, [setdiff(1:3, k) k]);
                T = tEngineProtocol.pose(10*[k 0 0]);
                phxEngine('add', w, 'revolution', 0, profile, axis, T, 1, [1 1 1]);
                [z, phi] = meshgrid(heights, azimuths);
                radial = e(:, 1)*cos(phi(:)') + e(:, 2)*sin(phi(:)');
                from = T(1:3, 4) + e(:, 3)*z(:)' + radial;
                P = phxEngine('raycast', w, from, from - radial);
                distance = sqrt(sum((P - T(1:3, 4) - e(:, 3)*z(:)').^2, 1));
                tc.verifyEqual(distance, repelem(radii, numel(azimuths)), "AbsTol", 1e-5, ...
                    "Radii along the profile, axis " + axis);
                from = T(1:3, 4) + e(:, 2)*[0.22 0.15] + e(:, 3)*[1 -1];
                P = phxEngine('raycast', w, from, from - e(:, 3)*[2 -2]);
                tc.verifyEqual(e(:, 3)'*(P - T(1:3, 4)), [0.212 -0.3], "AbsTol", 1e-5, ...
                    "Heights of the tip cone and the bottom end, axis " + axis);
            end
        end

        function revolutionCentreIsTheCentroid(tc)
            % The centre of mass of a revolution is the centroid of its
            % volume: the sum over its truncated cones
            profile = [-0.3 -0.1 0 0.1 0.1 0.2 0.3; 0.2 0.2 0.1 0.2 0.25 0.25 0];
            z1 = profile(1, 1:end - 1);
            h = diff(profile(1, :));
            r1 = profile(2, 1:end - 1);
            r2 = profile(2, 2:end);
            V = pi*h/3.*(r1.^2 + r1.*r2 + r2.^2);
            zc = z1 + h.*(r1.^2 + 2*r1.*r2 + 3*r2.^2)./(4*(r1.^2 + r1.*r2 + r2.^2));
            zc(V == 0) = 0;
            w = tc.newWorld([0 0 0]);
            c = phxEngine('add', w, 'revolution', 2, profile, 'y', eye(4), 1, [1 1 1]);
            tc.verifyEqual(phxEngine('get', w, c, 'com'), [0 sum(V.*zc)/sum(V) 0], "AbsTol", 1e-6);
        end

        function revolutionTakesOnlyOneRadiusPerHeight(tc)
            % Heights must not decrease (beyond 1e-6 of the size), radii
            % must not be negative, the profile must enclose a volume
            w = tc.newWorld([0 0 0]);
            add = @(profile) phxEngine('add', w, 'revolution', 2, profile, 'z', eye(4), 1, [1 1 1]);
            tc.verifyError(@() add([0 0.2 0.1 0.3; 0 0.1 0.2 0]), "phx:engine:badArgument", "Decreasing heights");
            tc.verifyError(@() add([0 0 1 1; 0 -0.1 0.1 0]), "phx:engine:badArgument", "Negative radius");
            tc.verifyError(@() add([0 0 1 1; 0 NaN 0.1 0]), "phx:engine:badArgument", "Not finite");
            tc.verifyError(@() add([0 0 1 1; 0 0.1 0.1 0; 0 0 0 0]), "phx:engine:badArgument", "Three rows");
            tc.verifyError(@() add([0 1; 0 0]), "phx:engine:badArgument", "No volume");
            tc.verifyError(@() add([0 0; 0 1]), "phx:engine:badArgument", "No height");
            tc.verifyWarningFree(@() add([0 0 1 1 - 1e-9; 0 0.1 0.1 0]), "A height off by less than the tolerance");
        end

        function centreOfMassIsTheShapesUnlessGiven(tc)
            % 'com' reads the centre of mass in body axes from the origin:
            % where the shape has it, or where 'add' and 'set' put it; a
            % centre that is not finite is refused
            w = tc.newWorld([0 0 0]);
            b = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            tc.verifyEqual(phxEngine('get', w, b, 'com'), [0 0 0], "AbsTol", 1e-7);
            c = phxEngine('add', w, 'cone', 2, 0.4, 0.6, 'z', tEngineProtocol.pose([2 0 0]), 1, [1 1 1]);
            tc.verifyEqual(phxEngine('get', w, c, 'com'), [0 0 -0.15], "AbsTol", 1e-6);
            d = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([4 0 0]), 1, [1 1 1], [0.1 0 0]);
            tc.verifyEqual(phxEngine('get', w, d, 'com'), [0.1 0 0], "AbsTol", 1e-7);
            phxEngine('set', w, d, 'com', [0 0 0]);
            tc.verifyEqual(phxEngine('get', w, d, 'com'), [0 0 0], "AbsTol", 1e-7);
            tc.verifyError(@() phxEngine('set', w, d, 'com', [NaN 0 0]), "phx:engine:badArgument");
        end

        function bodyTurnsAboutTheGivenCentre(tc)
            % With the centre 0.3 m off the origin and no linear velocity,
            % half a turn takes the origin 0.6 m across
            w = tc.newWorld([0 0 0]);
            b = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], eye(4), 1, 0.01*[1 1 1], [0.3 0 0]);
            phxEngine('set', w, b, 'angvel', [0 0 pi]);
            T = tc.lastPose(w, b, 1);
            tc.verifyEqual(T(1:3, 4), [0.6; 0; 0], "AbsTol", 1e-3);
        end

        function gravityActsAtTheCentre(tc)
            % A body pinned at its origin with the centre 0.3 m to the side
            % starts to fall about the pin: alpha = m g d/(I + m d^2)
            m = 2;
            I = 0.05;
            d = 0.3;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.1*[1 1 1]);
            b = phxEngine('add', w, 'box', 2, 0.05*[1 1 1], tEngineProtocol.pose([0 0 2]), m, I*[1 1 1], [d 0 0]);
            phxEngine('add', w, 'spherical', [a b], eye(4), eye(4), false);
            [~] = phxEngine('step', w, tc.Dt);
            om = phxEngine('get', w, b, 'angvel');
            tc.verifyEqual(om(2), m*tc.G*d/(I + m*d^2)*tc.Dt, "RelTol", 0.02);
        end

        function movedCentreKeepsTheMotionOfTheBody(tc)
            % Moving the centre of a spinning body changes neither its pose
            % nor the velocity of its points: the new centre takes the
            % velocity it had and from then on flies straight, the body
            % turning about it
            w = tc.newWorld([0 0 0]);
            b = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], eye(4), 1, 0.01*[1 1 1]);
            om = [0 0 pi];
            phxEngine('set', w, b, 'angvel', om);
            T = tc.lastPose(w, b, 0.25);
            tc.verifyEqual(T(1:3, 4), [0; 0; 0], "AbsTol", 1e-6);
            d = [0.3; 0; 0];
            c = T(1:3, 1:3)*d;
            phxEngine('set', w, b, 'com', d');
            v = cross(om, c');
            tc.verifyEqual(phxEngine('get', w, b, 'linvel'), v, "AbsTol", 1e-5);
            T = tc.lastPose(w, b, 0.5);
            tc.verifyEqual(T(1:3, 4), c + 0.5*v' - T(1:3, 1:3)*d, "AbsTol", 1e-4);
        end

        function jointHoldsWhenTheCentreMoves(tc)
            % The joints of a body keep their points on it when its centre
            % moves during the run
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            phxEngine('add', w, 'spherical', [a b], eye(4), tEngineProtocol.pose([-1 0 0]), false);
            tc.settle(w, 0.2);
            phxEngine('set', w, b, 'com', [0.4 0 0]);
            for k = 1:100
                [~, T] = phxEngine('step', w, tc.Dt);
                anchor = T(1:3, 1:3)*[-1; 0; 0] + T(1:3, 4);
                tc.assertEqual(anchor, [0; 0; 2], "AbsTol", 0.02);
            end
        end

        function kinematicBodyFollowsItsVelocity(tc)
            % A kinematic body moves with the velocity set to it
            w = tc.newWorld;
            k = tc.addBox(w, 1, [0 0 0], [1 1 0.2]);
            phxEngine('set', w, k, 'linvel', [1 0 0]);
            phxEngine('set', w, k, 'angvel', [0 0 0.5]);
            T = tc.lastPose(w, k, 1);
            tc.verifyEqual(T(1:3, 4), [1; 0; 0], "AbsTol", 1e-4);
            tc.verifyEqual(atan2(T(2, 1), T(1, 1)), 0.5, "AbsTol", 1e-4);
            tc.verifyEqual(phxEngine('get', w, k, 'linvel'), [1, 0, 0], "AbsTol", 1e-6);
        end

        function latestKinematicCommandHolds(tc)
            % A pose is reached in the next step, after which
            % the body stands still; a velocity holds until the next command;
            % a velocity set after a pose applies once the pose is reached
            w = tc.newWorld([0 0 0]);
            k = tc.addBox(w, 1, [0 0 0], [1 1 0.2]);
            dt = tc.Dt;

            % A velocity, then a pose before the step: the pose wins
            phxEngine('set', w, k, 'linvel', [0 1 0]);
            phxEngine('set', w, k, 'transform', tEngineProtocol.pose([1 0 0]));
            T = tc.lastPose(w, k, 0.5);
            tc.verifyEqual(T(1:3, 4), [1; 0; 0], "AbsTol", 1e-4, "The body stands at the pose.");
            tc.verifyEqual(phxEngine('get', w, k, 'linvel'), [0, 0, 0], "AbsTol", 1e-9);

            % A pose, then a velocity before the step: the pose, then the velocity
            phxEngine('set', w, k, 'transform', tEngineProtocol.pose([2 0 0]));
            phxEngine('set', w, k, 'linvel', [0 1 0]);
            T = tc.lastPose(w, k, 0.5 + dt);
            tc.verifyEqual(T(1:3, 4), [2; 0.5; 0], "AbsTol", 1e-4, "The body moves on from the pose.");

            % A velocity after a pose step: only the new velocity, nothing
            % left of the motion towards the pose
            phxEngine('set', w, k, 'transform', tEngineProtocol.pose([3 0 0]));
            tc.lastPose(w, k, dt);
            phxEngine('set', w, k, 'linvel', [0 0 1]);
            T = tc.lastPose(w, k, 0.5);
            tc.verifyEqual(T(1:3, 4), [3; 0; 0.5], "AbsTol", 1e-4);
        end

        function surfaceVelocityIsGivenInTheBodyFrame(tc)
            % A static belt turned by 90 deg about z drives along world y
            w = tc.newWorld;
            belt = phxEngine('add', w, 'box', 0, [5 5 0.5], ...
                makehgtform("translate", [0 0 -0.5], "zrotate", pi/2), 0, [0 0 0]);
            phxEngine('set', w, belt, 'friction', 1);
            phxEngine('set', w, belt, 'surfacevelocity', [1 0 0]);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phxEngine('set', w, h, 'friction', 1);
            tc.settle(w, 1);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 1, 0], "AbsTol", 0.05);
        end

        function frictionTakesFrictionRollingSpin(tc)
            % The friction is [friction rolling spin] (a coefficient and two
            % lengths) or the coefficient alone; the values are finite and
            % not negative
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phxEngine('set', w, h, 'friction', [0.5 0.001 0.005]);
            phxEngine('set', w, h, 'friction', 0.4);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyError(@() phxEngine('set', w, h, 'friction', [0.5 0.1]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, h, 'friction', [0.5 -0.001 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, h, 'friction', [0.5 0 NaN]), "phx:engine:badArgument");
        end

        function rollingResistanceSlowsABallByItsArm(tc)
            % A ball or a cylinder rolling freely meets the torque b N of the
            % rolling resistance b and slows by b g / (R (1 + I/(m R^2)))
            r = 0.1;
            b = 0.002;
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 b 0]), b*tc.G/(1.4*r), ...
                "RelTol", 0.01, "Ball");
            tc.verifyEqual(tc.rollingDeceleration("cylinder", [0.5 b 0]), b*tc.G/(1.5*r), ...
                "RelTol", 0.01, "Cylinder");
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 0 0]), 0, ...
                "AbsTol", 1e-3, "No rolling resistance");
        end

        function rollingResistanceIsTheSumOfBothBodies(tc)
            % The pair's rolling resistance is the sum of the two values,
            % whatever the friction coefficients
            expected = 0.002*tc.G/(1.4*0.1);
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 0.001 0], [1 0.001 0]), expected, ...
                "RelTol", 0.01, "Shared by ball and floor");
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 0 0], [1 0.002 0]), expected, ...
                "RelTol", 0.01, "Floor alone");
            tc.verifyEqual(tc.rollingDeceleration("sphere", [1 0.002 0], [0.3 0 0]), expected, ...
                "RelTol", 0.01, "Other friction coefficients");
        end

        function rollingResistanceHoldsForAnyMassAndStep(tc)
            % The limit follows the normal force, so a heavy ball and long
            % or short steps slow alike
            expected = 0.002*tc.G/(1.4*0.1);
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 0.002 0], [1 0 0], 100), expected, ...
                "RelTol", 0.01, "100 kg");
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 0.002 0], [1 0 0], 1, 0.001), expected, ...
                "RelTol", 0.01, "1 ms");
            tc.verifyEqual(tc.rollingDeceleration("sphere", [0.5 0.002 0], [1 0 0], 1, 0.02), expected, ...
                "RelTol", 0.01, "20 ms");
        end

        function rollingResistanceHasNoPreferredDirection(tc)
            % A ball rolling at an angle to the world axes slows as along
            % them and keeps its heading
            expected = 0.002*tc.G/(1.4*0.1);
            for azimuth = [30 45]
                [a, sideways] = tc.rollingDeceleration("sphere", [0.5 0.002 0], [1 0 0], 1, tc.Dt, azimuth);
                tc.verifyEqual(a, expected, "RelTol", 0.01, "Azimuth " + azimuth + " deg");
                tc.verifyLessThan(sideways, 1e-3, "Sideways speed (m/s) at azimuth " + azimuth + " deg");
            end
        end

        function spinRadiusStopsASpinningBall(tc)
            % A ball spinning about the vertical on a floor meets the twist
            % friction mu r N of the spin radius r (summed over the pair)
            % and slows by mu r m g / I; without friction nothing resists
            r = 0.1;
            I = 0.4*r^2;
            tc.verifyEqual(tc.spinDeceleration([0.5 0 0.01], [1 0 0]), 0.5*0.01*tc.G/I, ...
                "RelTol", 0.01, "Ball alone");
            tc.verifyEqual(tc.spinDeceleration([0.5 0 0.005], [0.5 0 0.005]), 0.25*0.01*tc.G/I, ...
                "RelTol", 0.01, "Shared by ball and floor");
            tc.verifyEqual(tc.spinDeceleration([0 0 0.01], [1 0 0]), 0, ...
                "AbsTol", 1e-3, "No friction");
            tc.verifyEqual(tc.spinDeceleration([0.5 0 0], [1 0 0]), 0, ...
                "AbsTol", 1e-3, "No spin radius");
        end

        function settingsAreOneStructOwnedByTheEngine(tc)
            % The engine settings go in one struct of any of their fields
            % and come back all together; an unknown one is refused.
            w = tc.newWorld;
            phxEngine('set', w, uint64(0), 'settings', struct('iterations', 7, 'margin', 0.003, 'contactcache', true));
            s = phxEngine('get', w, uint64(0), 'settings');
            tc.verifyEqual(s.iterations, 7);
            tc.verifyEqual(s.margin, 0.003, "AbsTol", 1e-9);
            tc.verifyTrue(s.contactcache);
            tc.verifyFalse(s.sleeping, "newWorld switches sleeping off.");
            tc.verifyEqual(sort(string(fieldnames(s)))', sort(["iterations" "positioniterations" "jointiterations" ...
                "springiterations" "margin" "penetrationslop" "speculativedistance" "contactcache" "threads" "sleeping" ...
                "jointtolerance" "contacttolerance" "tolerancelength" "diagnostics"]));
            tc.verifyError(@() phxEngine('set', w, uint64(0), 'settings', struct('nosuchsetting', 1)), ...
                "phx:engine:unsupported");
            tc.verifyError(@() phxEngine('set', w, uint64(0), 'numiterations', 7), "phx:engine:unsupported");
        end

        function totalForceIsTheResultantOfAllForces(tc)
            m = 2;
            % Falling freely: the weight plus what was applied
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 5], 0.2*[1 1 1], m);
            phxEngine('apply', w, h, 'centralforce', [3 0 0], false);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h, 'totalforce'), [3, 0, -m*tc.G], "AbsTol", 1e-4);

            % At rest on a floor: the contact cancels the weight
            w = tc.newWorld;
            tc.addFloor(w);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1], m);
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, h, 'totalforce'), [0, 0, 0], "AbsTol", 0.02*m*tc.G);
        end

        function totalTorqueIsTheResultantTorque(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phxEngine('apply', w, h, 'torque', [0.1 -0.2 0.3], false);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h, 'totaltorque'), [0.1, -0.2, 0.3], "AbsTol", 1e-6);
        end

        function totalTorqueOfAFreeTumblingBodyIsZero(tc)
            % The gyroscopic turning of a free asymmetric body (here about
            % 0.02 N m worth of w x Iw) is no torque
            s = [0.1 0.3 0.6];
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], s, 1, (sum(s.^2) - s.^2)/12);
            phxEngine('set', w, h, 'angvel', [0.2 3 0.2]);
            for k = 1:20
                [~] = phxEngine('step', w, tc.Dt);
                tc.verifyEqual(phxEngine('get', w, h, 'totaltorque'), [0, 0, 0], "AbsTol", 1e-5);
            end
        end

        function fallingBodyAcceleratesAtGravity(tc)
            % A free body reads the acceleration of gravity, no angular one;
            % the static floor and a body before its first step read none
            w = tc.newWorld;
            floor = tc.addFloor(w);
            h = tc.addBox(w, 2, [0 0 5], [0.2 0.2 0.2]);
            tc.verifyEqual(phxEngine('get', w, h, 'linacc'), [0, 0, 0], "Before the first step");
            for k = 1:3
                [~] = phxEngine('step', w, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', w, h, 'linacc'), [0, 0, -tc.G], "AbsTol", 1e-4);
            tc.verifyEqual(phxEngine('get', w, h, 'angacc'), [0, 0, 0], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, floor, 'linacc'), [0, 0, 0]);
        end

        function appliedTorqueGivesTheAngularAcceleration(tc)
            % A torque of 0.5 N m about the axis of the largest moment of
            % inertia of a box at rest: dw/dt = M / I
            s = [0.1 0.3 0.6];
            inertia = (sum(s.^2) - s.^2)/12;
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], s, 1, inertia);
            phxEngine('set', w, h, 'appliedtorque', [0.5 0 0]);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h, 'angacc'), [0.5/inertia(1), 0, 0], "RelTol", 1e-4);
            tc.verifyEqual(phxEngine('get', w, h, 'linacc'), [0, 0, 0], "AbsTol", 1e-6);
        end

        function kinematicBodyAcceleratesWithItsDrive(tc)
            % A kinematic body set to 1 m/s from rest accelerates by 1 m/s
            % in that step and not in the next one
            w = tc.newWorld([0 0 0]);
            k = tc.addBox(w, 1, [0 0 0], [1 1 0.2]);
            [~] = phxEngine('step', w, tc.Dt);
            phxEngine('set', w, k, 'linvel', [1 0 0]);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, k, 'linacc'), [1/tc.Dt, 0, 0], "RelTol", 1e-5);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, k, 'linacc'), [0, 0, 0], "AbsTol", 1e-4);
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
            tc.verifyEqual(phxEngine('get', w, support, 'totalforce'), [0, 0, -W], "AbsTol", 0.01*W);
            tc.verifyEqual(phxEngine('get', w, support, 'totaltorque'), [0, 0.3*W, 0], "AbsTol", 0.01*W);
        end

        function staticFloorReadsTheWeight(tc)
            % The same on a static plate
            w = tc.newWorld;
            floor = tc.addBox(w, 0, [0 0 0], [2 2 0.2]);
            tc.addBox(w, 2, [0.3 0 0.2], [0.2 0.2 0.2], 2);
            tc.settle(w, 1);
            W = 2*tEngineProtocol.G;
            tc.verifyEqual(phxEngine('get', w, floor, 'totalforce'), [0, 0, -W], "AbsTol", 0.01*W);
            tc.verifyEqual(phxEngine('get', w, floor, 'totaltorque'), [0, 0.3*W, 0], "AbsTol", 0.01*W);
        end

        function staticAnchorReadsTheHangingWeight(tc)
            % A 3 kg box hanging still from a static anchor by a point joint
            % 0.5 m above it: the anchor carries its weight
            w = tc.newWorld;
            anchor = tc.addBox(w, 0, [0 0 1], 0.1*[1 1 1]);
            box = tc.addBox(w, 2, [0 0 0.5], 0.2*[1 1 1], 3);
            phxEngine('add', w, 'spherical', [anchor box], eye(4), tEngineProtocol.pose([0 0 0.5]), false);
            tc.settle(w, 1);
            W = 3*tEngineProtocol.G;
            tc.verifyEqual(phxEngine('get', w, anchor, 'totalforce'), [0, 0, -W], "AbsTol", 0.01*W);
        end

        function appliedForcesActForOneStep(tc)
            w = tc.newWorld([0 0 0]);
            h = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            phxEngine('apply', w, h, 'centralforce', [1 0 0], false);
            [~] = phxEngine('step', w, 0.01);
            [~] = phxEngine('step', w, 0.01);
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0.01, 0, 0], "AbsTol", 1e-6);
        end

        function appliedBodyLoadsPersistUntilZero(tc)
            % Persistent loads in the body frame: a body turned +90 deg
            % about z (local x = world y) with 2 N along local x
            % accelerates at 1 m/s^2 along world y over several steps, and
            % keeps its speed once the force is set to zero; a torque of
            % 0.5 N*m about local z spins a second body up likewise
            w = tc.newWorld([0 0 0]);
            P = makehgtform("translate", [0 0 0], "zrotate", pi/2);
            h = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], P, 2, [1 1 1]);
            g = tc.addBox(w, 2, [3 0 0], 0.2*[1 1 1], 1, [0.5 0.5 0.5]);
            phxEngine('set', w, h, 'appliedforce', [2 0 0]);
            phxEngine('set', w, g, 'appliedtorque', [0 0 0.5]);
            tc.verifyEqual(phxEngine('get', w, h, 'appliedforce'), [2, 0, 0]);
            tc.verifyEqual(phxEngine('get', w, h, 'appliedtorque'), [0, 0, 0]);
            for k = 1:10
                [~] = phxEngine('step', w, 0.01);
            end
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 0.1, 0], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, g, 'angvel'), [0, 0, 0.1], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, h, 'totalforce'), [0, 2, 0], "AbsTol", 1e-4, ...
                "The load counts in the resultant.");
            phxEngine('set', w, h, 'appliedforce', [0 0 0]);
            phxEngine('set', w, g, 'appliedtorque', [0 0 0]);
            for k = 1:10
                [~] = phxEngine('step', w, 0.01);
            end
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), [0, 0.1, 0], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, g, 'angvel'), [0, 0, 0.1], "AbsTol", 1e-6);
            tc.verifyError(@() phxEngine('set', w, h, 'appliedforce', [NaN 0 0]), "phx:engine:badArgument");
        end

        function singleBodyForcesUseTheirFrames(tc)
            % Body turned +90 deg about z (local x = world y), unit mass and
            % inertia, far from the origin
            w = tc.newWorld([0 0 0]);
            P = makehgtform("translate", [10 0 0], "zrotate", pi/2);
            h = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], P, 1, [1 1 1]);
            io = @(varargin) phxEngine('apply', w, h, varargin{:});

            io('centralforce', [1 0 0], true);              % local x = world y
            io('force', [0 0 1], [1 0 0], false, true);     % at local x: lever along world y
            io('force', [0 0 1], [10 -1 0], false, false);  % at world point, lever along -y
            io('torque', [1 0 0], true);                    % about local x = world y
            [~] = phxEngine('step', w, 0.01);
            % Linear: (0,1,0) + (0,0,1) + (0,0,1); angular: (1,0,0) + (-1,0,0) + (0,1,0)
            tc.verifyEqual(phxEngine('get', w, h, 'linvel'), 0.01*[0, 1, 2], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, h, 'angvel'), 0.01*[0, 1, 0], "AbsTol", 1e-6);
        end

        function batchVerbsApplyPerBody(tc)
            w = tc.newWorld([0 0 0]);
            s = tc.addBox(w, 0, [0 5 0], 0.1*[1 1 1]);
            h1 = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([0 0 0]), 1, [1 1 1]);
            h2 = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([2 0 0]), 1, [1 1 1]);
            hs = [h1 s h2];  % a static body in a batch is harmless
            phxEngine('apply', w, hs, 'centralforce', [1 0 0, 5 5 5, 0 2 0], false);
            phxEngine('apply', w, hs, 'force', [0 0 1, 5 5 5, 0 0 1], [1 0 0, 0 0 0, 0 1 0], false, true);
            phxEngine('apply', w, hs, 'torque', [0 0 1, 5 5 5, 0 0 -1], false);
            [~] = phxEngine('step', w, 0.01);
            tc.verifyEqual(phxEngine('get', w, h1, 'linvel'), 0.01*[1, 0, 1], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, h2, 'linvel'), 0.01*[0, 2, 1], "AbsTol", 1e-6);
            % h1: lever (1,0,0) x (0,0,1) = (0,-1,0); h2: (0,1,0) x (0,0,1) = (1,0,0)
            tc.verifyEqual(phxEngine('get', w, h1, 'angvel'), 0.01*[0, -1, 1], "AbsTol", 1e-6);
            tc.verifyEqual(phxEngine('get', w, h2, 'angvel'), 0.01*[1, 0, -1], "AbsTol", 1e-6);
            % One vector per body, exactly; the bodies a row; the plural kinds
            % of protocol 1 are gone
            tc.verifyError(@() phxEngine('apply', w, hs, 'centralforce', [1 0 0, 5 5 5], false), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('apply', w, hs', 'centralforce', [1 0 0, 5 5 5, 0 2 0], false), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('apply', w, hs, 'centralforces', [1 0 0, 5 5 5, 0 2 0], false), "phx:engine:unsupported");
        end

        function raycastReportsTheClosestHit(tc)
            w = tc.newWorld([0 0 0]);
            near = tc.addBox(w, 0, [0 0 0], [1 1 1]);
            tc.addBox(w, 0, [0 0 -3], [1 1 1]);
            O = [0 0 5; 3 0 5]';
            E = [0 0 -5; 3 0 -5]';
            [p, n, d, ids] = phxEngine('raycast', w, O, E);
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
            tc.verifyEqual([p(:, 2); n(:, 2); d(2)], nan(7, 1), "A miss is NaN.");
            tc.verifyError(@() phxEngine('apply', w, uint64(0), 'raycast', O, E), "phx:engine:unsupported");
        end
    end

    % Joints
    methods (Test, TestTags = {'Engine'})
        function hingeHoldsItsAnchorAndReportsReactions(tc)
            m = 2;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1], m);
            j = phxEngine('add', w, 'revolute', [a b], tEngineProtocol.frame([0 0 0], [1 0 0]), tEngineProtocol.frame([0 0 1], [1 0 0]), false);
            tc.verifyClass(j, "uint64");
            T = tc.settle(w, 1);
            tc.verifyEqual(T(1:3, 4), [0; 0; 1], "AbsTol", 1e-3);
            % One 3x4 output [fA tA fB tB]
            F = phxEngine('get', w, j, 'feedback');
            tc.verifyClass(F, "double");
            tc.verifySize(F, [3 4]);
            tc.verifyEqual(F(:, 3)', [0, 0, m*tc.G], "AbsTol", 0.01*m*tc.G, "The joint holds body B up.");
            tc.verifyEqual(F(:, 1), -F(:, 3), "AbsTol", 0.01*m*tc.G, "Body A feels the opposite reaction.");
            % The reaction acts along the line of both centres of mass:
            % no torque about either
            tc.verifyEqual(F(:, [2 4]), zeros(3, 2), "AbsTol", 0.01*m*tc.G);
        end

        function jointKindsAreThePhxJoints(tc)
            % The kinds are named after the PHX joints; the constraint names
            % of protocol 1 are gone
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            kinds = ["fixed" "revolute" "prismatic" "spherical" "generic" "bushing"];
            for kind = kinds
                j = phxEngine('add', w, char(kind), [a b], eye(4), eye(4), false);
                tc.verifyClass(j, "uint64", kind);
                phxEngine('remove', w, j);
            end
            old = ["fixedconstraint" "hingeconstraint" "sliderconstraint" "pointtopointconstraint" ...
                "generic6dofconstraint" "generic6dofspringconstraint" "gearconstraint" "springconstraint" ...
                "ropeconstraint" "torsionspringconstraint" "conetwistconstraint"];
            for kind = old
                tc.verifyError(@() phxEngine('add', w, char(kind), [a b], eye(4), eye(4), false), ...
                    "phx:engine:unsupported", kind);
            end
            tc.verifyError(@() phxEngine('add', w, 'fixed', a, b, eye(4), eye(4), false), ...
                "phx:engine:badArgument", "The bodies come as one [hA hB] argument.");
        end

        function hingeMotorDrivesTheAxis(tc)
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            j = phxEngine('add', w, 'revolute', [a b], tEngineProtocol.frame([0 0 1], [1 0 0]), tEngineProtocol.frame([0 0 0], [1 0 0]), false);
            phxEngine('set', w, j, 'motor', true, 2, 100);
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, b, 'angvel'), [2, 0, 0], "AbsTol", 1e-3);
        end

        function lowDropBouncesToRestitutionSquared(tc)
            % A ball of restitution 0.8 dropped 2 cm onto a floor of
            % restitution 1 hits it at 0.63 m/s and bounces to e^2 h.
            % (Jolt bounces only impacts faster than 1 m/s by default.)
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'restitution', 1);
            r = 0.05;
            h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 r + 0.02]), 1, 0.4*r^2*[1 1 1]);
            phxEngine('set', w, h, 'restitution', 0.8);
            top = -inf;
            for k = 1:round(0.2/tc.Dt)   % the first bounce peaks at about 0.13 s
                [ids, P] = phxEngine('step', w, tc.Dt);
                if k*tc.Dt > 0.08 && any(ids == h)
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
            phxEngine('set', w, tc.addFloor(w), 'restitution', 0.9);
            boxes = zeros(1, 5, "uint64");
            for i = 1:5
                boxes(i) = tc.addBox(w, 2, [0 0 0.2*(i - 0.5)], 0.2*[1 1 1]);
                phxEngine('set', w, boxes(i), 'restitution', 0.9);
            end
            tc.settle(w, 1);
            speed = 0;
            for k = 1:round(1/tc.Dt)
                [~] = phxEngine('step', w, tc.Dt);
                for i = 1:5
                    speed = max(speed, norm(phxEngine('get', w, boxes(i), 'linvel')));
                end
            end
            tc.verifyLessThan(speed, 0.01, "Largest speed of the resting boxes (m/s)");
        end

        function stackOfTenStaysPut(tc)
            % Ten boxes of 0.2 m stacked on a floor stay where they are:
            % over 5 s after settling, the top box moves less than 0.05 mm.
            % The load of each face is statically indeterminate; it must not
            % drift from step to step. (The positions come from the steps
            % themselves: a step much shorter than the others rescales the
            % warm start of the next one and kicks the stack.)
            w = tc.newWorld;
            tc.addFloor(w);
            boxes = zeros(1, 10, "uint64");
            for i = 1:10
                boxes(i) = tc.addBox(w, 2, [0 0 0.2*(i - 0.5)], 0.2*[1 1 1]);
            end
            top = @(ids, T) T(1:3, 4, ids == boxes(end));
            for k = 1:round(1/tc.Dt)
                [ids, T] = phxEngine('step', w, tc.Dt);
            end
            p0 = top(ids, T);
            for k = 1:round(5/tc.Dt)
                [ids, T] = phxEngine('step', w, tc.Dt);
            end
            p1 = top(ids, T);
            tc.verifyLessThan(norm(p1(1:2) - p0(1:2)), 5e-5, "Horizontal drift of the top box in 5 s (m)");
        end

        function loadedChainHoldsItsLength(tc)
            % Ten 1 kg links of 0.2 m hung from an anchor by point joints,
            % the last one 100 kg: at 5 ms the chain stretches less than
            % 0.1 mm in 3 s. (Its joints swept one after another, the mass
            % ratio of 1:100 left it 3.4 mm long at the iteration cap.)
            w = tc.newWorld;
            s = [0.05 0.05 0.2];
            up = [0 0 0.1];
            anchor = tc.addBox(w, 0, [0 0 2], 0.05*[1 1 1]);
            links = zeros(1, 10, "uint64");
            for i = 1:10
                m = 1 + 99*(i == 10);
                links(i) = tc.addBox(w, 2, [0 0 2.1 - 0.2*i], s, m, m*(sum(s.^2) - s.^2)/12);
                if i == 1
                    phxEngine('add', w, 'spherical', [anchor links(i)], eye(4), [eye(3) up'; 0 0 0 1], false);
                else
                    phxEngine('add', w, 'spherical', [links(i - 1) links(i)], [eye(3) -up'; 0 0 0 1], [eye(3) up'; 0 0 0 1], false);
                end
            end
            for k = 1:round(3/tc.Dt)
                [ids, T] = phxEngine('step', w, tc.Dt);
            end
            last = T(:, :, ids == links(end));
            bottom = last(1:3, 4) - last(1:3, 1:3)*up';
            tc.verifyLessThan(norm(bottom - [0; 0; 2]) - 2, 1e-4, "Stretch of the chain (m)");
        end

        function tiltedRotorKeepsItsSpeed(tc)
            % A box rotor on a hinge, without gravity, its principal axes
            % tilted 10 deg to the hinge (dynamic imbalance), spinning at
            % 20 rad/s (w dt 0.1 at 5 ms): nothing acts about the axis, so
            % the speed stays within 1 % for 2 s. (With the gyroscopic step
            % applied before the joints' solve, which then projected the
            % angular velocity back onto the axis, it lost 2.6 %.)
            w = tc.newWorld([0 0 0]);
            s = [0.1 0.06 0.04];
            R0 = [cosd(10) 0 sind(10); 0 1 0; -sind(10) 0 cosd(10)];
            anchor = tc.addBox(w, 0, [0 0 0], 0.01*[1 1 1]);
            h = phxEngine('add', w, 'box', 2, s/2, [R0 [0; 0; 0]; 0 0 0 1], 1, (sum(s.^2) - s.^2)/12);
            phxEngine('add', w, 'revolute', [anchor h], eye(4), tEngineProtocol.frame([0 0 0], R0'*[0; 0; 1]), false);
            phxEngine('set', w, h, 'angvel', [0 0 20]);
            tc.settle(w, 2);
            tc.verifyEqual(phxEngine('get', w, h, 'angvel'), [0, 0, 20], "AbsTol", 0.2, "Angular velocity after 2 s (rad/s)");
        end

        function topKeepsItsPrecession(tc)
            % A spinning disc held by a point joint below its centre of
            % mass, its axis tilted 30 deg and started in steady precession:
            % the tilt stays within 0.5 deg for 5 s at 5 ms. (With the
            % gyroscopic step applied before the joint's solve the top sank
            % 17 deg.)
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
            h = phxEngine('add', w, 'cylinder', 2, rad, 2*hh, 'z', [R0 l*e3; 0 0 0 1], m, Ic);
            phxEngine('add', w, 'spherical', [anchor h], eye(4), tEngineProtocol.pose([0 0 -l]), false);
            om0 = W*[0; 0; 1] + (w3 - W*cosd(th))*e3;
            phxEngine('set', w, h, 'angvel', om0);
            phxEngine('set', w, h, 'linvel', cross(om0, l*e3));
            tilt = 0;
            for k = 1:round(5/tc.Dt)
                [ids, P] = phxEngine('step', w, tc.Dt);
                if any(ids == h)
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
                phxEngine('set', w, uint64(0), 'settings', struct('jointiterations', 10));
            catch
            end
            anchor = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            box = tc.addBox(w, 2, [0.5 0 0], [0.2 0.2 0.2], 10);
            j = phxEngine('add', w, 'revolute', [anchor box], eye(4), tEngineProtocol.pose([-0.5 0 0]), false);
            phxEngine('set', w, box, 'angvel', [0 0 2]);
            phxEngine('set', w, box, 'linvel', [0 1 0]);
            n = 400;
            T = zeros(3, n);
            for k = 1:n
                [~] = phxEngine('step', w, tc.Dt);
                F = phxEngine('get', w, j, 'feedback');
                T(:, k) = F(:, 2);
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
            % of its last target, rang by 5.6 mrad.)
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
                joints(i) = phxEngine('add', w, 'revolute', [previous links(i)], ...
                    tEngineProtocol.frame(pivotA, [0 1 0]), tEngineProtocol.frame([-L/2 0 0], [0 1 0]), false);
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
                    phxEngine('set', w, joints(i), 'motor', true, Kp*e(k, i), 1000);
                end
                [ids, P] = phxEngine('step', w, dt);
                for i = 1:n
                    R(:, :, i + 1) = P(1:3, 1:3, ids == links(i));
                end
            end
            last = e(1001:end, :);
            ringing = rms(last - movmean(last, 101), "all");
            tc.verifyLessThan(ringing, 1e-4, "Ringing of the joints (rad) about their smooth tracking error");
            tc.verifyLessThan(max(abs(last), [], "all"), 0.01, "Tracking error (rad)");
        end

        function prismaticSlidesAlongTheZAxisOfItsFrames(tc)
            % Frames turned so that their Z is world -z: the body slides
            % down under gravity and nowhere else; the motor drives along Z
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0.5 0 2], 0.2*[1 1 1]);
            R = diag([1 -1 -1]);
            j = phxEngine('add', w, 'prismatic', [a b], [R [0.5; 0; 0]; 0 0 0 1], [R [0; 0; 0]; 0 0 0 1], false);
            T = tc.settle(w, 0.3);
            tc.verifyEqual(T(1:2, 4), [0.5; 0], "AbsTol", 1e-3);
            tc.verifyLessThan(T(3, 4), 1.8);
            phxEngine('set', w, j, 'motor', true, 0.5, 1000);
            tc.settle(w, 0.3);
            tc.verifyEqual(phxEngine('get', w, b, 'linvel'), [0, 0, -0.5], "AbsTol", 1e-3);
        end

        function revoluteMotorWorksAgainstAnAppliedTorque(tc)
            % A rotor of inertia 0.5 about z, no gravity: its motor holds
            % 2 rad/s against an applied torque of -1 N*m, which stays in
            % force and decelerates the rotor at 2 rad/s^2 once the motor is
            % off; the feedback carries the applied torque, B getting it, A
            % the opposite
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1], 1, [0.5 0.5 0.5]);
            j = phxEngine('add', w, 'revolute', [a b], tEngineProtocol.pose([0 0 1]), eye(4), false);
            phxEngine('set', w, j, 'motor', true, 2, 50);
            phxEngine('set', w, j, 'appliedtorque', -1);
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, b, 'angvel'), [0, 0, 2], "AbsTol", 1e-3, ...
                "The motor holds its speed against the applied torque.");
            F = phxEngine('get', w, j, 'feedback');
            tc.verifyEqual(F(3, 4), 0, "AbsTol", 1e-3, "Motor and applied torque balance on body B.");
            phxEngine('set', w, j, 'motor', false, 2, 50);
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, b, 'angvel'), [0, 0, 2 - 2*0.5], "AbsTol", 1e-3, ...
                "The applied torque acts alone once the motor is off.");
            F = phxEngine('get', w, j, 'feedback');
            tc.verifyEqual(F(:, 4)', [0, 0, -1], "AbsTol", 1e-3, "The applied torque on body B.");
            tc.verifyEqual(F(:, 2)', [0, 0, 1], "AbsTol", 1e-3, "Its reaction on body A.");
            phxEngine('set', w, j, 'appliedtorque', 0);
            v = phxEngine('get', w, b, 'angvel');
            tc.settle(w, 0.2);
            tc.verifyEqual(phxEngine('get', w, b, 'angvel'), v, "AbsTol", 1e-6, "Set to 0, it stops.");
        end

        function prismaticAppliedForceActsAlongTheAxis(tc)
            % 3 N along the slide axis (world x) on a 2 kg slider, no
            % gravity: 1.5 m/s^2, the opposite force on the free body A
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 4);
            b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1], 2);
            X = tEngineProtocol.frame([0 0 0], [1 0 0]);
            j = phxEngine('add', w, 'prismatic', [a b], X, X, false);
            phxEngine('set', w, j, 'appliedforce', 3);
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, b, 'linvel'), [0.75, 0, 0], "AbsTol", 1e-4);
            tc.verifyEqual(phxEngine('get', w, a, 'linvel'), [-0.375, 0, 0], "AbsTol", 1e-4);
            tc.verifyEqual(phxEngine('get', w, b, 'angvel'), [0, 0, 0], "AbsTol", 1e-6, ...
                "The force passes through the joint point and turns nothing.");
        end

        function fixedJointHoldsThePose(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            phxEngine('add', w, 'fixed', [a b], tEngineProtocol.pose([0.5 0 0]), tEngineProtocol.pose([-0.5 0 0]), false);
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
            phxEngine('add', w, 'revolute', [a b], tEngineProtocol.frame([0 0 0], [1 0 0]), tEngineProtocol.frame([0 -sind(60) cosd(60)], [1 0 0]), false);
            n = round(10/tc.Dt);
            E = zeros(1, n);
            for k = 1:n
                [ids, P] = phxEngine('step', w, tc.Dt);
                T = P(:, :, ids == b);
                v = phxEngine('get', w, b, 'linvel');
                om = phxEngine('get', w, b, 'angvel');
                E(k) = 0.5*m*(v*v') + 0.5*om*diag(I)*om' + m*g*T(3, 4);
            end
            window = round(3/tc.Dt);
            tc.verifyEqual(mean(E(end-window+1:end)), mean(E(1:window)), "RelTol", 0.01);
        end

        function pointJointHoldsItsAnchor(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            phxEngine('add', w, 'spherical', [a b], eye(4), tEngineProtocol.pose([-1 0 0]), false);
            lowest = 2;
            for k = 1:200
                [~, T] = phxEngine('step', w, tc.Dt);
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
            j = phxEngine('add', w, 'generic', [a b], tEngineProtocol.pose([0.3 0 -1]), eye(4), false);
            phxEngine('set', w, j, 'linlimits', [0 0 1], [0 0 -1]);
            phxEngine('set', w, j, 'anglimits', [0 0 0], [0 0 0]);
            phxEngine('set', w, b, 'linvel', [1 1 0]);
            phxEngine('set', w, b, 'angvel', [1 1 1]);
            T = tc.settle(w, 0.5);
            % The locked axes absorb the initial kick with a small error
            tc.verifyEqual(T(1:2, 4), [0.3; 0], "AbsTol", 5e-3);
            tc.verifyLessThan(T(3, 4), 0, "Body B falls along the free axis.");
            tc.verifyEqual(T(1:3, 1:3), eye(3), "AbsTol", 5e-3);
        end

        function genericJointLimitsChangeWhileRunning(tc)
            % The limits of a running joint take effect at once: locked, the
            % body hangs at the joint; bounded to [-0.3 0] along z, it drops
            % onto the lower stop; freed, it falls away
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            j = phxEngine('add', w, 'generic', [a b], tEngineProtocol.pose([0 0 -1]), eye(4), false);
            phxEngine('set', w, j, 'anglimits', [0 0 0], [0 0 0]);
            phxEngine('set', w, j, 'linlimits', [0 0 0], [0 0 0]);
            tc.settle(w, 0.2);
            tc.verifyEqual(tEngineProtocol.positionOf(w, b)', [0 0 1], "AbsTol", 1e-3, "Locked");
            phxEngine('set', w, j, 'linlimits', [0 0 -0.3], [0 0 0]);
            tc.settle(w, 1);
            tc.verifyEqual(tEngineProtocol.positionOf(w, b)', [0 0 0.7], "AbsTol", 2e-3, "On the lower stop");
            phxEngine('set', w, j, 'linlimits', [0 0 1], [0 0 -1]);
            tc.settle(w, 0.5);
            p = tEngineProtocol.positionOf(w, b);
            tc.verifyLessThan(p(3), 0, "Freed, it falls away.");
        end

        function genericAngularLimitsAreRightHanded(tc)
            % A rotation bounded to [0.1 0.3] rad about each axis of the
            % frames in turn, driven by a persistent torque about that axis
            % in world (= frame) axes, no gravity: turned positively it
            % stops at +0.3, negatively at +0.1
            angleOf = {@(R) atan2(R(3, 2), R(2, 2)), @(R) atan2(R(1, 3), R(3, 3)), @(R) atan2(R(2, 1), R(1, 1))};
            for ax = 1:3
                for s = [1 -1]
                    w = tc.newWorld([0 0 0]);
                    a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
                    b = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [1 1 1]);
                    j = phxEngine('add', w, 'generic', [a b], eye(4), eye(4), false);
                    phxEngine('set', w, j, 'linlimits', [0 0 0], [0 0 0]);
                    lower = [0 0 0];
                    upper = [0 0 0];
                    lower(ax) = 0.1;
                    upper(ax) = 0.3;
                    phxEngine('set', w, j, 'anglimits', lower, upper);
                    torque = [0 0 0];
                    torque(ax) = 5*s;
                    phxEngine('set', w, b, 'appliedtorque', torque);
                    T = tc.lastPose(w, b, 1);
                    expected = 0.2 + 0.1*s;
                    tc.verifyEqual(angleOf{ax}(T(1:3, 1:3)), expected, "AbsTol", 0.01, ...
                        sprintf("Axis %d turned %+d", ax, s));
                end
            end
        end

        function revoluteLimitsBoundTheAngleRightHanded(tc)
            % A hinge about z bounded to [0.1 0.3] rad, a range without the
            % zero, driven by a persistent torque about z, no gravity:
            % turned positively it stops at +0.3, negatively at +0.1
            for s = [1 -1]
                [w, b, j] = tc.hingeRig;
                phxEngine('set', w, j, 'anglimits', 0.1, 0.3);
                phxEngine('set', w, b, 'appliedtorque', [0 0 5*s]);
                tc.settle(w, 1);
                tc.verifyEqual(tEngineProtocol.zAngleOf(w, b), 0.2 + 0.1*s, "AbsTol", 0.01, ...
                    sprintf("Turned %+d", s));
            end
        end

        function revoluteLimitsLockFreeAndChangeWhileRunning(tc)
            % Equal limits lock the hinge at that angle against the torque;
            % lower > upper and infinite limits free it; a limit clamped to
            % pi with the other one inside stops it at the back point
            [w, b, j] = tc.hingeRig;
            phxEngine('set', w, b, 'appliedtorque', [0 0 5]);
            phxEngine('set', w, j, 'anglimits', 0.5, 0.5);
            tc.settle(w, 1);
            tc.verifyEqual(tEngineProtocol.zAngleOf(w, b), 0.5, "AbsTol", 0.01, "Locked at 0.5");
            for free = {[1 -1], [-Inf Inf]}
                phxEngine('set', w, j, 'anglimits', 0.5, 0.5);
                tc.settle(w, 0.2);
                phxEngine('set', w, j, 'anglimits', free{1}(1), free{1}(2));
                tc.settle(w, 0.4);
                om = phxEngine('get', w, b, 'angvel');
                tc.verifyEqual(om(3), 5*0.4, "RelTol", 0.05, "Freed by " + mat2str(free{1}));
            end
            [w, b, j] = tc.hingeRig;
            phxEngine('set', w, j, 'anglimits', -Inf, 0.5);
            phxEngine('set', w, b, 'appliedtorque', [0 0 -5]);
            tc.settle(w, 2);
            tc.verifyEqual(abs(tEngineProtocol.zAngleOf(w, b)), pi, "AbsTol", 0.02, "Stopped at the back point");
        end

        function prismaticLimitsBoundTheTravel(tc)
            % A slider along world z hanging under gravity, bounded to
            % [-0.3 -0.1] m (a range without the zero): it drops onto the
            % lower stop, which carries its weight; locked at -0.2 it holds
            % there; with no lower stop it falls away
            m = 2;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1], m);
            j = phxEngine('add', w, 'prismatic', [a b], tEngineProtocol.pose([0 0 -1]), eye(4), false);
            phxEngine('set', w, j, 'linlimits', -0.3, -0.1);
            tc.settle(w, 1);
            tc.verifyEqual(tc.meanForceB(w, j), [0 0 m*tc.G], "AbsTol", 0.02*m*tc.G, "The stop carries the weight.");
            tc.verifyEqual(tEngineProtocol.positionOf(w, b)', [0 0 0.7], "AbsTol", 2e-3, "On the lower stop");
            phxEngine('set', w, j, 'linlimits', -0.2, -0.2);
            tc.settle(w, 1);
            tc.verifyEqual(tEngineProtocol.positionOf(w, b)', [0 0 0.8], "AbsTol", 2e-3, "Locked at -0.2");
            phxEngine('set', w, j, 'linlimits', -Inf, 0);
            tc.settle(w, 0.5);
            p = tEngineProtocol.positionOf(w, b);
            tc.verifyLessThan(p(3), 0, "No lower stop, it falls away.");
        end

        function jointLimitsRejectNaNAndOtherKinds(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            hinge = phxEngine('add', w, 'revolute', [a b], eye(4), eye(4), false);
            slider = phxEngine('add', w, 'prismatic', [a b], eye(4), eye(4), false);
            generic = phxEngine('add', w, 'generic', [a b], eye(4), eye(4), false);
            tc.verifyError(@() phxEngine('set', w, hinge, 'anglimits', NaN, 1), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, slider, 'linlimits', 0, NaN), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, generic, 'linlimits', [0 NaN 0], [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, generic, 'anglimits', [0 0 0], [NaN 0 0]), "phx:engine:badArgument");
            % Each joint has the limits of its own axis only
            tc.verifyError(@() phxEngine('set', w, hinge, 'linlimits', 0, 1), "phx:engine:unsupported");
            tc.verifyError(@() phxEngine('set', w, slider, 'anglimits', 0, 1), "phx:engine:unsupported");
            ball = phxEngine('add', w, 'spherical', [a b], eye(4), eye(4), false);
            tc.verifyError(@() phxEngine('set', w, ball, 'linlimits', 0, 1), "phx:engine:unsupported");
            tc.verifyError(@() phxEngine('set', w, hinge, 'cone', [1 1]), "phx:engine:unsupported");
        end

        function sphericalConeBoundsTheSwingElliptically(tc)
            % An arm 1 m long held horizontally along x by a ball joint whose
            % frame A has its Z axis along the arm, under gravity: the arm
            % swings down until the cone stops it. With the cone [0.6 0.3]
            % the stop is at 0.3 rad when the fall turns the arm about Y of
            % frame A and at 0.6 rad when it turns it about X; a cone of
            % [Inf Inf] frees it again
            RA = {[0 0 1; 0 -1 0; 1 0 0], 0.3    % X_A up: the fall turns about Y_A
                  [0 0 1; 1 0 0; 0 1 0], 0.6};   % Y_A up: the fall turns about X_A
            for k = 1:2
                w = tc.newWorld;
                a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
                b = tc.addBox(w, 2, [1 0 0], 0.1*[1 1 1]);
                TA = eye(4); TA(1:3, 1:3) = RA{k, 1};
                TB = TA; TB(1:3, 4) = [-1 0 0];
                j = phxEngine('add', w, 'spherical', [a b], TA, TB, false);
                phxEngine('set', w, j, 'cone', [0.6 0.3]);
                tc.settle(w, 2);
                tc.verifyEqual(tEngineProtocol.tiltOf(w, b), RA{k, 2}, "AbsTol", 0.01, sprintf("Frame %d", k));
                phxEngine('set', w, j, 'cone', [Inf Inf]);
                tc.settle(w, 0.3);
                tc.verifyGreaterThan(tEngineProtocol.tiltOf(w, b), RA{k, 2} + 0.2, sprintf("Freed, frame %d", k));
            end
        end

        function sphericalTwistLimitsBoundTheTwist(tc)
            % A ball joint at the centre of a free body of unit inertia, its
            % swing locked by a cone of [0 0], no gravity, turned about z by
            % a persistent torque: the twist limits [0.1 0.3] stop it at
            % +0.3 or +0.1 as for a hinge; lower > upper frees the twist
            for s = [1 -1]
                w = tc.newWorld([0 0 0]);
                a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
                b = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [1 1 1]);
                j = phxEngine('add', w, 'spherical', [a b], eye(4), eye(4), false);
                phxEngine('set', w, j, 'cone', [0 0]);
                phxEngine('set', w, j, 'anglimits', 0.1, 0.3);
                phxEngine('set', w, b, 'appliedtorque', [0 0 5*s]);
                tc.settle(w, 1);
                tc.verifyEqual(tEngineProtocol.zAngleOf(w, b), 0.2 + 0.1*s, "AbsTol", 0.01, sprintf("Turned %+d", s));
                phxEngine('set', w, j, 'anglimits', 1, -1);
                tc.settle(w, 0.4);
                om = phxEngine('get', w, b, 'angvel');
                tc.verifyEqual(om, [0 0 5*s*0.4], "AbsTol", 0.1, sprintf("Freed, turned %+d", s));
            end
        end

        function sphericalLimitsRejectBadValues(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            j = phxEngine('add', w, 'spherical', [a b], eye(4), eye(4), false);
            for bad = {[NaN 0.5], [-0.1 0.5], 0.5, [0.1 0.2 0.3]}
                tc.verifyError(@() phxEngine('set', w, j, 'cone', bad{1}), "phx:engine:badArgument", mat2str(bad{1}));
            end
            tc.verifyError(@() phxEngine('set', w, j, 'anglimits', NaN, 0), "phx:engine:badArgument");
        end

        function cylindricalLeavesOnlySlideAndTurnFree(tc)
            % A body on a cylindrical joint along world z, no gravity, pushed
            % along and turned about the axis and loaded across it: it moves
            % and turns along the axis only
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [1 1 1]);
            phxEngine('add', w, 'cylindrical', [a b], eye(4), eye(4), false);
            phxEngine('set', w, b, 'appliedforce', [3 -2 1]);
            phxEngine('set', w, b, 'appliedtorque', [2 1 1]);
            T = tc.settle(w, 1);
            tc.verifyEqual(T(1:2, 4)', [0 0], "AbsTol", 1e-4, "Stays on the axis");
            tc.verifyEqual(T(3, 4), 0.5, "RelTol", 0.02, "Slides by F t^2/(2 m)");
            tc.verifyEqual(T(1:3, 3)', [0 0 1], "AbsTol", 1e-4, "The axis stays put");
            tc.verifyEqual(atan2(T(2, 1), T(1, 1)), 0.5, "AbsTol", 0.01, "Turns by M t^2/(2 I)");
        end

        function cylindricalLimitsBoundTravelAndAngle(tc)
            % Hanging along z on [-0.3 -0.1] m and turned on [0.1 0.3] rad by
            % a torque about z: the lower stop carries the weight, the angle
            % stops at whichever limit the torque drives it against; equal
            % limits lock both
            m = 2;
            for s = [1 -1]
                w = tc.newWorld;
                a = tc.addBox(w, 0, [0 0 2], 0.1*[1 1 1]);
                b = tc.addBox(w, 2, [0 0 1], 0.1*[1 1 1], m, [1 1 1]);
                j = phxEngine('add', w, 'cylindrical', [a b], tEngineProtocol.pose([0 0 -1]), eye(4), false);
                phxEngine('set', w, j, 'linlimits', -0.3, -0.1);
                phxEngine('set', w, j, 'anglimits', 0.1, 0.3);
                phxEngine('set', w, b, 'appliedtorque', [0 0 5*s]);
                T = tc.settle(w, 1);
                tc.verifyEqual(T(3, 4), 0.7, "AbsTol", 2e-3, "On the lower stop");
                tc.verifyEqual(atan2(T(2, 1), T(1, 1)), 0.2 + 0.1*s, "AbsTol", 0.01, sprintf("Turned %+d", s));
                F = tc.meanForceB(w, j);
                tc.verifyEqual(F(3), m*tc.G, "AbsTol", 0.02*m*tc.G, "The stop carries the weight");
            end
            phxEngine('set', w, j, 'linlimits', -0.2, -0.2);
            phxEngine('set', w, j, 'anglimits', 0.2, 0.2);
            T = tc.settle(w, 1);
            tc.verifyEqual(T(3, 4), 0.8, "AbsTol", 2e-3, "Locked travel");
            tc.verifyEqual(atan2(T(2, 1), T(1, 1)), 0.2, "AbsTol", 0.01, "Locked angle");
            tc.verifyError(@() phxEngine('set', w, j, 'linlimits', NaN, 0), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, j, 'cone', [1 1]), "phx:engine:unsupported");
        end

        function cylindricalChainSwingsLikeAHingeChain(tc)
            % A chain of 20 links hanging from 30 deg on cylindrical joints
            % across the swing plane keeps its joints closed and its energy
            % as the same chain on revolute joints does, and slides nowhere
            dt = 0.005; L = 0.2; n = 20; m = 0.1;
            E = zeros(1, 2); gap = zeros(1, 2); spread = zeros(1, 2);
            kinds = ["revolute" "cylindrical"];
            for k = 1:2
                w = tc.newWorld;
                phxEngine('set', w, uint64(0), 'settings', struct('diagnostics', true));
                half = [0.02 0.02 L/2];
                inertia = m*(sum((2*half).^2) - (2*half).^2)/12;
                R = [cos(pi/6) 0 sin(pi/6); 0 1 0; -sin(pi/6) 0 cos(pi/6)];
                Ry = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1];
                prev = tc.addBox(w, 0, [0 0 5], 0.02*[1 1 1]);
                TA = [R zeros(3, 1); 0 0 0 1]*Ry;
                bodies = zeros(1, n, 'uint64');
                for i = 1:n
                    c = [0 0 5] + ((i - 0.5)*L*R*[0; 0; -1])';
                    bodies(i) = phxEngine('add', w, 'box', 2, half, [R c'; 0 0 0 1], m, inertia);
                    phxEngine('add', w, char(kinds(k)), [prev bodies(i)], TA, [eye(3) [0; 0; L/2]; 0 0 0 1]*Ry, false);
                    prev = bodies(i);
                    TA = [eye(3) [0; 0; -L/2]; 0 0 0 1]*Ry;
                end
                E0 = tEngineProtocol.chainEnergy(w, bodies, m, L);
                phxEngine('get', w, uint64(0), 'diagnostics');
                for s = 1:1000
                    phxEngine('step', w, dt);
                end
                D = phxEngine('get', w, uint64(0), 'diagnostics');
                gap(k) = D.jointgap;
                E(k) = tEngineProtocol.chainEnergy(w, bodies, m, L)/E0 - 1;
                P = phxEngine('get', w, bodies, 'position');
                spread(k) = max(P(2, :)) - min(P(2, :));
            end
            tc.verifyLessThan(gap(2), 1e-4, "Joint gap of the cylindrical chain");
            tc.verifyEqual(E(2), E(1), "AbsTol", 0.01, "Energy kept as the hinge chain keeps it");
            tc.verifyLessThan(spread(2), 1e-6, "The cylindrical chain slid along its joint axes");
        end

        function springJointDeflectsByWeightOverStiffness(tc)
            m = 1;
            k = 1000;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1], m);
            j = phxEngine('add', w, 'bushing', [a b], tEngineProtocol.pose([0 0 -1]), eye(4), false);
            phxEngine('set', w, j, 'springs', [0 0 k], [0 0 0], [0 0 20], [0 0 0]);
            phxEngine('set', w, j, 'linlimits', [0 0 1], [0 0 -1]);
            phxEngine('set', w, j, 'anglimits', [0 0 0], [0 0 0]);
            T = tc.settle(w, 3);
            tc.verifyEqual(T(3, 4), 1 - m*tc.G/k, "AbsTol", 0.05*m*tc.G/k);
            % A stiffer spring, set while running, halves the deflection
            phxEngine('set', w, j, 'springs', [0 0 2*k], [0 0 0], [0 0 40], [0 0 0]);
            T = tc.settle(w, 3);
            tc.verifyEqual(T(3, 4), 1 - m*tc.G/(2*k), "AbsTol", 0.05*m*tc.G/k);
            tc.verifyError(@() phxEngine('set', w, j, 'springs', [0 0 -1], [0 0 0], [0 0 0], [0 0 0]), ...
                "phx:engine:badArgument");
        end

        function gearCouplesTheAngularVelocities(tc)
            % omegaA.axisA + ratio*omegaB.axisB = 0
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1]);
            phxEngine('add', w, 'gear', [a b], eye(4), eye(4), false, 2);
            phxEngine('set', w, a, 'angvel', [0 0 1]);
            tc.settle(w, 0.25);
            wa = phxEngine('get', w, a, 'angvel');
            wb = phxEngine('get', w, b, 'angvel');
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
                phxEngine('add', w, 'gear', [a b], eye(4), eye(4), false, ratio);
                for k = 1:n
                    phxEngine('apply', w, a, 'torque', [0 0 torque], false);
                    [~] = phxEngine('step', w, tc.Dt);
                end
                wa = torque/(IA + IB/ratio^2)*n*tc.Dt;
                tc.verifyEqual(phxEngine('get', w, a, 'angvel'), [0, 0, wa], "AbsTol", 0.01*wa, ...
                    "Body A, ratio " + ratio);
                tc.verifyEqual(phxEngine('get', w, b, 'angvel'), [0, 0, -wa/ratio], "AbsTol", 0.01*wa, ...
                    "Body B, ratio " + ratio);
            end
        end

        function mutualCollisionsFollowTheFlag(tc)
            % Two overlapping boxes on a point joint: with mutual collisions
            % off (false) they stay put, with them on (true) the contact
            % pushes them apart
            d = zeros(1, 2);
            flags = [false true];
            for i = 1:2
                w = tc.newWorld([0 0 0]);
                a = tc.addBox(w, 2, [0 0 0], 0.4*[1 1 1]);
                b = tc.addBox(w, 2, [0.1 0 0], 0.4*[1 1 1]);
                phxEngine('add', w, 'spherical', [a b], tEngineProtocol.pose([0.05 0 0]), tEngineProtocol.pose([-0.05 0 0]), flags(i));
                tc.settle(w, 0.25);
                pa = tEngineProtocol.positionOf(w, a);
                pb = tEngineProtocol.positionOf(w, b);
                d(i) = norm(pa - pb);
            end
            tc.verifyEqual(d(1), 0.1, "AbsTol", 1e-3);
            tc.verifyGreaterThan(d(2), 0.2);
        end
    end

    % Soft bodies
    methods (Test, TestTags = {'Engine'})
        function softBodyRestsOnTheFloorKeepingItsShape(tc)
            % A closed cube mesh dropped as a soft body comes to rest on the
            % floor, its lowest vertex at z = 0 and its sides upright. Its
            % vertices are read back in world coordinates in the order given;
            % the step output reports rigid bodies only, and a soft body is
            % removed as a body is.
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            w = tc.newWorld;
            tc.addFloor(w);
            h = phxEngine('add', w, 'softbody', V, F, tEngineProtocol.pose([0 0 1]), 1, 0);
            tc.verifyClass(h, "uint64");
            tc.verifyEqual(phxEngine('get', w, h, 'vertices'), V + [0; 0; 1], "AbsTol", 1e-6);
            for k = 1:round(2/tc.Dt)
                ids = phxEngine('step', w, tc.Dt);
            end
            tc.verifyEmpty(ids, "The soft body is not among the moved bodies.");
            X = phxEngine('get', w, h, 'vertices');
            tc.verifySize(X, [3 8]);
            tc.verifyEqual(min(X(3, :)), 0, "AbsTol", 2e-3, "The cube rests on the floor.");
            tc.verifyEqual(X(:, 5:8) - X(:, 1:4), repmat([0; 0; 0.4], 1, 4), "AbsTol", 2e-3, ...
                "The cube keeps its shape.");
            phxEngine('remove', w, h);
            tc.verifyError(@() phxEngine('get', w, h, 'vertices'), ?MException);
        end
    end

    % Particle groups
    methods (Test, TestTags = {'Engine'})
        function particlesRestOnTheFloorUnreportedByStep(tc)
            % Particles dropped on the floor are read back as 3xN in the
            % order given and come to rest touching it; the step reports
            % only the rigid body beside them.
            r = 0.05;
            P = [(-2:2)*0.3; zeros(1, 5); 0.5*ones(1, 5)];
            w = tc.newWorld;
            tc.addFloor(w);
            b = tc.addBox(w, 2, [0 1 0.5], 0.2*[1 1 1]);
            g = phxEngine('add', w, 'particles', P, r, 0.1, false);
            tc.verifyClass(g, "uint64");
            tc.verifyEqual(phxEngine('get', w, g, 'count'), 5);
            tc.verifyEqual(phxEngine('get', w, g, 'positions'), P, "AbsTol", 1e-6);
            for k = 1:round(1/tc.Dt)
                ids = phxEngine('step', w, tc.Dt);
                tc.assertEqual(ids, b, "Only the rigid body is reported.");
            end
            X = phxEngine('get', w, g, 'positions');
            tc.verifySize(X, [3 5]);
            tc.verifyEqual(X(1:2, :), P(1:2, :), "AbsTol", 1e-3);
            tc.verifyEqual(X(3, :), r*ones(1, 5), "AbsTol", 2e-3, "The particles rest on the floor.");
            tc.verifyEqual(phxEngine('get', w, g, 'velocities'), zeros(3, 5), "AbsTol", 1e-2);
            phxEngine('remove', w, g);
            tc.verifyError(@() phxEngine('get', w, g, 'positions'), "phx:engine:badHandle");
        end

        function particlesKeepTheirOrder(tc)
            % Removing particles keeps the order of the rest, appended ones
            % come last, and a subset is set by one-based indices.
            w = tc.newWorld([0 0 0]);
            P = [1; 0; 0] * (1:5);
            g = phxEngine('add', w, 'particles', P, 0.1, 1, false);
            phxEngine('remove', w, g, [2 4]);
            tc.verifyEqual(phxEngine('get', w, g, 'positions'), P(:, [1 3 5]), "AbsTol", 1e-6);
            n = phxEngine('add', w, g, [0 0; 2 3; 0 0], [0 0; 0 0; 1 2]);
            tc.verifyEqual(n, 5);
            tc.verifyEqual(phxEngine('get', w, g, 'velocities'), [zeros(3, 3), [0; 0; 1], [0; 0; 2]], "AbsTol", 1e-6);
            phxEngine('set', w, g, 'positions', [0; 0; 7], 5);
            X = phxEngine('get', w, g, 'positions');
            tc.verifyEqual(X(:, [1 5]), [1 0; 0 0; 0 7], "AbsTol", 1e-6);
            tc.verifyError(@() phxEngine('remove', w, g, 6), "phx:engine:badArgument");
        end

        function particleForcesActForOneStep(tc)
            % A force for all (3x1) or one per particle (3xN) changes the
            % velocity by F/m dt in the next step only.
            w = tc.newWorld([0 0 0]);
            m = 0.5;
            g = phxEngine('add', w, 'particles', [0 1; 0 0; 0 0], 0.1, m, false);
            phxEngine('apply', w, g, 'centralforce', [2; 0; 0], false);
            phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, g, 'velocities'), [1 1; 0 0; 0 0]*2/m*tc.Dt, "AbsTol", 1e-6);
            phxEngine('apply', w, g, 'centralforce', [0 0; 1 -1; 0 0], false);
            phxEngine('step', w, tc.Dt);
            phxEngine('step', w, tc.Dt);
            V = phxEngine('get', w, g, 'velocities');
            tc.verifyEqual(V(2, :), [1 -1]/m*tc.Dt, "AbsTol", 1e-6, "The force acts in one step only.");
        end

        function particleSelfCollisionsCanBeSwitchedOff(tc)
            % Two particles dropped one above the other stack; without
            % self-collisions the upper one passes through to the floor.
            r = 0.05;
            heights = zeros(2, 2);
            for on = [true false]
                w = tc.newWorld;
                tc.addFloor(w);
                g = phxEngine('add', w, 'particles', [0 0; 0 0; r 4*r], r, 0.1, false);
                phxEngine('set', w, g, 'selfcollisions', on);
                for k = 1:round(1/tc.Dt)
                    phxEngine('step', w, tc.Dt);
                end
                X = phxEngine('get', w, g, 'positions');
                heights(:, 2 - on) = X(3, :)';
            end
            tc.verifyEqual(heights(:, 1), [r; 3*r], "AbsTol", 3e-3, "With self-collisions they stack.");
            tc.verifyEqual(heights(:, 2), [r; r], "AbsTol", 3e-3, "Without, both rest on the floor.");
        end

        function particlesRollOnlyWithRotation(tc)
            % A particle sliding over the floor starts rolling (w = v/r about
            % y) only when its group rotates.
            r = 0.05;
            w = tc.newWorld;
            tc.addFloor(w);
            sliding = phxEngine('add', w, 'particles', [0; 0; r], r, 0.1, false);
            rolling = phxEngine('add', w, 'particles', [0; 1; r], r, 0.1, true);
            phxEngine('set', w, sliding, 'velocities', [1; 0; 0]);
            phxEngine('set', w, rolling, 'velocities', [1; 0; 0]);
            for k = 1:round(0.5/tc.Dt)
                phxEngine('step', w, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', w, sliding, 'angvel'), [0; 0; 0]);
            v = phxEngine('get', w, rolling, 'velocities');
            tc.verifyEqual(phxEngine('get', w, rolling, 'angvel'), [0; v(1)/r; 0], "AbsTol", 0.05*v(1)/r);
        end

        function particlesSleepByTheirOwnSetting(tc)
            % In a world that keeps its bodies awake, a group allowed to sleep
            % falls asleep once at rest; forbidding it wakes the particles.
            w = tc.newWorld;
            tc.addFloor(w);
            g = phxEngine('add', w, 'particles', [0 0.3; 0 0; 0.05 0.05], 0.05, 0.1, false);
            phxEngine('set', w, g, 'sleeping', true);
            for k = 1:round(2/tc.Dt)
                phxEngine('step', w, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', w, g, 'active'), [false false]);
            phxEngine('set', w, g, 'sleeping', false);
            tc.verifyEqual(phxEngine('get', w, g, 'active'), [true true]);
        end
    end

    % Spring constraint
    methods (Test, TestTags = {'Engine'})
        function springHoldsAMassAtItsStaticDeflection(tc)
            % 2 kg hanging from a static anchor on a spring of free length
            % 0.5 m, k = 400 N/m, damped to rest: stretched by m*g/k
            m = 2;
            k = 400;
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 1.5], 0.2*[1 1 1], m);
            s = phxEngine('add', w, 'spring', [a b], eye(4), eye(4), false, 0.5, k, 40);
            tc.verifyClass(s, "uint64");
            tc.settle(w, 3);
            [f, L] = phxEngine('get', w, s, 'spring');
            W = m*tc.G;
            tc.verifyEqual(L, 0.5 + W/k, "AbsTol", 1e-4);
            tc.verifyEqual(f, W, "RelTol", 1e-3, "A stretched spring reads a positive tension.");
            F = phxEngine('get', w, s, 'feedback');
            tc.verifySize(F, [3 4]);
            fA = F(:, 1)';
            fB = F(:, 3)';
            tc.verifyEqual(fB, [0, 0, W], "AbsTol", 1e-3*W);
            tc.verifyEqual(fA, -fB, "AbsTol", 1e-3*W);

            % New free length and spring at run time
            phxEngine('set', w, s, 'lengths', 0.7);
            tc.settle(w, 3);
            [~, L] = phxEngine('get', w, s, 'spring');
            tc.verifyEqual(L, 0.7 + W/k, "AbsTol", 1e-4);
            phxEngine('set', w, s, 'spring', 2*k, 40);
            tc.settle(w, 3);
            [~, L] = phxEngine('get', w, s, 'spring');
            tc.verifyEqual(L, 0.7 + W/(2*k), "AbsTol", 1e-4);
        end

        function springOscillatesAtItsNaturalFrequency(tc)
            % 1 kg on k = 100 N/m without damping, released 0.1 m out:
            % period 2*pi/10 s, and the motion neither grows nor decays
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1.1 0 0], 0.2*[1 1 1]);
            s = phxEngine('add', w, 'spring', [a b], eye(4), eye(4), false, 1, 100, 0);
            n = round(2/tc.Dt);
            x = zeros(1, n);
            for i = 1:n
                [~] = phxEngine('step', w, tc.Dt);
                [~, L] = phxEngine('get', w, s, 'spring');
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
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            lengths = [1 2 0.3];
            for i = 3:-1:1
                b = tc.addBox(w, 2, [0 3*i lengths(i)], 0.1*[1 1 1]);
                s(i) = phxEngine('add', w, 'spring', [a b], tEngineProtocol.pose([0 3*i 0]), eye(4), false, [0.5 1.5], 100, 0);
            end
            [~] = phxEngine('step', w, tc.Dt);
            f = arrayfun(@(h) phxEngine('get', w, h, 'spring'), s);
            tc.verifyEqual(f(1), 0, "Inside the range the spring is slack.");
            tc.verifyEqual(f(2), 100*0.5, "RelTol", 0.02, "Beyond the range it pulls.");
            tc.verifyEqual(f(3), -100*0.2, "RelTol", 0.02, "Below the range it pushes.");
        end

        function inerterAddsItsInertanceToTheMass(tc)
            % 1 kg on k = 100 N/m with an inerter of 3 kg to a static anchor,
            % released 0.1 m out: period 2*pi*sqrt((m + b)/k), amplitude kept
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1.1 0 0], 0.2*[1 1 1]);
            s = phxEngine('add', w, 'spring', [a b], eye(4), eye(4), false, 1, 100, 0);
            phxEngine('set', w, s, 'inertance', 3);
            n = round(4/tc.Dt);
            x = zeros(1, n);
            for i = 1:n
                [~] = phxEngine('step', w, tc.Dt);
                [~, L] = phxEngine('get', w, s, 'spring');
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
            w = tc.newWorld([0 0 0]);
            hA = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([0 0 0]), 1, [1 1 1]);
            hB = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([1 0 0]), 3, [1 1 1]);
            s = phxEngine('add', w, 'spring', [hA hB], eye(4), eye(4), false, 1, 0, 0);
            phxEngine('set', w, s, 'inertance', 2);
            phxEngine('apply', w, hA, 'centralforce', [-10 0 0], false);
            [~] = phxEngine('step', w, tc.Dt);
            f = 2*10/(1 + 2 + 2/3);
            tc.verifyEqual(phxEngine('get', w, hA, 'linvel'), [-(10 - f)*tc.Dt, 0, 0], "RelTol", 1e-4);
            tc.verifyEqual(phxEngine('get', w, hB, 'linvel'), [-f/3*tc.Dt, 0, 0], "RelTol", 1e-4);
        end

        function inerterFarAboveTheMassIsStable(tc)
            % 1 kg on k = 1e4 N/m with 1e4 kg of inertance, released 0.1 m
            % out: period 2*pi*sqrt(1.0001) s, the motion neither grows nor
            % decays
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1.1 0 0], 0.2*[1 1 1]);
            s = phxEngine('add', w, 'spring', [a b], eye(4), eye(4), false, 1, 1e4, 0);
            phxEngine('set', w, s, 'inertance', 1e4);
            T = 2*pi*sqrt(1.0001);
            tc.settle(w, T/4);
            [~, L] = phxEngine('get', w, s, 'spring');
            tc.verifyEqual(L - 1, 0, "AbsTol", 2e-3, "A quarter period passes zero.");
            tc.settle(w, T/4);
            [~, L] = phxEngine('get', w, s, 'spring');
            tc.verifyEqual(L - 1, -0.1, "RelTol", 0.01, "Half a period swings to the other side.");
            tc.verifyError(@() phxEngine('set', w, s, 'inertance', -1), "phx:engine:badArgument");
        end

        function torsionSpringTwistsByTorqueOverStiffness(tc)
            % 2 N*m about z on a body held by k = 10 N*m/rad (damped): it
            % turns by M/k; the spring holds it with -M, its base feels +M
            [w, a, b, s] = tc.torsionPair(1, 0, 10, 5);
            for i = 1:round(5/tc.Dt)
                phxEngine('apply', w, b, 'torque', [0 0 2], false);
                [~] = phxEngine('step', w, tc.Dt);
            end
            [M, angle] = phxEngine('get', w, s, 'spring');
            tc.verifyEqual(angle, 0.2, "AbsTol", 1e-4);
            tc.verifyEqual(M, -2, "RelTol", 1e-3);
            F = phxEngine('get', w, s, 'feedback');
            tA = F(:, 2)';
            tB = F(:, 4)';
            tc.verifyEqual(tB, [0, 0, -2], "AbsTol", 2e-3);
            tc.verifyEqual(tA, [0, 0, 2], "AbsTol", 2e-3);
            tc.verifyEqual(phxEngine('get', w, a, 'totaltorque'), [0, 0, 2], "AbsTol", 2e-3);
        end

        function torsionSpringOscillatesAtSqrtKOverI(tc)
            % I = 0.5 kg*m^2 on k = 2 N*m/rad, rest angle 0.3 rad, released
            % at 0: period pi, swinging between 0 and 0.6 rad; with an
            % inertance of 1.5 kg*m^2 the period doubles
            for inertance = [0 1.5]
                [w, ~, ~, s] = tc.torsionPair(0.5, 0.3, 2, 0);
                phxEngine('set', w, s, 'inertance', inertance);
                T = pi*sqrt((0.5 + inertance)/0.5);
                n = round(3*T/tc.Dt);
                x = zeros(1, n);
                for i = 1:n
                    [~] = phxEngine('step', w, tc.Dt);
                    [~, x(i)] = phxEngine('get', w, s, 'spring');
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
            [w, ~, b, s] = tc.torsionPair(1, 0, 0.1, 0);
            phxEngine('set', w, b, 'angvel', [0 0 10]);
            top = 0;
            for i = 1:round(6/tc.Dt)
                [~] = phxEngine('step', w, tc.Dt);
                [~, angle] = phxEngine('get', w, s, 'spring');
                top = max(top, angle);
            end
            tc.verifyEqual(top, 10/sqrt(0.1), "RelTol", 0.01);
        end

        function torsionSpringRejectsBadSettings(tc)
            [w, a, b, s] = tc.torsionPair(1, 0, 1, 0);
            tc.verifyError(@() phxEngine('add', w, 'torsionspring', [a b], eye(4), eye(4), true, NaN, 1, 0), ...
                "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'torsionspring', [a b], eye(4), eye(4), true, 0, -1, 0), ...
                "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, s, 'inertance', -1), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, s, 'angle', Inf), "phx:engine:badArgument");
        end

        function ropeNeverPushes(tc)
            % Lengths [-Inf 1]: no force shorter than 1 m, even where the
            % points meet, a pull beyond it
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            lengths = [0 0.3 1.5];
            for i = 3:-1:1
                b = tc.addBox(w, 2, [0 3*i lengths(i)], 0.1*[1 1 1]);
                phxEngine('set', w, b, 'collisions', false);
                s(i) = phxEngine('add', w, 'spring', [a b], tEngineProtocol.pose([0 3*i 0]), eye(4), false, [-Inf 1], 100, 0);
            end
            [~] = phxEngine('step', w, tc.Dt);
            f = arrayfun(@(h) phxEngine('get', w, h, 'spring'), s);
            tc.verifyEqual(f(1:2), [0 0], "A rope does not push.");
            tc.verifyEqual(f(3), 100*0.5, "RelTol", 0.02, "Beyond its length it pulls.");
        end

        function ropeTackleHoldsHalfTheLoad(tc)
            % A 2 kg block hangs in the bight of a rope: from the static beam
            % down under the block (two points 0.4 m apart, the rope wraps
            % it), up over the beam and across to a static drum. Two
            % vertical runs carry it, T = m*g/2; the block sinks by the
            % stretch over two runs, T/k/2; the drum feels T toward the beam
            [w, h] = tc.tackle(4.2);
            m = 2;
            k = 1e4;
            tc.settle(w, 3);
            [T, L] = phxEngine('get', w, h.rope, 'spring');
            tc.verifyEqual(T, m*tc.G/2, "RelTol", 1e-3);
            tc.verifyEqual(L, 4.2 + T/k, "AbsTol", 1e-5);
            p = tEngineProtocol.positionOf(w, h.block);
            tc.verifyEqual(p(3), 2 - T/k/2, "AbsTol", 1e-5);
            tc.verifyEqual(phxEngine('get', w, h.drum, 'totalforce'), [-T, 0, 0], "AbsTol", 1e-3*T);
        end

        function ropeIsSlackBelowItsLength(tc)
            % The same tackle with 0.3 m of rope to spare: the block falls
            % freely and the rope reads no tension
            [w, h] = tc.tackle(4.5);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h.rope, 'spring'), 0);
            tc.verifyEqual(phxEngine('get', w, h.block, 'linvel'), [0, 0, -tc.G*tc.Dt], "AbsTol", 1e-6);
        end

        function ropeGoesWithAnyOfItsBodies(tc)
            [w, h] = tc.tackle(4.2);
            tc.settle(w, 0.1);
            phxEngine('remove', w, h.block);
            [~] = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(phxEngine('get', w, h.rope, 'spring'), 0);
            tc.verifyError(@() phxEngine('set', w, h.rope, 'lengths', 4), "phx:engine:badHandle");
        end

        function ropeRejectsBadSettings(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.1*[1 1 1]);
            add = @(hs, L) phxEngine('add', w, 'rope', hs, zeros(3, numel(hs)), L, 100, 0);
            tc.verifyError(@() add(a, 1), "phx:engine:badArgument");
            tc.verifyError(@() add([a a], 1), "phx:engine:badArgument");
            tc.verifyError(@() add([a b], -1), "phx:engine:badArgument");
            tc.verifyClass(add([a b a], 1), "uint64");
        end

        function damperAloneIsNotRigid(tc)
            % No stiffness, c = 2 N*s/m on 1 kg moving apart at 1 m/s: the
            % speed decays as exp(-2*t) and the bodies drift apart
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.2*[1 1 1]);
            s = phxEngine('add', w, 'spring', [a b], eye(4), eye(4), false, 1, 0, 2);
            phxEngine('set', w, b, 'linvel', [1 0 0]);
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, b, 'linvel'), [exp(-1), 0, 0], "RelTol", 0.02);
            [~, L] = phxEngine('get', w, s, 'spring');
            tc.verifyEqual(L, 1 + (1 - exp(-1))/2, "RelTol", 0.01);

            % Neither stiffness nor damping: no force at all
            phxEngine('set', w, s, 'spring', 0, 0);
            v = phxEngine('get', w, b, 'linvel');
            tc.settle(w, 0.5);
            tc.verifyEqual(phxEngine('get', w, b, 'linvel'), v, "AbsTol", 1e-9);
            tc.verifyEqual(phxEngine('get', w, s, 'spring'), 0);
        end

        function springRejectsBadSettings(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 0], 0.1*[1 1 1]);
            add = @(lengths, k, c) phxEngine('add', w, 'spring', [a b], eye(4), eye(4), false, lengths, k, c);
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
            phxEngine('remove', w, h1);
            ids = phxEngine('step', w, tc.Dt);
            tc.verifyEqual(ids, h2);
        end

        function removedJointReleasesTheBody(tc)
            w = tc.newWorld;
            a = tc.addBox(w, 0, [0 0 2], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [1 0 2], 0.2*[1 1 1]);
            j = phxEngine('add', w, 'fixed', [a b], tEngineProtocol.pose([0.5 0 0]), tEngineProtocol.pose([-0.5 0 0]), false);
            tc.settle(w, 0.1);
            phxEngine('remove', w, j);
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
            w = phxEngine('setup');
            tc.addTeardown(@() phxEngine('clear', w));
            phxEngine('set', w, uint64(0), 'gravity', gravity);
            phxEngine('set', w, uint64(0), 'settings', struct('sleeping', false));
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
            h = phxEngine('add', w, 'box', type, size/2, tEngineProtocol.pose(position), mass, inertia);
        end

        function [w, a, b, s] = torsionPair(tc, inertia, restAngle, k, c)
            % A torsion spring about z from a static body to a free one of
            % the given (isotropic) inertia, in zero gravity, apart enough
            % not to touch
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.1*[1 1 1]);
            b = phxEngine('add', w, 'box', 2, 0.1*[1 1 1], tEngineProtocol.pose([1 0 0]), 1, inertia*[1 1 1]);
            s = phxEngine('add', w, 'torsionspring', [a b], eye(4), eye(4), true, restAngle, k, c);
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
            h.rope = phxEngine('add', w, 'rope', hs, points, freeLength, 1e4, 200);
        end

        function h = addFloor(tc, w)
            % Static 10 x 10 m floor with its top at z = 0
            h = tc.addBox(w, 0, [0 0 -0.5], [10 10 1], 0, [0 0 0]);
        end

        function [w, b, j] = hingeRig(tc)
            % A hinge about world z from a static body to a free one of
            % unit inertia at the same place, no gravity
            w = tc.newWorld([0 0 0]);
            a = tc.addBox(w, 0, [0 0 0], 0.2*[1 1 1]);
            b = tc.addBox(w, 2, [0 0 0], 0.2*[1 1 1], 1, [1 1 1]);
            j = phxEngine('add', w, 'revolute', [a b], eye(4), eye(4), false);
        end

        function T = lastPose(tc, w, h, duration)
            % Steps for the given time; returns the last pose reported for
            % the body (a body standing still is not reported)
            T = [];
            for k = 1:round(duration/tc.Dt)
                [ids, P] = phxEngine('step', w, tc.Dt);
                if any(ids == h)
                    T = P(:, :, ids == h);
                end
            end
            tc.assertNotEmpty(T, "The body was never reported as moving.");
        end

        function [a, sideways] = rollingDeceleration(tc, shape, bodyFriction, floorFriction, mass, dt, azimuth)
            % Launches a 1 kg ball (or a cylinder about its axis) of radius
            % 0.1 m rolling without slip at 1 m/s on a floor along the
            % azimuth (deg); returns its deceleration over 0.5 s after 0.1 s
            % and its sideways speed at the end
            arguments
                tc
                shape (1, 1) string
                bodyFriction (1, 3) double
                floorFriction (1, 3) double = [1 0 0]
                mass (1, 1) double = 1
                dt (1, 1) double = tc.Dt
                azimuth (1, 1) double = 0
            end
            r = 0.1;
            d = [cosd(azimuth) sind(azimuth) 0];
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'friction', floorFriction);
            if shape == "sphere"
                h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 r]), mass, 0.4*mass*r^2*[1 1 1]);
            else
                Rz = makehgtform("zrotate", deg2rad(azimuth));
                I = mass*[(3*r^2 + 0.1^2)/12, r^2/2, (3*r^2 + 0.1^2)/12];
                h = phxEngine('add', w, 'cylinder', 2, r, 0.1, 'y', ...
                    Rz*tEngineProtocol.pose([0 0 r]), mass, I);
            end
            phxEngine('set', w, h, 'friction', bodyFriction);
            phxEngine('set', w, h, 'linvel', d);
            phxEngine('set', w, h, 'angvel', cross([0 0 1], d)/r);
            for k = 1:round(0.1/dt)
                [~] = phxEngine('step', w, dt);
            end
            v1 = phxEngine('get', w, h, 'linvel');
            for k = 1:round(0.5/dt)
                [~] = phxEngine('step', w, dt);
            end
            v2 = phxEngine('get', w, h, 'linvel');
            a = (dot(v1, d) - dot(v2, d))/0.5;
            sideways = norm(v2 - dot(v2, d)*d);
        end

        function alpha = spinDeceleration(tc, ballFriction, floorFriction)
            % Spins a 1 kg ball of radius 0.1 m at 10 rad/s about the
            % vertical on a floor; returns its angular deceleration over
            % 0.3 s after 0.05 s
            r = 0.1;
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'friction', floorFriction);
            h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 r]), 1, 0.4*r^2*[1 1 1]);
            phxEngine('set', w, h, 'friction', ballFriction);
            phxEngine('set', w, h, 'angvel', [0 0 10]);
            tc.settle(w, 0.05);
            w1 = phxEngine('get', w, h, 'angvel');
            tc.settle(w, 0.3);
            w2 = phxEngine('get', w, h, 'angvel');
            alpha = (w1(3) - w2(3))/0.3;
        end

        function T = settle(tc, w, duration)
            % Steps for the given time; returns the poses of the last step
            % (4x4xN, or false when nothing moved)
            for k = 1:round(duration/tc.Dt)
                [~, T] = phxEngine('step', w, tc.Dt);
            end
        end

        function F = meanForceB(tc, w, j)
            % Steps 40 times and returns the force of joint j on its body B
            % (third column of the feedback) averaged over those steps, as a
            % row. A stop engages only at or beyond it, so a body resting on
            % it leaves it now and then for a step and is caught with twice
            % the impulse
            n = 40;
            F = zeros(n, 3);
            for k = 1:n
                phxEngine('step', w, tc.Dt);
                fb = phxEngine('get', w, j, 'feedback');
                F(k, :) = fb(:, 3)';
            end
            F = mean(F);
        end
    end

    methods (Static, Access = private)
        function out = oneOutput(f)
            % Calls f asking for one output
            out = f();
        end

        function P = pose(position)
            P = [eye(3) position(:); 0 0 0 1];
        end

        function P = frame(position, axis)
            % Joint frame at the position whose Z axis is the given axis
            z = axis(:)/norm(axis);
            helper = [0; 0; 1];
            if abs(z(3)) > 0.9
                helper = [1; 0; 0];
            end
            x = cross(helper, z);
            x = x/norm(x);
            P = [x cross(z, x) z position(:); 0 0 0 1];
        end

        function p = positionOf(w, h)
            % Position of a moving body, read off the poses of one more
            % (negligibly short) step, as PHX reads it
            [ids, T] = phxEngine('step', w, 1e-6);
            p = T(1:3, 4, ids == h);
        end

        function E = chainEnergy(w, bodies, m, L)
            % Potential and kinetic energy of a chain of links of mass m and
            % length L (thin rods), from the engine
            P = phxEngine('get', w, bodies, 'position');
            E = 0;
            for i = 1:numel(bodies)
                v = phxEngine('get', w, bodies(i), 'linvel');
                om = phxEngine('get', w, bodies(i), 'angvel');
                E = E + m*tEngineProtocol.G*P(3, i) + 0.5*m*(v*v') + 0.5*(m*L^2/12)*(om*om');
            end
        end

        function a = tiltOf(w, h)
            % Angle between the x axis of a body and the world x axis
            T = phxEngine('get', w, h, 'transform');
            a = acos(min(1, T(1, 1)));
        end

        function a = zAngleOf(w, h)
            % Rotation of a body about world z, read from its pose
            T = phxEngine('get', w, h, 'transform');
            a = atan2(T(2, 1), T(1, 1));
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
            s.box = row(@(w, P) phxEngine('add', w, 'box', 2, [0.2 0.3 0.4], P, 1, I), 0.4);
            s.sphere = row(@(w, P) phxEngine('add', w, 'sphere', 2, 0.3, P, 1, I), 0.3);
            s.cylinderLying = row(@(w, P) phxEngine('add', w, 'cylinder', 2, 0.2, 1.0, 'x', P, 1, I), 0.2);
            s.cylinderStanding = row(@(w, P) phxEngine('add', w, 'cylinder', 2, 0.4, 0.2, 'z', P, 1, I), 0.1);
            s.cone = row(@(w, P) phxEngine('add', w, 'cone', 2, 0.3, 0.8, 'z', P, 1, I), 0.4);
            s.capsule = row(@(w, P) phxEngine('add', w, 'capsule', 2, 0.2, 0.6, 'z', P, 1, I), 0.5);
            bottle = [-0.3 0.1 0.3; 0.2 0.2 0];
            s.revolutionStanding = row(@(w, P) phxEngine('add', w, 'revolution', 2, bottle, 'z', P, 1, I), 0.3);
            s.revolutionLying = row(@(w, P) phxEngine('add', w, 'revolution', 2, bottle, 'x', P, 1, I), 0.2);
            s.convexHull = row(@(w, P) phxEngine('add', w, 'convexhull', 2, V, P, 1, I), 0.2);
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
            h = phxEngine('add', w, 'trimesh', type, V, F, P, 1, [0.1 0.1 0.1]);
        end

        function [V, F] = cubeMesh(half)
            % Closed cube of the given half size: vertices 3x8, outward
            % one-based faces 3x12
            V = half*[-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1]';
            F = 1 + [0 2 1; 0 3 2; 4 5 6; 4 6 7; 0 1 5; 0 5 4; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 0 4; 3 4 7]';
        end
    end

end
