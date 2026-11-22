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

    % Spherical shell (protocol 2, step W): a static shell of middle radius
    % 1 m and a 5 cm wall, its inner face at 0.975 m, its outer at 1.025 m
    methods (Test, TestTags = {'Engine'})
        function shellHoldsASphereInsideAndBearsOneOutside(tc)
            % A ball thrown about in the cavity never gets past the inner
            % face; a ball dropped on the top never sinks into the outer one.
            % Hitting the wall at 3.7 m/s it sinks in a few millimetres at
            % this step, as into any wall.
            w = tc.newWorld;
            tEngineProtocol.addShell(w, 0, 1, 0.05);
            r = 0.1;
            I = 0.4*r^2*[1 1 1];
            in = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0.3 0 0]), 1, I);
            out = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0.05 0 1.6]), 1, I);
            phxEngine('set', w, in, 'linvel', [3 2 1]);
            farthest = 0;
            nearest = inf;
            for k = 1:400
                phxEngine('step', w, tc.Dt);
                farthest = max(farthest, norm(phxEngine('get', w, in, 'position')));
                nearest = min(nearest, norm(phxEngine('get', w, out, 'position')));
            end
            tc.verifyLessThan(farthest, 0.975 - r + 5e-3, "Farthest the inner ball's centre got (m)");
            tc.verifyGreaterThan(nearest, 1.025 + r - 2e-3, "Nearest the outer ball's centre got (m)");
        end

        function shellHoldsABoxOnItsCorners(tc)
            % A box let down flat in the cavity comes to rest on the four
            % corners of its bottom face, all on the inner face
            w = tc.newWorld;
            tEngineProtocol.addShell(w, 0, 1, 0.05);
            half = [0.1 0.05 0.05];
            h = tc.addBox(w, 2, [0 0 -0.5], 2*half);
            tc.settle(w, 3);
            T = phxEngine('get', w, h, 'transform');
            [a, b, c] = ndgrid([-1 1], [-1 1], [-1 1]);
            corners = T*[a(:)*half(1) b(:)*half(2) c(:)*half(3) ones(8, 1)]';
            radii = sort(vecnorm(corners(1:3, :)), "descend");
            tc.verifyEqual(radii(1:4), 0.975*[1 1 1 1], "AbsTol", 2e-3, "Radii of the four lowest corners (m)");
            tc.verifyLessThan(norm(phxEngine('get', w, h, 'linvel')), 1e-3, "The box comes to rest.");
        end

        function shellCollidesWithMeshesTerrainsAndShells(tc)
            % A mesh cube and a smaller shell thrown about in the cavity stay
            % in it; a small shell rests on a flat terrain on its outer face
            [V, F] = tEngineProtocol.cubeMesh(0.08);
            w = tc.newWorld;
            tEngineProtocol.addShell(w, 0, 1, 0.05);
            cube = tEngineProtocol.addTrimesh(w, V, F, tEngineProtocol.pose([0.2 0 0]));
            small = tEngineProtocol.addShell(w, 2, 0.2, 0.02, [-0.4 0 0]);
            phxEngine('set', w, cube, 'linvel', [3 1 2]);
            phxEngine('set', w, cube, 'angvel', [4 -2 3]);
            phxEngine('set', w, small, 'linvel', [-2 2 1]);
            cubeReach = 0;
            shellReach = 0;
            for k = 1:400
                phxEngine('step', w, tc.Dt);
                T = phxEngine('get', w, cube, 'transform');
                cubeReach = max(cubeReach, max(vecnorm(T(1:3, :)*[V; ones(1, 8)])));
                shellReach = max(shellReach, norm(phxEngine('get', w, small, 'position')));
            end
            tc.verifyLessThan(cubeReach, 0.975 + 2e-3, "Farthest a vertex of the mesh cube got (m)");
            tc.verifyLessThan(shellReach, 0.975 - 0.21 + 2e-3, "Farthest the small shell's centre got (m)");

            w = tc.newWorld;
            phxEngine('add', w, 'terrain', 0, zeros(11), [0.2 0.2], tEngineProtocol.pose([0 0 0]), 0, [0 0 0]);
            small = tEngineProtocol.addShell(w, 2, 0.2, 0.02, [0 0 0.5]);
            tc.settle(w, 1);
            p = phxEngine('get', w, small, 'position');
            tc.verifyEqual(p(3), 0.21, "AbsTol", 2e-3, "Height of the shell resting on the terrain (m)");
        end

        function raycastHitsTheShellFaces(tc)
            % From the cavity a ray hits the inner face, facing the cavity;
            % from outside the outer face; from inside the wall it passes out
            w = tc.newWorld([0 0 0]);
            h = tEngineProtocol.addShell(w, 0, 1, 0.05);
            O = [0 0 0; 0 0 3; 0 0 1]';
            E = [2 0 0; 0 0 0; 0 0 3]';
            [p, n, d, ids] = phxEngine('raycast', w, O, E);
            tc.verifyEqual(ids(1:2), [h h]);
            tc.verifyEqual(p(:, 1), [0.975; 0; 0], "AbsTol", 1e-5);
            tc.verifyEqual(n(:, 1), [-1; 0; 0], "AbsTol", 1e-5);
            tc.verifyEqual(d(1), 0.975, "AbsTol", 1e-5);
            tc.verifyEqual(p(:, 2), [0; 0; 1.025], "AbsTol", 1e-5);
            tc.verifyEqual(n(:, 2), [0; 0; 1], "AbsTol", 1e-5);
            tc.verifyEqual(d(2), 1.975, "AbsTol", 1e-5);
            tc.verifyEqual(ids(3), uint64(0), "A ray starting in the wall passes out of it.");
        end

        function shellArgumentsAreChecked(tc)
            % A positive radius and a positive wall thinner than the diameter
            w = tc.newWorld;
            P = tEngineProtocol.pose([0 0 0]);
            tc.verifyError(@() phxEngine('add', w, 'sphereshell', 0, 1, 2, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'sphereshell', 0, 0, 0.1, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'sphereshell', 0, 1, 0, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'sphereshell', 0, 1), "phx:engine:badArgument");
        end
    end

    % Capsule shell (protocol 2, step X): a static shell of middle radius
    % 0.5 m about a 2 m segment, a 4 cm wall, its inner face 0.48 m and its
    % outer 0.52 m from the segment
    methods (Test, TestTags = {'Engine'})
        function capsuleShellHoldsBodiesInsideAndBearsThemOutside(tc)
            % In a lying shell a ball thrown about never gets past the inner
            % face (3.9 m/s against the wall sinks a few mm) and a ball
            % dropped on the top never sinks into the outer face; a box laid
            % alone along the axis comes to rest on its four lowest corners
            % on the inner face
            w = tc.newWorld;
            tEngineProtocol.addCapsuleShell(w, 0, 0.5, 2, 'x', 0.04);
            r = 0.08;
            I = 0.4*r^2*[1 1 1];
            in = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0.5 0 0]), 1, I);
            out = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0.3 0.02 1]), 1, I);
            phxEngine('set', w, in, 'linvel', [3 2 1.5]);
            farthest = 0;
            nearest = inf;
            for k = 1:600
                phxEngine('step', w, tc.Dt);
                farthest = max(farthest, tEngineProtocol.fromSegment(reshape(phxEngine('get', w, in, 'position'), 3, 1), 1, 1));
                nearest = min(nearest, tEngineProtocol.fromSegment(reshape(phxEngine('get', w, out, 'position'), 3, 1), 1, 1));
            end
            tc.verifyLessThan(farthest, 0.48 - r + 5e-3, "Farthest the inner ball's centre got from the segment (m)");
            tc.verifyGreaterThan(nearest, 0.52 + r - 2e-3, "Nearest the outer ball's centre got to the segment (m)");

            w = tc.newWorld;
            tEngineProtocol.addCapsuleShell(w, 0, 0.5, 2, 'x', 0.04);
            half = [0.4 0.1 0.05];
            box = tc.addBox(w, 2, [-0.3 0 -0.2], 2*half);
            tc.settle(w, 3);
            T = phxEngine('get', w, box, 'transform');
            [a, b, c] = ndgrid([-1 1], [-1 1], [-1 1]);
            corners = T*[a(:)*half(1) b(:)*half(2) c(:)*half(3) ones(8, 1)]';
            d = sort(tEngineProtocol.fromSegment(corners(1:3, :), 1, 1), "descend");
            tc.verifyEqual(d(1:4), 0.48*[1 1 1 1], "AbsTol", 2e-3, "Distances of the four lowest corners from the segment (m)");
            tc.verifyLessThan(norm(phxEngine('get', w, box, 'linvel')), 1e-3, "The box comes to rest.");
        end

        function capsuleShellsNestInShells(tc)
            % A spherical shell thrown about in a lying capsule shell stays in
            % it, and a capsule shell in a spherical one stays in it with both
            % ends of its core
            w = tc.newWorld;
            tEngineProtocol.addCapsuleShell(w, 0, 0.5, 2, 'x', 0.04);
            ball = tEngineProtocol.addShell(w, 2, 0.15, 0.02, [0.5 0 0]);
            phxEngine('set', w, ball, 'linvel', [3 2 1]);
            reach = 0;
            for k = 1:400
                phxEngine('step', w, tc.Dt);
                reach = max(reach, tEngineProtocol.fromSegment(reshape(phxEngine('get', w, ball, 'position'), 3, 1), 1, 1));
            end
            tc.verifyLessThan(reach, 0.48 - 0.16 + 2e-3, "Farthest the spherical shell's centre got from the segment (m)");

            w = tc.newWorld;
            tEngineProtocol.addShell(w, 0, 1, 0.05);
            pill = tEngineProtocol.addCapsuleShell(w, 2, 0.15, 0.6, 'x', 0.02, [0 0 -0.3]);
            phxEngine('set', w, pill, 'linvel', [2 1 -2]);
            phxEngine('set', w, pill, 'angvel', [1 3 2]);
            reach = 0;
            for k = 1:400
                phxEngine('step', w, tc.Dt);
                T = phxEngine('get', w, pill, 'transform');
                ends = T*[0.3 -0.3; 0 0; 0 0; 1 1];
                reach = max(reach, max(vecnorm(ends(1:3, :))));
            end
            tc.verifyLessThan(reach, 0.975 - 0.16 + 2e-3, "Farthest an end of the capsule's core got from the centre (m)");
        end

        function raycastHitsTheCapsuleShellFaces(tc)
            % From the cavity a ray hits the inner face on the side and in the
            % end cap; from outside the outer face; from inside the wall of
            % the cap it passes out
            w = tc.newWorld([0 0 0]);
            h = tEngineProtocol.addCapsuleShell(w, 0, 0.5, 2, 'z', 0.04);
            O = [0 0 0; 0 0 0; 0 0 3; 3 0 0.5; 0 0 1.5]';
            E = [2 0 0; 0 0 3; 0 0 0; 0 0 0.5; 0 0 3]';
            [p, n, d, ids] = phxEngine('raycast', w, O, E);
            tc.verifyEqual(ids(1:4), [h h h h]);
            tc.verifyEqual(p(:, 1:4), [0.48 0 0; 0 0 1.48; 0 0 1.52; 0.52 0 0.5]', "AbsTol", 1e-5);
            tc.verifyEqual(n(:, 1:4), [-1 0 0; 0 0 -1; 0 0 1; 1 0 0]', "AbsTol", 1e-5);
            tc.verifyEqual(d(1:4), [0.48 1.48 1.48 2.48], "AbsTol", 1e-5);
            tc.verifyEqual(ids(5), uint64(0), "A ray starting in the wall passes out of it.");
        end

        function capsuleShellArgumentsAreChecked(tc)
            % A positive radius, a height not negative, an axis char and a
            % positive wall thinner than the diameter
            w = tc.newWorld;
            P = tEngineProtocol.pose([0 0 0]);
            tc.verifyError(@() phxEngine('add', w, 'capsuleshell', 0, 1, 1, 'z', 2, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'capsuleshell', 0, 1, -1, 'z', 0.1, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'capsuleshell', 0, 1, 1, 'w', 0.1, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'capsuleshell', 0, 0, 1, 'z', 0.1, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'capsuleshell', 0, 1, 1, 'z'), "phx:engine:badArgument");
        end
    end

    % Box shell (protocol 2, step Y): a static shell of outer half extents
    % [0.6 0.4 0.5], a 4 cm wall and a cavity rounded by 1 cm, its core of
    % half extents [0.55 0.35 0.45], its cavity reaching [0.56 0.36 0.46]
    methods (Test, TestTags = {'Engine'})
        function boxShellHoldsBodiesInsideAndBearsThemOutside(tc)
            % A ball thrown about in the cavity never gets past the inner
            % face, edges and corners included (each flat wall in reach gives
            % its own contact, as separate plates would), a ball dropped on
            % the flat top rests on it, and a box let down in the cavity comes
            % to rest on its four lowest corners on the floor
            core = [0.55 0.35 0.45];
            w = tc.newWorld;
            tEngineProtocol.addBoxShell(w, 0, [0.6 0.4 0.5], 0.04, 0.01);
            r = 0.05;
            I = 0.4*r^2*[1 1 1];
            in = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0.2 0.1 0]), 1, I);
            on = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0.1 0.05 0.8]), 1, I);
            phxEngine('set', w, in, 'linvel', [4 3 2]);
            farthest = -inf;
            for k = 1:400
                phxEngine('step', w, tc.Dt);
                farthest = max(farthest, tEngineProtocol.fromBox(reshape(phxEngine('get', w, in, 'position'), 3, 1), core));
            end
            tc.verifyLessThan(farthest, 0.01 - r + 2e-3, "Farthest the inner ball's centre got from the core (m)");
            p = phxEngine('get', w, on, 'position');
            tc.verifyEqual(p(3), 0.5 + r, "AbsTol", 2e-3, "Height of the ball resting on the top (m)");

            w = tc.newWorld;
            tEngineProtocol.addBoxShell(w, 0, [0.6 0.4 0.5], 0.04, 0.01);
            half = [0.1 0.08 0.05];
            box = tc.addBox(w, 2, [0.1 0.05 -0.3], 2*half);
            tc.settle(w, 2);
            T = phxEngine('get', w, box, 'transform');
            [a, b, c] = ndgrid([-1 1], [-1 1], [-1 1]);
            corners = T*[a(:)*half(1) b(:)*half(2) c(:)*half(3) ones(8, 1)]';
            z = sort(corners(3, :));
            tc.verifyEqual(z(1:4), -0.46*[1 1 1 1], "AbsTol", 2e-3, "Heights of the four lowest corners (m)");
            tc.verifyLessThan(norm(phxEngine('get', w, box, 'linvel')), 1e-3, "The box comes to rest.");
        end

        function boxShellCornerTouchesEveryWall(tc)
            % Pulled into a lower corner of the cavity, a box rests against
            % the floor and both walls it reaches (the pull along the floor
            % beats the friction on the floor and on the walls)
            w = tc.newWorld([-8 -8 -4]);
            tEngineProtocol.addBoxShell(w, 0, [0.6 0.4 0.5], 0.04, 0.01);
            half = [0.1 0.08 0.05];
            box = tc.addBox(w, 2, [0 0 -0.3], 2*half);
            tc.settle(w, 3);
            T = phxEngine('get', w, box, 'transform');
            [a, b, c] = ndgrid([-1 1], [-1 1], [-1 1]);
            corners = T*[a(:)*half(1) b(:)*half(2) c(:)*half(3) ones(8, 1)]';
            tc.verifyEqual(min(corners(1:3, :), [], 2), -[0.56; 0.36; 0.46], "AbsTol", 3e-3, ...
                "Lowest x, y and z of the box's corners (m)");
            tc.verifyLessThan(norm(phxEngine('get', w, box, 'linvel')), 1e-3, "The box comes to rest.");
        end

        function boxShellsNestAndStack(tc)
            % A box shell thrown about in a spherical shell stays in it with
            % every corner of its core, and a small box shell dropped on a
            % large one rests on its flat top
            w = tc.newWorld;
            tEngineProtocol.addShell(w, 0, 2, 0.05);
            inner = tEngineProtocol.addBoxShell(w, 2, [0.3 0.2 0.15], 0.02, 0.005, [0 0 -1], 2);
            phxEngine('set', w, inner, 'linvel', [2 1 -1]);
            phxEngine('set', w, inner, 'angvel', [1 2 3]);
            core = [0.3 0.2 0.15] - 0.025;
            [a, b, c] = ndgrid([-1 1], [-1 1], [-1 1]);
            corners = [a(:)*core(1) b(:)*core(2) c(:)*core(3) ones(8, 1)]';
            reach = 0;
            for k = 1:400
                phxEngine('step', w, tc.Dt);
                T = phxEngine('get', w, inner, 'transform');
                reach = max(reach, max(vecnorm(T(1:3, :)*corners)));
            end
            tc.verifyLessThan(reach, 1.975 - 0.025 + 2e-3, "Farthest a core corner got from the centre (m)");

            w = tc.newWorld;
            tEngineProtocol.addBoxShell(w, 0, [0.6 0.4 0.5], 0.04, 0.01);
            top = tEngineProtocol.addBoxShell(w, 2, [0.2 0.2 0.1], 0.02, 0, [0.1 0 0.8]);
            tc.settle(w, 2);
            p = phxEngine('get', w, top, 'position');
            tc.verifyEqual(p(3), 0.6, "AbsTol", 2e-3, "Height of the small shell resting on the large one (m)");
        end

        function raycastHitsTheBoxShellFaces(tc)
            % From the cavity a ray hits the inner faces, from outside the
            % outer face and the rounded outer corner; from inside the wall it
            % passes out
            w = tc.newWorld([0 0 0]);
            h = tEngineProtocol.addBoxShell(w, 0, [0.6 0.4 0.5], 0.04, 0.01);
            diagonal = [1 1 1]/sqrt(3);
            O = [0 0 0; 0 0 0; 3 0 0; 3*diagonal; 0 0 0.48]';
            E = [2 0 0; 0 0 2; 0 0 0; 0 0 0; 0 0 3]';
            [p, n, d, ids] = phxEngine('raycast', w, O, E);
            tc.verifyEqual(ids(1:4), [h h h h]);
            tc.verifyEqual(p(:, 1:3), [0.56 0 0; 0 0 0.46; 0.6 0 0]', "AbsTol", 1e-5);
            tc.verifyEqual(n(:, 1:3), [-1 0 0; 0 0 -1; 1 0 0]', "AbsTol", 1e-5);
            tc.verifyEqual(d(1:3), [0.56 0.46 2.4], "AbsTol", 1e-5);
            tc.verifyEqual(tEngineProtocol.fromBox(p(:, 4), [0.55 0.35 0.45]), 0.05, "AbsTol", 1e-5, ...
                "The diagonal ray hits the outer face 5 cm from the core.");
            tc.verifyEqual(ids(5), uint64(0), "A ray starting in the wall passes out of it.");
        end

        function boxShellArgumentsAreChecked(tc)
            % A positive wall, a rounding not negative, and a core left in
            % every direction
            w = tc.newWorld;
            P = tEngineProtocol.pose([0 0 0]);
            tc.verifyError(@() phxEngine('add', w, 'boxshell', 0, [0.6 0.4 0.5], 0.04, -0.01, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'boxshell', 0, [0.6 0.4 0.04], 0.04, 0.01, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'boxshell', 0, [0.6 0.4 0.5], 0, 0.01, P, 0, [0 0 0]), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('add', w, 'boxshell', 0, [0.6 0.4 0.5]), "phx:engine:badArgument");
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

        function frictionIsTheGeometricMeanOfTheTwoBodies(tc)
            % A box sliding on a floor decelerates by sqrt(mu_box*mu_floor)*g
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'friction', 0.8);
            h = tc.addBox(w, 2, [0 0 0.2], 0.4*[1 1 1]);
            phxEngine('set', w, h, 'friction', 0.5);
            tc.settle(w, 0.1);
            phxEngine('set', w, h, 'linvel', [3 0 0]);
            v0 = phxEngine('get', w, h, 'linvel');
            tc.settle(w, 0.2);
            v1 = phxEngine('get', w, h, 'linvel');
            tc.verifyEqual((v0(1) - v1(1))/0.2, sqrt(0.5*0.8)*tc.G, "RelTol", 0.03);
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
            phxEngine('set', w, h, 'friction', 0.25);   % 0.5 with the floor's 1
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
            % The first touch is the step after which the ball first rises:
            % the lowest point of the run can be a later landing, as every
            % landing ends within about 1e-8 m of the floor
            touch = find(diff(z) > 0, 1);
            tc.verifyEqual(max(z(touch:end)), 0.64, "RelTol", 0.15, "Height of the first bounce");
        end

        function restitutionIsTheSmallerOfTheTwoBodies(tc)
            % A ball dropped from 1 m bounces with the smaller of its and
            % the floor's restitution, whichever body has it: 0.8 and 0.5
            % either way round rise to 0.5^2 = 0.25 m
            r = 0.1;
            for e = [0.8 0.5; 0.5 0.8]'
                w = tc.newWorld;
                phxEngine('set', w, tc.addFloor(w), 'restitution', e(1));
                h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 1 + r]), 1, 0.4*r^2*[1 1 1]);
                phxEngine('set', w, h, 'restitution', e(2));
                z = nan(1, round(1.5/tc.Dt));
                for k = 1:numel(z)
                    [ids, T] = phxEngine('step', w, tc.Dt);
                    if any(ids == h)
                        z(k) = T(3, 4, ids == h) - r;
                    end
                end
                touch = find(diff(z) > 0, 1);
                tc.verifyEqual(max(z(touch:end)), 0.25, "RelTol", 0.15, ...
                    sprintf("Bounce with floor %.1f and ball %.1f", e(1), e(2)));
            end
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

        function rollingDiskStaysOnItsRimBelowFiveDegrees(tc)
            % The steel disk of phxex_eulerdisk (radius 0.15 m, 4 cm thick,
            % no margin) launched rolling on its rim at 25 deg in steady
            % precession, rolling resistance 1 mm, 1 ms steps: it rolls
            % down to a tilt below 3 deg before it falls flat (1.9 deg).
            % (Jolt 5.6.0 let a cap within 5 deg of its contact touch on the
            % chords of a fixed octagon; the disk fell flat from 4.6 deg.)
            g = tEngineProtocol.G;
            r = 0.15;
            hh = 0.02;
            m = 7800*pi*r^2*2*hh;
            I = m*[(3*r^2 + (2*hh)^2)/12*[1 1] r^2/2];
            dt = 0.001;
            w = tc.newWorld;
            phxEngine('set', w, uint64(0), 'settings', struct('margin', 0));
            floor = tc.addFloor(w);
            phxEngine('set', w, floor, 'friction', [1 0 0]);
            a = 25;
            R = [1 0 0; 0 cosd(a) -sind(a); 0 sind(a) cosd(a)];
            h = phxEngine('add', w, 'cylinder', 2, r, 2*hh, 'z', [R [0; 0; r*sind(a) + hh*cosd(a)]; 0 0 0 1], m, I);
            phxEngine('set', w, h, 'friction', [1 0.001 0]);
            precession = sqrt(m*g*r/(I(1)*sind(a)));
            phxEngine('set', w, h, 'angvel', precession*sind(a)*[0 cosd(a) sind(a)]);
            n = round(10/dt);
            tilt = a*ones(n, 1);
            for k = 1:n
                [ids, T] = phxEngine('step', w, dt);
                if any(ids == h)
                    tilt(k) = acosd(T(3, 3, ids == h));
                elseif k > 1
                    tilt(k) = tilt(k - 1);
                end
            end
            flat = find(tilt < 0.5, 1);
            tc.assertNotEmpty(flat, "The disk lies flat within 10 s");
            rolling = tilt(1:flat - round(0.1/dt));
            tc.verifyLessThan(min(rolling), 3, "Lowest tilt 0.1 s before the disk lies flat (deg)");
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
            phxEngine('set', w, pallet, 'friction', 0.25);
            offsets = [0.02 -0.03; -0.04 0.01; 0.03 0.04; -0.01 -0.02; 0.04 0.02; -0.03 0.03];
            boxes = zeros(1, 6, "uint64");
            for i = 1:6
                boxes(i) = tc.addBox(w, 2, [offsets(i, :) 0.25 + (i - 0.5) + 0.02*i], [3 3 1], 1, [0.1 0.1 0.1]);
                phxEngine('set', w, boxes(i), 'friction', 0.25);
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
            % it along, held by static friction (tan 0.15 < 0.64)
            w = tc.newWorld;
            plate = tc.addBox(w, 1, [0 0 -0.1], [4 4 0.2]);
            phxEngine('set', w, plate, 'friction', 0.64);
            h = tc.addBox(w, 2, [1 0 0.2], 0.4*[1 1 1]);
            phxEngine('set', w, h, 'friction', 0.64);
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
            tc.verifyEqual(tc.spinDeceleration([0.25 0 0.01], [1 0 0]), 0.5*0.01*tc.G/I, ...
                "RelTol", 0.01, "Ball alone");
            tc.verifyEqual(tc.spinDeceleration([0.25 0 0.005], [0.25 0 0.005]), 0.25*0.01*tc.G/I, ...
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

    % Compliant contacts (body field 'contact', protocol 2 step T)
    methods (Test, TestTags = {'Engine'})
        function softContactSinksByTheWeightOverTheStiffness(tc)
            % A 10 kg box resting on a soft contact sinks m g / k into the
            % floor, at a coarse step as at a fine one, and stays level
            k = 1e6;
            for dt = [1/60 tc.Dt]
                [d, speed] = tc.restOnSoftContact(10, [k 2*sqrt(k*10)], dt, 1);
                tc.verifyEqual(d, 10*tc.G/k, "RelTol", 0.01, sprintf("Penetration at dt %.4f s", dt));
                tc.verifyLessThan(speed, 1e-3, sprintf("Speed at rest at dt %.4f s", dt));
            end
        end

        function softImpactFollowsTheSpringDamperLaw(tc)
            % A 1 kg ball dropped 0.2 m onto a soft contact, at a step of
            % a fortieth of the contact time pi sqrt(m/k), sinks and rebounds
            % as the unilateral Kelvin-Voigt law gives (k 1e5; damping
            % ratios 0 and 0.3)
            k = 1e5;
            for zeta = [0 0.3]
                c = 2*zeta*sqrt(k);
                [rebound, depth] = tc.dropOnSoftContact([k c], [Inf 0], 1/4000);
                [refRebound, refDepth] = tEngineProtocol.kelvinVoigtDrop(1, k, c, 0.2, tc.G);
                tc.verifyEqual(rebound, refRebound, "RelTol", 0.03, sprintf("Rebound at zeta %.1f", zeta));
                tc.verifyEqual(depth, refDepth, "RelTol", 0.03, sprintf("Largest penetration at zeta %.1f", zeta));
            end
        end

        function softContactsCombineInSeries(tc)
            % Ball 2e5 on a floor 2e5 (dampers 40 and 40) is the contact
            % 1e5 with 20, as is a soft floor under a rigid ball
            [r0, d0] = tc.dropOnSoftContact([1e5 20], [Inf 0], 1/2000);
            [r1, d1] = tc.dropOnSoftContact([2e5 40], [2e5 40], 1/2000);
            [r2, d2] = tc.dropOnSoftContact([Inf 0], [1e5 20], 1/2000);
            tc.verifyEqual([r1 r2], [r0 r0], "RelTol", 1e-3, "Rebound");
            tc.verifyEqual([d1 d2], [d0 d0], "RelTol", 1e-3, "Largest penetration");
        end

        function softContactNeverPulls(tc)
            % A heavily damped soft contact (damping ratio 3) slows the
            % ball down but never holds it back: no step accelerates it
            % downwards by more than gravity
            k = 1e5;
            w = tc.newWorld;
            tc.addFloor(w);
            h = phxEngine('add', w, 'sphere', 2, 0.1, tEngineProtocol.pose([0 0 0.3]), 1, 0.004*[1 1 1]);
            phxEngine('set', w, h, 'contact', [k 6*sqrt(k)]);
            a = nan(1, 1200);
            for i = 1:numel(a)
                phxEngine('step', w, 1/2000);
                acc = phxEngine('get', w, h, 'linacc');
                a(i) = acc(3);
            end
            tc.verifyGreaterThan(min(a), -tc.G*(1 + 1e-3), "Downward acceleration beyond gravity");
        end

        function softContactKeepsCoulombFriction(tc)
            % A box on a soft contact slides down a 25 deg incline (gravity
            % tilted) with friction 0.3 at g (sin - 0.3 cos) and holds on a
            % 10 deg one
            for slope = [25 10]
                g = tc.G*[sind(slope) 0 -cosd(slope)];
                w = tc.newWorld(g);
                phxEngine('set', w, tc.addFloor(w), 'friction', 0.3);
                h = tc.addBox(w, 2, [0 0 0.1], 0.2*[1 1 1], 10);
                phxEngine('set', w, h, 'friction', 0.3);
                phxEngine('set', w, h, 'contact', [1e6 2*sqrt(1e7)]);
                tc.settle(w, 0.2);
                v0 = phxEngine('get', w, h, 'linvel');
                tc.settle(w, 0.5);
                v1 = phxEngine('get', w, h, 'linvel');
                expected = max(0, tc.G*(sind(slope) - 0.3*cosd(slope)));
                tc.verifyEqual((v1(1) - v0(1))/0.5, expected, "AbsTol", 0.01*tc.G, sprintf("Acceleration on %d deg", slope));
            end
        end

        function stiffSoftContactStaysStableAtALargeStep(tc)
            % k 1e9 under a 10 kg box at 1/60 s (omega dt 167), lightly
            % damped (damping ratio 0.1): the box rests, sunk m g / k
            [d, speed] = tc.restOnSoftContact(10, [1e9 0.2*sqrt(1e10)], 1/60, 4);
            tc.verifyEqual(d, 10*tc.G/1e9, "RelTol", 0.1, "Penetration");
            tc.verifyLessThan(speed, 1e-3, "Speed at rest");
        end

        function fastImpactOnASoftContactGainsNoEnergy(tc)
            % A ball dropped 1 m at 1/60 s approaches by 7 cm a step, more
            % than any speculative contact catches: undamped, it bounces
            % back to at most its height
            w = tc.newWorld;
            tc.addFloor(w);
            h = phxEngine('add', w, 'sphere', 2, 0.1, tEngineProtocol.pose([0 0 1.1]), 1, 0.004*[1 1 1]);
            phxEngine('set', w, h, 'contact', [1e5 0]);
            z = nan(1, 120);
            for i = 1:numel(z)
                phxEngine('step', w, 1/60);
                p = phxEngine('get', w, h, 'position');
                z(i) = p(3) - 0.1;
            end
            touch = find(z < 0, 1);
            tc.assertNotEmpty(touch, "The ball never reached the floor.");
            tc.verifyLessThan(max(z(touch:end)), 1.05, "Height of the bounce");
        end

        function rigidContactStiffnessRestoresTheHardContact(tc)
            % Stiffness Inf is the rigid contact of before: a box set soft
            % and then back rests on the floor's surface
            w = tc.newWorld;
            tc.addFloor(w);
            h = tc.addBox(w, 2, [0 0 0.1], 0.2*[1 1 1], 10);
            phxEngine('set', w, h, 'contact', [1e4 0]);
            phxEngine('set', w, h, 'contact', [Inf 0]);
            tc.settle(w, 0.5);
            p = phxEngine('get', w, h, 'position');
            tc.verifyLessThan(0.1 - p(3), 1e-3, "Penetration of the hard contact");
        end

        function diagnosticsTellCompliantFromRigidPenetration(tc)
            % A compliant box sunk m g / k (9.8 mm) is reported as
            % compliantpenetration with its bodies, not as penetration,
            % which a rigid box beside it keeps at the rigid contact's depth
            w = tc.newWorld;
            phxEngine('set', w, uint64(0), 'settings', struct('diagnostics', true));
            floor = tc.addFloor(w);
            soft = tc.addBox(w, 2, [0 0 0.1], 0.2*[1 1 1], 10);
            phxEngine('set', w, soft, 'contact', [1e4 2*sqrt(1e5)]);
            tc.addBox(w, 2, [1 0 0.1], 0.2*[1 1 1], 10);
            tc.settle(w, 1);
            phxEngine('get', w, uint64(0), 'diagnostics');
            tc.settle(w, 0.1);
            D = phxEngine('get', w, uint64(0), 'diagnostics');
            tc.verifyEqual(D.compliantpenetration, 10*tc.G/1e4, "RelTol", 0.02, "Depth of the compliant contact");
            tc.verifyEqual(sort(D.compliantbodies), sort([floor soft]), "Bodies of the compliant contact");
            tc.verifyLessThan(D.penetration, 2e-3, "Penetration of the rigid contacts");
        end

        function contactRejectsBadValues(tc)
            w = tc.newWorld;
            h = tc.addBox(w, 2, [0 0 1], 0.2*[1 1 1]);
            bad = {1e5, [1e5 1 2], [0 1], [-1 1], [NaN 1], [1e5 -1], [1e5 Inf], [1e5 NaN]};
            for i = 1:numel(bad)
                tc.verifyError(@() phxEngine('set', w, h, 'contact', bad{i}), "phx:engine:badArgument", ...
                    "contact " + mat2str(bad{i}));
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

        function softBodyVelocitiesAreSetAndReadInTheWorld(tc)
            % The vertex velocities of a soft body are set and read as 3xN
            % in the world; given all the same, the mesh flies off at that
            % velocity without gravity. A pinned vertex keeps standing
            % still, and a matrix of another size fails.
            [V, F] = tEngineProtocol.cubeMesh(0.2);
            w = tc.newWorld([0 0 0]);
            h = phxEngine('add', w, 'softbody', V, F, tEngineProtocol.pose([0 0 1]), 1, 0);
            tc.verifyEqual(phxEngine('get', w, h, 'velocities'), zeros(3, 8));
            v = [1; 0.5; -0.2];
            phxEngine('set', w, h, 'velocities', repmat(v, 1, 8));
            tc.verifyEqual(phxEngine('get', w, h, 'velocities'), repmat(v, 1, 8), "AbsTol", 1e-6);
            t = 0.2;
            for k = 1:round(t/tc.Dt)
                phxEngine('step', w, tc.Dt);
            end
            X = phxEngine('get', w, h, 'vertices');
            tc.verifyEqual(X, V + [0; 0; 1] + v*t, "AbsTol", 1e-4, "The mesh moves at the velocity.");
            tc.verifyEqual(phxEngine('get', w, h, 'velocities'), repmat(v, 1, 8), "AbsTol", 1e-4);

            phxEngine('set', w, h, 'pinned', 1);
            phxEngine('set', w, h, 'velocities', repmat(v, 1, 8));
            U = phxEngine('get', w, h, 'velocities');
            tc.verifyEqual(U(:, 1), zeros(3, 1), "A pinned vertex stays held.");
            tc.verifyEqual(U(:, 2:end), repmat(v, 1, 7), "AbsTol", 1e-6);
            tc.verifyError(@() phxEngine('set', w, h, 'velocities', zeros(3, 7)), "phx:engine:badArgument");
        end

        function flexibleBodyVelocitiesAreSetAndReadInTheWorld(tc)
            % A flexible body of linear and of quadratic tetrahedra takes
            % its vertex velocities as a soft body does; the Newton solver of
            % the quadratic ones keeps them in double precision. A uniform
            % velocity carries the body along without gravity, unstrained,
            % also when it is set again during the run.
            w = tc.newWorld([0 0 0]);
            for order = [4 10]
                [V, F, T] = tEngineProtocol.tetrahedronMesh(order);
                n = size(V, 2);
                h = phxEngine('add', w, 'flexiblebody', V, F, tEngineProtocol.pose([0 0 1]), 1, [1e6 0.3], T);
                tol = 1e-8 + (order == 4)*1e-5;   % float velocities of Jolt's solver
                tc.verifyEqual(phxEngine('get', w, h, 'velocities'), zeros(3, n), ...
                    sprintf("At rest at first (order %d).", order));
                v = [0.3; -0.4; 1];
                phxEngine('set', w, h, 'velocities', repmat(v, 1, n));
                tc.verifyEqual(phxEngine('get', w, h, 'velocities'), repmat(v, 1, n), "AbsTol", 1e-6);
                t = 0.1;
                for k = 1:round(t/tc.Dt)
                    phxEngine('step', w, tc.Dt);
                end
                X = phxEngine('get', w, h, 'vertices');
                tc.verifyEqual(X, V + [0; 0; 1] + v*t, "AbsTol", tol, ...
                    sprintf("The body moves at the velocity (order %d).", order));
                tc.verifyEqual(phxEngine('get', w, h, 'velocities'), repmat(v, 1, n), "AbsTol", tol);

                phxEngine('set', w, h, 'velocities', repmat(-v, 1, n));
                for k = 1:round(t/tc.Dt)
                    phxEngine('step', w, tc.Dt);
                end
                tc.verifyEqual(phxEngine('get', w, h, 'vertices'), V + [0; 0; 1], "AbsTol", 2*tol, ...
                    sprintf("Set again, the velocity brings it back (order %d).", order));
                phxEngine('remove', w, h);
            end
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

    % Liquids (particle groups with the field 'fluid', protocol 2 step U)
    methods (Test, TestTags = {'Engine'})
        function fluidFieldIsSetReadAndCleared(tc)
            % The fluid struct reads back as set and [] makes the group plain
            % again; incomplete or invalid structs and the densities of a
            % plain group are refused.
            w = tc.newWorld([0 0 0]);
            g = phxEngine('add', w, 'particles', [0; 0; 0], 0.01, 0.01, false);
            tc.verifyEqual(phxEngine('get', w, g, 'fluid'), []);
            tc.verifyError(@() phxEngine('get', w, g, 'density'), "phx:engine:badArgument");
            S = struct("density", 1000, "soundspeed", 20, "viscosity", 0.1);
            phxEngine('set', w, g, 'fluid', S);
            tc.verifyEqual(phxEngine('get', w, g, 'fluid'), S);
            tc.verifyError(@() phxEngine('set', w, g, 'fluid', rmfield(S, "viscosity")), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, g, 'fluid', setfield(S, "extra", 1)), "phx:engine:badArgument"); %#ok<SFLD>
            tc.verifyError(@() phxEngine('set', w, g, 'fluid', setfield(S, "density", -1)), "phx:engine:badArgument"); %#ok<SFLD>
            tc.verifyError(@() phxEngine('set', w, g, 'fluid', setfield(S, "soundspeed", 0)), "phx:engine:badArgument"); %#ok<SFLD>
            tc.verifyError(@() phxEngine('set', w, g, 'fluid', setfield(S, "viscosity", -0.1)), "phx:engine:badArgument"); %#ok<SFLD>
            phxEngine('set', w, g, 'fluid', []);
            tc.verifyEqual(phxEngine('get', w, g, 'fluid'), []);
            tc.verifyError(@() phxEngine('get', w, g, 'pressure'), "phx:engine:badArgument");
        end

        function fluidGroupIsCreatedAsALiquid(tc)
            % add with a fluid struct (step AA): a liquid from the start,
            % its mass the liquid's whatever mass is given (the interior of a
            % lattice reads the rest density), more particles than a world
            % holds bodies; [] makes a plain group, an invalid struct fails.
            r = 0.01;
            S = struct("density", 1000, "soundspeed", 20, "viscosity", 0.1);
            w = tc.newWorld([0 0 0]);
            [x, y, z] = ndgrid(-2:2);
            g = phxEngine('add', w, 'particles', 2*r*[x(:) y(:) z(:)]', r, 0.123, false, S);
            tc.verifyEqual(phxEngine('get', w, g, 'fluid'), S);
            rho = phxEngine('get', w, g, 'density');
            tc.verifyEqual(rho(63), S.density, "RelTol", 1e-6, "The centre of the lattice reads rho0.");
            [x, y, z] = ndgrid(0:49, 0:49, 0:27);
            g = phxEngine('add', w, 'particles', 2*r*[x(:) y(:) z(:)]', r, 0.01, false, S);
            tc.verifyEqual(phxEngine('get', w, g, 'count'), 70000);
            tc.verifySize(phxEngine('get', w, g, 'pressure'), [1 70000]);
            g = phxEngine('add', w, 'particles', [0; 0; 0], r, 0.01, false, []);
            tc.verifyEqual(phxEngine('get', w, g, 'fluid'), []);
            tc.verifyError(@() phxEngine('add', w, 'particles', [0; 0; 0], r, 0.01, false, rmfield(S, "viscosity")), ...
                "phx:engine:badArgument");
        end

        function fluidBoundsRemoveParticlesBeforeTheStep(tc)
            % bounds (step AB): read back as set, none by default and after
            % []; the particles outside the box (turned with its transform)
            % or not finite go before the next step, the others keep their
            % order; reading the densities removes nothing. A plain group
            % has no bounds, and setting fluid [] drops them.
            r = 0.01;
            S = struct("density", 1000, "soundspeed", 20, "viscosity", 0.1);
            w = tc.newWorld([0 0 0]);
            % Five lone particles along x, 0.3 m apart
            P = [0 0.3 0.6 0.9 1.2; zeros(2, 5)];
            g = phxEngine('add', w, 'particles', P, r, 0.01, false, S);
            tc.verifyEqual(phxEngine('get', w, g, 'bounds'), []);
            % A box 1 m along its x and 0.1 m across, centred at x = 0.6 and
            % turned 90 deg about z: unturned it would hold the particles
            % at 0.3, 0.6 and 0.9, turned only the one at 0.6
            T = [0 -1 0 0.6; 1 0 0 0; 0 0 1 0; 0 0 0 1];
            B = struct("transform", T, "size", [1 0.1 Inf]);
            phxEngine('set', w, g, 'bounds', B);
            got = phxEngine('get', w, g, 'bounds');
            tc.verifyEqual(got.transform, T, "AbsTol", 1e-12);
            tc.verifyEqual(got.size, [1 0.1 Inf], "RelTol", 1e-7);
            phxEngine('get', w, g, 'density');
            tc.verifyEqual(phxEngine('get', w, g, 'count'), 5, "Reading the densities removes nothing.");
            phxEngine('step', w, 1e-3);
            tc.verifyEqual(phxEngine('get', w, g, 'positions'), [0.6; 0; 0], "AbsTol", 1e-6, "Only the particle at 0.6 stays.");
            % An axis-aligned box of +-1 m in x: of 0.6, 0, 0.3, 0.9, 1.2
            % the last goes, and the third as not a number; the order of
            % the rest is kept
            phxEngine('add', w, g, P(:, [1 2 4 5]));
            phxEngine('set', w, g, 'bounds', struct("transform", eye(4), "size", [2 1 1]));
            phxEngine('set', w, g, 'positions', [NaN; 0; 0], 3);
            phxEngine('step', w, 1e-3);
            X = phxEngine('get', w, g, 'positions');
            tc.verifyEqual(X(1, :), [0.6 0 0.9], "AbsTol", 1e-6);
            phxEngine('set', w, g, 'bounds', []);
            tc.verifyEqual(phxEngine('get', w, g, 'bounds'), []);
            tc.verifyError(@() phxEngine('set', w, g, 'bounds', struct("transform", eye(4), "size", [1 0 1])), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, g, 'bounds', struct("transform", eye(4))), "phx:engine:badArgument");
            phxEngine('set', w, g, 'bounds', B);
            phxEngine('set', w, g, 'fluid', []);
            tc.verifyError(@() phxEngine('get', w, g, 'bounds'), "phx:engine:badArgument");
            tc.verifyError(@() phxEngine('set', w, g, 'bounds', B), "phx:engine:badArgument");
        end

        function fluidLatticeReadsTheRestDensity(tc)
            % A simple cubic lattice of spacing 2 r: its interior particle
            % reads the rest density, whatever mass the group was made with;
            % the same lattice squeezed by 1 % reads more, with the Tait
            % pressure of that density.
            r = 0.01;
            rho0 = 1000;
            c = 20;
            [x, y, z] = ndgrid(-2:2);
            w = tc.newWorld([0 0 0]);
            for squeeze = [1 0.99]
                P = 2*r*squeeze*[x(:) y(:) z(:)]';
                g = phxEngine('add', w, 'particles', P, r, 0.123, false);
                phxEngine('set', w, g, 'fluid', struct("density", rho0, "soundspeed", c, "viscosity", 0.1));
                rho = phxEngine('get', w, g, 'density');
                p = phxEngine('get', w, g, 'pressure');
                tc.verifySize(rho, [1 125]);
                centre = find(all(P == 0, 1));
                if squeeze == 1
                    % Positions and neighbour offsets in single precision,
                    % and the pressure of that density error
                    tc.verifyEqual(rho(centre), rho0, "RelTol", 1e-6);
                    tc.verifyEqual(p(centre), 0, "AbsTol", rho0*c^2*1e-6);
                else
                    tc.verifyGreaterThan(rho(centre), rho0*1.02);
                    tc.verifyEqual(p(centre), rho0*c^2/7*((rho(centre)/rho0)^7 - 1), "RelTol", 1e-9);
                end
                tc.verifyLessThan(rho(1), rho(centre), "A corner particle lacks neighbours.");
                phxEngine('remove', w, g);
            end
        end

        function fluidForcesConserveMomentum(tc)
            % A squeezed block in free space expands: its outer particles move
            % outwards and the momentum of the block stays zero, up to the
            % single precision of the vertex positions (each step derives
            % the velocities from them).
            r = 0.01;
            [x, y, z] = ndgrid(-2:2);
            P = 2*r*0.97*[x(:) y(:) z(:)]';
            w = tc.newWorld([0 0 0]);
            g = phxEngine('add', w, 'particles', P, r, 0.01, false);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
            for k = 1:10
                phxEngine('step', w, 2.5e-4);
            end
            V = phxEngine('get', w, g, 'velocities');
            tc.verifyEqual(sum(V, 2), [0; 0; 0], "AbsTol", 1e-4*max(abs(V), [], "all"));
            corner = find(all(P == 2*r*0.97*2, 1));
            tc.verifyGreaterThan(V(:, corner), [0; 0; 0], "The corner particle moves outwards.");
        end

        function fluidParticlesDoNotSleep(tc)
            % Particles of a liquid never sleep, whatever the group's
            % sleeping says.
            w = tc.newWorld;
            tc.addFloor(w);
            g = phxEngine('add', w, 'particles', [0 0.3; 0 0; 0.01 0.01], 0.01, 0.01, false);
            phxEngine('set', w, g, 'sleeping', true);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
            for k = 1:round(2/tc.Dt)
                phxEngine('step', w, tc.Dt);
            end
            tc.verifyEqual(phxEngine('get', w, g, 'active'), [true true]);
        end

        function fluidRestsInATank(tc)
            % A column of liquid in a tank of five static plates keeps its
            % level within half a spacing (the free surface settles about a
            % quarter of a spacing lower: its particles lack neighbours above)
            % and its density within 2 %. At rest by the rms speed (about
            % 0.02 m/s): single particles at the floor flick at 0.1-0.4 m/s
            % now and then, so the fastest one of an instant says nothing.
            d = 0.02;
            w = tc.newLiquidWorld;
            P = tc.addTank(w, [0.2 0.2 0.25], d, 0.16);
            g = phxEngine('add', w, 'particles', P, d/2, 0.01, false);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
            for k = 1:4000
                phxEngine('step', w, 2.5e-4);
            end
            X = phxEngine('get', w, g, 'positions');
            top = sort(X(3, :), "descend");
            tc.verifyEqual(mean(top(1:25)) + d/2, 0.16, "AbsTol", d/2, "The level stays.");
            tc.verifyLessThan(max(phxEngine('get', w, g, 'density')), 1000*1.02);
            V = phxEngine('get', w, g, 'velocities');
            tc.verifyLessThan(sqrt(mean(vecnorm(V).^2)), 0.05, "It comes to rest (rms speed, m/s).");
        end

        function fluidStaysInASphereShell(tc)
            % Liquid filling the lower half of a spherical shell (inner face
            % at 0.14 m) stays in it: its particles touch the inner face with
            % their radius d/2 and keep the density near the rest one
            d = 0.02;
            w = tc.newLiquidWorld;
            tEngineProtocol.addShell(w, 0, 0.15, 0.02);
            r = 0.14 - d/2;
            [x, y, z] = ndgrid(-r:d:r, -r:d:r, -r:d:0);
            P = [x(:) y(:) z(:)]';
            P = P(:, vecnorm(P) <= r);
            g = phxEngine('add', w, 'particles', P, d/2, 0.01, false);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
            for k = 1:1200
                phxEngine('step', w, 2.5e-4);
            end
            X = phxEngine('get', w, g, 'positions');
            tc.verifyLessThan(max(vecnorm(X)), r + 2e-3, "Farthest particle centre from the shell centre (m)");
            tc.verifyLessThan(max(phxEngine('get', w, g, 'density')), 1000*1.03);
        end

        function fluidStaysInACapsuleShell(tc)
            % Liquid filling the lower half of a lying capsule shell (inner
            % face 0.19 m from a 0.6 m segment) stays in it
            d = 0.02;
            w = tc.newLiquidWorld;
            tEngineProtocol.addCapsuleShell(w, 0, 0.2, 0.6, 'x', 0.02);
            r = 0.19 - d/2;
            [x, y, z] = ndgrid(-0.3 - r:d:0.3 + r, -r:d:r, -r:d:0);
            P = [x(:) y(:) z(:)]';
            P = P(:, tEngineProtocol.fromSegment(P, 1, 0.3) <= r);
            g = phxEngine('add', w, 'particles', P, d/2, 0.01, false);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
            for k = 1:1200
                phxEngine('step', w, 2.5e-4);
            end
            X = phxEngine('get', w, g, 'positions');
            tc.verifyLessThan(max(tEngineProtocol.fromSegment(X, 1, 0.3)), r + 2e-3, ...
                "Farthest particle centre from the shell's segment (m)");
            tc.verifyLessThan(max(phxEngine('get', w, g, 'density')), 1000*1.03);
        end

        function fluidStaysInABoxShell(tc)
            % Liquid filling the lower half of a box shell with a sharp cavity
            % of half extents [0.13 0.08 0.1] stays in it
            d = 0.02;
            w = tc.newLiquidWorld;
            tEngineProtocol.addBoxShell(w, 0, [0.15 0.1 0.12], 0.02, 0);
            limit = [0.13 0.08 0.1] - d/2;
            [x, y, z] = ndgrid(-limit(1):d:limit(1), -limit(2):d:limit(2), -limit(3):d:0);
            g = phxEngine('add', w, 'particles', [x(:) y(:) z(:)]', d/2, 0.01, false);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
            for k = 1:1200
                phxEngine('step', w, 2.5e-4);
            end
            X = phxEngine('get', w, g, 'positions');
            tc.verifyLessThan(max(abs(X), [], 2)', limit + 2e-3, "Largest |x|, |y| and |z| of the particle centres (m)");
            tc.verifyLessThan(max(phxEngine('get', w, g, 'density')), 1000*1.03);
        end

        function fluidFloatsADynamicBox(tc)
            % A box of half the liquid's density floats: it neither sinks nor
            % leaves the surface, its draft near half its height. (The
            % boundary model gives about 10 % less at this resolution;
            % FLUID.md.)
            d = 0.02;
            a = 0.12;
            w = tc.newLiquidWorld;
            P = tc.addTank(w, [0.24 0.24 0.3], d, 0.12);
            P = P(:, ~(max(abs(P(1:2, :)), [], 1) < a/2 + d/2 & P(3, :) > 0.12 - a/2 - d/2));
            box = tc.addBox(w, 2, [0 0 0.12], a*[1 1 1], 500*a^3);
            g = phxEngine('add', w, 'particles', P, d/2, 0.01, false);
            phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.3));
            for k = 1:round(0.6/2.5e-4)
                phxEngine('step', w, 2.5e-4);
            end
            X = phxEngine('get', w, g, 'positions');
            far = max(abs(X(1:2, :)), [], 1) > 0.09;
            top = sort(X(3, far), "descend");
            level = mean(top(1:20)) + d/2;
            p = phxEngine('get', w, box, 'position');
            draft = level - (p(3) - a/2);
            tc.verifyEqual(draft, a/2, "RelTol", 0.25, "The box floats at about half its height.");
            tc.verifyEqual(phxEngine('get', w, box, 'linvel'), [0 0 0], "AbsTol", 0.05, "It has come to rest.");
        end

        function fluidResultsDoNotDependOnThreads(tc)
            % A liquid hitting a dynamic box moves the same, bit for bit, on
            % one thread and on four.
            d = 0.02;
            X = cell(1, 2);
            threads = [1 4];
            for t = 1:2
                w = tc.newLiquidWorld;
                phxEngine('set', w, uint64(0), 'settings', struct('threads', threads(t)));
                P = tc.addTank(w, [0.4 0.1 0.2], d, 0.16);
                P = P(:, P(1, :) < -0.1);
                tc.addBox(w, 2, [0.1 0 0.03], [0.06 0.06 0.06], 0.1);
                g = phxEngine('add', w, 'particles', P, d/2, 0.01, false);
                phxEngine('set', w, g, 'fluid', struct("density", 1000, "soundspeed", 20, "viscosity", 0.1));
                for k = 1:400
                    phxEngine('step', w, 2.5e-4);
                end
                X{t} = phxEngine('get', w, g, 'positions');
            end
            tc.verifyEqual(X{2}, X{1});
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

        function w = newLiquidWorld(tc)
            % A world with gravity and the contact tolerances PHX sets for
            % LengthScale 0.1, as the centimetre particles of a liquid need
            % (at Jolt's defaults they rest 9 mm deep in the walls)
            w = tc.newWorld;
            phxEngine('set', w, uint64(0), 'settings', struct('margin', 0.004, 'penetrationslop', 1e-4, ...
                'speculativedistance', 0.002, 'tolerancelength', 0.01));
        end

        function P = addTank(tc, w, inner, d, height)
            % Static tank of five 0.05 m plates around the inner size
            % [x y z], its floor top at z = 0 and centred on the z axis;
            % returns a simple cubic lattice of spacing d filling it up to
            % the given height (3xN), the particles d/2 from the plates
            t = 0.05;
            tc.addBox(w, 0, [0 0 -t/2], [inner(1:2) + 2*t, t], 0, [0 0 0]);
            for s = [-1 1]
                tc.addBox(w, 0, [s*(inner(1) + t)/2, 0, inner(3)/2], [t, inner(2) + 2*t, inner(3)], 0, [0 0 0]);
                tc.addBox(w, 0, [0, s*(inner(2) + t)/2, inner(3)/2], [inner(1), t, inner(3)], 0, [0 0 0]);
            end
            [x, y, z] = ndgrid(-inner(1)/2 + d/2 : d : inner(1)/2 - d/4, ...
                -inner(2)/2 + d/2 : d : inner(2)/2 - d/4, d/2 : d : height - d/4);
            P = [x(:) y(:) z(:)]';
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

        function [d, speed] = restOnSoftContact(tc, mass, contact, dt, duration)
            % A 0.2 m box of the given mass with the given contact, its
            % bottom just touching a rigid floor, stepped at dt; returns
            % its penetration and its largest speed over the last quarter
            w = tc.newWorld;
            tc.addFloor(w);
            h = tc.addBox(w, 2, [0 0 0.1], 0.2*[1 1 1], mass);
            phxEngine('set', w, h, 'contact', contact);
            n = round(duration/dt);
            d = nan(1, n);
            speed = d;
            for i = 1:n
                phxEngine('step', w, dt);
                p = phxEngine('get', w, h, 'position');
                d(i) = 0.1 - p(3);
                speed(i) = norm(phxEngine('get', w, h, 'linvel'));
            end
            tail = round(0.75*n):n;
            d = mean(d(tail));
            speed = max(speed(tail));
        end

        function [rebound, depth] = dropOnSoftContact(tc, ballContact, floorContact, dt)
            % A 1 kg ball of radius 0.1 m dropped 0.2 m onto the floor,
            % each with the given contact, stepped at dt for 0.6 s; returns
            % the rebound (the speed of the bounce's apex height over the
            % impact speed) and the largest penetration
            r = 0.1;
            w = tc.newWorld;
            phxEngine('set', w, tc.addFloor(w), 'contact', floorContact);
            h = phxEngine('add', w, 'sphere', 2, r, tEngineProtocol.pose([0 0 r + 0.2]), 1, 0.4*r^2*[1 1 1]);
            phxEngine('set', w, h, 'contact', ballContact);
            z = nan(1, round(0.6/dt));
            for i = 1:numel(z)
                phxEngine('step', w, dt);
                p = phxEngine('get', w, h, 'position');
                z(i) = p(3) - r;
            end
            touch = find(z < 0, 1);
            leave = touch - 1 + find(z(touch:end) >= 0, 1);
            tc.assertNotEmpty(leave, "The ball did not leave the floor.");
            depth = -min(z);
            rebound = sqrt(max(z(leave:end))/0.2);
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
        function [rebound, depth] = kelvinVoigtDrop(m, k, c, height, g)
            % The exact unilateral Kelvin-Voigt impact of a mass dropped
            % from the given height onto a rigid support: the force
            % max(0, k x + c x') of the penetration x; returns the rebound
            % (as dropOnSoftContact measures it) and the largest penetration
            v0 = sqrt(2*g*height);
            f = @(~, y) [y(2); g - max(0, k*y(1) + c*y(2))/m];
            % The contact ends where its force falls to 0 (at x = 0 without
            % damping, before it with damping)
            options = odeset("RelTol", 1e-10, "AbsTol", 1e-12, ...
                "Events", @(~, y) tEngineProtocol.contactForceFalls(y, k, c));
            [~, y] = ode45(f, [0 1], [0; v0], options);
            depth = max(y(:, 1));
            % After the release the mass flies from x_l at the speed -v_l
            apex = y(end, 2)^2/(2*g) - y(end, 1);
            rebound = sqrt(max(apex, 0)/height);
        end

        function [value, terminal, direction] = contactForceFalls(y, k, c)
            % Event of kelvinVoigtDrop: the contact force falling to 0
            value = k*y(1) + c*y(2);
            terminal = true;
            direction = -1;
        end

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
            % A shell rests on its outer face, radius + thickness/2
            s.sphereShell = row(@(w, P) phxEngine('add', w, 'sphereshell', 2, 0.3, 0.04, P, 1, I), 0.32);
            % A capsule shell lying rests on the outer radius, standing on its
            % end cap at half its cylinder plus that radius
            s.capsuleShellLying = row(@(w, P) phxEngine('add', w, 'capsuleshell', 2, 0.2, 0.6, 'x', 0.04, P, 1, I), 0.22);
            s.capsuleShellStanding = row(@(w, P) phxEngine('add', w, 'capsuleshell', 2, 0.2, 0.6, 'z', 0.04, P, 1, I), 0.52);
            % A box shell rests on its flat outer bottom
            s.boxShell = row(@(w, P) phxEngine('add', w, 'boxshell', 2, [0.3 0.2 0.4], 0.04, 0.01, P, 1, I), 0.4);
        end

        function h = addBoxShell(w, type, half, thickness, innerRadius, position, mass)
            % Box shell of the given outer half extents, wall and rounding
            % of the cavity; a dynamic one gets the inertia of a solid box
            arguments
                w
                type
                half (1, 3) double
                thickness
                innerRadius
                position (1, 3) double = [0 0 0]
                mass (1, 1) double = 1
            end
            h = phxEngine('add', w, 'boxshell', type, half, thickness, innerRadius, ...
                tEngineProtocol.pose(position), mass, mass*(sum(half.^2) - half.^2)/3);
        end

        function d = fromBox(P, half)
            % Signed distances of the points P (3xN) from the centred box of
            % the given half extents (negative inside)
            q = abs(P) - half(:);
            d = vecnorm(max(q, 0), 2, 1) + min(max(q, [], 1), 0);
        end

        function h = addCapsuleShell(w, type, radius, height, axis, thickness, position, mass)
            % Capsule shell of the given middle radius, cylinder height, axis
            % and wall; a dynamic one gets a rough isotropic inertia
            arguments
                w
                type
                radius
                height
                axis
                thickness
                position (1, 3) double = [0 0 0]
                mass (1, 1) double = 1
            end
            h = phxEngine('add', w, 'capsuleshell', type, radius, height, axis, thickness, ...
                tEngineProtocol.pose(position), mass, mass*(radius^2 + height^2/12)*[1 1 1]);
        end

        function d = fromSegment(P, axis, halfHeight)
            % Distances of the points P (3xN) from the segment of the given
            % half height along the axis (1, 2 or 3) through the origin
            Q = zeros(size(P));
            Q(axis, :) = max(min(P(axis, :), halfHeight), -halfHeight);
            d = vecnorm(P - Q, 2, 1);
        end

        function h = addShell(w, type, radius, thickness, position, mass)
            % Spherical shell of the given middle radius and wall; a dynamic
            % one gets the inertia of a thin shell, 2/3 m r^2
            arguments
                w
                type
                radius
                thickness
                position (1, 3) double = [0 0 0]
                mass (1, 1) double = 1
            end
            h = phxEngine('add', w, 'sphereshell', type, radius, thickness, tEngineProtocol.pose(position), ...
                mass, 2/3*mass*radius^2*[1 1 1]);
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

        function [V, F, T] = tetrahedronMesh(order)
            % One tetrahedron of 0.2 m legs: vertices 3xN, the outward
            % faces of its corners 3x4 and its vertex indices 4x1 (order 4)
            % or, with the vertices of the edges 12 23 13 14 24 34, 10x1
            % (order 10)
            C = 0.2*[0 0 0; 1 0 0; 0 1 0; 0 0 1]';
            F = [1 3 2; 1 2 4; 2 3 4; 1 4 3]';
            if order == 4
                V = C;
            else
                E = [1 2; 2 3; 1 3; 1 4; 2 4; 3 4];
                V = [C, (C(:, E(:, 1)) + C(:, E(:, 2)))/2];
            end
            T = (1:order)';
        end
    end

end
