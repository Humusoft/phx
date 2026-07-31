classdef tImport < matlab.unittest.TestCase
%tImport Tests for the phx.assembly.import robot importer.
%
%   Untagged tests cover the file/XML-level error handling and need neither
%   graphics nor the engine. Graphics-tagged tests verify the parsed object
%   structure, name sanitization, the link world poses (pinning the URDF
%   rpy convention against an independent rotation implementation), the
%   base-pose options and the mesh path resolution. The Engine-tagged test verifies that an imported
%   robot is collected by phx.Simulation from the bodies struct alone and
%   that its joints hold together during simulation.
%
%   See also phx.assembly.import

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function missingFileRaisesError(tc)
            tc.verifyError(@() phx.assembly.import("definitely_missing_robot.urdf"), ...
                "phx:import:fileNotFound");
        end

        function invalidXMLRaisesError(tc)
            file = tc.writeFile("broken.urdf", "<robot name='x'><link");
            tc.verifyError(@() phx.assembly.import(file), "phx:import:parseError");
        end

        function nonRobotRootRaisesError(tc)
            file = tc.writeFile("notrobot.urdf", "<?xml version='1.0'?><model name='x'/>");
            tc.verifyError(@() phx.assembly.import(file), "phx:import:invalidRoot");
        end

        function unknownLinkRaisesError(tc)
            file = tc.writeFile("badlink.urdf", "<robot name='x'>" + ...
                "<link name='a'/>" + ...
                "<joint name='j' type='fixed'><parent link='a'/><child link='ghost'/></joint>" + ...
                "</robot>");
            tc.verifyError(@() phx.assembly.import(file), "phx:import:unknownLink");
        end

        function conflictingBaseRotationRaisesError(tc)
            tc.verifyError(@() phx.assembly.import(tc.fixtureFile, ...
                "Orientation", [0 -1 0; 1 0 0; 0 0 1], "EulerAngles", [0 0 pi/2]), ...
                "phx:import:conflictingOptions");
        end

        function crossBranchOptionRaisesError(tc)
            % Mesh options on a URDF and MeshPath on a mesh file are rejected
            % with a phx: identifier, not MATLAB's generic TooManyInputs.
            urdf = tc.fixtureFile;
            for opt = ["Scale", "Envelope", "FlipFaces", "Density"]
                tc.verifyError(@() phx.assembly.import(urdf, opt, 1), ...
                    "phx:import:unsupportedOption", opt);
            end
            stl = fullfile(fileparts(mfilename("fullpath")), "..", "examples", "res", "cat.stl");
            tc.assumeTrue(isfile(stl), "cat.stl fixture not available");
            tc.verifyError(@() phx.assembly.import([], stl, "MeshPath", tempdir), ...
                "phx:import:unsupportedOption");
        end

        function unknownOptionRaisesError(tc)
            tc.verifyError(@() phx.assembly.import(tc.fixtureFile, "Bogus", 1), ...
                "phx:import:unsupportedOption");
        end

        function branchOptionsAreAccepted(tc)
            % Drift guard: every name the dispatcher allows must really reach
            % its branch, so the two lists cannot silently diverge.
            stl = fullfile(fileparts(mfilename("fullpath")), "..", "examples", "res", "cat.stl");
            tc.assumeTrue(isfile(stl), "cat.stl fixture not available");
            meshOpts = {"Scale", [0.02 0.02 0.02], "Envelope", "box", ...
                "FlipFaces", true, "Density", 500};
            for i = 1:2:numel(meshOpts)
                tc.verifyWarningFree(@() phx.assembly.import([], stl, meshOpts{i:i+1}), meshOpts{i});
            end
            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));
            tc.verifyWarningFree(@() phx.assembly.import([], tc.fixtureFile, "MeshPath", tempdir));
            baseOpts = {"Position", [0 0 1], "Orientation", eye(3), "EulerAngles", [0 0 0]};
            for i = 1:2:numel(baseOpts)
                tc.verifyWarningFree(@() phx.assembly.import([], stl, baseOpts{i:i+1}), baseOpts{i});
                tc.verifyWarningFree(@() phx.assembly.import([], tc.fixtureFile, baseOpts{i:i+1}), baseOpts{i});
            end
        end

        function kinematicLoopRaisesError(tc)
            file = tc.writeFile("loop.urdf", "<robot name='x'>" + ...
                "<link name='a'/><link name='b'/>" + ...
                "<joint name='j1' type='fixed'><parent link='a'/><child link='b'/></joint>" + ...
                "<joint name='j2' type='fixed'><parent link='b'/><child link='a'/></joint>" + ...
                "</robot>");
            tc.verifyError(@() phx.assembly.import(file), "phx:import:invalidTree");
        end
    end

    methods (Test, TestTags = {'Graphics'})
        function importsExpectedObjects(tc)
            [bodies, joints] = tc.importFixture;

            tc.verifyEqual(string(fieldnames(bodies)), ...
                ["base"; "upper_arm"; "forearm"; "hand"; "tool_frame"]);
            tc.verifyEqual(string(fieldnames(joints)), ...
                ["shoulder_joint"; "elbow_fix"; "wrist_slide"; "tool_mount"]);

            tc.verifyClass(joints.shoulder_joint, "phx.RevoluteJoint");
            tc.verifyClass(joints.elbow_fix, "phx.FixedJoint");
            tc.verifyClass(joints.wrist_slide, "phx.PrismaticJoint");
            tc.verifyClass(joints.tool_mount, "phx.FixedJoint");

            % Original URDF names are preserved in the Name property
            tc.verifyEqual(bodies.upper_arm.Name, "upper-arm");
            tc.verifyEqual(bodies.tool_frame.Name, "tool-frame");
            tc.verifyEqual(joints.shoulder_joint.Name, "shoulder-joint");
            tc.verifyEqual(joints.elbow_fix.Name, "elbow.fix");

            % Mass properties come from the inertial elements
            tc.verifyEqual(bodies.base.Mass, 5.0);
            tc.verifyEqual(bodies.base.Inertia, [0.04 0.04 0.075], "AbsTol", 1e-12);

            % A geometry-less dummy link gets placeholder mass properties
            tc.verifyLessThanOrEqual(bodies.tool_frame.Mass, 1e-3);
        end

        function explicitAxesTargetIsHonored(tc)
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            axTarget = subplot(1, 2, 2, "Parent", f);
            axCurrent = subplot(1, 2, 1, "Parent", f);
            axes(axCurrent);

            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));
            bodies = phx.assembly.import(axTarget, tc.fixtureFile);

            tc.verifyEqual(bodies.base.ParentAxes, axTarget, ...
                "The robot was not drawn into the requested axes.");
            tc.verifyEqual(gca, axCurrent, "The current axes changed.");
        end

        function emptyTargetImportsWithoutGraphics(tc)
            % An explicit [] target follows the phx.Body([], ...) headless
            % convention: no parent axes and no figure gets created
            nFigures = numel(findall(groot, "Type", "figure"));
            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));

            bodies = phx.assembly.import([], tc.fixtureFile);

            tc.verifyEmpty(bodies.base.ParentAxes, ...
                "A headless import got a parent axes.");
            tc.verifyEqual(numel(findall(groot, "Type", "figure")), nFigures, ...
                "A headless import created a figure.");
        end

        function planarJointMapsToGenericJoint(tc)
            % A planar joint has no direct PHX equivalent yet, so it is
            % approximated by a phx.GenericJoint with a warning
            urdf = tc.writeFile("planar.urdf", "<robot name='s'>" + ...
                "<link name='a'/><link name='b'/>" + ...
                "<joint name='j' type='planar'>" + ...
                "<parent link='a'/><child link='b'/><axis xyz='0 0 1'/>" + ...
                "</joint></robot>");
            tc.prepareAxes;
            [~, joints] = tc.verifyWarning(@() phx.assembly.import(urdf), ...
                "phx:import:substitutedJoint", ...
                "The planar joint did not warn about the substitution.");
            tc.verifyClass(joints.j, "phx.GenericJoint", ...
                "The planar joint was not approximated by a phx.GenericJoint.");
        end

        function floatingJointCreatesNoJoint(tc)
            % A floating joint imposes no constraint, so no joint is created:
            % the importer warns and the child link is left free, with no
            % entry in the returned joints struct
            urdf = tc.writeFile("floating.urdf", "<robot name='s'>" + ...
                "<link name='a'/><link name='b'/>" + ...
                "<joint name='j' type='floating'>" + ...
                "<parent link='a'/><child link='b'/></joint></robot>");
            tc.prepareAxes;
            [bodies, joints] = tc.verifyWarning(@() phx.assembly.import(urdf), ...
                "phx:import:floatingJoint", ...
                "The floating joint did not warn that it was dropped.");

            tc.verifyEqual(string(fieldnames(joints)), string.empty(0, 1), ...
                "A floating joint should not appear in the joints struct.");
            % both links are still imported as free bodies
            tc.verifyEqual(sort(string(fieldnames(bodies))), ["a"; "b"], ...
                "The floating joint's links were not both imported.");
        end

        function prismaticSlidingAxisFollowsJointAxis(tc)
            % The imported prismatic joint slides along the URDF joint axis:
            % AxisA and AxisB - the local Z of both joint frames - map to one
            % and the same world direction, equal to the URDF axis in the joint
            % (child) frame
            [bodies, joints] = tc.importFixture; %#ok<ASGLU> bodies keep the joints alive
            j = joints.wrist_slide;
            tc.verifyClass(j, "phx.PrismaticJoint");

            TShoulder = tImport.trf([0 0 0.1], [0.1 0.2 0.3]);
            TElbow = TShoulder*tImport.trf([0 0 0.4], [0 -0.4 0]);
            TWrist = TElbow*tImport.trf([0.1 0 0], [0 0 1.0]);
            expected = TWrist(1:3, 1:3)*[1; 0; 0]; % URDF axis "1 0 0" in world

            axisA = j.Parents{1}.Transform(1:3, 1:3)*j.AxisA';
            axisB = j.Parents{2}.Transform(1:3, 1:3)*j.AxisB';
            tc.verifyEqual(axisA, expected, "AbsTol", 1e-12, ...
                "Slider axis on body A does not match the URDF joint axis.");
            tc.verifyEqual(axisB, expected, "AbsTol", 1e-12, ...
                "Slider axis on body B does not match the URDF joint axis.");
        end

        function linkPosesFollowKinematicTree(tc)
            bodies = tc.importFixture;

            % Joint frames composed with an independent implementation of the
            % URDF rpy convention (extrinsic X-Y-Z: Rz(yaw)*Ry(pitch)*Rx(roll))
            TShoulder = tImport.trf([0 0 0.1], [0.1 0.2 0.3]);
            TElbow = TShoulder*tImport.trf([0 0 0.4], [0 -0.4 0]);
            TWrist = TElbow*tImport.trf([0.1 0 0], [0 0 1.0]);
            TTool = TWrist*tImport.trf([0.1 0 0], [0 0 0]);

            % Body frames sit at the origin of the first visual geometry
            tc.verifyEqual(bodies.base.Transform, ...
                tImport.trf([0 0 0.05], [0 0 0]), "AbsTol", 1e-12);
            tc.verifyEqual(bodies.upper_arm.Transform, ...
                TShoulder*tImport.trf([0 0 0.2], [0 0 0]), "AbsTol", 1e-12);
            tc.verifyEqual(bodies.forearm.Transform, TElbow, "AbsTol", 1e-12);
            tc.verifyEqual(bodies.hand.Transform, ...
                TWrist*tImport.trf([0.05 0 0], [0 1.5707963 0]), "AbsTol", 1e-12);
            tc.verifyEqual(bodies.tool_frame.Transform, TTool, "AbsTol", 1e-12);
        end

        function basePoseTransformsTheWholeRobot(tc)
            % The Position/Orientation/EulerAngles options rigidly move the
            % complete zero-pose assembly, root link frame first
            ref = tc.importFixture;
            TBase = tImport.trf([0.5 -1 2], [0.2 -0.3 0.4]);

            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));
            moved = phx.assembly.import(tc.fixtureFile, ...
                "Position", TBase(1:3, 4)', "EulerAngles", [0.2 -0.3 0.4]);
            rotated = phx.assembly.import(tc.fixtureFile, ...
                "Orientation", TBase(1:3, 1:3));

            names = string(fieldnames(ref));
            for name = names'
                tc.verifyEqual(moved.(name).Transform, ...
                    TBase*ref.(name).Transform, "AbsTol", 1e-12, ...
                    "Base pose was not applied to link '" + name + "'.");
                TR = rotated.(name).Transform;
                TExpected = ref.(name).Transform;
                TExpected(1:3, :) = TBase(1:3, 1:3)*TExpected(1:3, :);
                tc.verifyEqual(TR, TExpected, "AbsTol", 1e-12, ...
                    "Orientation option was not applied to link '" + name + "'.");
            end
        end

        function jointFramesAreConsistentWithInitialPoses(tc)
            % Regression guard: joint frames that disagree with the initial
            % body poses would deform the robot at the first simulation step
            [bodies, joints] = tc.importFixture; %#ok<ASGLU> bodies keep the joints alive
            tc.verifyAnchorsCoincide(joints, 1e-9);
        end

        function meshGeometryResolvesPackageURI(tc)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            meshDir = fullfile(folder, "mypkg", "meshes");
            mkdir(meshDir);
            tc.writeTetrahedronSTL(fullfile(meshDir, "part.stl"));
            urdf = fullfile(folder, "mesh_robot.urdf");
            tc.writeTextFile(urdf, "<robot name='m'><link name='part'><visual><geometry>" + ...
                "<mesh filename='package://mypkg/meshes/part.stl' scale='2 2 2'/>" + ...
                "</geometry></visual></link></robot>");

            tc.prepareAxes;
            bodies = phx.assembly.import(urdf, "MeshPath", folder);

            shape = tc.bodyShape(bodies.part);
            tc.verifyClass(shape, "phx.shape.Mesh");
            tc.verifyEqual(shape.Scale, [2 2 2]);
            tc.verifyFalse(shape.Centered);
            tc.verifyEqual(string(shape.Envelope), "convex");
        end

        function capsuleExtensionMapsToCapsuleShape(tc)
            % <capsule> is a common vendor extension of URDF; length is the
            % cylindrical part of the shape (phx.shape.Capsule.Height)
            urdf = tc.writeFile("capsule.urdf", "<robot name='c'><link name='limb'>" + ...
                "<visual><geometry><capsule radius='0.04' length='0.2'/></geometry></visual>" + ...
                "</link></robot>");
            tc.prepareAxes;
            bodies = phx.assembly.import(urdf);

            shape = tc.bodyShape(bodies.limb);
            tc.verifyClass(shape, "phx.shape.Capsule");
            tc.verifyEqual(shape.Radius, 0.04);
            tc.verifyEqual(shape.Height, 0.2);
        end

        function missingMeshRaisesError(tc)
            urdf = tc.writeFile("nomesh.urdf", "<robot name='m'><link name='part'>" + ...
                "<visual><geometry><mesh filename='package://nope/part.stl'/></geometry></visual>" + ...
                "</link></robot>");
            tc.prepareAxes;
            tc.verifyError(@() phx.assembly.import(urdf), "phx:import:meshNotFound");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function planarJointLocksThePlaneNormal(tc)
            % A planar joint is approximated by a phx.GenericJoint that frees
            % the two in-plane translations and the rotation about the plane
            % normal, and locks the rest. With the normal along world Z, the
            % child must not translate along Z (the locked normal) under
            % gravity, which pulls straight along it.
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");

            urdf = tc.writeFile("planar.urdf", "<robot name='p'>" + ...
                "<link name='base'/><link name='puck'/>" + ...
                "<joint name='slab' type='planar'>" + ...
                "<parent link='base'/><child link='puck'/><axis xyz='0 0 1'/>" + ...
                "</joint></robot>");
            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));

            [bodies, joints] = phx.assembly.import([], urdf);
            bodies.base.Type = "static";
            tc.verifyClass(joints.slab, "phx.GenericJoint");

            sim = phx.Simulation(bodies);
            tc.addTeardown(@() delete(sim));
            z0 = bodies.puck.Position(3);
            sim.step(0.5, 250);

            p = bodies.puck.Position;
            tc.verifyEqual(p(3), z0, "AbsTol", 1e-3, ...
                "The planar joint did not lock translation along the plane normal.");
            tc.verifyTrue(all(isfinite(p)), "The planar-jointed body left the scene.");
        end

        function importedRobotHoldsTogether(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");

            [bodies, joints] = tc.importFixture;

            % The bodies struct alone is enough; joints are collected
            % automatically through the object hierarchy
            sim = phx.Simulation(bodies);
            tc.addTeardown(@() delete(sim));
            jointNames = fieldnames(joints);
            for i = 1:numel(jointNames)
                tc.verifyNotEmpty(joints.(jointNames{i}).ObjectHandle, ...
                    "Joint '" + jointNames{i} + "' was not added to the engine.");
            end

            relBefore = tc.fixedRelativePoses(joints);

            % Free fall of the whole (unanchored) arm; dt = 2 ms keeps the
            % constraint network stable
            sim.step(0.4, 200);

            % Joint anchor points still coincide
            tc.verifyAnchorsCoincide(joints, 0.01);

            % Bodies welded by fixed joints keep their relative pose
            relAfter = tc.fixedRelativePoses(joints);
            for i = 1:numel(relBefore)
                dp = norm(relBefore{i}(1:3, 4) - relAfter{i}(1:3, 4));
                R = relBefore{i}(1:3, 1:3)'*relAfter{i}(1:3, 1:3);
                da = acos(min(1, (trace(R) - 1)/2));
                tc.verifyLessThan(dp, 0.01, "A fixed pair drifted apart.");
                tc.verifyLessThan(da, 0.05, "A fixed pair rotated relatively.");
            end

            % Nothing explodes: a 0.4 s free fall stays well within 10 m
            bodyNames = fieldnames(bodies);
            for i = 1:numel(bodyNames)
                p = bodies.(bodyNames{i}).Position;
                tc.verifyTrue(all(isfinite(p)) && norm(p) < 10, ...
                    "Body '" + bodyNames{i} + "' left the expected region.");
            end
        end
    end

    methods (Access = private)
        function file = fixtureFile(~)
            file = fullfile(fileparts(mfilename("fullpath")), "fixtures", "three_link_arm.urdf");
        end

        function [bodies, joints] = importFixture(tc)
            tc.prepareAxes;
            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));
            [bodies, joints] = phx.assembly.import(tc.fixtureFile);
        end

        function prepareAxes(tc)
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            axes(f);
        end

        function file = writeFile(tc, name, content)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            file = fullfile(folder, name);
            tc.writeTextFile(file, content);
        end

        function verifyAnchorsCoincide(tc, joints, tol)
            names = fieldnames(joints);
            for i = 1:numel(names)
                j = joints.(names{i});
                pa = phx.internal.transformPoint(j.Parents{1}.Transform, j.PointA);
                pb = phx.internal.transformPoint(j.Parents{2}.Transform, j.PointB);
                tc.verifyEqual(pa, pb, "AbsTol", tol, ...
                    "Anchor points of joint '" + names{i} + "' do not coincide.");
            end
        end

        function rel = fixedRelativePoses(~, joints)
            names = ["elbow_fix", "tool_mount"];
            rel = cell(1, numel(names));
            for i = 1:numel(names)
                j = joints.(names(i));
                rel{i} = j.Parents{1}.Transform\j.Parents{2}.Transform;
            end
        end

        function shape = bodyShape(~, body)
            shape = [];
            for ch = body.Graphics.Children'
                s = getappdata(ch, "phxShape");
                if ~isempty(s)
                    shape = s;
                    return
                end
            end
        end
    end

    methods (Static, Access = private)
        function T = trf(xyz, rpy)
            % Independent reference for the URDF origin transform:
            % rpy is the extrinsic X-Y-Z rotation Rz(yaw)*Ry(pitch)*Rx(roll)
            cx = cos(rpy(1)); sx = sin(rpy(1));
            cy = cos(rpy(2)); sy = sin(rpy(2));
            cz = cos(rpy(3)); sz = sin(rpy(3));
            Rx = [1 0 0; 0 cx -sx; 0 sx cx];
            Ry = [cy 0 sy; 0 1 0; -sy 0 cy];
            Rz = [cz -sz 0; sz cz 0; 0 0 1];
            T = eye(4);
            T(1:3, 1:3) = Rz*Ry*Rx;
            T(1:3, 4) = xyz;
        end

        function writeTextFile(file, content)
            fid = fopen(file, "w");
            fwrite(fid, content);
            fclose(fid);
        end

        function writeTetrahedronSTL(file)
            lines = [
                "solid part"
                "  facet normal 0 0 -1"
                "    outer loop"
                "      vertex 0 0 0"
                "      vertex 0 1 0"
                "      vertex 1 0 0"
                "    endloop"
                "  endfacet"
                "  facet normal 0 -1 0"
                "    outer loop"
                "      vertex 0 0 0"
                "      vertex 1 0 0"
                "      vertex 0 0 1"
                "    endloop"
                "  endfacet"
                "  facet normal -1 0 0"
                "    outer loop"
                "      vertex 0 0 0"
                "      vertex 0 0 1"
                "      vertex 0 1 0"
                "    endloop"
                "  endfacet"
                "  facet normal 0.5774 0.5774 0.5774"
                "    outer loop"
                "      vertex 1 0 0"
                "      vertex 0 1 0"
                "      vertex 0 0 1"
                "    endloop"
                "  endfacet"
                "endsolid part"
                ];
            tImport.writeTextFile(file, strjoin(lines, newline));
        end
    end

end
