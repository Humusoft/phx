classdef tAssembly < matlab.unittest.TestCase
%tAssembly Tests for the phx.assembly scene builders.
%
%   Untagged tests cover the option validation and need neither graphics
%   nor the engine. Graphics-tagged tests verify the built object
%   structure and geometry (arena: inner dimensions, floor surface through
%   the origin, closed corners; chain: link poses along the polyline,
%   joint types per axis row, anchors; wall: brick sizes, the running bond
%   and both ways of ending the shifted rows) and the shared base-pose
%   options. The Engine-tagged tests verify that a moving body cannot
%   escape the arena, that an anchored chain swings as a pendulum without
%   falling apart and that a brick wall stands on its own yet collapses
%   when hit.
%
%   See also phx.assembly.arena, phx.assembly.chain, phx.assembly.wall

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function conflictingBaseRotationRaisesError(tc)
            tc.verifyError(@() phx.assembly.arena( ...
                "Orientation", [0 -1 0; 1 0 0; 0 0 1], "EulerAngles", [0 0 pi/2]), ...
                "phx:arena:conflictingOptions");
        end

        function invalidSizeRaisesError(tc)
            tc.verifyError(@() phx.assembly.arena("Size", [2 -1 0.5]), ...
                "MATLAB:validators:mustBePositive");
            tc.verifyError(@() phx.assembly.arena("Thickness", 0), ...
                "MATLAB:validators:mustBePositive");
        end

        function invalidScatterInputsRaiseErrors(tc)
            tc.verifyError(@() phx.assembly.scatter("box", 5), ...
                "phx:scatter:invalidShape");
            tc.verifyError(@() phx.assembly.scatter({"Sphere"}, 5, ...
                "Color", zeros(3)), "phx:scatter:invalidColor");
            tc.verifyError(@() phx.assembly.scatter({"Sphere"}, 10, ...
                "Region", [0.1 0.1 0], "Spacing", 1), "phx:scatter:regionFull");
            tc.verifyError(@() phx.assembly.scatter({"Sphere"}, 2, ...
                "Orientation", [0 -1 0; 1 0 0; 0 0 1], "EulerAngles", [0 0 pi/2]), ...
                "phx:scatter:conflictingOptions");
        end

        function invalidChainInputsRaiseErrors(tc)
            tc.verifyError(@() phx.assembly.chain([0 0 0]), ...
                "phx:chain:invalidPoints");
            tc.verifyError(@() phx.assembly.chain([0 0 0; 0 0 0; 1 0 0]), ...
                "phx:chain:invalidPoints");
            tc.verifyError(@() phx.assembly.chain([0 0 0; 1 0 0], "Axis", eye(3)), ...
                "phx:chain:invalidAxis");
            tc.verifyError(@() phx.assembly.chain([0 0 0; 0.05 0 0], "Diameter", 0.1), ...
                "phx:chain:linkTooShort");
            tc.verifyError(@() phx.assembly.chain([0 0 0; 1 0 0], ...
                "Orientation", [0 -1 0; 1 0 0; 0 0 1], "EulerAngles", [0 0 pi/2]), ...
                "phx:chain:conflictingOptions");
        end

        function invalidWallInputsRaiseErrors(tc)
            % Open even rows hold one brick less, so they need a second column
            tc.verifyError(@() phx.assembly.wall("Columns", 1, ...
                "HalfBricks", false), "phx:wall:invalidColumns");
            tc.verifyError(@() phx.assembly.wall("Rows", 2.5), ...
                "MATLAB:validators:mustBeInteger");
            tc.verifyError(@() phx.assembly.wall("Size", [2 0 1]), ...
                "MATLAB:validators:mustBePositive");
            tc.verifyError(@() phx.assembly.wall("RandomTint", 1.5), ...
                "MATLAB:validators:mustBeInRange");
            tc.verifyError(@() phx.assembly.wall("RandomTint", -0.1), ...
                "MATLAB:validators:mustBeInRange");
            tc.verifyError(@() phx.assembly.wall( ...
                "Orientation", [0 -1 0; 1 0 0; 0 0 1], "EulerAngles", [0 0 pi/2]), ...
                "phx:wall:conflictingOptions");
        end
    end

    methods (Test, TestTags = {'Graphics'})
        function arenaEnclosesTheInnerSpace(tc)
            tc.prepareAxes;
            s = [3 2 0.6];
            t = 0.15;
            parts = phx.assembly.arena("Size", s, "Thickness", t);

            tc.verifySize(parts.walls, [1 4]);
            all = [parts.floor parts.walls];
            for b = all
                tc.verifyClass(b, "phx.Body");
                tc.verifyEqual(string(b.Type), "static");
                tc.verifyEqual(b.Orientation, eye(3), "AbsTol", 1e-12);
            end

            % The floor extends under the walls and its top surface passes
            % through the arena origin
            tc.verifyEqual(parts.floor.Position, [0 0 -t/2], "AbsTol", 1e-12);
            tc.verifyEqual(tc.bodyShape(parts.floor).Size, ...
                [s(1) + 2*t, s(2) + 2*t, t], "AbsTol", 1e-12);

            % Wall order -x, +x, -y, +y; inner faces at +-s/2, tops at s(3)
            expectedPos = {[-(s(1) + t)/2, 0, s(3)/2], [(s(1) + t)/2, 0, s(3)/2], ...
                [0, -(s(2) + t)/2, s(3)/2], [0, (s(2) + t)/2, s(3)/2]};
            expectedSize = {[t, s(2), s(3)], [t, s(2), s(3)], ...
                [s(1) + 2*t, t, s(3)], [s(1) + 2*t, t, s(3)]};
            for i = 1:4
                tc.verifyEqual(parts.walls(i).Position, expectedPos{i}, "AbsTol", 1e-12);
                tc.verifyEqual(tc.bodyShape(parts.walls(i)).Size, expectedSize{i}, "AbsTol", 1e-12);
            end
        end

        function basePoseTransformsTheWholeArena(tc)
            tc.prepareAxes;
            ref = phx.assembly.arena;
            TBase = eye(4);
            TBase(1:3, 1:3) = phx.internal.Math.rot321([0.2 -0.3 0.4]);
            TBase(1:3, 4) = [0.5 -1 2];
            moved = phx.assembly.arena("Position", [0.5 -1 2], "EulerAngles", [0.2 -0.3 0.4]);

            refParts = [ref.floor ref.walls];
            movedParts = [moved.floor moved.walls];
            for i = 1:numel(refParts)
                tc.verifyEqual(movedParts(i).Transform, ...
                    TBase*refParts(i).Transform, "AbsTol", 1e-12, ...
                    "Base pose was not applied to arena part #" + i + ".");
            end
        end
        function chainLinksFollowThePolyline(tc)
            tc.prepareAxes;
            % An L-shaped chain with a revolute elbow and a spherical wrist,
            % anchored at the start
            pts = [0 0 0; 0.4 0 0; 0.4 0 -0.3];
            axis = [0 1 0; 0 1 0; 0 0 0];
            parts = phx.assembly.chain(pts, "Axis", axis, "Anchor", "start", ...
                "Diameter", 0.05);

            tc.verifySize(parts.links, [1 2]);
            tc.verifySize(parts.joints, [1 2]); % elbow + anchor mount
            tc.verifySize(parts.anchors, [1 1]);
            tc.verifyEqual(string(parts.anchors.Type), "static");

            % Body frames sit at the segment centres with z along the segment
            tc.verifyEqual(parts.links(1).Position, [0.2 0 0], "AbsTol", 1e-12);
            tc.verifyEqual(parts.links(2).Position, [0.4 0 -0.15], "AbsTol", 1e-12);
            tc.verifyEqual(parts.links(1).Orientation*[0; 0; 1], [1; 0; 0], "AbsTol", 1e-12);
            tc.verifyEqual(parts.links(2).Orientation*[0; 0; 1], [0; 0; -1], "AbsTol", 1e-12);

            % A capsule is shortened by one diameter, tips meet at the points
            shape = tc.bodyShape(parts.links(1));
            tc.verifyClass(shape, "phx.shape.Capsule");
            tc.verifyEqual(shape.Height, 0.35, "AbsTol", 1e-12);

            % Joint types follow the axis rows and their anchors coincide
            % with the polyline points in the world
            tc.verifyClass(parts.joints{1}, "phx.RevoluteJoint"); % point 2
            tc.verifyClass(parts.joints{2}, "phx.RevoluteJoint"); % anchor, point 1
            tc.verifyJointAt(parts.joints{1}, pts(2, :));
            tc.verifyJointAt(parts.joints{2}, pts(1, :));

            % The default zero axis makes the joints spherical
            sph = phx.assembly.chain(pts, "Diameter", 0.05);
            tc.verifyClass(sph.joints{1}, "phx.SphericalJoint");
        end

        function scatterPlacesBodiesInTheRegion(tc)
            tc.prepareAxes;
            n = 25;
            region = [2 2 1];
            spacing = 0.3;
            offset = [1 -1 0.5];
            palette = hsv(n);

            rng(7);
            bodies = phx.assembly.scatter({"Sphere", "Diameter", 0.2}, n, ...
                "Region", region, "Spacing", spacing, "Position", offset, ...
                "RandomOrientation", true, "Color", palette);

            tc.verifySize(bodies, [1 n]);
            p = vertcat(bodies.Position) - offset;
            tc.verifyLessThanOrEqual(max(abs(p(:, 1:2)), [], 1), region(1:2)/2 + 1e-12, ...
                "A body centre left the region in the ground plane.");
            tc.verifyGreaterThanOrEqual(min(p(:, 3)), -1e-12, "A body centre lies below the region.");
            tc.verifyLessThanOrEqual(max(p(:, 3)), region(3) + 1e-12, "A body centre lies above the region.");

            % All pairwise centre distances respect the spacing
            dist = sqrt(sum((permute(p, [1 3 2]) - permute(p, [3 1 2])).^2, 3));
            dist(1:n + 1:end) = inf;
            tc.verifyGreaterThanOrEqual(min(dist(:)), spacing - 1e-12, ...
                "Two bodies were placed closer than the spacing.");

            % Per-body colors and random orientations were applied
            tc.verifyEqual(bodies(3).Color, palette(3, :), "AbsTol", 1e-12);
            tc.verifyGreaterThan(norm(bodies(1).Orientation - eye(3)), 1e-6, ...
                "RandomOrientation left the identity orientation.");

            % The layout is reproducible through the global generator
            rng(7);
            again = phx.assembly.scatter({"Sphere", "Diameter", 0.2}, n, ...
                "Region", region, "Spacing", spacing, "Position", offset, ...
                "RandomOrientation", true, "Color", palette);
            tc.verifyEqual(vertcat(again.Transform), vertcat(bodies.Transform), ...
                "AbsTol", 1e-12, "The same seed did not reproduce the layout.");
        end

        function wallLaysBricksInARunningBond(tc)
            tc.prepareAxes;
            s = [2 0.2 1];
            nRows = 4;
            nCols = 5;
            len = s(1)/nCols;
            hgt = s(3)/nRows;
            density = 1800;

            for halfBricks = [true false]
                bricks = phx.assembly.wall("Size", s, "Rows", nRows, ...
                    "Columns", nCols, "HalfBricks", halfBricks, "Density", density);

                % Odd rows hold nCols bricks, even rows one more (closed by
                % half bricks) or one less (left open)
                nEven = nCols - 1 + 2*halfBricks;
                tc.verifySize(bricks, [1 2*nCols + 2*nEven]);
                tc.verifyEqual(string(bricks(1).Type), "dynamic");
                tc.verifyEqual(bricks(1).Mass, density*len*s(2)*hgt, "RelTol", 1e-9);

                names = string({bricks.Name});
                for row = 1:nRows
                    rowBricks = bricks(startsWith(names, "brick" + row + "_"));
                    pos = vertcat(rowBricks.Position);
                    sz = zeros(numel(rowBricks), 3);
                    for i = 1:numel(rowBricks)
                        sz(i, :) = tc.bodyShape(rowBricks(i)).Size;
                    end
                    n = numel(rowBricks);

                    % Every brick is as thick as the wall, one row high and
                    % centred in the wall plane
                    tc.verifyEqual(sz(:, 2), repmat(s(2), n, 1), "AbsTol", 1e-12);
                    tc.verifyEqual(sz(:, 3), repmat(hgt, n, 1), "AbsTol", 1e-12);
                    tc.verifyEqual(pos(:, 2), zeros(n, 1), "AbsTol", 1e-12);
                    tc.verifyEqual(pos(:, 3), repmat((row - 0.5)*hgt, n, 1), "AbsTol", 1e-12);

                    % The bricks come back left to right and lie side by side
                    % without gaps or overlaps
                    left = pos(:, 1) - sz(:, 1)/2;
                    right = pos(:, 1) + sz(:, 1)/2;
                    tc.verifyEqual(left(2:end), right(1:end - 1), "AbsTol", 1e-12, ...
                        "Bricks of row " + row + " are not laid side by side.");

                    if mod(row, 2) == 1
                        % Unshifted row: full bricks over the whole length
                        tc.verifySize(rowBricks, [1 nCols]);
                        tc.verifyEqual(sz(:, 1), repmat(len, n, 1), "AbsTol", 1e-12);
                        tc.verifyEqual([left(1) right(end)], [-s(1)/2 s(1)/2], "AbsTol", 1e-12);
                    elseif halfBricks
                        % Shifted row closed by a half brick at each side, so
                        % the ends of the wall are flush and the joints of the
                        % row sit at the middle of the bricks below
                        tc.verifySize(rowBricks, [1 nCols + 1]);
                        tc.verifyEqual(sz([1 end], 1), [len/2; len/2], "AbsTol", 1e-12);
                        tc.verifyEqual(sz(2:end - 1, 1), repmat(len, n - 2, 1), "AbsTol", 1e-12);
                        tc.verifyEqual([left(1) right(end)], [-s(1)/2 s(1)/2], "AbsTol", 1e-12);
                    else
                        % Shifted row left open: full bricks only, inset by
                        % half a brick at both ends
                        tc.verifySize(rowBricks, [1 nCols - 1]);
                        tc.verifyEqual(sz(:, 1), repmat(len, n, 1), "AbsTol", 1e-12);
                        tc.verifyEqual([left(1) right(end)], ...
                            [-s(1)/2 + len/2, s(1)/2 - len/2], "AbsTol", 1e-12);
                    end
                end
                delete(bricks);
            end

            % The base pose moves the whole wall rigidly
            ref = phx.assembly.wall("Rows", 2, "Columns", 3);
            TBase = eye(4);
            TBase(1:3, 1:3) = phx.internal.Math.rot321([0.2 -0.3 0.4]);
            TBase(1:3, 4) = [0.5 -1 2];
            moved = phx.assembly.wall("Rows", 2, "Columns", 3, ...
                "Position", [0.5 -1 2], "EulerAngles", [0.2 -0.3 0.4]);
            for i = 1:numel(ref)
                tc.verifyEqual(moved(i).Transform, TBase*ref(i).Transform, ...
                    "AbsTol", 1e-12, "Base pose was not applied to brick #" + i + ".");
            end
        end

        function wallRandomTintShadesTheBricks(tc)
            tc.prepareAxes;
            color = [0.7 0.35 0.25];
            tint = 0.3;

            rng(3);
            bricks = phx.assembly.wall("Rows", 4, "Columns", 4, ...
                "Color", color, "RandomTint", tint);
            shades = vertcat(bricks.Color);

            % Every brick keeps the hue and is darkened by its own factor
            % within the requested band
            factors = shades(:, 1)/color(1);
            tc.verifyEqual(shades, factors.*color, "AbsTol", 1e-12, ...
                "A brick was tinted per channel instead of darkened.");
            tc.verifyGreaterThan(min(factors), 1 - tint, "A brick was darkened too much.");
            tc.verifyLessThanOrEqual(max(factors), 1, "A brick came out brighter than Color.");
            tc.verifyGreaterThan(max(factors) - min(factors), 0.05, ...
                "The bricks did not get different shades.");

            % The tinting is reproducible through the global generator
            rng(3);
            again = phx.assembly.wall("Rows", 4, "Columns", 4, ...
                "Color", color, "RandomTint", tint);
            tc.verifyEqual(vertcat(again.Color), shades, "AbsTol", 1e-12, ...
                "The same seed did not reproduce the shades.");

            % No tint means one uniform color and no draw from the generator
            rng(3);
            expected = rand;
            rng(3);
            plain = phx.assembly.wall("Rows", 2, "Columns", 2, ...
                "Color", color, "RandomTint", 0);
            tc.verifyEqual(vertcat(plain.Color), repmat(color, numel(plain), 1), ...
                "AbsTol", 1e-12, "RandomTint 0 did not leave the color alone.");
            tc.verifyEqual(rand, expected, "AbsTol", 1e-12, ...
                "An untinted wall consumed random numbers.");
        end

        function explicitAxesTargetIsHonored(tc)
            % All builders take an optional leading axes target and leave
            % the current axes untouched
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            axTarget = subplot(1, 2, 2, "Parent", f);
            axCurrent = subplot(1, 2, 1, "Parent", f);
            axes(axCurrent);

            parts = phx.assembly.arena(axTarget);
            links = phx.assembly.chain(axTarget, [0 0 0; 0.4 0 0]).links;
            balls = phx.assembly.scatter(axTarget, {"Sphere", "Diameter", 0.2}, 3);
            bricks = phx.assembly.wall(axTarget, "Rows", 2, "Columns", 2);

            for b = [parts.floor parts.walls links balls bricks]
                tc.verifyEqual(b.ParentAxes, axTarget, ...
                    "A body was not drawn into the requested axes.");
            end
            tc.verifyEqual(gca, axCurrent, "The current axes changed.");
        end

        function emptyTargetBuildsWithoutGraphics(tc)
            % An explicit [] target follows the phx.Body([], ...) headless
            % convention: no parent axes and no figure gets created
            nFigures = numel(findall(groot, "Type", "figure"));

            parts = phx.assembly.arena([]);
            ch = phx.assembly.chain([], [0 0 0; 0.4 0 0], "Anchor", "start");
            balls = phx.assembly.scatter([], {"Sphere", "Diameter", 0.2}, 3);
            bricks = phx.assembly.wall([], "Rows", 2, "Columns", 2);

            for b = [parts.floor parts.walls ch.links ch.anchors balls bricks]
                tc.verifyEmpty(b.ParentAxes, "A headless body got a parent axes.");
            end
            tc.verifyEqual(numel(findall(groot, "Type", "figure")), nFigures, ...
                "A headless build created a figure.");
        end

        function basePoseTransformsTheWholeChain(tc)
            tc.prepareAxes;
            pts = [0 0 0; 0.4 0 0; 0.8 0 0.2];
            ref = phx.assembly.chain(pts, "Anchor", "both");
            TBase = eye(4);
            TBase(1:3, 1:3) = phx.internal.Math.rot321([0.2 -0.3 0.4]);
            TBase(1:3, 4) = [0.5 -1 2];
            moved = phx.assembly.chain(pts, "Anchor", "both", ...
                "Position", [0.5 -1 2], "EulerAngles", [0.2 -0.3 0.4]);

            for i = 1:numel(ref.links)
                tc.verifyEqual(moved.links(i).Transform, ...
                    TBase*ref.links(i).Transform, "AbsTol", 1e-12, ...
                    "Base pose was not applied to link #" + i + ".");
            end
            for i = 1:numel(ref.anchors)
                tc.verifyEqual(moved.anchors(i).Position, ...
                    (TBase(1:3, 1:3)*ref.anchors(i).Position' + TBase(1:3, 4))', ...
                    "AbsTol", 1e-12, "Base pose was not applied to anchor #" + i + ".");
            end
        end
    end

    methods (Test, TestTags = {'Engine'})
        function arenaKeepsBodiesInside(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
            tc.prepareAxes;

            s = [1 1 0.4];
            phx.assembly.arena("Size", s, "Thickness", 0.05);
            ball = phx.Body(gca, "Position", [0 0 0.2], ...
                "Shape", {"Sphere", "Radius", 0.1}, ...
                "LinearVelocity", [3 2 0]);

            sim = phx.Simulation(gca);
            tc.addTeardown(@() delete(sim));
            sim.step(2, 400); % dt = 5 ms, no redraw

            % The ball ricocheted off the walls but stayed in the inner space
            p = ball.Position;
            tc.verifyTrue(all(isfinite(p)), "The ball state diverged.");
            tc.verifyLessThan(abs(p(1:2)), s(1:2)/2, ...
                "The ball escaped the arena sideways.");
            tc.verifyGreaterThan(p(3), 0, "The ball fell through the floor.");
            tc.verifyLessThan(p(3), s(3), "The ball jumped over the walls.");
        end

        function anchoredChainSwingsAsAPendulum(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
            tc.prepareAxes;

            % A horizontal double pendulum released from rest
            parts = phx.assembly.chain([0 0 0; 0.3 0 0; 0.6 0 0], ...
                "Anchor", "start", "Axis", [0 1 0], "Diameter", 0.06);
            sim = phx.Simulation(gca);
            tc.addTeardown(@() delete(sim));

            % The tip must swing through the lower half-circle at some point
            tipLow = 0;
            for k = 1:10
                sim.step(0.2, 100); % dt = 2 ms for the constraint chain
                tip = phx.internal.transformPoint(parts.links(2).Transform, ...
                    [0 0 0.15]);
                tc.verifyTrue(all(isfinite(tip)), "The chain state diverged.");
                tipLow = min(tipLow, tip(3));
            end
            tc.verifyLessThan(tipLow, -0.3, "The pendulum did not swing down.");

            % The pendulum stays in its swing plane (y = 0) and in one piece
            tc.verifyEqual(tip(2), 0, "AbsTol", 1e-3, ...
                "The revolute joints did not keep the swing planar.");
            for i = 1:numel(parts.joints)
                tc.verifyJointAt(parts.joints{i}, [], 0.01);
            end
        end

        function brickWallStandsAndCollapses(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
            tc.prepareAxes;

            phx.Body(gca, "Type", "static", "Position", [0 0 -0.1], ...
                "Shape", {"Box", "Size", [6 2 0.2]});
            bricks = phx.assembly.wall(gca, "Size", [2 0.2 1], ...
                "Rows", 8, "Columns", 8);
            sim = phx.Simulation(gca);
            tc.addTeardown(@() delete(sim));

            % Left alone the wall carries its own weight: the rows settle by
            % a fraction of a brick and come to rest
            z0 = vertcat(bricks.Position);
            z0 = z0(:, 3);
            sim.step(2, 400); % dt = 5 ms
            z1 = vertcat(bricks.Position);
            z1 = z1(:, 3);
            tc.verifyTrue(all(isfinite(z1)), "The wall state diverged.");
            tc.verifyLessThan(max(abs(z1 - z0)), 0.01, "The wall settled too much.");
            tc.verifyLessThan(max(vecnorm(vertcat(bricks.LinearVelocity), 2, 2)), 0.05, ...
                "The wall did not come to rest.");

            % A heavy ball thrown at it brings the wall down
            ball = phx.Body(gca, "Position", [-2 0 0.5], ...
                "Shape", {"Sphere", "Radius", 0.2, "Density", 4000}, ...
                "LinearVelocity", [10 0 0]);
            sim.addObjects(ball); % the ball joined the scene after the build
            sim.step(1.5, 300);
            z2 = vertcat(bricks.Position);
            z2 = z2(:, 3);
            tc.verifyGreaterThan(max(z0 - z2), 0.2, ...
                "The wall did not collapse after the impact.");
        end
    end

    methods (Access = private)
        function verifyJointAt(tc, j, point, tol)
            % Both world-space anchor points of the joint coincide (and
            % optionally sit at the given point)
            if nargin < 4
                tol = 1e-9;
            end
            pa = phx.internal.transformPoint(j.Parents{1}.Transform, j.PointA);
            pb = phx.internal.transformPoint(j.Parents{2}.Transform, j.PointB);
            tc.verifyEqual(pa, pb, "AbsTol", tol, ...
                "Anchor points of joint '" + j.Name + "' do not coincide.");
            if nargin > 2 && ~isempty(point)
                tc.verifyEqual(pa, point, "AbsTol", tol, ...
                    "Joint '" + j.Name + "' does not sit at the expected point.");
            end
        end

        function prepareAxes(tc)
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            axes(f);
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

end
