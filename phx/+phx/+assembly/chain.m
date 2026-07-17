function parts = chain(varargin)
%phx.assembly.chain Build a chain of rigid links along a polyline
%
%   parts = phx.assembly.chain(points) creates a chain of rigid links in the
%   current axes: between every two consecutive rows of the N-by-3 points
%   matrix a link body is stretched, and at every interior point the two
%   neighbouring links are connected by a joint. Like with phx.Body, a
%   target axes to draw into may be passed as an optional first argument:
%   phx.assembly.chain(ax, points, ___), where an empty target ([])
%   creates the parts without graphics. With N points the chain
%   has N-1 links and N-2 interior joints; a two-row matrix gives a single
%   free rod. The parts are returned in a struct with the fields:
%   - links: the link bodies (1x(N-1) phx.Body, in the order of the points)
%   - joints: cell row of the joints, interior joints first, then the
%     anchor joints (a cell array, because revolute and spherical joints
%     may mix in one chain)
%   - anchors: static mount bodies created by the Anchor option (possibly
%     empty)
%
%   parts = phx.assembly.chain(points, Name, Value, ...) specifies options
%   as name-value pairs:
%   - Shape: link geometry, one of "capsule" (default), "cylinder" or
%     "box". The link body frame sits at the segment centre with its z axis
%     running along the segment. A capsule is shortened by one diameter so
%     that its rounded tips meet exactly at the joint points.
%   - Diameter: thickness of the links (the side length of a box link),
%     default 0.1.
%   - Axis: rotation axis of the joints. Either a single 1x3 vector used
%     by all joints, or an N-by-3 matrix with one row per point (rows 1 and
%     N are used only by the anchor joints). A non-zero row creates a
%     phx.RevoluteJoint around that axis, a [0 0 0] row creates a
%     phx.SphericalJoint. The axes are expressed in the same frame as the
%     points. Default [0 0 0], i.e. spherical joints everywhere.
%   - Anchor: pins an end of the chain to a static mount ball, one of
%     "none" (default), "start", "end" or "both". The mount joint uses the
%     axis row of the anchored point.
%   - Density: density of the link material (kg/m^3), default 1000; the
%     link masses and inertias follow from the geometry.
%   - Color: common color of all parts, default [0.75 0.73 0.7].
%   - Friction: friction coefficients of all links, default [0.5 0 0].
%   - Position, Orientation, EulerAngles: pose of the chain frame (in
%     which the points and axes are expressed) in the world, default at
%     the world origin. Same conventions as phx.Body.
%
%   Neighbouring links do not collide with each other (the connecting
%   joint disables their mutual collisions, see MutualCollisions), but the
%   rest of the chain does collide with itself, so it can be piled up.
%
%   Example - a double pendulum swinging in the x-z plane:
%       parts = phx.assembly.chain([0 0 0; 0.4 0 0; 0.8 0 0], ...
%           "Anchor", "start", "Axis", [0 1 0]);
%       sim = phx.Simulation(gca);
%       sim.step(5, 2000, 5);
%
%   See also phx.assembly.arena, phx.RevoluteJoint, phx.SphericalJoint,
%   phx.Rope, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    [ax, args] = axesTarget(varargin);
    parts = build(ax, args{:});
end

function parts = build(ax, points, Options)
    arguments
        ax
        points (:, 3) double {mustBeFinite}
        Options.Shape (1, 1) string {mustBeMember(Options.Shape, ["capsule", "cylinder", "box"])} = "capsule"
        Options.Diameter (1, 1) double {mustBePositive} = 0.1
        Options.Axis double {mustBeFinite} = [0 0 0]
        Options.Anchor (1, 1) string {mustBeMember(Options.Anchor, ["none", "start", "end", "both"])} = "none"
        Options.Density (1, 1) double {mustBePositive} = 1000
        Options.Color (1, 3) double = [1 1 1]
        Options.Friction (1, 3) double {mustBeGreaterThanOrEqual(Options.Friction, 0)} = [0.5 0 0]
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
    end

    TBase = basePose(Options, "chain");

    % Validate the polyline and the joint axes
    n = size(points, 1);
    if n < 2
        error("phx:chain:invalidPoints", "The points matrix must have at least two rows (got %d).", n);
    end
    segments = diff(points);
    lengths = sqrt(sum(segments.^2, 2));
    if any(lengths < 10*eps)
        error("phx:chain:invalidPoints", "Consecutive points must not coincide (rows %d and %d).", ...
            find(lengths < 10*eps, 1), find(lengths < 10*eps, 1) + 1);
    end

    axes_ = Options.Axis;
    if isequal(size(axes_), [1 3])
        axes_ = repmat(axes_, n, 1);
    elseif ~isequal(size(axes_), [n 3])
        error("phx:chain:invalidAxis", "Axis must be a 1x3 vector or an N-by-3 matrix matching the %d points (got %dx%d).", ...
            n, size(axes_, 1), size(axes_, 2));
    end

    d = Options.Diameter;
    if Options.Shape == "capsule" && any(lengths <= d)
        error("phx:chain:linkTooShort", "Capsule links must be longer than their diameter (%g); link %d has length %g.", ...
            d, find(lengths <= d, 1), min(lengths(lengths <= d)));
    end

    % Create the links: body frame at the segment centre, z along the
    % segment; the whole assembly is built in the chain frame and then
    % rigidly moved by the base pose
    if isa(ax, "missing")
        ax = gca;
    end
    nLinks = n - 1;
    links = phx.Body.empty;
    R = cell(1, nLinks);
    for i = 1:nLinks
        switch Options.Shape
            case "capsule"
                shape = {"Capsule", "Diameter", d, "Height", lengths(i) - d};
            case "cylinder"
                shape = {"Cylinder", "Diameter", d, "Height", lengths(i)};
            case "box"
                shape = {"Box", "Size", [d d lengths(i)]};
        end
        links(i) = phx.Body(ax, "Name", "link" + i, ...
            "Shape", [shape {"Color", Options.Color, "Density", Options.Density}], ...
            "Friction", Options.Friction);

        R{i} = alignZ(segments(i, :)/lengths(i));
        T = eye(4);
        T(1:3, 1:3) = R{i};
        T(1:3, 4) = (points(i, :) + points(i + 1, :))'/2;
        links(i).Transform = TBase*T;
    end

    % Interior joints at the points 2..N-1; the joint axis is expressed in
    % the local frames of both connected links
    joints = {};
    for i = 1:nLinks - 1
        pa = [0 0 lengths(i)/2];      % lower end of link i = point i+1
        pb = [0 0 -lengths(i + 1)/2]; % upper end of link i+1 = point i+1
        joints{end + 1} = makeJoint(links(i), links(i + 1), pa, pb, ...
            axes_(i + 1, :), R{i}, R{i + 1}); %#ok<AGROW> few joints
    end

    % Anchor joints pin the chain ends to static mount balls; the mount
    % keeps the chain-frame orientation, so the joint axes stay consistent
    anchors = phx.Body.empty;
    if ismember(Options.Anchor, ["start", "both"])
        anchors(end + 1) = makeAnchor(ax, TBase, points(1, :), Options);
        joints{end + 1} = makeJoint(anchors(end), links(1), ...
            [0 0 0], [0 0 -lengths(1)/2], axes_(1, :), eye(3), R{1});
    end
    if ismember(Options.Anchor, ["end", "both"])
        anchors(end + 1) = makeAnchor(ax, TBase, points(n, :), Options);
        joints{end + 1} = makeJoint(anchors(end), links(nLinks), ...
            [0 0 0], [0 0 lengths(nLinks)/2], axes_(n, :), eye(3), R{nLinks});
    end

    parts.links = links;
    parts.joints = joints;
    parts.anchors = anchors;
end

function R = alignZ(dir)
% Rotation matrix turning the z axis into the given unit direction
    c = cross([0 0 1], dir);
    s = norm(c);
    if s < 1e-12
        R = phx.internal.Math.rotAA([1 0 0], pi*(dir(3) < 0));
    else
        R = phx.internal.Math.rotAA(c/s, atan2(s, dir(3)));
    end
end

function j = makeJoint(bodyA, bodyB, pa, pb, axis, RA, RB)
% Revolute joint around the world-space axis, or a spherical joint for a
% zero axis; the axis is transformed into the local frames of the bodies
    if any(axis)
        axis = axis/norm(axis);
        j = phx.RevoluteJoint(bodyA, bodyB, "PointA", pa, "PointB", pb, ...
            "AxisA", (RA'*axis')', "AxisB", (RB'*axis')');
    else
        j = phx.SphericalJoint(bodyA, bodyB, "PointA", pa, "PointB", pb);
    end
end

function b = makeAnchor(ax, TBase, point, Options)
% Static mount ball at the given chain-frame point, oriented like the
% chain frame (its local space stands in for the chain frame in joints)
    T = TBase;
    T(1:3, 4) = T(1:3, 4) + T(1:3, 1:3)*point';
    b = phx.Body(ax, "Type", "static", "Name", "anchor", ...
        "Shape", {"Sphere", "Diameter", Options.Diameter, "Color", Options.Color}, ...
        "Collisions", false);
    b.Transform = T;
end
