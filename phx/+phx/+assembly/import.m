function [bodies, joints] = import(varargin)
%phx.assembly.import Import a robot model from a URDF file
%
%   bodies = phx.assembly.import(file) reads a robot description from the
%   given URDF (Unified Robot Description Format) file and creates a
%   phx.Body object for every link of the robot. The bodies are returned in
%   a struct whose field names are the link names (made valid MATLAB
%   identifiers by matlab.lang.makeValidName; the original URDF name is
%   preserved in the Name property of each object). The bodies are drawn
%   into the current axes and are placed at the world poses of the robot in
%   its zero (home) configuration, with the base (root link frame) at the
%   world origin unless a pose is given by the Position and Orientation or
%   EulerAngles options. Like with phx.Body, a target axes to draw into
%   may be passed as an optional first argument:
%   phx.assembly.import(ax, file, ___), where an empty target ([]) creates
%   the bodies without graphics for headless simulations.
%
%   [bodies, joints] = phx.assembly.import(file) also returns all created
%   joints in a struct whose field names are the joint names.
%
%   [___] = phx.assembly.import(___, Name, Value, ...) specifies additional
%   options as name-value pairs:
%   - Position: world position of the base (root link frame), default
%     [0 0 0].
%   - Orientation: world rotation of the base as a 3x3 rotation matrix,
%     default eye(3). Same convention as phx.Body.Orientation.
%   - EulerAngles: world rotation of the base as Euler angles for the
%     z->y->x order, an alternative to Orientation (do not combine the
%     two). Same convention as phx.Body.EulerAngles.
%   - MeshPath: root folder used to resolve "package://" URIs and relative
%     file names of mesh geometries. Mesh files are always searched relative
%     to the folder of the URDF file as well.
%
%   To simulate the imported robot, pass the bodies to a simulation; the
%   joints are collected automatically through the object hierarchy. All
%   bodies are created as dynamic; anchor the robot by making its base
%   static:
%       [bodies, joints] = phx.assembly.import("robot.urdf");
%       bodies.base.Type = "static";
%       sim = phx.Simulation(bodies);
%       sim.step(1, 500, 5);
%
%   URDF to PHX mapping:
%   - Every link becomes a phx.Body. The body frame is placed at the origin
%     of the link's first visual geometry (or first collision geometry when
%     the link has no visual) because PHX shapes are always centred at the
%     body origin. Links without any geometry get a small placeholder
%     sphere.
%   - Geometries map to box -> phx.shape.Box, cylinder -> phx.shape.Cylinder,
%     sphere -> phx.shape.Sphere and mesh -> phx.shape.STL or phx.shape.OBJ
%     (selected by the file extension, with the scale attribute applied).
%     Mesh shapes use the "convex" collision envelope. The capsule element
%     <capsule radius="r" length="l"/> - a common vendor extension of the
%     URDF format - maps to phx.shape.Capsule.
%   - Joints of type "revolute" and "continuous" become phx.RevoluteJoint,
%     "prismatic" joints become phx.PrismaticJoint (sliding along the joint
%     axis) and "fixed" joints become phx.FixedJoint. Joint frames are derived
%     from the initial link poses, so the assembled robot is in equilibrium
%     at its zero configuration.
%   - Masses are taken from the inertial elements; the inertia tensor is
%     expressed in the axes of the body frame and its diagonal is used as
%     the Inertia property.
%
%   Limitations of the importer:
%   - Joint limits of revolute and prismatic joints are ignored.
%   - Joints of type "planar" and "floating" are replaced by a
%     phx.FixedJoint (their degrees of freedom are locked in the zero pose)
%     and the warning phx:import:substitutedJoint is issued.
%   - All joints are passive; no motors or transmissions are created.
%   - Products of inertia (off-diagonal tensor elements) and the offset of
%     the centre of mass from the body frame are ignored. Links with a zero
%     or missing mass get small placeholder mass properties so that they
%     remain dynamic.
%   - Only the first visual geometry of a link is imported and it also
%     serves as the collision shape. Additional geometry elements are
%     reported by the warning phx:import:extraGeometry.
%   - Of the material definitions only the diffuse color is applied.
%
%   See also phx.Body, phx.RevoluteJoint, phx.PrismaticJoint, phx.FixedJoint, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    [ax, args] = axesTarget(varargin);
    [bodies, joints] = build(ax, args{:});
end

function [bodies, joints] = build(ax, file, Options)
    arguments
        ax
        file (1, 1) string
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
        Options.MeshPath (1, 1) string = ""
    end

    % Resolve the requested base pose
    TBase = basePose(Options, "import");

    % Locate and parse the XML document
    if ~isfile(file)
        error("phx:import:fileNotFound", "URDF file '%s' was not found.", file);
    end
    info = dir(file);
    urdfDir = string(info.folder);
    fullFile = fullfile(info.folder, info.name);

    % readstruct parses the XML natively (no JVM/Xerces start-up, unlike
    % xmlread, which costs tens of seconds on its first call) into a MATLAB
    % struct: element attributes become fields with an "Attribute" suffix and
    % repeated child elements become struct arrays.
    try
        root = readstruct(fullFile, "FileType", "xml", "AttributeSuffix", "Attribute");
    catch err
        error("phx:import:parseError", "Could not parse '%s' as an XML document: %s", file, err.message);
    end

    % readstruct discards the root element's own tag name, so the <robot>
    % check reads it back from the file text
    rootName = rootElementName(fullFile);
    if rootName ~= "robot"
        error("phx:import:invalidRoot", "File '%s' is not a robot description (root element is <%s>, expected <robot>).", file, rootName);
    end

    % Parse materials, links and joints
    materials = parseMaterials(root);

    linkEls = getChildren(root, "link");
    nLinks = numel(linkEls);
    links = cell(1, nLinks);
    for i = 1:nLinks
        links{i} = parseLink(linkEls(i), materials);
    end
    links = [links{:}];

    bodies = struct;
    joints = struct;
    if nLinks == 0
        return
    end

    jointEls = getChildren(root, "joint");
    nJoints = numel(jointEls);
    jointDefs = cell(1, nJoints);
    for k = 1:nJoints
        jointDefs{k} = parseJoint(jointEls(k));
    end
    jointDefs = [jointDefs{:}];

    linkNames = [links.name];
    if numel(unique(linkNames)) ~= nLinks
        error("phx:import:duplicateName", "The URDF file contains duplicate link names.");
    end
    if nJoints > 0 && numel(unique([jointDefs.name])) ~= nJoints
        error("phx:import:duplicateName", "The URDF file contains duplicate joint names.");
    end

    % Resolve the kinematic tree: world pose of every link frame in the
    % zero configuration (the frame of a child link coincides with the
    % joint frame given by the joint origin in the parent link frame)
    parentID = zeros(1, nJoints);
    childID = zeros(1, nJoints);
    hasParent = false(1, nLinks);
    for k = 1:nJoints
        parentID(k) = linkIndex(linkNames, jointDefs(k).parent, jointDefs(k).name);
        childID(k) = linkIndex(linkNames, jointDefs(k).child, jointDefs(k).name);
        if hasParent(childID(k))
            error("phx:import:invalidTree", "Link '%s' is the child of more than one joint; the links must form a tree.", jointDefs(k).child);
        end
        hasParent(childID(k)) = true;
    end

    TLink = repmat({TBase}, 1, nLinks);
    known = ~hasParent; % root links sit at the requested base pose
    resolved = false(1, nJoints);
    progress = true;
    while progress
        progress = false;
        for k = find(~resolved)
            if known(parentID(k))
                TLink{childID(k)} = TLink{parentID(k)}*jointDefs(k).T;
                known(childID(k)) = true;
                resolved(k) = true;
                progress = true;
            end
        end
    end
    if ~all(known)
        error("phx:import:invalidTree", "The joints form a kinematic loop around link(s) %s; the links must form a tree.", strjoin(linkNames(~known), ", "));
    end

    % Create the bodies; the body frame sits at the origin of the used
    % geometry so that the PHX shape (always centred at the body origin)
    % appears at the correct place
    if isa(ax, "missing")
        ax = gca;
    end
    fields = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(linkNames));
    bodyList = cell(1, nLinks);
    TBody = cell(1, nLinks);
    for i = 1:nLinks
        geom = links(i).geometry;
        if isempty(geom)
            G = eye(4);
            shape = phx.shape.Sphere("Radius", 0.005);
        else
            G = geom.T;
            shape = createShape(geom, Options.MeshPath, urdfDir);
        end

        b = phx.Body(ax, "Name", links(i).name, "Shape", shape);
        b.Transform = TLink{i}*G;
        [b.Mass, b.Inertia] = massProperties(links(i), G);

        TBody{i} = b.Transform;
        bodyList{i} = b;
        bodies.(fields(i)) = b;
    end

    % Create the joints; both joint frames are expressed in the local space
    % of the connected bodies and must be consistent with the initial poses,
    % otherwise the constraints would deform the robot at the first step
    if nJoints > 0
        jointFields = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName([jointDefs.name]));
    end
    for k = 1:nJoints
        def = jointDefs(k);
        TJ = TLink{childID(k)}; % joint frame = child link frame at zero position
        TA = cleanTransform(TBody{parentID(k)}\TJ);
        TB = cleanTransform(TBody{childID(k)}\TJ);
        parentBody = bodyList{parentID(k)};
        childBody = bodyList{childID(k)};

        switch def.type
            case {"revolute", "continuous"}
                if ~any(def.axis)
                    error("phx:import:invalidAttribute", "Joint '%s' has a zero-length rotation axis.", def.name);
                end
                axis = def.axis/norm(def.axis);
                j = phx.RevoluteJoint(parentBody, childBody, ...
                    "PointA", TA(1:3, 4)', "PointB", TB(1:3, 4)', ...
                    "AxisA", (TA(1:3, 1:3)*axis')', "AxisB", (TB(1:3, 1:3)*axis')', ...
                    "Name", def.name);
            case "prismatic"
                if ~any(def.axis)
                    error("phx:import:invalidAttribute", "Joint '%s' has a zero-length sliding axis.", def.name);
                end
                % The slider axis is the local X of both joint frames, so the
                % frame is rebuilt with its X along the joint axis (fixed
                % joints, in contrast, only need the two frames to coincide)
                TS = sliderFrame(TJ, def.axis/norm(def.axis));
                j = phx.PrismaticJoint(parentBody, childBody, ...
                    "TransformA", cleanTransform(TBody{parentID(k)}\TS), ...
                    "TransformB", cleanTransform(TBody{childID(k)}\TS), ...
                    "Name", def.name);
            case "fixed"
                j = phx.FixedJoint(parentBody, childBody, ...
                    "TransformA", TA, "TransformB", TB, "Name", def.name);
            otherwise % planar, floating
                warning("phx:import:substitutedJoint", "Joint '%s' of unsupported type '%s' was replaced by a fixed joint; its degrees of freedom are locked in the zero pose.", def.name, def.type);
                j = phx.FixedJoint(parentBody, childBody, ...
                    "TransformA", TA, "TransformB", TB, "Name", def.name);
        end

        joints.(jointFields(k)) = j;
    end

end

%% XML helpers -----------------------------------------------------------
% These navigate the struct produced by readstruct: a child element is a
% (scalar or array) struct field, repeated children form a struct array and
% an attribute "attr" is a field "attrAttribute". Because readstruct only
% turns direct children into fields, nested elements (such as
% <transmission>) do not leak into the sibling lists on their own.

function name = rootElementName(file)
% Name of the root element, read from the file text because readstruct
% keeps only the root's contents, not its own tag name
    txt = fileread(file);
    txt = regexprep(txt, "(?s)<!--.*?-->", ""); % ignore commented-out markup
    token = regexp(txt, "<([A-Za-z_][\w.:\-]*)", "tokens", "once");
    if isempty(token)
        name = "";
    else
        name = string(token{1});
    end
end

function els = getChildren(node, name)
% Direct child elements of the given name as a struct array (empty when
% there are none, so numel/indexing behave uniformly)
    if isstruct(node) && isfield(node, name) && isstruct(node.(name))
        els = node.(name);
    else
        els = struct([]);
    end
end

function el = firstElement(node, name)
    els = getChildren(node, name);
    if isempty(els)
        el = [];
    else
        el = els(1);
    end
end

function tf = hasAttribute(el, name)
    field = name + "Attribute";
    tf = isstruct(el) && isfield(el, field) && ~ismissing(el.(field));
end

function value = attribute(el, name, default)
    if hasAttribute(el, name)
        value = string(el.(name + "Attribute")); % readstruct may type single numbers as double
    else
        value = default;
    end
end

function value = requiredAttribute(el, name, context)
    if ~hasAttribute(el, name)
        error("phx:import:missingAttribute", "A <%s> element is missing the required '%s' attribute.", context, name);
    end
    value = string(el.(name + "Attribute"));
end

function v = numbers(str, n, context)
    v = sscanf(char(str), '%f')';
    if numel(v) ~= n || ~all(isfinite(v))
        error("phx:import:invalidAttribute", "Could not read %d numeric value(s) from the %s attribute value '%s'.", n, context, str);
    end
end

%% URDF element parsers --------------------------------------------------

function T = originTransform(el, context)
    T = eye(4);
    if isempty(el)
        return
    end
    xyz = numbers(attribute(el, "xyz", "0 0 0"), 3, context + " origin xyz");
    rpy = numbers(attribute(el, "rpy", "0 0 0"), 3, context + " origin rpy");
    % URDF rpy is the extrinsic X-Y-Z rotation Rz(yaw)*Ry(pitch)*Rx(roll),
    % which matches the z->y->x convention of rot321
    T(1:3, 1:3) = phx.internal.Math.rot321(rpy);
    T(1:3, 4) = xyz;
end

function materials = parseMaterials(root)
    materials.names = string.empty(1, 0);
    materials.colors = zeros(0, 3);
    els = getChildren(root, "material");
    for i = 1:numel(els)
        colorEl = firstElement(els(i), "color");
        if ~isempty(colorEl) && hasAttribute(els(i), "name")
            rgba = numbers(attribute(colorEl, "rgba", "0.5 0.5 0.5 1"), 4, "color rgba");
            materials.names(end + 1) = attribute(els(i), "name", "");
            materials.colors(end + 1, :) = rgba(1:3);
        end
    end
end

function link = parseLink(el, materials)
    link.name = requiredAttribute(el, "name", "link");
    link.mass = [];
    link.inertiaTensor = [];
    link.inertialT = eye(4);
    link.geometry = [];

    inertialEl = firstElement(el, "inertial");
    if ~isempty(inertialEl)
        link.inertialT = originTransform(firstElement(inertialEl, "origin"), "inertial");
        massEl = firstElement(inertialEl, "mass");
        if ~isempty(massEl)
            link.mass = numbers(attribute(massEl, "value", "0"), 1, "mass value");
        end
        tensorEl = firstElement(inertialEl, "inertia");
        if ~isempty(tensorEl)
            m = @(name) numbers(attribute(tensorEl, name, "0"), 1, "inertia " + name);
            ixx = m("ixx"); ixy = m("ixy"); ixz = m("ixz");
            iyy = m("iyy"); iyz = m("iyz"); izz = m("izz");
            link.inertiaTensor = [ixx ixy ixz; ixy iyy iyz; ixz iyz izz];
        end
    end

    % The first visual geometry defines both the appearance and the
    % collision shape; collision elements are used only as a fallback
    geometryEls = getChildren(el, "visual");
    if isempty(geometryEls)
        geometryEls = getChildren(el, "collision");
    end
    if numel(geometryEls) > 1
        warning("phx:import:extraGeometry", "Link '%s' has %d geometry elements; only the first one is imported.", link.name, numel(geometryEls));
    end
    if ~isempty(geometryEls)
        link.geometry = parseGeometry(geometryEls(1), materials, link.name);
    end
end

function geom = parseGeometry(el, materials, linkName)
    geom = [];
    geomEl = firstElement(el, "geometry");
    if isempty(geomEl)
        return
    end

    geom.T = originTransform(firstElement(el, "origin"), "geometry");
    geom.color = [];

    matEl = firstElement(el, "material");
    if ~isempty(matEl)
        colorEl = firstElement(matEl, "color");
        if ~isempty(colorEl)
            rgba = numbers(attribute(colorEl, "rgba", "0.5 0.5 0.5 1"), 4, "color rgba");
            geom.color = rgba(1:3);
        elseif hasAttribute(matEl, "name")
            id = find(materials.names == attribute(matEl, "name", ""), 1);
            if ~isempty(id)
                geom.color = materials.colors(id, :);
            end
        end
    end

    boxEl = firstElement(geomEl, "box");
    cylinderEl = firstElement(geomEl, "cylinder");
    capsuleEl = firstElement(geomEl, "capsule");
    sphereEl = firstElement(geomEl, "sphere");
    meshEl = firstElement(geomEl, "mesh");
    if ~isempty(boxEl)
        geom.kind = "box";
        geom.size = numbers(requiredAttribute(boxEl, "size", "box"), 3, "box size");
    elseif ~isempty(cylinderEl)
        geom.kind = "cylinder";
        geom.radius = numbers(requiredAttribute(cylinderEl, "radius", "cylinder"), 1, "cylinder radius");
        geom.length = numbers(requiredAttribute(cylinderEl, "length", "cylinder"), 1, "cylinder length");
    elseif ~isempty(capsuleEl)
        % Capsule is not part of the core URDF specification but is a
        % widely used extension; length is the cylindrical part only
        geom.kind = "capsule";
        geom.radius = numbers(requiredAttribute(capsuleEl, "radius", "capsule"), 1, "capsule radius");
        geom.length = numbers(requiredAttribute(capsuleEl, "length", "capsule"), 1, "capsule length");
    elseif ~isempty(sphereEl)
        geom.kind = "sphere";
        geom.radius = numbers(requiredAttribute(sphereEl, "radius", "sphere"), 1, "sphere radius");
    elseif ~isempty(meshEl)
        geom.kind = "mesh";
        geom.filename = requiredAttribute(meshEl, "filename", "mesh");
        geom.scale = [1 1 1];
        if hasAttribute(meshEl, "scale")
            s = sscanf(char(attribute(meshEl, "scale", "")), '%f')';
            if isscalar(s)
                s = [s s s];
            end
            if numel(s) ~= 3 || ~all(isfinite(s))
                error("phx:import:invalidAttribute", "Could not read the mesh scale attribute of link '%s'.", linkName);
            end
            geom.scale = s;
        end
    else
        error("phx:import:unsupportedGeometry", "Link '%s' uses an unsupported geometry element.", linkName);
    end
end

function joint = parseJoint(el)
    joint.name = requiredAttribute(el, "name", "joint");
    joint.type = requiredAttribute(el, "type", "joint");
    if ~ismember(joint.type, ["revolute", "continuous", "fixed", "prismatic", "planar", "floating"])
        error("phx:import:unsupportedJoint", "Joint '%s' has unknown type '%s'.", joint.name, joint.type);
    end

    parentEl = firstElement(el, "parent");
    childEl = firstElement(el, "child");
    if isempty(parentEl) || isempty(childEl)
        error("phx:import:missingElement", "Joint '%s' is missing the required <parent> or <child> element.", joint.name);
    end
    joint.parent = requiredAttribute(parentEl, "link", "parent");
    joint.child = requiredAttribute(childEl, "link", "child");

    joint.T = originTransform(firstElement(el, "origin"), "joint");
    joint.axis = [1 0 0];
    axisEl = firstElement(el, "axis");
    if ~isempty(axisEl)
        joint.axis = numbers(attribute(axisEl, "xyz", "1 0 0"), 3, "axis xyz");
    end
end

%% Object builders -------------------------------------------------------

function id = linkIndex(linkNames, name, jointName)
    id = find(linkNames == name, 1);
    if isempty(id)
        error("phx:import:unknownLink", "Joint '%s' references the unknown link '%s'.", jointName, name);
    end
end

function shape = createShape(geom, meshPath, urdfDir)
    switch geom.kind
        case "box"
            shape = phx.shape.Box("Size", geom.size);
        case "cylinder"
            shape = phx.shape.Cylinder("Radius", geom.radius, "Height", geom.length);
        case "capsule"
            shape = phx.shape.Capsule("Radius", geom.radius, "Height", geom.length);
        case "sphere"
            shape = phx.shape.Sphere("Radius", geom.radius);
        case "mesh"
            source = resolveMeshFile(geom.filename, meshPath, urdfDir);
            [~, ~, ext] = fileparts(source);
            switch lower(ext)
                case ".stl"
                    shape = phx.shape.STL;
                case ".obj"
                    shape = phx.shape.OBJ;
                otherwise
                    error("phx:import:unsupportedGeometry", "Unsupported mesh file format '%s' of '%s'; only STL and OBJ meshes are supported.", ext, geom.filename);
            end
            shape.Centered = false; % keep the vertices in the URDF geometry frame
            shape.Scale = geom.scale;
            shape.Envelope = "convex";
            shape.Source = source;
    end

    if ~isempty(geom.color)
        shape.Color = geom.color;
    end
end

function source = resolveMeshFile(uri, meshPath, urdfDir)
    uri = strrep(uri, "\", "/");
    bases = [meshPath urdfDir];
    bases(strlength(bases) == 0) = [];

    if startsWith(uri, "package://")
        rel = extractAfter(uri, "package://");
        candidates = fullfile(bases, rel);
        if contains(rel, "/")
            % also try the URI path without the package name itself
            candidates = [candidates fullfile(bases, extractAfter(rel, "/"))];
        end
    elseif startsWith(uri, "file://")
        candidates = extractAfter(uri, "file://");
    else
        candidates = [uri fullfile(bases, uri)];
    end

    for candidate = candidates
        if isfile(candidate)
            source = candidate;
            return
        end
    end

    error("phx:import:meshNotFound", "Mesh file '%s' was not found (tried: %s). Use the MeshPath option to point to the mesh root folder.", uri, strjoin(candidates, ", "));
end

function [mass, inertia] = massProperties(link, G)
    if isempty(link.mass)
        mass = 0;
    else
        mass = link.mass;
    end

    if isempty(link.inertiaTensor)
        inertia = [0 0 0];
    else
        % Express the tensor in the axes of the body frame and keep the
        % diagonal; products of inertia and the offset of the centre of
        % mass are ignored (see the limitations in the help)
        R = G(1:3, 1:3)'*link.inertialT(1:3, 1:3);
        inertia = diag(R*link.inertiaTensor*R')';
    end

    % Placeholder values keep massless links dynamic
    mass = max(mass, 1e-3);
    inertia = max(inertia, 1e-9);
end

function T = cleanTransform(T)
    T(4, :) = [0 0 0 1];
end

function T = sliderFrame(TJ, axis)
% World joint frame of a prismatic joint: the origin sits at the joint
% point and the local X points along the sliding axis (URDF axis, given in
% the joint frame); Y and Z complete a right-handed orthonormal basis. Both
% body-side frames are later derived from this single world frame, so they
% stay consistent and share the same sliding direction.
    x = TJ(1:3, 1:3)*axis(:);
    x = x/norm(x);
    ref = [0; 0; 1];
    if abs(x'*ref) > 0.9
        ref = [1; 0; 0]; % avoid a degenerate cross product for a near-Z axis
    end
    y = cross(ref, x); y = y/norm(y);
    z = cross(x, y);
    T = eye(4);
    T(1:3, 1:3) = [x y z];
    T(1:3, 4) = TJ(1:3, 4);
end
