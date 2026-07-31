function [bodies, joints] = import(varargin)
%phx.assembly.import Import a multi-body model from a file
%
%   bodies = phx.assembly.import(file) reads a model from the given file and
%   creates the phx.Body objects it describes. The file format is selected
%   from the extension:
%   - URDF (.urdf, .xml) - one body per robot link, connected by joints;
%   - OBJ (.obj) - one body per Wavefront object (o/g group), with no joints;
%   - STL (.stl) and PLY (.ply) - a single body from the whole mesh.
%   The bodies are returned in a struct whose field names are the link or
%   object names (made valid MATLAB identifiers by matlab.lang.makeValidName;
%   the original name is preserved in the Name property of each object). Like
%   with phx.Body, a target axes to draw into may be passed as an optional
%   first argument: phx.assembly.import(ax, file, ___), where an empty target
%   ([]) creates the bodies without graphics for headless simulations.
%
%   For an OBJ file the mesh objects become separate dynamic bodies with a
%   convex collision envelope (set Envelope to override); each body frame
%   sits at its object's centroid. Only genuine 3D solids are imported - flat
%   objects (ground planes, light quads, decals) and line- or point-only
%   objects carry no useful collision volume and are skipped. Use
%   phx.shape.Mesh instead to load a whole OBJ as a single merged body. The
%   URDF path is described below.
%
%   A URDF file creates a phx.Body for every link. The bodies are drawn into
%   the current axes and placed at the world poses of the robot in its zero
%   (home) configuration, with the base (root link frame) at the world
%   origin unless a pose is given by the Position and Orientation or
%   EulerAngles options.
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
%     sphere -> phx.shape.Sphere and mesh -> phx.shape.Mesh (STL or OBJ,
%     selected by the file extension, with the scale attribute applied).
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
%   - Joints of type "planar" have no direct PHX equivalent yet and are
%     approximated by a phx.GenericJoint that frees the two in-plane
%     translations and the rotation about the plane normal; the warning
%     phx:import:substitutedJoint is issued.
%   - Joints of type "floating" impose no constraint, so no joint is created
%     and the child link is left free at its zero-configuration pose; the
%     warning phx:import:floatingJoint is issued and the link has no entry in
%     the returned joints struct.
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
    if isempty(args)
        error("phx:import:missingFile", "A model file name is required.");
    end
    file = string(args{1});
    [~, ~, ext] = fileparts(file);

    % Base-pose options are shared, the rest belongs to one branch only. The
    % branch functions validate them again (types, membership); checking the
    % names here just turns MATLAB's generic MATLAB:TooManyInputs into a
    % phx:import:unsupportedOption that says which format the option belongs to.
    BASE = ["Position", "Orientation", "EulerAngles"];
    MESH = ["Scale", "Envelope", "FlipFaces", "Density"];
    URDF = "MeshPath";

    switch lower(ext)
        case {".urdf", ".xml"}
            checkOptions(args, [BASE URDF], "URDF", MESH, "OBJ, STL and PLY files");
            [bodies, joints] = importURDF(ax, args{:});
        case ".obj"
            checkOptions(args, [BASE MESH], "OBJ", URDF, "URDF files");
            [bodies, joints] = importOBJ(ax, args{:});
        case {".stl", ".ply"}
            checkOptions(args, [BASE MESH], upper(extractAfter(lower(ext), ".")), URDF, "URDF files");
            [bodies, joints] = importMeshFile(ax, args{:});
        otherwise
            error("phx:import:unsupportedFormat", "Unsupported model file format '%s'; supported formats are URDF, OBJ, STL and PLY.", ext);
    end
end

function checkOptions(args, allowed, format, foreign, foreignFormats)
% Reject option names this branch does not take, naming the format they suit.
    for i = 2:2:numel(args) - 1
        name = args{i};
        if ~(isstring(name) || ischar(name)) || isscalar(string(name)) == false
            continue % not a name-value name; leave it to the branch function
        end
        name = string(name);
        if any(strcmpi(name, allowed))
            continue
        end
        if any(strcmpi(name, foreign))
            error("phx:import:unsupportedOption", ...
                "Option '%s' does not apply to %s files; it applies to %s. Valid options here are %s.", ...
                name, format, foreignFormats, strjoin("'" + allowed + "'", ", "));
        end
        error("phx:import:unsupportedOption", ...
            "Unknown option '%s' for %s files. Valid options are %s.", ...
            name, format, strjoin("'" + allowed + "'", ", "));
    end
end

function [bodies, joints] = importURDF(ax, file, Options)
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
                % The sliding axis is the local Z of both joint frames, so the
                % frame is rebuilt with its Z along the joint axis (fixed
                % joints, in contrast, only need the two frames to coincide)
                TS = axisAlignedFrame(TJ, def.axis/norm(def.axis));
                j = phx.PrismaticJoint(parentBody, childBody, ...
                    "TransformA", cleanTransform(TBody{parentID(k)}\TS), ...
                    "TransformB", cleanTransform(TBody{childID(k)}\TS), ...
                    "Name", def.name);
            case "fixed"
                j = phx.FixedJoint(parentBody, childBody, ...
                    "TransformA", TA, "TransformB", TB, "Name", def.name);
            case "planar"
                if ~any(def.axis)
                    error("phx:import:invalidAttribute", "Joint '%s' has a zero-length plane normal.", def.name);
                end
                % TEMPORARY substitution: PHX has no dedicated planar joint
                % yet, so the motion (two translations in the plane plus a
                % rotation about the plane normal) is approximated with a
                % phx.GenericJoint - the plane normal (URDF axis) is put on
                % the joint Z, which frees the two in-plane translations (X, Y)
                % and the rotation about Z while locking the rest. Z also keeps
                % the free rotation off the generic joint's degenerate Y axis.
                % Replace with a direct phx.PlanarJoint once it exists.
                warning("phx:import:substitutedJoint", "Joint '%s' of type 'planar' has no direct PHX equivalent yet and was approximated by a phx.GenericJoint.", def.name);
                TP = axisAlignedFrame(TJ, def.axis/norm(def.axis));
                j = phx.GenericJoint(parentBody, childBody, ...
                    "TransformA", cleanTransform(TBody{parentID(k)}\TP), ...
                    "TransformB", cleanTransform(TBody{childID(k)}\TP), ...
                    "LowerLinearLimits", [1 1 0], "UpperLinearLimits", [-1 -1 0], ...
                    "LowerAngularLimits", [0 0 1], "UpperAngularLimits", [0 0 -1], ...
                    "Name", def.name);
            case "floating"
                % A floating joint imposes no constraint - all six degrees of
                % freedom are free - so no PHX joint is created and the child
                % link is left as a free dynamic body at its zero-configuration
                % pose. This matches the URDF semantics of a free-floating link
                % (e.g. a mobile robot's base) more faithfully than any
                % constraint could. The link therefore has no entry in the
                % returned joints struct.
                warning("phx:import:floatingJoint", "Joint '%s' of type 'floating' imposes no constraint; link '%s' is left free and no joint is created.", def.name, def.child);
                continue
            otherwise
                error("phx:import:unsupportedJoint", "Joint '%s' has unhandled type '%s'.", def.name, def.type);
        end

        joints.(jointFields(k)) = j;
    end

end

%% OBJ / mesh import -----------------------------------------------------

function [bodies, joints] = importOBJ(ax, file, Options)
% Import a Wavefront OBJ as one phx.Body per object (o/g group).
    arguments
        ax
        file (1, 1) string
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
        Options.Scale (1, 3) double = [1 1 1]
        Options.Envelope {mustBeMember(Options.Envelope, ["box", "cylinder", "sphere", "convex", "concave"])} = "convex"
        Options.FlipFaces (1, 1) logical = false
        Options.Density (1, 1) double = 1000
    end

    TBase = basePose(Options, "import");
    if ~isfile(file)
        error("phx:import:fileNotFound", "OBJ file '%s' was not found.", file);
    end

    mesh = phx.internal.readMesh(file);
    joints = struct;
    bodies = struct;

    groups = mesh.groups;
    n = numel(groups);
    if n == 0
        return
    end
    if isa(ax, "missing")
        ax = gca;
    end

    names = strings(1, n);
    for i = 1:n
        nm = groups(i).name;
        if strlength(nm) == 0
            nm = "part";
        end
        names(i) = nm;
    end
    fields = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(names));

    for i = 1:n
        % Recentre each object on its own bounding box, so the body frame
        % sits at the object's centroid (see phx.shape.Mesh, Centered)
        V = groupVertices(groups(i));

        % Import only genuine 3D solids; flat objects (ground planes, light
        % quads, decals) and line/point-only objects make no useful physics
        % body and are skipped
        if ~isSolidMesh(V)
            continue
        end

        [pmin, pmax] = bounds(V);
        center = (pmin + pmax)/2;

        shape = phx.shape.Mesh.fromGroup(groups(i), "Envelope", Options.Envelope, ...
            "FlipFaces", Options.FlipFaces, "Scale", Options.Scale, ...
            "Centered", true, "Density", Options.Density);

        b = phx.Body(ax, "Name", char(names(i)), "Shape", shape);
        T = TBase;
        T(1:3, 4) = TBase(1:3, 4) + TBase(1:3, 1:3)*(center.*Options.Scale)';
        b.Transform = T;

        bodies.(fields(i)) = b;
    end
end

function [bodies, joints] = importMeshFile(ax, file, Options)
% Import a single-solid mesh file (STL, PLY) as one phx.Body.
    arguments
        ax
        file (1, 1) string
        Options.Position (1, 3) double = [0 0 0]
        Options.Orientation (3, 3) double = eye(3)
        Options.EulerAngles (1, 3) double = [0 0 0]
        Options.Scale (1, 3) double = [1 1 1]
        Options.Envelope {mustBeMember(Options.Envelope, ["box", "cylinder", "sphere", "convex", "concave"])} = "convex"
        Options.FlipFaces (1, 1) logical = false
        Options.Density (1, 1) double = 1000
    end

    TBase = basePose(Options, "import");
    if ~isfile(file)
        error("phx:import:fileNotFound", "Mesh file '%s' was not found.", file);
    end
    if isa(ax, "missing")
        ax = gca;
    end

    [~, name] = fileparts(file);
    shape = phx.shape.Mesh("Source", file, "Envelope", Options.Envelope, ...
        "FlipFaces", Options.FlipFaces, "Scale", Options.Scale, "Density", Options.Density);
    b = phx.Body(ax, "Name", char(name), "Shape", shape);
    b.Transform = TBase;

    joints = struct;
    bodies = struct(matlab.lang.makeValidName(name), b);
end

function V = groupVertices(group)
    V = zeros(0, 3);
    for s = 1:numel(group.submeshes)
        V = [V; group.submeshes(s).vertices]; %#ok<AGROW>
    end
end

function tf = isSolidMesh(V)
% True when the vertices span three dimensions, i.e. form a real 3D solid
% rather than a flat sheet. Uses the smallest principal extent, so the test
% is independent of how the plane is oriented.
    if size(V, 1) < 4
        tf = false;
        return
    end
    s = svd(V - mean(V, 1), "econ");
    tf = numel(s) >= 3 && s(3) > 1e-4*s(1);
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
            if ~ismember(lower(ext), [".stl", ".obj"])
                error("phx:import:unsupportedGeometry", "Unsupported mesh file format '%s' of '%s'; only STL and OBJ meshes are supported.", ext, geom.filename);
            end
            % Keep the vertices in the URDF geometry frame (no recentring)
            shape = phx.shape.Mesh("Source", source, "Scale", geom.scale, "Envelope", "convex", "Centered", false);
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

function T = axisAlignedFrame(TJ, axis)
% World joint frame whose local axis Z points along the given axis (URDF axis,
% expressed in the joint frame); the other two axes complete a right-handed
% orthonormal basis and the origin sits at the joint point. Deriving both
% body-side frames from this single world frame keeps them consistent, so the
% joint neither deforms nor snaps at the first step. Used to align a prismatic
% sliding axis or a planar plane normal, both of which are the local axis Z.
    T = eye(4);
    T(1:3, 1:3) = phx.internal.Math.alignZ(eye(3), TJ(1:3, 1:3)*axis(:));
    T(1:3, 4) = TJ(1:3, 4);
end
