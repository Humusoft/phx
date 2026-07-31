function mesh = readPly(fileName)
%readPly Read a Stanford PLY mesh into the unified readMesh structure.
%
%   mesh = phx.internal.readPly(filename) parses a PLY (Stanford Polygon)
%   file and returns the same draw-ready structure that phx.internal.readMesh
%   produces for the other formats: a single group with a single submesh
%   holding the vertices, triangulated faces and any optional normals, texture
%   coordinates and color the file carries. The reader is dependency-free
%   (plain fopen/fgetl/fread, no toolbox APIs) and understands all three
%   standard encodings: ascii, binary_little_endian and binary_big_endian.
%
%   Recognised vertex properties are the position x/y/z, the optional normal
%   nx/ny/nz, the optional texture coordinates s/t (also accepted as u/v or
%   texture_u/texture_v) and the optional per-vertex color red/green/blue.
%   Polygon faces of more than three corners are fan-triangulated, exactly
%   like phx.internal.readObj. Unknown elements and properties in the header
%   are parsed and skipped so their bytes do not desync the reader.
%
%   The returned structure matches emptySubmesh() in phx.internal.readMesh
%   (fields vertices, faces, normals, uv, color, texture, flip), so the rest
%   of phx.shape.Mesh (drawing, mass, collision) works with it unchanged.
%
%   See also phx.internal.readMesh, phx.internal.readObj, phx.shape.Mesh

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    fid = fopen(fileName, "r"); % binary mode; fgetl still handles \n and \r\n
    if fid < 0
        error("phx:readPly:fileNotFound", "Could not open PLY file '%s'.", fileName);
    end
    cleanup = onCleanup(@() fclose(fid));

    [format, elements] = readHeader(fid, fileName);

    % Read the element data, then keep only the vertex/face elements
    switch format
        case "ascii"
            elements = readAsciiBody(fid, elements, fileName);
        case "binary_little_endian"
            elements = readBinaryBody(fid, elements, false, fileName);
        case "binary_big_endian"
            elements = readBinaryBody(fid, elements, true, fileName);
        otherwise
            error("phx:readPly:unsupportedFormat", "Unsupported PLY format '%s' in '%s'.", format, fileName);
    end

    mesh.groups = struct('name', "", 'submeshes', buildSubmesh(elements, fileName));
end

%% Header ----------------------------------------------------------------

function [format, elements] = readHeader(fid, fileName)
% Parse the PLY header into the format keyword and a list of elements, each
% with its ordered property definitions. The file cursor is left at the
% first byte of the element data (right after the end_header line).
    magic = fgetl(fid);
    if ~ischar(magic) || strtrim(string(magic)) ~= "ply"
        error("phx:readPly:invalidHeader", "File '%s' is not a PLY file (missing 'ply' magic).", fileName);
    end

    format = "";
    elements = struct('name', {}, 'count', {}, 'props', {});
    cur = 0;
    while true
        line = fgetl(fid);
        if ~ischar(line)
            error("phx:readPly:invalidHeader", "Unexpected end of file in the header of '%s'.", fileName);
        end
        tokens = split(strtrim(string(line)));
        tokens(strlength(tokens) == 0) = [];
        if isempty(tokens)
            continue
        end
        switch tokens(1)
            case "format"
                format = lower(tokens(2));
            case "element"
                cur = cur + 1;
                elements(cur).name = tokens(2);
                elements(cur).count = str2double(tokens(3));
                elements(cur).props = emptyProp;
            case "property"
                if cur == 0
                    error("phx:readPly:invalidHeader", "A 'property' precedes any 'element' in '%s'.", fileName);
                end
                elements(cur).props(end + 1) = parseProperty(tokens, fileName);
            case "end_header"
                return
            otherwise
                % comment, obj_info, version, ... - ignored
        end
    end
end

function p = parseProperty(tokens, fileName)
    p = emptyProp;
    if tokens(2) == "list"
        % property list <countType> <valueType> <name>
        [p(1).countClass, p(1).countSize] = plyType(tokens(3), fileName);
        [p(1).valClass, p(1).valSize] = plyType(tokens(4), fileName);
        p(1).isList = true;
        p(1).name = tokens(5);
    else
        % property <valueType> <name>
        [p(1).valClass, p(1).valSize] = plyType(tokens(2), fileName);
        p(1).isList = false;
        p(1).name = tokens(3);
    end
end

function p = emptyProp
    p = struct('name', "", 'isList', false, 'valClass', "", 'valSize', 0, ...
        'countClass', "", 'countSize', 0);
    p(1) = [];
end

function [cls, sz] = plyType(t, fileName)
% Map a PLY scalar type name to the matching MATLAB class and byte size.
    switch lower(t)
        case {"char", "int8"};    cls = "int8";   sz = 1;
        case {"uchar", "uint8"};  cls = "uint8";  sz = 1;
        case {"short", "int16"};  cls = "int16";  sz = 2;
        case {"ushort", "uint16"};cls = "uint16"; sz = 2;
        case {"int", "int32"};    cls = "int32";  sz = 4;
        case {"uint", "uint32"};  cls = "uint32"; sz = 4;
        case {"float", "float32"};cls = "single"; sz = 4;
        case {"double", "float64"};cls = "double"; sz = 8;
        otherwise
            error("phx:readPly:invalidType", "Unknown PLY property type '%s' in '%s'.", t, fileName);
    end
end

%% ASCII body ------------------------------------------------------------

function elements = readAsciiBody(fid, elements, fileName)
% In an ASCII PLY every element instance sits on its own line. Read each
% line, flatten it to numbers and hand them out to the properties in order
% (a scalar takes one number, a list takes a leading count then that many).
    for e = 1:numel(elements)
        props = elements(e).props;
        n = elements(e).count;
        scalarIdx = find(~[props.isList]);
        listIdx = find([props.isList]);
        S = zeros(n, numel(scalarIdx));
        L = cell(n, numel(listIdx));

        for it = 1:n
            nums = readAsciiLine(fid, fileName);
            c = 0;
            si = 0;
            li = 0;
            for pi = 1:numel(props)
                if props(pi).isList
                    li = li + 1;
                    k = nums(c + 1);
                    L{it, li} = nums(c + 2 : c + 1 + k);
                    c = c + 1 + k;
                else
                    si = si + 1;
                    S(it, si) = nums(c + 1);
                    c = c + 1;
                end
            end
        end
        elements(e).scalarNames = [props(scalarIdx).name];
        elements(e).scalar = S;
        elements(e).listNames = [props(listIdx).name];
        elements(e).list = L;
    end
end

function nums = readAsciiLine(fid, fileName)
    line = fgetl(fid);
    while ischar(line) && isempty(strtrim(line))
        line = fgetl(fid);
    end
    if ~ischar(line)
        error("phx:readPly:truncated", "The PLY body of '%s' ended earlier than the header promised.", fileName);
    end
    nums = sscanf(line, "%f")';
end

%% Binary body -----------------------------------------------------------

function elements = readBinaryBody(fid, elements, swap, fileName)
% Read the whole binary payload at once, then walk it element by element.
% Scalar-only elements (the usual vertex block) are sliced out in one
% vectorised pass; elements carrying a list property (the usual face block)
% are decoded instance by instance because their stride varies.
    bytes = fread(fid, inf, "*uint8");
    pos = 1; % 1-based cursor into bytes

    for e = 1:numel(elements)
        props = elements(e).props;
        n = elements(e).count;
        scalarIdx = find(~[props.isList]);
        listIdx = find([props.isList]);

        if isempty(listIdx)
            [S, pos] = readScalarBlock(bytes, pos, props, n, swap, fileName);
            L = cell(n, 0);
        else
            [S, L, pos] = readMixedBlock(bytes, pos, props, scalarIdx, listIdx, n, swap, fileName);
        end
        elements(e).scalarNames = [props(scalarIdx).name];
        elements(e).scalar = S;
        elements(e).listNames = [props(listIdx).name];
        elements(e).list = L;
    end
end

function [S, pos] = readScalarBlock(bytes, pos, props, n, swap, fileName)
% Vectorised read of an element whose properties are all fixed-size scalars.
    stride = sum([props.valSize]);
    need = stride*n;
    ensureBytes(bytes, pos, need, fileName);
    block = reshape(bytes(pos : pos + need - 1), stride, n); % stride x n (columns are items)
    pos = pos + need;

    S = zeros(n, numel(props));
    off = 0;
    for pi = 1:numel(props)
        sz = props(pi).valSize;
        raw = block(off + 1 : off + sz, :); % sz x n, contiguous per item down each column
        S(:, pi) = castValues(raw(:), props(pi).valClass, swap);
        off = off + sz;
    end
end

function [S, L, pos] = readMixedBlock(bytes, pos, props, scalarIdx, listIdx, n, swap, fileName)
% Instance-by-instance read for elements that contain a list property.
    S = zeros(n, numel(scalarIdx));
    L = cell(n, numel(listIdx));
    for it = 1:n
        si = 0;
        li = 0;
        for pi = 1:numel(props)
            p = props(pi);
            if p.isList
                li = li + 1;
                [k, pos] = readOne(bytes, pos, p.countClass, p.countSize, swap, fileName);
                [vals, pos] = readMany(bytes, pos, p.valClass, p.valSize, k, swap, fileName);
                L{it, li} = vals';
            else
                si = si + 1;
                [S(it, si), pos] = readOne(bytes, pos, p.valClass, p.valSize, swap, fileName);
            end
        end
    end
end

function [v, pos] = readOne(bytes, pos, cls, sz, swap, fileName)
    ensureBytes(bytes, pos, sz, fileName);
    v = castValues(bytes(pos : pos + sz - 1), cls, swap);
    pos = pos + sz;
end

function [v, pos] = readMany(bytes, pos, cls, sz, k, swap, fileName)
    need = sz*k;
    ensureBytes(bytes, pos, need, fileName);
    v = castValues(bytes(pos : pos + need - 1), cls, swap);
    pos = pos + need;
end

function v = castValues(rawBytes, cls, swap)
    v = typecast(uint8(rawBytes(:)'), char(cls));
    if swap
        v = swapbytes(v);
    end
    v = double(v(:));
end

function ensureBytes(bytes, pos, need, fileName)
    if pos + need - 1 > numel(bytes)
        error("phx:readPly:truncated", "The binary PLY body of '%s' is shorter than the header promised.", fileName);
    end
end

%% Assemble the submesh --------------------------------------------------

function sub = buildSubmesh(elements, fileName)
% Turn the parsed vertex/face elements into the unified submesh structure.
    vId = findElement(elements, "vertex");
    fId = findElement(elements, "face");
    if isempty(vId)
        error("phx:readPly:noVertices", "The PLY file '%s' defines no vertex element.", fileName);
    end
    vE = elements(vId);

    % Positions are mandatory; coordinates are taken as-is. PLY, unlike OBJ,
    % has no fixed up-axis convention, so - like STL - the file is assumed to
    % already be in PHX axes (Z up) and no axis remap is applied.
    x = scalarColumn(vE, "x");
    y = scalarColumn(vE, "y");
    z = scalarColumn(vE, "z");
    if isempty(x) || isempty(y) || isempty(z)
        error("phx:readPly:noVertices", "The PLY vertex element of '%s' lacks x/y/z coordinates.", fileName);
    end
    sub.vertices = [x y z];

    % Faces: PLY indices are 0-based; fan-triangulate any polygon. (The field
    % order here mirrors emptySubmesh() in readMesh: vertices, faces, normals,
    % uv, color, texture, flip.)
    if isempty(fId)
        sub.faces = zeros(0, 3);
    else
        sub.faces = triangulateFaces(elements(fId), fileName);
    end

    % Optional per-vertex normals (used directly, no remap)
    nx = scalarColumn(vE, "nx");
    ny = scalarColumn(vE, "ny");
    nz = scalarColumn(vE, "nz");
    if ~isempty(nx) && ~isempty(ny) && ~isempty(nz)
        sub.normals = [nx ny nz]; % per-vertex, aligned with vertices
    else
        sub.normals = []; % derived from the geometry on draw (Style-aware)
    end

    % Optional texture coordinates (s/t, or the u/v and texture_u/texture_v
    % aliases). Passed through as-is; a texture image itself is not read from
    % PLY, so these only take effect if the user assigns a Texture.
    s = firstColumn(vE, ["s", "u", "texture_u"]);
    t = firstColumn(vE, ["t", "v", "texture_v"]);
    if ~isempty(s) && ~isempty(t)
        sub.uv = [s t];
    else
        sub.uv = [];
    end

    % Optional per-vertex color. The unified submesh (and the draw path in
    % phx.base.ShapeMesh) carries a single color, not per-vertex color, so the
    % values are averaged into one representative diffuse color; per-vertex
    % variation is not rendered. Leave empty when the file has no color.
    sub.color = averageColor(vE);
    sub.texture = "";

    % PLY faces follow the CCW / right-hand-rule (outward normals) convention,
    % like OBJ, so the winding is used as-is (unlike STL, which is flipped).
    sub.flip = false;
end

function faces = triangulateFaces(fE, fileName)
    if isempty(fE.listNames)
        error("phx:readPly:noFaces", "The PLY face element of '%s' has no vertex-index list property.", fileName);
    end
    % The vertex-index list is the property named like vertex_indices; fall
    % back to the first list property when the name is non-standard.
    col = find(contains(lower(fE.listNames), "vert") & contains(lower(fE.listNames), "ind"), 1);
    if isempty(col)
        col = 1;
    end
    polys = fE.list(:, col);

    % Pre-size to the exact triangle count (sum of corners-2 over all faces)
    counts = cellfun(@numel, polys);
    nTri = sum(max(counts - 2, 0));
    faces = zeros(nTri, 3);
    r = 0;
    for i = 1:numel(polys)
        idx = polys{i}(:)' + 1; % 0-based -> 1-based
        for k = 2:numel(idx) - 1
            r = r + 1;
            faces(r, :) = idx([1 k k + 1]);
        end
    end
end

function col = scalarColumn(elem, name)
    col = [];
    j = find(elem.scalarNames == name, 1);
    if ~isempty(j)
        col = elem.scalar(:, j);
    end
end

function col = firstColumn(elem, names)
    col = [];
    for name = names
        col = scalarColumn(elem, name);
        if ~isempty(col)
            return
        end
    end
end

function color = averageColor(vE)
    r = firstColumn(vE, ["red", "r", "diffuse_red"]);
    g = firstColumn(vE, ["green", "g", "diffuse_green"]);
    b = firstColumn(vE, ["blue", "b", "diffuse_blue"]);
    if isempty(r) || isempty(g) || isempty(b)
        color = [];
        return
    end
    rgb = mean([r g b], 1);
    % Integer color channels are stored 0-255, float channels already 0-1
    if max(rgb) > 1
        rgb = rgb/255;
    end
    color = min(max(rgb, 0), 1);
end

function id = findElement(elements, name)
    id = find([elements.name] == name, 1);
end
