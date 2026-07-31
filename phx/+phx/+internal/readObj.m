function OBJ = readObj(objFileName)
%readObj Read geometry, groups and materials from a Wavefront OBJ file.
%
%   OBJ = phx.internal.readObj(filename) parses the OBJ file and returns a
%   struct with the raw geometry (in the file's own coordinates), the object
%   grouping and the referenced materials. No coordinate or texture-space
%   conversion is applied here - that is the caller's responsibility (see
%   phx.internal.readMesh).
%
%   Fields of OBJ:
%     vertices  - (nv x 3)  vertex positions
%     normals   - (nvn x 3) vertex normals
%     uv        - (nvt x 2) texture coordinates (u, v)
%     groups    - struct array, one entry per object (see below)
%     materials - struct array of the materials from the .mtl libraries
%
%   Each element of OBJ.groups describes one body-level part:
%     name      - object name (from the "o", or "g" fallback, marker)
%     submeshes - struct array, one entry per contiguous material run:
%         material - material name ("" when none is active)
%         v        - (F x 3) vertex indices of the triangulated faces
%         vt       - (F x 3) texture indices (0 where absent)
%         vn       - (F x 3) normal indices (0 where absent)
%
%   Faces are grouped into bodies by their "o" markers; if the file has no
%   "o" marker the "g" markers are used, and a file with neither becomes a
%   single group. Faces of more than three vertices are fan-triangulated.
%
%   Each element of OBJ.materials has the fields:
%     name   - material name
%     Kd     - (1 x 3) diffuse color, or [] when not specified
%     map_Kd - diffuse texture file (resolved against the .mtl folder), or ""
%
%   See also phx.internal.readMesh, phx.shape.Mesh

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    % Read the whole file as lines
    fid = fopen(objFileName, "r");
    if fid < 0
        error("phx:readObj:fileNotFound", "Could not open OBJ file '%s'.", objFileName);
    end
    raw = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
    fclose(fid);
    lines = raw{1};
    nLines = numel(lines);

    % Geometry pools (over-allocated, trimmed at the end)
    vertices = zeros(nLines, 3);  nv = 0;
    normals = zeros(nLines, 3);   nvn = 0;
    uv = zeros(nLines, 2);        nvt = 0;

    % Flat face list, each face tagged with its object/group/material context.
    % Faces are fan-triangulated on the fly, so one input face may yield
    % several rows here.
    faceV = zeros(nLines, 3);
    faceVT = zeros(nLines, 3);
    faceVN = zeros(nLines, 3);
    faceObj = strings(nLines, 1);
    faceGrp = strings(nLines, 1);
    faceMtl = strings(nLines, 1);
    nf = 0;

    materials = struct('name', {}, 'Kd', {}, 'map_Kd', {});
    anyObj = false;
    anyGrp = false;
    curObj = "";
    curGrp = "";
    curMtl = "";

    for iln = 1:nLines
        line = strtrim(lines{iln});
        if isempty(line)
            continue
        end
        sp = find(line == ' ' | line == char(9), 1);
        if isempty(sp)
            continue
        end
        type = line(1:sp - 1);
        data = strtrim(line(sp + 1:end));

        switch type
            case 'v'
                nv = nv + 1;
                p = sscanf(data, '%f', 3);
                vertices(nv, :) = p(1:3)';
            case 'vn'
                nvn = nvn + 1;
                p = sscanf(data, '%f', 3);
                normals(nvn, :) = p(1:3)';
            case 'vt'
                nvt = nvt + 1;
                p = sscanf(data, '%f', 2);
                if numel(p) < 2
                    p(2) = 0;
                end
                uv(nvt, :) = p(1:2)';
            case 'o'
                anyObj = true;
                curObj = string(data);
            case 'g'
                anyGrp = true;
                curGrp = string(data);
            case 'usemtl'
                curMtl = string(data);
            case 'mtllib'
                folder = fileparts(objFileName);
                for libName = split(string(data))'
                    if strlength(libName) == 0
                        continue
                    end
                    materials = [materials, readMtl(fullfile(folder, libName))]; %#ok<AGROW>
                end
            case 'f'
                [vi, ti, ni] = parseFace(data, nv, nvt, nvn);
                % Fan-triangulate the (possibly polygonal) face
                for k = 2:numel(vi) - 1
                    nf = nf + 1;
                    faceV(nf, :) = vi([1 k k + 1]);
                    faceVT(nf, :) = ti([1 k k + 1]);
                    faceVN(nf, :) = ni([1 k k + 1]);
                    faceObj(nf) = curObj;
                    faceGrp(nf) = curGrp;
                    faceMtl(nf) = curMtl;
                end
            otherwise
                % vp, l, s, comments, ... - ignored
        end
    end

    % Trim pools
    OBJ.vertices = vertices(1:nv, :);
    OBJ.normals = normals(1:nvn, :);
    OBJ.uv = uv(1:nvt, :);
    OBJ.materials = materials;

    faceV = faceV(1:nf, :);
    faceVT = faceVT(1:nf, :);
    faceVN = faceVN(1:nf, :);
    faceObj = faceObj(1:nf);
    faceGrp = faceGrp(1:nf);
    faceMtl = faceMtl(1:nf);

    % Choose the body-splitting key: "o" primarily, "g" as a fallback
    if anyObj
        faceKey = faceObj;
    elseif anyGrp
        faceKey = faceGrp;
    else
        faceKey = strings(nf, 1);
    end

    % Build the groups (in first-appearance order), each split into
    % contiguous material submeshes
    OBJ.groups = buildGroups(faceKey, faceMtl, faceV, faceVT, faceVN);
end

function [vi, ti, ni] = parseFace(data, nv, nvt, nvn)
% Parse the vertex references of one face line into 1-based index vectors.
% Missing texture/normal indices become 0; negative (relative) indices are
% resolved against the current pool sizes.
    tokens = split(string(strtrim(data)));
    tokens(strlength(tokens) == 0) = [];
    n = numel(tokens);
    vi = zeros(1, n);
    ti = zeros(1, n);
    ni = zeros(1, n);
    for i = 1:n
        parts = split(tokens(i), "/");
        vi(i) = resolveIndex(parts(1), nv);
        if numel(parts) >= 2
            ti(i) = resolveIndex(parts(2), nvt);
        end
        if numel(parts) >= 3
            ni(i) = resolveIndex(parts(3), nvn);
        end
    end
end

function idx = resolveIndex(str, count)
    if strlength(str) == 0
        idx = 0;
        return
    end
    idx = str2double(str);
    if isnan(idx)
        idx = 0;
    elseif idx < 0
        idx = count + idx + 1; % relative reference
    end
end

function groups = buildGroups(faceKey, faceMtl, faceV, faceVT, faceVN)
    groups = struct('name', {}, 'submeshes', {});
    [keyNames, ~, keyIdx] = unique(faceKey, 'stable');
    for g = 1:numel(keyNames)
        rows = find(keyIdx == g);
        groups(g).name = keyNames(g);
        groups(g).submeshes = buildSubmeshes(faceMtl(rows), faceV(rows, :), faceVT(rows, :), faceVN(rows, :));
    end
    if isempty(groups)
        groups = struct('name', "", 'submeshes', emptySubmesh());
    end
end

function subs = buildSubmeshes(mtl, v, vt, vn)
% Split a group's faces into runs of a constant material, preserving order.
    subs = emptySubmesh();
    if isempty(mtl)
        return
    end
    % A new submesh starts wherever the active material changes
    starts = [1; find(mtl(2:end) ~= mtl(1:end-1)) + 1];
    stops = [starts(2:end) - 1; numel(mtl)];
    for s = 1:numel(starts)
        r = starts(s):stops(s);
        subs(s).material = mtl(starts(s));
        subs(s).v = v(r, :);
        subs(s).vt = vt(r, :);
        subs(s).vn = vn(r, :);
    end
end

function s = emptySubmesh()
    s = struct('material', {}, 'v', {}, 'vt', {}, 'vn', {});
end

function mtl = readMtl(mtlFileName)
% Parse a .mtl library into a struct array of materials (name, Kd, map_Kd).
    mtl = struct('name', {}, 'Kd', {}, 'map_Kd', {});
    fid = fopen(mtlFileName, "r");
    if fid < 0
        return % a missing library is not fatal - materials stay unresolved
    end
    raw = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', '');
    fclose(fid);
    lines = raw{1};
    folder = fileparts(mtlFileName);
    cur = 0;

    for iln = 1:numel(lines)
        line = strtrim(lines{iln});
        if isempty(line)
            continue
        end
        sp = find(line == ' ' | line == char(9), 1);
        if isempty(sp)
            continue
        end
        type = line(1:sp - 1);
        data = strtrim(line(sp + 1:end));

        switch type
            case 'newmtl'
                cur = numel(mtl) + 1;
                mtl(cur).name = string(data);
                mtl(cur).Kd = [];
                mtl(cur).map_Kd = "";
            case 'Kd'
                if cur > 0
                    c = sscanf(data, '%f', 3);
                    if numel(c) == 3
                        mtl(cur).Kd = c';
                    end
                end
            case 'map_Kd'
                if cur > 0
                    mtl(cur).map_Kd = resolveMap(data, folder);
                end
        end
    end
end

function path = resolveMap(data, folder)
% Extract the texture file name from a map_Kd line (dropping any -options)
% and resolve it against the .mtl folder.
    tokens = split(string(strtrim(data)));
    tokens(strlength(tokens) == 0) = [];
    % The file name is the last token that is not an option flag or its value
    name = "";
    i = numel(tokens);
    while i >= 1
        if startsWith(tokens(i), "-")
            break
        end
        name = tokens(i);
        i = i - 1;
        % stop at the first non-option token from the end
        break
    end
    if strlength(name) == 0
        path = "";
    else
        path = string(fullfile(folder, name));
    end
end
