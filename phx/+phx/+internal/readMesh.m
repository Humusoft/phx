function mesh = readMesh(fileName, Details)
%readMesh Load a triangular mesh file into a uniform, draw-ready structure.
%
%   mesh = phx.internal.readMesh(fileName) reads a mesh file and returns a
%   structure that phx.shape.Mesh can draw and collide directly. The file
%   format is selected from the extension:
%       .stl - binary or ASCII STL (single solid)
%       .obj - Wavefront OBJ (objects/groups and materials, see readObj)
%       .ply - Stanford PLY (ASCII or binary, single solid, see readPly)
%   The mesh geometry is expressed in PHX axes (Z up), ready for drawing.
%
%   mesh = phx.internal.readMesh(fileName, Details) decimates STL meshes to
%   the given fraction of their faces (0 < Details < 1); the default 1 keeps
%   the mesh intact. Details has no effect on other formats.
%
%   The returned structure has the fields:
%       groups - struct array, one entry per body-level part:
%           name      - part name ("" when the file defines none)
%           submeshes - struct array, one entry per material section:
%               vertices - (V x 3) vertex positions
%               faces    - (F x 3) triangle indices into vertices
%               normals  - (V x 3) or (F x 3) normals, [] to derive on draw
%               uv       - (V x 2) texture coordinates, or []
%               color    - (1 x 3) diffuse color, or []
%               texture  - texture image file, or "" (only when it exists)
%
%   A single-body shape merges all groups; phx.assembly.import turns each
%   group into its own phx.Body.
%
%   See also phx.internal.readObj, phx.internal.readPly, phx.shape.Mesh, phx.assembly.import

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        fileName (1, 1) string
        Details (1, 1) double = 1
    end

    [~, ~, ext] = fileparts(fileName);
    switch lower(ext)
        case ".stl"
            mesh = readStl(fileName, Details);
        case ".obj"
            mesh = readObjMesh(fileName);
        case ".ply"
            mesh = phx.internal.readPly(fileName);
        otherwise
            error("phx:readMesh:unsupportedFormat", "Unsupported mesh file format '%s'; supported formats are STL, OBJ and PLY.", ext);
    end
end

function mesh = readStl(fileName, Details)
    cFileName = matlab.io.internal.validators.validateFileName(fileName);
    stl = matlab.internal.meshio.stlread(cFileName{1});
    if Details < 1
        nfv = reducepatch(stl.Faces, stl.Vertices, Details);
        sub.vertices = nfv.vertices;
        sub.faces = nfv.faces;
    else
        sub.vertices = stl.Vertices;
        sub.faces = stl.Faces;
    end
    sub.normals = []; % derived from the geometry on draw (Style-aware)
    sub.uv = [];
    sub.color = [];
    sub.texture = "";
    sub.flip = true; % STL winding is flipped for display/collision, matching legacy behaviour
    mesh.groups = struct('name', "", 'submeshes', sub);
end

function mesh = readObjMesh(fileName)
    obj = phx.internal.readObj(fileName);

    % OBJ is Y-up; PHX is Z-up. Swap Y and Z on positions and normals.
    V = obj.vertices(:, [1 3 2]);
    N = obj.normals(:, [1 3 2]);
    UV = obj.uv;

    for g = numel(obj.groups):-1:1
        subsIn = obj.groups(g).submeshes;
        subsOut = emptySubmesh();
        s = 0;
        for j = 1:numel(subsIn)
            sm = subsIn(j);
            F = size(sm.v, 1);
            if F == 0
                continue
            end
            s = s + 1;

            % De-index: one own vertex per triangle corner, so independent
            % vertex/texture/normal index spaces collapse into a flat mesh
            vidx = reshape(sm.v', [], 1);
            subsOut(s).vertices = V(vidx, :);
            subsOut(s).faces = reshape(1:3*F, 3, F)';

            if all(sm.vn(:) > 0)
                nidx = reshape(sm.vn', [], 1);
                subsOut(s).normals = N(nidx, :);
            else
                subsOut(s).normals = []; % derived on draw
            end

            if all(sm.vt(:) > 0) && ~isempty(UV)
                tidx = reshape(sm.vt', [], 1);
                raw = UV(tidx, :);
                % Match the historical texture-space mapping (V flipped, axes swapped)
                subsOut(s).uv = [1 - raw(:, 2), raw(:, 1)];
            else
                subsOut(s).uv = [];
            end

            [subsOut(s).color, subsOut(s).texture] = resolveMaterial(obj.materials, sm.material);
            subsOut(s).flip = false; % OBJ winding is used as-is
        end
        mesh.groups(g).name = obj.groups(g).name;
        mesh.groups(g).submeshes = subsOut;
    end
end

function [color, texture] = resolveMaterial(materials, name)
    color = [];
    texture = "";
    if strlength(name) == 0 || isempty(materials)
        return
    end
    id = find([materials.name] == name, 1);
    if isempty(id)
        return
    end
    color = materials(id).Kd;
    map = materials(id).map_Kd;
    if strlength(map) > 0 && isfile(map)
        texture = map; % only adopt textures that actually exist on disk
    end
end

function s = emptySubmesh()
    s = struct('vertices', {}, 'faces', {}, 'normals', {}, 'uv', {}, 'color', {}, 'texture', {}, 'flip', {});
end
