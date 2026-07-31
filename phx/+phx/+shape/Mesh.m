classdef Mesh < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Mesh Triangular mesh shape (imported or from data)
%
%   A general triangular-mesh shape whose geometry is either loaded from a
%   file (the Source property) or supplied directly as data (the Vertices,
%   Normals, Faces and UV properties). Supported file formats are STL,
%   Wavefront OBJ and Stanford PLY; an OBJ file is loaded as a single body
%   with all of its objects and materials merged (use phx.assembly.import to
%   turn a multi-object OBJ into separate bodies). OBJ textures and diffuse
%   colors are taken automatically from the referenced .mtl materials. A PLY
%   file loads as a single mesh; its per-vertex color is averaged into one
%   diffuse color (per-vertex color is not rendered).
%
%   The mesh can be scaled (Scale), recentred to its bounding-box origin
%   (Centered) and, for STL, decimated (Details).
%
%   The collision shape is selected by the Envelope property:
%     "box", "cylinder", "sphere" - a bounding primitive fitted to the mesh
%        (cylinder and sphere keep rolling bodies smooth, unlike a hull); the
%        cylinder is aligned along the Axis property ("x", "y" or "z"),
%     "convex"  - the convex hull of the mesh (dynamic, the default),
%     "concave" - the exact triangle mesh (best kept static). Set FlipFaces
%        to reverse the triangle winding when the solid side comes out wrong.
%   Mass properties are computed from the mesh volume and Density, falling
%   back to the bounding box for non-solid meshes.
%
%   phx.shape.Mesh() creates a shape with default parameters.
%
%   phx.shape.Mesh(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.assembly.import, phx.shape.Box

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        % Loaded mesh: struct with a groups(name, submeshes) field. Held only
        % as scratch for drawing and mass - the geometry itself is carried by
        % the graphics primitives, so it is not persisted (see the class help).
        Data
    end

    properties (Access = private)
        BBox
        Center
    end

    properties
        % Scale factors [x y z], or a scalar for uniform scaling
        Scale (1, 3) double = [1 1 1]

        % Move the origin to the centre of the shape's bounding box
        Centered (1, 1) logical = true

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Collision envelope: bounding "box", "cylinder" or "sphere",
        % "convex" hull, or "concave" triangle mesh
        Envelope {mustBeMember(Envelope, ["box", "cylinder", "sphere", "convex", "concave"])} = "convex"

        % Reverse triangle winding of the "concave" collision mesh
        FlipFaces (1, 1) logical = false

        % Axis of the "cylinder" collision envelope
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"
    end

    properties (Transient)
        % Source file name (.stl, .obj or .ply)
        Source (1, 1) string

        % Keep only this named OBJ object/group (default: all, merged)
        Group (1, 1) string

        % Mesh details reduction factor (STL only, 0-1)
        Details (1, 1) double = 1

        % Matrix Nx3 of vertices (direct geometry input)
        Vertices (:, 3) double

        % Matrix Nx3 of vertex normals (direct geometry input)
        Normals (:, 3) double

        % Matrix Mx3 of vertex indices (direct geometry input)
        Faces (:, 3) double

        % Matrix Nx2 of vertex texture coordinates (direct geometry input)
        UV (:, 2) double
    end

    methods
        function obj = Mesh(Options)
            arguments
                Options.?phx.shape.Mesh
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end

            % Assemble the mesh (from a file or from the supplied data) once
            % all options are known, so the argument order does not matter
            obj = obj.build;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            for g = 1:numel(obj.Data.groups)
                subs = obj.Data.groups(g).submeshes;
                for s = 1:numel(subs)
                    obj.drawSubmesh(target, subs(s));
                end
            end
        end

        function [mass, inertia] = computeMass(obj)
            [V, F] = obj.mergedGeometry;
            V = V.*obj.Scale;
            [mass, I0] = phx.internal.Geometry.meshMass(V, F, obj.Density);
            inertia = I0([1 5 9]);
            if ~(mass > 0) || any(~isfinite(inertia)) || any(inertia <= 0)
                % Non-solid or open mesh: fall back to the bounding box
                s = obj.BBox.*obj.Scale;
                mass = obj.Density*s(1)*s(2)*s(3);
                q = s.^2;
                inertia = [q(2) + q(3), q(1) + q(3), q(1) + q(2)]*mass/12;
            end
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            bsize = (obj.BBox.*obj.Scale)/2; % half-extents
            switch obj.Envelope
                case "box"
                    eh = phx.engine.io('add', body.WorldHandle, 'box', body.TypeID, bsize, body.Transform, body.Mass, body.Inertia);
                case "cylinder"
                    eh = phx.engine.io('add', body.WorldHandle, 'cylinder', body.TypeID, bsize, char(obj.Axis), body.Transform, body.Mass, body.Inertia);
                case "sphere"
                    eh = phx.engine.io('add', body.WorldHandle, 'sphere', body.TypeID, max(bsize), body.Transform, body.Mass, body.Inertia);
                case "convex"
                    vertices = obj.gatherVertices(body)';
                    eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
                case "concave"
                    [V, F] = obj.gatherMesh(body);
                    if obj.FlipFaces
                        F = fliplr(F);
                    end
                    vertices = V';
                    faces = int32(F') - 1;
                    sh_id = phx.engine.io('prepare', body.WorldHandle, uint64(0), 'concaveshape', vertices(:), size(vertices, 2), faces(:), size(faces, 2));
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'dynamictrimesh');
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'validation');
                    eh = phx.engine.io('add', body.WorldHandle, 'rigidbody', body.TypeID, sh_id, body.Transform, body.Mass, body.Inertia);
            end
        end

        function createComponent(obj, body)
        end
    end

    methods (Access = private)
        function obj = build(obj)
            if strlength(obj.Source) > 0
                obj.Data = phx.internal.readMesh(obj.Source, obj.Details);
                if strlength(obj.Group) > 0
                    keep = [obj.Data.groups.name] == obj.Group;
                    if ~any(keep)
                        error("phx:Mesh:unknownGroup", "The mesh '%s' has no object named '%s'.", obj.Source, obj.Group);
                    end
                    obj.Data.groups = obj.Data.groups(keep);
                end
            elseif ~isempty(obj.Vertices)
                if isempty(obj.Faces)
                    error("phx:Mesh:missingFaces", "A data mesh requires the Faces property along with Vertices.");
                end
                sub.vertices = obj.Vertices;
                sub.faces = obj.Faces;
                if ~isempty(obj.Normals)
                    sub.normals = obj.Normals;
                else
                    sub.normals = [];
                end
                sub.uv = obj.UV;
                sub.color = [];
                sub.texture = "";
                sub.flip = false;
                obj.Data.groups = struct('name', "", 'submeshes', sub);
            else
                obj.Data.groups = struct('name', "", 'submeshes', struct('vertices', {}, 'faces', {}, 'normals', {}, 'uv', {}, 'color', {}, 'texture', {}, 'flip', {}));
            end
            obj = obj.computeBounds;
        end

        function obj = computeBounds(obj)
            % Overall bounding box over every submesh of every group
            V = obj.mergedGeometry;
            if isempty(V)
                obj.BBox = [0 0 0];
                obj.Center = [0 0 0];
            else
                [pmin, pmax] = bounds(V);
                obj.Center = (pmin + pmax)/2;
                obj.BBox = pmax - pmin;
            end
        end

        function drawSubmesh(obj, target, sm)
            if obj.Centered
                vertices = (sm.vertices - obj.Center).*obj.Scale;
            else
                vertices = sm.vertices.*obj.Scale;
            end

            faces = sm.faces;
            if isempty(sm.normals)
                tri = triangulation(faces, sm.vertices);
                if obj.Style == "flat"
                    normals = tri.faceNormal;
                else
                    normals = tri.vertexNormal;
                end
                if sm.flip
                    faces = fliplr(faces); % display winding, after deriving normals
                end
            else
                normals = sm.normals;
            end

            % Per-submesh appearance: OBJ material color/texture, with the
            % shape Color/Texture as the fallback and manual override
            clrShape = obj.Color;
            obj.Color = obj.submeshColor(sm);
            texture = obj.submeshTexture(sm);

            primitive = obj.drawMesh(target, vertices, normals, faces, sm.uv, texture);
            setappdata(primitive, "phxShape", obj);
            obj.Color = clrShape;
        end

        function clr = submeshColor(obj, sm)
            if ~isempty(sm.color)
                clr = sm.color;
            else
                clr = obj.Color;
            end
        end

        function texture = submeshTexture(obj, sm)
            % A material texture wins; a manually set Texture fills in
            % materials that have none and overrides untextured meshes
            if ~isempty(sm.uv) && strlength(sm.texture) > 0
                texture = phx.base.ShapeMesh.loadTexture(sm.texture);
            elseif ~isempty(sm.uv)
                texture = obj.TextureData;
            else
                texture = [];
            end
        end

        function [V, F] = mergedGeometry(obj)
            V = zeros(0, 3);
            F = zeros(0, 3);
            for g = 1:numel(obj.Data.groups)
                subs = obj.Data.groups(g).submeshes;
                for s = 1:numel(subs)
                    F = [F; subs(s).faces + size(V, 1)]; %#ok<AGROW>
                    V = [V; subs(s).vertices]; %#ok<AGROW>
                end
            end
        end

        function V = gatherVertices(obj, body)
            % Collision vertices read back from the body's own mesh
            % primitives, so they survive a save/load round-trip
            V = zeros(0, 3);
            for ch = obj.meshPrimitives(body)
                ph = phx.internal.PrimitiveHelper(ch);
                V = [V; ph.Vertices]; %#ok<AGROW>
            end
        end

        function [V, F] = gatherMesh(obj, body)
            V = zeros(0, 3);
            F = zeros(0, 3);
            for ch = obj.meshPrimitives(body)
                ph = phx.internal.PrimitiveHelper(ch);
                lf = ph.LinearizedFaces;
                F = [F; reshape(lf, 3, [])' + size(V, 1)]; %#ok<AGROW>
                V = [V; ph.Vertices]; %#ok<AGROW>
            end
        end

        function prims = meshPrimitives(~, body)
            % The body carries a single shape, so every child primitive that
            % was tagged with phxShape is one of this mesh's submeshes
            children = body.Graphics.Children;
            keep = false(size(children));
            for i = 1:numel(children)
                keep(i) = ~isempty(getappdata(children(i), "phxShape"));
            end
            prims = reshape(children(keep), 1, []);
        end
    end

    methods (Static, Hidden)
        function obj = fromGroup(group, Options)
        %fromGroup Build a mesh from a pre-parsed readMesh group (internal).
        %   Used by phx.assembly.import to turn one OBJ object into a body
        %   without re-reading the file. Options are the usual name-value
        %   pairs (Envelope, Scale, Centered, Density, ...).
            arguments
                group
                Options.?phx.shape.Mesh
            end
            args = namedargs2cell(Options);
            obj = phx.shape.Mesh(args{:});
            obj.Data.groups = group;
            obj = obj.computeBounds;
        end
    end

end
