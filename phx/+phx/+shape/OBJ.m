classdef OBJ < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.OBJ OBJ imported shape
%
%   Visual appearance is loaded from a Wavefront OBJ file specified by the
%   Source property, including its texture coordinates. The mesh can be
%   scaled (Scale) and recentred to its bounding box origin (Centered).
%
%   The collision shape is selected by the Envelope property and can be a
%   bounding box, cylinder, sphere, convex hull or concave triangle mesh.
%   Mass properties are modeled from the bounding box and Density.
%
%   phx.shape.OBJ() creates a shape with default parameters.
%
%   phx.shape.OBJ(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.STL, phx.shape.Mesh

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        Data
    end

    properties (Access = private)
        BBox
        Center
    end

    properties
        % Scale factors [x y z], or a scalar for uniform scaling
        Scale (1, 3) double = [1 1 1]

        % Move the origin to the centre of the shape
        Centered (1, 1) logical = false

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Collision envelope: bounding "box", "cylinder" or "sphere",
        % "convex" hull, or "concave" triangle mesh
        Envelope {mustBeMember(Envelope, ["box", "cylinder", "sphere", "convex", "concave"])} = "box"
    end

    properties (Transient)
        % Source file name
        Source (1, 1) string
    end

    methods
        function obj = OBJ(Options)
            arguments
                Options.?phx.shape.OBJ
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function obj = set.Source(obj, fileName)
            if isempty(obj.Data)
                obj.Data = phx.internal.readObj(fileName);
                obj.Data.vertices = obj.Data.vertices(:, [1 3 2]);
                obj.Data.vertices_normal = obj.Data.vertices_normal(:, [1 3 2]);
                obj.Data.vertices_texture = obj.Data.vertices_texture(:, [2 1]);

                [pmin, pmax] = bounds(obj.Data.vertices);
                obj.Center = (pmin + pmax)/2;
                obj.BBox = pmax - pmin;
            end
            obj.Source = fileName;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);

            shp = obj.Data.objects(end).data;

            if obj.Centered
                vertices = (obj.Data.vertices - obj.Center).*obj.Scale;
            else
                vertices = obj.Data.vertices.*obj.Scale;
            end

            vid = shp.vertices';
            vertices = vertices(vid(:), :);
            nid = shp.normal';
            normals = obj.Data.vertices_normal(nid(:), :);
            tid = shp.texture';
            uvs = obj.Data.vertices_texture(tid(:), :);
            faces = 1:size(vertices, 1);

            primitive = obj.drawMesh(target, vertices, normals, faces, uvs, obj.TextureData);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            s = obj.BBox.*obj.Scale;
            mass = obj.Density*s(1)*s(2)*s(3);
            q = s.^2;
            inertia = [q(2) + q(3), q(1) + q(3), q(1) + q(2)]*mass/12;
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            bsize = (obj.BBox.*obj.Scale)/2; % half-extents
            switch obj.Envelope
                case "box"
                    eh = phx.engine.io('add', body.WorldHandle, 'box', body.TypeID, bsize, body.Transform, body.Mass, body.Inertia);
                case "cylinder"
                    ax = obj.detectAxis;
                    eh = phx.engine.io('add', body.WorldHandle, 'cylinder', body.TypeID, bsize, ax, body.Transform, body.Mass, body.Inertia);
                case "sphere"
                    eh = phx.engine.io('add', body.WorldHandle, 'sphere', body.TypeID, max(bsize), body.Transform, body.Mass, body.Inertia);
                case "convex"
                    vertices = double(primitive.VertexData);
                    eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
                case "concave"
                    vertices = double(primitive.VertexData);
                    faces = int32(primitive.VertexIndices) - 1;
                    sh_id = phx.engine.io('prepare', body.WorldHandle, uint64(0), 'concaveshape', vertices(:), numel(vertices)/3, faces(:), numel(faces)/3);
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'dynamictrimesh');
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'validation');
                    eh = phx.engine.io('add', body.WorldHandle, 'rigidbody', body.TypeID, sh_id, body.Transform, body.Mass, body.Inertia);
            end
        end

        function createComponent(obj, body)
        end
    end

    methods (Access = private)
        function ax = detectAxis(obj)
            v = obj.Data.Points.^2;
            s = std([v(:, 2) + v(:, 3), v(:, 1) + v(:, 3), v(:, 1) + v(:, 2)]);
            [~, id] = max(s);
            ch = 'xyz';
            ax = ch(id);
        end
    end

end