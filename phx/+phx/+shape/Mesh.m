classdef Mesh < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Mesh Custom shape with texture
%
%   Visual appearance is based on a user-supplied triangular mesh defined by
%   the Vertices, Normals, Faces and UV properties, optionally textured.
%
%   Collision shape and mass properties are modeled as an ideal block based
%   on the axis-aligned bounding box of the mesh (scaled by Scale).
%
%   phx.shape.Mesh() creates a shape with default parameters.
%
%   phx.shape.Mesh(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.OBJ, phx.shape.STL

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        BBox
        Center
    end

    properties
        % Matrix Nx3 of vertices
        Vertices (:, 3) double

        % Matrix Nx3 of vertex normals
        Normals (:, 3) double

        % Matrix Mx3 of vertex indices
        Faces (:, 3) double

        % Matrix Nx2 of vertex texture coordinates
        UV (:, 2) double

        % Scale
        Scale (1, 3) double = [1 1 1]
        
        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    methods
        function obj = Mesh(Options)
            arguments
                Options.?phx.shape.Mesh
            end

            % Defaults
            obj.Style = "flat";

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end

            % Bounding box
            [pmin, pmax] = bounds(obj.Vertices);
            obj.Center = (pmin + pmax)/2;
            obj.BBox = pmax - pmin;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            primitive = obj.drawMesh(target, obj.Vertices, obj.Normals, obj.Faces, obj.UV, obj.getTexture);
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
            eh = phx.engine.io('add', body.WorldHandle, 'box', body.TypeID, bsize, body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end

    methods (Static)
    end

end