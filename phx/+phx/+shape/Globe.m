classdef Globe < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Globe Globe shape
%
%   Visual appearance is based on a geometry of revolved profile with 
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal sphere
%   independent on visual details.
%   
%   phx.shape.Globe() creates a shape with default parameters.
%
%   phx.shape.Globe(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Sphere

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        Vertices
        Normals
        Faces
        TextCoords
    end

    properties
        % Globe diameter
        Diameter (1, 1) double = 1

        % Number of meridian and parallel segments
        Segments (1, 1) double = 32

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent)
        % Globe radius
        Radius (1, 1) double
    end

    methods
        function obj = Globe(Options)
            arguments
                Options.?phx.shape.Globe
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end

            % Prepare geometry
            [obj.Vertices, obj.Normals, obj.Faces, obj.TextCoords] = phx.internal.Geometry.sphere(2, obj.Segments);
        end

        function obj = set.Radius(obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius(obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            primitive = obj.drawMesh(target, obj.Vertices*obj.Radius, obj.Normals, obj.Faces, obj.TextCoords, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            r = obj.Diameter/2;
            mass = obj.Density*pi*r^3*4/3;
            inertia = r^2*mass*2/5;
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phx.engine.io('add', body.WorldHandle, 'sphere', body.TypeID, obj.Radius, body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end

end