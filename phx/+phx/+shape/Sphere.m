classdef Sphere < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Sphere Sphere shape
% 
%   Visual appearance is based on a geometry of icosahedron with 
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal sphere
%   independent on visual details.
%   
%   phx.shape.Sphere() creates a shape with default parameters.
%
%   phx.shape.Sphere(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Globe

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        Vertices
        Normals
        Faces
    end

    properties
        % Sphere diameter
        Diameter (1, 1) double = 1

        % Number of division of the icosahedron shape
        Division (1, 1) double = 3

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent)
        % Sphere radius
        Radius (1, 1) double
    end

    methods
        function obj = Sphere(Options)
            arguments
                Options.?phx.shape.Sphere
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end

            % Prepare geometry
            [obj.Vertices, obj.Normals, obj.Faces] = phx.internal.Geometry.icosphere(obj.Division);
        end

        function obj = set.Radius(obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius(obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            primitive = obj.drawMesh(target, obj.Vertices*obj.Radius, obj.Normals, obj.Faces, [], []);
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