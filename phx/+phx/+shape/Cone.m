classdef Cone < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Cone Cone shape
%
%   Visual appearance is based on a geometry of revolved profile with 
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal cone
%   independent on visual details.
%   
%   phx.shape.Cone() creates a shape with default parameters.
%
%   phx.shape.Cone(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Cylinder

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Cone diameter
        Diameter (1, 1) double = 1

        % Cone height
        Height (1, 1) double = 1

        % Modeling axis of the cone
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of cone segments
        Segments (1, 1) double = 24

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent)
        % Cone radius
        Radius (1, 1) double
    end

    methods
        function obj = Cone(Options)
            arguments
                Options.?phx.shape.Cone
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function obj = set.Radius (obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius (obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);

            [V, N, F, T] = phx.internal.Geometry.revolution([-0.5 0.5; 0.5 0], obj.Segments, true, false);
            V = V.*[obj.Diameter obj.Diameter obj.Height];
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            rq = (obj.Diameter/2)^2;
            h = obj.Height;
            mass = obj.Density*pi*rq*h/3;
            ia = mass*rq*3/10;
            io = mass*(h^2/80 + rq/20)*3;
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io io ia]);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phx.engine.io('add', body.WorldHandle, 'cone', body.TypeID, obj.Diameter/2, obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end

end