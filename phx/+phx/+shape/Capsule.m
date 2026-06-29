classdef Capsule < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Capsule Capsule shape
%
%   Visual appearance is based on a geometry of revolved profile with 
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal capsule
%   independent on visual details.
%   
%   phx.shape.Capsule() creates a shape with default parameters.
%
%   phx.shape.Capsule(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Cone

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Capsule diameter
        Diameter (1, 1) double = 1

        % Capsule height
        Height (1, 1) double = 1

        % Modeling axis of the capsule
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of Capsule segments
        Segments (1, 1) double = 24

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent)
        % Capsule radius
        Radius (1, 1) double
    end

    methods
        function obj = Capsule(Options)
            arguments
                Options.?phx.shape.Capsule
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

            sh = sin(linspace(0, pi/2, obj.Segments/3))*obj.Radius;
            sr = cos(linspace(0, pi/2, obj.Segments/3))*obj.Radius;
            sl = fliplr(sr);
            hr = sh + obj.Height/2;
            hl = -fliplr(hr);
            [V, N, F, T] = phx.internal.Geometry.revolution([hl hr; sl sr]', obj.Segments, true, true);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            r = obj.Diameter/2;
            rq = r^2;
            h = obj.Height;
            mc = obj.Density*pi*rq*h;
            ms = obj.Density*pi*r^3*4/3;
            mass = mc + ms;
            is = rq*ms*2/5;
            ia = mc*rq/2 + is;
            io = mc*(h^2 + 3*rq)/12 + is + ms*(h/2)^2;
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io io ia]);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phx.engine.io('add', body.WorldHandle, 'capsule', body.TypeID, obj.Diameter/2, obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end

end