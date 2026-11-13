classdef Cone < phx.base.Shape
%phx.shape.Cone Cone or truncated cone shape
%
%   Visual appearance is based on a geometry of revolved profile with
%   an adjustable level of details.
%
%   Collision shape and mass properties are modeled as an ideal cone
%   independent on visual details. Diameter gives the diameters at the
%   negative and the positive end of Axis; a single value is a sharp cone
%   with its apex on the positive axis, two values a truncated cone (or a
%   cylinder when they are equal).
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
        % Diameters at the negative and the positive end of Axis
        % A single value d is [d 0], a sharp cone with its apex on the
        % positive axis; two values make a truncated cone.
        Diameter (1, :) double {mustBeConeSize} = [1 0]

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
        % Radii at the negative and the positive end of Axis (as Diameter)
        Radius (1, :) double {mustBeConeSize}
    end

    methods
        function obj = Cone(Options)
            arguments
                Options.?phx.shape.Cone
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.Diameter(obj, diameter)
            if isscalar(diameter)
                diameter = [diameter 0];
            end
            obj.Diameter = diameter;
        end

        function diameter = get.Diameter(obj)
            % A cone saved by PHX 1.0 kept a single diameter
            diameter = obj.Diameter;
            if isscalar(diameter)
                diameter = [diameter 0];
            end
        end

        function obj = set.Radius (obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius (obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            % The profile from the negative to the positive end, each end
            % capped unless it is an apex
            d = obj.Diameter;
            [V, N, F, T] = phx.internal.Geometry.revolution([-0.5 d(1)/2; 0.5 d(2)/2], obj.Segments, d(1) > 0, d(2) > 0);
            V = V.*[1 1 obj.Height];
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);

            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            % A solid of revolution of radius r(z), linear in z, integrated
            % by the 3-point Gauss rule, exact for the polynomials of up to
            % the fifth degree here: mass, centre of mass zc, the moment
            % about the axis and the transverse one about zc
            r = obj.Radius;
            h = obj.Height;
            s = [-sqrt(3/5) 0 sqrt(3/5)];
            g = [5 8 5]/18;
            z = s*h/2;
            rz = r(1) + (r(2) - r(1))*(z/h + 0.5);
            dm = obj.Density*pi*rz.^2.*g*h;
            mass = sum(dm);
            zc = sum(dm.*z)/mass;
            ia = sum(dm.*rz.^2/2);
            io = sum(dm.*(rz.^2/4 + (z - zc).^2));
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io io ia]);
            center = phx.internal.Geometry.switchZAxis(obj.Axis, [0 0 zc]);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            eh = phxEngine('add', body.WorldHandle, 'cone', body.TypeID, obj.Radius, obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

end

function mustBeConeSize(value)
%mustBeConeSize Validates Diameter and Radius: one or two values, not negative, not both zero
    if ~any(numel(value) == [1 2]) || any(~isfinite(value)) || any(value < 0) || ~any(value > 0)
        error("phx:Cone:badSize", ...
            "Diameter (Radius) must be one or two finite, nonnegative values, not all zero.");
    end
end