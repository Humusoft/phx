classdef Tube < phx.base.Shape
%phx.shape.Tube Tube, pipe or funnel shape
%
%   Visual appearance is a straight or tapered tube with an open bore.
%   InnerDiameter gives the bore at the negative and the positive end of
%   Axis; a single value is a straight tube, two values taper the bore while
%   the wall keeps its thickness, so a single shape covers a pipe, a funnel,
%   a hopper and a closed cup.
%
%   Collision shape is selected by the Envelope property: the exact concave
%   triangle mesh, which is the only one that leaves the bore open, the convex
%   hull of the mesh, or a bounding cylinder about the tube axis, which keeps
%   a rolling body smooth where a faceted hull would not. Mass properties are
%   always calculated from the actual triangular mesh.
%
%   phx.shape.Tube() creates a shape with default parameters.
%
%   phx.shape.Tube(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Cylinder, phx.shape.Revolution

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Inner diameters at the negative and the positive end of Axis
        % A single value d is [d d], a straight tube; two values taper the
        % bore while the wall keeps its thickness. A zero closes the bore at
        % that end, leaving a disc as wide as the wall is thick.
        InnerDiameter (1, :) double {mustBeBoreSize} = [0.9 0.9]

        % Thickness of the tube wall
        WallThickness (1, 1) double {mustBePositive, mustBeFinite} = 0.05

        % Tube height along the modeling axis
        Height (1, 1) double = 1

        % Modeling axis of the tube
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of tube segments
        Segments (1, 1) double = 24

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Collision envelope: "concave" triangle mesh, "convex" hull or
        % bounding "cylinder" about the tube axis
        Envelope {mustBeMember(Envelope, ["concave", "convex", "cylinder"])} = "concave"
    end

    properties (Dependent)
        % Inner radii at the negative and the positive end of Axis (as
        % InnerDiameter)
        InnerRadius (1, :) double {mustBeBoreSize}

        % Outer diameters at the negative and the positive end of Axis
        % The bore plus twice the wall; setting it keeps WallThickness and
        % moves the bore.
        Diameter (1, :) double

        % Outer radii at the negative and the positive end of Axis (as
        % Diameter)
        Radius (1, :) double
    end

    methods
        function obj = Tube(Options)
            arguments
                Options.?phx.shape.Tube
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.InnerDiameter(obj, diameter)
            if isscalar(diameter)
                diameter = [diameter diameter];
            end
            obj.InnerDiameter = diameter;
        end

        function diameter = get.InnerDiameter(obj)
            % A tube saved by PHX 1.0 kept a single diameter
            diameter = obj.InnerDiameter;
            if isscalar(diameter)
                diameter = [diameter diameter];
            end
        end

        function obj = set.InnerRadius (obj, radius)
            obj.InnerDiameter = radius*2;
        end

        function radius = get.InnerRadius (obj)
            radius = obj.InnerDiameter/2;
        end

        function obj = set.Diameter (obj, diameter)
            obj.InnerDiameter = diameter - obj.WallThickness*2;
        end

        function diameter = get.Diameter (obj)
            diameter = obj.InnerDiameter + obj.WallThickness*2;
        end

        function obj = set.Radius (obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius (obj)
            radius = obj.Diameter/2;
        end

        function drawTo(obj, target)
            [V, N, F, T] = phx.internal.Geometry.revolution(obj.wallProfile, obj.Segments, false, false);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);

            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            [V, ~, F] = phx.internal.Geometry.revolution(obj.wallProfile, obj.Segments, false, false);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);

            [mass, I0] = phx.internal.Geometry.meshMass(V, F, obj.Density);
            inertia = I0([1 5 9]);
        end
    end

    methods (Access = private)
        function ZX = wallProfile(obj)
            % Closed [z x] loop of the tube wall. It is traversed outer wall
            % first so that the outer surface, both end faces and the bore all
            % come out facing away from the material.
            rb = obj.InnerDiameter/2;
            t = obj.WallThickness;
            h = obj.Height/2;
            ZX = [-h rb(1) + t; h rb(2) + t; h rb(2); -h rb(1); -h rb(1) + t];
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            switch obj.Envelope
                case "cylinder"
                    % Bounding cylinder (centered on the shape origin)
                    eh = phx.engine.io('add', body.WorldHandle, 'cylinder', body.TypeID, max(obj.Radius), obj.Height, char(obj.Axis), body.Transform, body.Mass, body.Inertia);
                case "convex"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = obj.createHullBody(body, vertices);
                case "concave"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = obj.createTrimeshBody(body, vertices, ph.LinearizedFaces);
            end
        end
    end

end

function mustBeBoreSize(value)
%mustBeBoreSize Validates InnerDiameter and InnerRadius: one or two finite, nonnegative values
    if ~any(numel(value) == [1 2]) || any(~isfinite(value)) || any(value < 0)
        error("phx:Tube:badSize", ...
            "The bore must be one or two finite, nonnegative values (Diameter at least twice WallThickness).");
    end
end
