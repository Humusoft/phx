classdef Rock < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Rock Rock shape
%
%   Visual appearance is a procedurally generated irregular rock created by
%   perturbing an icosahedron. Roundness controls how close the result is to
%   the ideal spherical shape and Diameter (or Radius) sets its size.
%
%   The collision shape is the convex hull of the generated mesh, while the
%   mass properties are modeled as an ideal sphere of the given Density.
%
%   phx.shape.Rock() creates a shape with default parameters.
%
%   phx.shape.Rock(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Sphere, phx.shape.Globe

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Rock diameter
        Diameter (1, 1) double = 1

        % Roundness factor (of the ideal icosahedron shape)
        Roundness (1, 1) double = 0.75

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent)
        % Rock radius
        Radius (1, 1) double
    end

    methods
        function obj = Rock(Options)
            arguments
                Options.?phx.shape.Rock
            end

            % Defaults
            obj.Style = "flat";

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
            [V, N, F] = phx.internal.Geometry.rock(obj.Radius*2, obj.Roundness);
            primitive = obj.drawMesh(target, V, N, F, [], []);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            r = obj.Diameter/2;
            mass = obj.Density*pi*r^3*4/3;
            inertia = [1 1 1]*r^2*mass*2/5;
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            ph = phx.internal.PrimitiveHelper(primitive);
            vertices = ph.Vertices';
            eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end

end