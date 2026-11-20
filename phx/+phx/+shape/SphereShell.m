classdef SphereShell < phx.base.Shape
%phx.shape.SphereShell Spherical shell shape with a wall of given thickness
%
%   A hollow sphere: a wall of WallThickness inside the outer Diameter,
%   solid from both sides, like a vessel of sheet metal. A body outside
%   the shell touches its outer face, a body inside it (in the cavity) its
%   inner face; particles and a phx.Liquid inside stay in. Visual
%   appearance is the outer and the inner face, each a latitude-longitude
%   grid of Segments meridians; a texture is mapped onto both as onto a
%   phx.shape.Sphere. Alpha is the opacity of the wall as a whole: a line
%   of sight through the wall crosses both faces, so each face is drawn
%   with the opacity 1 - sqrt(1 - Alpha).
%
%   Collision shape and mass properties are modeled as an ideal spherical
%   shell, independent of the visual details. The shell collides with all
%   other shapes, meshes, terrains and shells included, but has no
%   continuous collision detection: a body passing through the wall in one
%   step, faster than the wall thickness plus its own size per step, goes
%   through it.
%
%   phx.shape.SphereShell() creates a shape with default parameters.
%
%   phx.shape.SphereShell(Name, Value, ...) creates a shape and sets
%   properties values according to given name-value pairs:
%
%       tank = phx.Body(gca, "Type", "static", "Shape", ...
%           phx.shape.SphereShell("Diameter", 1, "WallThickness", 0.02));
%
%   See also phx.Body, phx.shape.Sphere, phx.shape.Tube, phx.Liquid

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Outer diameter of the shell
        Diameter (1, 1) double {mustBePositive, mustBeFinite} = 1

        % Thickness of the wall, inwards from the outer diameter; less than
        % the outer radius
        WallThickness (1, 1) double {mustBePositive, mustBeFinite} = 0.05

        % Number of meridian segments of each face; the parallels follow at
        % the same angular step
        Segments (1, 1) double = 28

        % Volumetric density of the wall (kg/m^3)
        Density (1, 1) double = 1000
    end

    properties (Dependent)
        % Outer radius of the shell
        Radius (1, 1) double

        % Diameter of the cavity, Diameter - 2*WallThickness
        InnerDiameter (1, 1) double
    end

    methods
        function obj = SphereShell(Options)
            arguments
                Options.?phx.shape.SphereShell
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function obj = set.Radius(obj, radius)
            obj.Diameter = radius*2;
        end

        function radius = get.Radius(obj)
            radius = obj.Diameter/2;
        end

        function d = get.InnerDiameter(obj)
            d = obj.Diameter - 2*obj.WallThickness;
        end

        function drawTo(obj, target)
            obj.checkWall;

            % The outer face, and the inner face turned inside out: its
            % normals and its winding face the cavity
            [V, N, F, T] = phx.internal.Geometry.sphere(obj.Diameter, obj.Segments);
            [Vi, Ni, Fi, Ti] = phx.internal.Geometry.sphere(obj.InnerDiameter, obj.Segments);
            V = [V; Vi];
            N = [N; -Ni];
            F = [F; fliplr(Fi) + size(Vi, 1)];
            T = [T; Ti];
            % Alpha is the opacity of the wall, which a line of sight
            % crosses at two faces: each takes 1 - sqrt(1 - Alpha)
            faces = obj;
            faces.Alpha = 1 - sqrt(1 - obj.Alpha);
            primitive = faces.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            obj.checkWall;
            ro = obj.Diameter/2;
            ri = obj.InnerDiameter/2;
            mass = obj.Density*pi*(ro^3 - ri^3)*4/3;
            inertia = mass*2/5*(ro^5 - ri^5)/(ro^3 - ri^3);
            center = [0 0 0];
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            % The engine takes the radius of the middle of the wall
            obj.checkWall;
            middle = (obj.Diameter - obj.WallThickness)/2;
            eh = phxEngine('add', body.WorldHandle, 'sphereshell', body.TypeID, middle, obj.WallThickness, ...
                body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

    methods (Access = private)
        function checkWall(obj)
            % The wall must leave a cavity; checked where the shape is used,
            % so that Diameter and WallThickness may be set in any order
            if ~(obj.WallThickness < obj.Diameter/2)
                error("phx:SphereShell:badWallThickness", ...
                    "WallThickness (%g) must be less than the outer radius (%g); a solid ball is phx.shape.Sphere.", ...
                    obj.WallThickness, obj.Diameter/2);
            end
        end
    end

end
