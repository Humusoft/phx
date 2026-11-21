classdef CapsuleShell < phx.base.Shape
%phx.shape.CapsuleShell Capsule shell shape with a wall of given thickness
%
%   A hollow capsule: a wall of WallThickness inside the outer Diameter,
%   around a cylindrical part of Height along Axis closed by two
%   hemispherical ends, as phx.shape.Capsule has them. The wall is solid
%   from both sides, like a vessel of sheet metal: a body outside the
%   shell touches its outer face, a body inside it (in the cavity) its
%   inner face; particles and a phx.Liquid inside stay in. Visual
%   appearance is the outer and the inner face, each a revolved profile of
%   Segments segments. Alpha is the opacity of the wall as a whole: a line
%   of sight through the wall crosses both faces, so each face is drawn
%   with the opacity 1 - sqrt(1 - Alpha).
%
%   Collision shape and mass properties are modeled as an ideal capsule
%   shell, independent of the visual details. The shell collides with all
%   other shapes, meshes, terrains and shells included. As any thin wall,
%   it lets through a body that moves farther in one step than the wall
%   thickness plus its own size; shorten the step for fast bodies.
%
%   phx.shape.CapsuleShell() creates a shape with default parameters.
%
%   phx.shape.CapsuleShell(Name, Value, ...) creates a shape and sets
%   properties values according to given name-value pairs:
%
%       drum = phx.Body(gca, "Shape", phx.shape.CapsuleShell( ...
%           "Diameter", 0.6, "Height", 1, "Axis", "x", "WallThickness", 0.02));
%
%   See also phx.Body, phx.shape.Capsule, phx.shape.SphereShell, phx.Liquid

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Outer diameter of the shell
        Diameter (1, 1) double {mustBePositive, mustBeFinite} = 1

        % Height of the cylindrical part, without the hemispherical ends
        Height (1, 1) double {mustBeNonnegative, mustBeFinite} = 1

        % Modeling axis of the capsule
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Thickness of the wall, inwards from the outer diameter; less than
        % the outer radius
        WallThickness (1, 1) double {mustBePositive, mustBeFinite} = 0.05

        % Number of segments of each face around the axis
        Segments (1, 1) double = 24

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
        function obj = CapsuleShell(Options)
            arguments
                Options.?phx.shape.CapsuleShell
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
            [V, N, F, T] = obj.face(obj.Diameter);
            [Vi, Ni, Fi, Ti] = obj.face(obj.InnerDiameter);
            V = [V; Vi];
            N = [N; -Ni];
            F = [F; fliplr(Fi) + size(V, 1) - size(Vi, 1)];   % V holds both faces now
            T = [T; Ti];
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);

            % Alpha is the opacity of the wall, which a line of sight
            % crosses at two faces: each takes 1 - sqrt(1 - Alpha)
            faces = obj;
            faces.Alpha = 1 - sqrt(1 - obj.Alpha);
            primitive = faces.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [volume, inertia, center] = computeVolume(obj)
            % The solid capsule of the outer face less that of the cavity
            obj.checkWall;
            [mo, io, ao] = solidCapsule(obj.Diameter/2, obj.Height, 1);
            [mi, ii, ai] = solidCapsule(obj.InnerDiameter/2, obj.Height, 1);
            volume = mo - mi;
            inertia = phx.internal.Geometry.switchZAxis(obj.Axis, [io - ii, io - ii, ao - ai]);
            center = [0 0 0];
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            % The engine takes the radius of the middle of the wall
            obj.checkWall;
            middle = (obj.Diameter - obj.WallThickness)/2;
            eh = phxEngine('add', body.WorldHandle, 'capsuleshell', body.TypeID, middle, obj.Height, char(obj.Axis), ...
                obj.WallThickness, body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

    methods (Access = private)
        function checkWall(obj)
            % The wall must leave a cavity; checked where the shape is used,
            % so that Diameter and WallThickness may be set in any order
            if ~(obj.WallThickness < obj.Diameter/2)
                error("phx:CapsuleShell:badWallThickness", ...
                    "WallThickness (%g) must be less than the outer radius (%g); a solid capsule is phx.shape.Capsule.", ...
                    obj.WallThickness, obj.Diameter/2);
            end
        end

        function [V, N, F, T] = face(obj, diameter)
            % One face, a capsule of the given diameter about z, as
            % phx.shape.Capsule draws it
            r = diameter/2;
            sh = sin(linspace(0, pi/2, obj.Segments/3))*r;
            sr = cos(linspace(0, pi/2, obj.Segments/3))*r;
            sl = fliplr(sr);
            hr = sh + obj.Height/2;
            hl = -fliplr(hr);
            [V, N, F, T] = phx.internal.Geometry.revolution([hl hr; sl sr]', obj.Segments, true, true);
        end
    end

end

function [mass, inertia, axial] = solidCapsule(r, h, density)
%solidCapsule Mass and principal moments of a solid capsule about z.
%   The moment about the transverse axes and about the axis, through the
%   centre; h is the height of the cylindrical part.
    mc = density*pi*r^2*h;
    ms = density*pi*r^3*4/3;
    mass = mc + ms;
    is = r^2*ms*2/5;
    axial = mc*r^2/2 + is;
    inertia = mc*(h^2 + 3*r^2)/12 + is + ms*(h/2)^2;
end
