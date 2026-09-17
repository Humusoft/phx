classdef Tube < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Tube Tube, pipe or funnel shape
%
%   Visual appearance is a straight or tapered tube with an open bore. The
%   Taper property narrows the bore towards one end while the wall keeps its
%   thickness, so a single shape covers a pipe, a funnel, a hopper and a
%   closed cup.
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
        % Inner diameter of the tube at its wide end
        InnerDiameter (1, 1) double = 0.9

        % Thickness of the tube wall
        WallThickness (1, 1) double = 0.05

        % Tube height along the modeling axis
        Height (1, 1) double = 1

        % Taper of the bore
        % Zero is a straight tube, a positive value narrows the bore at the
        % end on the positive side of Axis and a negative value at the
        % opposite end. The magnitude is the fraction by which the inner
        % radius shrinks, so 0.3 leaves a bore 70 % as wide and 1 closes it
        % completely. The wall keeps its thickness all along the tube.
        Taper (1, 1) double {mustBeInRange(Taper, -1, 1)} = 0

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
        % Inner radius of the tube at its wide end
        InnerRadius (1, 1) double

        % Outer diameter of the tube
        Diameter (1, 1) double

        % Outer radius of the tube
        Radius (1, 1) double
    end

    methods
        function obj = Tube(Options)
            arguments
                Options.?phx.shape.Tube
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
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
            obj.drawSkelet(target, obj.Color);

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
            ri = obj.InnerDiameter/2;
            t = obj.WallThickness;
            h = obj.Height/2;

            % Bore radius at the negative and the positive end of the axis
            if obj.Taper >= 0
                rb = [ri ri*(1 - obj.Taper)];
            else
                rb = [ri*(1 + obj.Taper) ri];
            end

            ZX = [-h rb(1) + t; h rb(2) + t; h rb(2); -h rb(1); -h rb(1) + t];
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            switch obj.Envelope
                case "cylinder"
                    % Bounding cylinder (centered on the shape origin)
                    bsize = phx.internal.Geometry.switchZAxis(obj.Axis, [obj.Radius obj.Radius obj.Height/2]);
                    eh = phx.engine.io('add', body.WorldHandle, 'cylinder', body.TypeID, bsize, char(obj.Axis), body.Transform, body.Mass, body.Inertia);
                case "convex"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
                case {"concave"}
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    faces = int32(ph.LinearizedFaces - 1);
                    sh_id = phx.engine.io('prepare', body.WorldHandle, uint64(0), 'concaveshape', vertices(:), numel(vertices)/3, faces, numel(faces)/3);
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'dynamictrimesh');
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'validation');
                    eh = phx.engine.io('add', body.WorldHandle, 'rigidbody', body.TypeID, sh_id, body.Transform, body.Mass, body.Inertia);
            end
        end

        function createComponent(obj, body)
        end
    end

end
