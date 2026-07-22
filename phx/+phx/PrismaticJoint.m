classdef PrismaticJoint < phx.base.Joint
%phx.PrismaticJoint Prismatic joint
%
%   Prismatic joint realizes a kinematic constraint with 1 degree of freedom
%   specified as translation along the sliding axis. The sliding axis is the
%   X axis of the joint coordinate systems of both connected bodies.
%
%   phx.PrismaticJoint(bodyA, bodyB) creates a joint between two bodies A and B
%   attached to their points of origin, with the sliding axis aligned to axis X
%   of the local coordinate system of each body.
%   Custom joint coordinate systems can be set using the TransformA and TransformB
%   properties, or via the PointA, PointB, EulerAnglesA and EulerAnglesB helpers;
%   in every case the sliding axis is the local X axis of each coordinate system.
%
%   phx.PrismaticJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   See also phx.RevoluteJoint, phx.FixedJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
        hM
    end

    properties
        % Transformation matrix relative to the first body
        TransformA (4, 4) double = eye(4)

        % Transformation matrix relative to the second body
        TransformB (4, 4) double = eye(4)

        % Draw joint as overlay
        Overlay (1, 1) logical = false
    end

    properties (Dependent)
        % Connecting point in the local space of the first body
        PointA (1, 3) double

        % Connecting point in the local space of the second body
        PointB (1, 3) double

        % Euler angles rotation (for z->y->x order) of the first body
        EulerAnglesA (1, 3) double

        % Euler angles rotation (for z->y->x order) of the second body
        EulerAnglesB (1, 3) double
    end

    methods
        function obj = PrismaticJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.PrismaticJoint
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            clr = uint8([obj.Color*255 255]');
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 1.0, 'ColorBinding', 'object', 'ColorData', clr, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, 'EdgeColorData', clr, 'Style', 'circle', 'Size', 10, 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.PrismaticJoint.updateView({obj});
        end

        function set.PointA(obj, value)
            obj.TransformA(13:15) = value;
        end

        function value = get.PointA(obj)
            value = obj.TransformA(13:15)';
        end

        function set.PointB(obj, value)
            obj.TransformB(13:15) = value;
        end

        function value = get.PointB(obj)
            value = obj.TransformB(13:15)';
        end

        function set.EulerAnglesA(obj, value)
            obj.TransformA(1:3, 1:3) = phx.internal.Math.rot321(value);
        end

        function value = get.EulerAnglesA(obj)
            value = phx.internal.Math.decomp321(obj.TransformA(1:3, 1:3));
        end

        function set.EulerAnglesB(obj, value)
            obj.TransformB(1:3, 1:3) = phx.internal.Math.rot321(value);
        end

        function value = get.EulerAnglesB(obj)
            value = phx.internal.Math.decomp321(obj.TransformB(1:3, 1:3));
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'sliderconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, ...
                    obj.TransformA(:), obj.TransformB(:), true, ~obj.MutualCollisions);
            end
        end

        function destroyObject(obj)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('remove', obj.WorldHandle, obj.ObjectHandle);
                obj.ObjectHandle = [];
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
                pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.PointB);
                vd = single([pa' pb']);

                obj.hL.VertexData = vd;
                obj.hM.VertexData = vd;
            end
        end
    end

end