classdef CylindricalJoint < phx.base.Joint
%phx.CylindricalJoint Cylindrical joint
%
%   Cylindrical joint realizes a kinematic constraint with 2 degrees of freedom
%   specified as translation along an axis and rotation about the same axis - a
%   shaft free to slide and spin inside a sleeve. The axis is the Z axis of the
%   joint coordinate systems of both connected bodies; the remaining four
%   degrees of freedom are locked.
%
%   phx.CylindricalJoint(bodyA, bodyB) creates a joint between two bodies A and
%   B attached to their points of origin, with the joint axis aligned to axis Z
%   of the local coordinate system of each body.
%   Custom connection points and direction vectors of the joint axis can be set
%   using the PointA, PointB, AxisA and AxisB properties, whole joint coordinate
%   systems using TransformA and TransformB or the EulerAngles helpers.
%   Only the components perpendicular to the joint axis place the bodies with
%   respect to each other; an offset along the axis merely shifts the zero of
%   the free translation, and a roll about it the zero of the free rotation.
%
%   Unlike phx.PrismaticJoint, this joint leaves the rotation about the axis
%   free, so AxisA and AxisB alone fully describe it - the two coordinate
%   systems need not agree on how they are rolled about the axis.
%
%   phx.CylindricalJoint(___, name, value, ...) creates a joint and sets
%   properties values according to given name-value pairs.
%
%   See also phx.PrismaticJoint, phx.RevoluteJoint, phx.GenericJoint

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
        % Draw joint as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = CylindricalJoint(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.CylindricalJoint
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
            phx.CylindricalJoint.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'generic6dofconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, ...
                    obj.TransformA(:), obj.TransformB(:), 'xyz', ~obj.MutualCollisions);
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'linlimits', [0 0 1], [0 0 -1]);
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'anglimits', [0 0 1], [0 0 -1]);
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
