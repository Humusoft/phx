classdef PrismaticJoint < phx.base.Joint
%phx.PrismaticJoint Prismatic joint
%
%   Prismatic joint realizes a kinematic constraint with 1 degree of freedom
%   specified as translation along the sliding axis. The sliding axis is the
%   Z axis of the joint coordinate systems of both connected bodies.
%
%   phx.PrismaticJoint(bodyA, bodyB) creates a joint between two bodies A and B
%   attached to their points of origin, with the sliding axis aligned to axis Z
%   of the local coordinate system of each body.
%   Custom sliding axes can be set using the AxisA and AxisB properties, custom
%   joint coordinate systems using TransformA and TransformB or the PointA,
%   PointB, EulerAnglesA and EulerAnglesB helpers; in every case the sliding
%   axis is the local Z axis of each coordinate system.
%   Only the components perpendicular to the sliding axis place the bodies with
%   respect to each other; an offset along the sliding axis merely shifts the
%   zero of the free translation.
%
%   Because this joint also locks the rotation about the sliding axis, AxisA
%   and AxisB alone do not fully describe it: they fix where each coordinate
%   system points its Z axis, but not how the two are rolled about it. They are
%   enough when both bodies share an orientation, as a body and a part sliding
%   straight out of it usually do. When they do not, set TransformA and
%   TransformB (or the EulerAngles helpers) so that both coordinate systems
%   coincide in every axis, otherwise the solver twists the bodies into line
%   over the first steps.
%
%   phx.PrismaticJoint(___, name, value, ...) creates a joint and sets properties
%   values according to given name-value pairs.
%
%   See also phx.RevoluteJoint, phx.CylindricalJoint, phx.FixedJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Constant, Access = private)
        % The engine slides along the frame's axis X while PHX exposes axis Z,
        % so both frames are handed over turned by -90 degrees about Y. The
        % matrix is written out as integers to keep the frames exact.
        EngineFrame = [0 0 -1 0; 0 1 0 0; 1 0 0 0; 0 0 0 1]
    end

    properties (Access = private)
        hL
        hM
    end

    properties
        % Draw joint as overlay
        Overlay (1, 1) logical = false
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
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
            if valid
                obj.WorldHandle = world;
                M = phx.PrismaticJoint.EngineFrame;
                TA = obj.TransformA*M;
                TB = obj.TransformB*M;
                obj.ObjectHandle = phx.engine.io('add', world, 'sliderconstraint', obj.Parents{1}.ObjectHandle, obj.Parents{2}.ObjectHandle, ...
                    TA(:), TB(:), true, ~obj.MutualCollisions);
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