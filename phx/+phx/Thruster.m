classdef Thruster < phx.base.Object
%phx.Thruster Thrust actuator
%
%   Thruster applies a body-fixed thrust force to a body. The mounting
%   point and the thrust direction are defined in the local space of the
%   body, so the thrust follows the body orientation - like an attached
%   rocket engine, a propeller or a fan.
%
%   The actual thrust is MaxThrust*Throttle. Optionally it follows the
%   throttle with a first-order lag (TimeConstant) and applies a reaction
%   torque about the thrust axis (ReactionFactor) - the counter-torque of
%   a spinning propeller.
%
%   The Throttle property can be changed while the simulation runs, driven
%   by a phx.Script curve, or set from Simulink; the resulting Thrust can
%   be recorded with a phx.Logger.
%
%   phx.Thruster(body) creates a thruster acting at the point of origin
%   of the body, thrusting along its local Z axis.
%
%   phx.Thruster(___, name, value, ...) creates a thruster and sets
%   properties values according to given name-value pairs.
%
%   See also phx.Spring, phx.Resistance, phx.Script, phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
    end

    properties (SetAccess = private)
        % Current thrust force magnitude (after the throttle lag)
        Thrust (1, 1) double = 0
    end

    properties
        % Mounting point in the local space of the body
        Point (1, 3) double = [0 0 0]

        % Thrust direction in the local space of the body (unit vector)
        Direction (1, 3) double = [0 0 1]

        % Maximal thrust force at full throttle
        MaxThrust (1, 1) double = 1

        % Throttle setting (-1 to 1, negative for reverse thrust)
        Throttle (1, 1) double {mustBeInRange(Throttle, -1, 1)} = 0

        % Time constant of the first-order throttle response (0 = immediate)
        TimeConstant (1, 1) double {mustBeNonnegative} = 0

        % Reaction torque about the thrust axis per unit thrust (N*m/N)
        ReactionFactor (1, 1) double = 0

        % Exhaust vector multiplication factor for drawing
        ForceVectorSize (1, 1) double = 0.001

        % Draw the exhaust vector as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Thruster(Parent, Options)
            arguments
                Parent (1, 1) {mustBeA(Parent, "phx.Body")}
                Options.?phx.Thruster
            end

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parent.ParentAxes;

            % Process input arguments
            obj.Parents = addChild(Parent, obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 2.0, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color*255 255]'), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.Thruster.updateView({obj});
        end

        function set.Direction(obj, value)
            len = norm(value);
            if len == 0
                error("phx:Thruster:zeroDirection", "Direction must be a nonzero vector.");
            end
            obj.Direction = value/len;
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = isscalar(obj.Parents) && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj)
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                body = obj.Parents{1};

                % The thrust follows the throttle, optionally with a lag
                target = obj.MaxThrust*obj.Throttle;
                if obj.TimeConstant > 0
                    obj.Thrust = obj.Thrust + (target - obj.Thrust)*min(dt/obj.TimeConstant, 1);
                else
                    obj.Thrust = target;
                end

                if obj.Thrust ~= 0
                    % The thrust is fixed to the body, so rotate it to the
                    % global frame here and apply as a global force at the
                    % local mounting point
                    R = body.Matrix(1:3, 1:3);
                    phx.engine.io('apply', world, body.ObjectHandle, 'force', (R*(obj.Thrust*obj.Direction)')', obj.Point, false, true);
                    if obj.ReactionFactor ~= 0
                        phx.engine.io('apply', world, body.ObjectHandle, 'torque', (R*(obj.ReactionFactor*obj.Thrust*obj.Direction)')', false);
                    end
                end
            end
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                % Draw the exhaust opposite to the thrust direction
                M = obj.Parents{1}.Matrix;
                p0 = phx.internal.transformPoint(M, obj.Point);
                p1 = phx.internal.transformPoint(M, obj.Point - obj.Direction*obj.Thrust*obj.ForceVectorSize);
                obj.hL.VertexData = single([p0' p1']);
            end
        end
    end

end
