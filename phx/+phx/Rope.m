classdef Rope < phx.base.Object
%phx.Rope Rope routed over pulleys
%
%   Rope connects a chain of two or more bodies. The first and the last
%   body are the rope ends; all bodies in between act as ideal
%   (frictionless) pulleys, so a single tension acts along the whole rope.
%   The routing points are defined in the local spaces of the bodies by
%   the rows of the Points matrix.
%
%   A body may appear in the chain repeatedly with different routing
%   points - e.g. the moving block of a tackle that the rope wraps twice.
%
%   The rope transfers tension only: it acts as a spring and damper when
%   stretched beyond its free length and goes slack otherwise. The free
%   length is measured from the initial configuration of the bodies
%   (InitialLength) and can be changed while the simulation runs through
%   the Displacement property - positive values pay the rope out, negative
%   values winch it in (drivable by a phx.Script, loggable along with the
%   Length and Force readouts by a phx.Logger).
%
%   phx.Rope(bodies) creates a rope through the points of origin of the
%   given bodies.
%
%   phx.Rope(___, name, value, ...) creates a rope and sets properties
%   values according to given name-value pairs.
%
%   The engine holds the rope over its pulleys and solves it together with
%   the joints and contacts, by a step that keeps its energy and is stable
%   at any stiffness and damping. It needs at least two different bodies.
%   Force is the tension applied over the last step; Length is read from
%   the current poses.
%
%   See also phx.Spring, phx.Script, phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
        hM
        CMap
    end

    properties (Dependent, SetAccess = private)
        % Current tension in the rope (N)
        Force (1, 1) double

        % Current total length of the rope (m)
        Length (1, 1) double
    end

    properties
        % Routing points in the local spaces of the bodies (one row per body)
        Points (:, 3) double

        % Unstretched rope length; NaN = measure the initial configuration
        InitialLength (1, 1) double = NaN

        % Free length change: positive pays out, negative winches in
        Displacement (1, 1) double = 0

        % Stiffness of the stretched rope (N/m)
        Stiffness (1, 1) double {mustBeNonnegative, mustBeFinite} = 1000

        % Damping of the stretched rope (N*s/m)
        Damping (1, 1) double {mustBeNonnegative, mustBeFinite} = 10

        % Name of the used colormap (tension coloring)
        Colormap (1, 1) string = "none"

        % Tension range mapped on the whole color map
        ColorRange (1, 2) double = [0 100]
    end

    properties (Dependent, SetAccess = private)
        % Free length of the rope (InitialLength + Displacement)
        FreeLength

        % Elastic energy stored in the stretched rope (J)
        Energy (1, 1) double
    end

    methods
        function obj = Rope(Parents, Options)
            arguments
                Parents (1, :) {mustBeA(Parents, "phx.Body")}
                Options.?phx.Rope
            end

            if numel(Parents) < 2
                error("phx:Rope:tooFewBodies", "Rope requires at least two bodies.");
            end

            % Set default values; the engine does the work of the steps
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parents(1).ParentAxes;

            % Process input arguments; a body may appear in the chain more
            % than once, but it must be registered as a parent only once.
            % A failure leaves no half-built object attached to the parents.
            n = numel(Parents);
            try
                for a = 1:n
                    if ~any(Parents(1:a-1) == Parents(a))
                        addChild(Parents(a), obj);
                    end
                end
                obj.Parents = num2cell(Parents);
                phx.internal.applyArguments(Options, obj);

                % Default routing points and their validation
                if isempty(obj.Points)
                    obj.Points = zeros(n, 3);
                elseif size(obj.Points, 1) ~= n
                    error("phx:Rope:pointsSize", "Points must have one row per body.");
                end
            catch err
                obj.abandon(Parents);
                rethrow(err);
            end

            % Measure the unstretched length from the initial configuration
            if isnan(obj.InitialLength)
                obj.InitialLength = phx.Rope.totalLength(obj);
            end

            % Create graphics objects
            clr = uint8([obj.Color*255 255]');
            layer = phx.internal.choose({'middle', 'front'}, obj.Overlay + 1);
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 2.0, 'ColorBinding', 'object', 'ColorData', clr, 'StripData', uint32([1 n + 1]), 'Layer', layer);
            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, 'EdgeColorData', clr, 'Style', 'circle', 'Size', 10, 'Layer', layer, 'PickableParts', 'none');
            phx.Rope.updateView({obj});
        end

        function len = get.FreeLength(obj)
            len = obj.InitialLength + obj.Displacement;
        end

        function value = get.Energy(obj)
            stretch = phx.Rope.totalLength(obj) - obj.FreeLength;
            value = obj.Stiffness*max(stretch, 0)^2/2;
        end

        function value = get.Force(obj)
            value = 0;
            if ~isempty(obj.ObjectHandle)
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'spring');
            end
        end

        function value = get.Length(obj)
            value = phx.Rope.totalLength(obj);
        end

        function set.InitialLength(obj, value)
            obj.InitialLength = value;
            obj.pushLength;
        end

        function set.Displacement(obj, value)
            obj.Displacement = value;
            obj.pushLength;
        end

        function set.Stiffness(obj, value)
            obj.Stiffness = value;
            obj.pushSpring;
        end

        function set.Damping(obj, value)
            obj.Damping = value;
            obj.pushSpring;
        end

        function set.Colormap(obj, map)
            try
                cmap = feval(map, 100);
                cmap(:, 4) = 1;
                obj.Colormap = map;
            catch
                cmap = [];
                obj.Colormap = "none";
            end
            obj.CMap = uint8(cmap*255);
        end
    end

    methods (Access = private)
        function value = engineLength(obj)
            % A rope winched in past zero length pulls from zero
            value = max(obj.FreeLength, 0);
        end

        function pushLength(obj)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'lengths', obj.engineLength);
            end
        end

        function pushSpring(obj)
            if ~isempty(obj.ObjectHandle)
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'spring', obj.Stiffness, obj.Damping);
            end
        end
    end

    methods (Access = protected)
        function valid = checkObject(obj)
            % A rope is strung through at least two bodies.
            valid = numel(obj.Parents) >= 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % Built once and kept across pipeline rebuilds; the setters
                % update it
                obj.WorldHandle = world;
                handles = cellfun(@(p) p.ObjectHandle, obj.Parents);
                obj.ObjectHandle = phx.engine.io('add', world, 'ropeconstraint', handles, obj.Points', ...
                    obj.engineLength, obj.Stiffness, obj.Damping);
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

                P = obj.Parents;
                n = numel(P);
                pts = zeros(3, n, 'single');
                for a = 1:n
                    pts(:, a) = phx.internal.transformPoint(P{a}.Matrix, obj.Points(a, :))';
                end

                hL = obj.hL;
                hL.VertexData = pts;
                obj.hM.VertexData = pts;
                if ~isempty(obj.CMap)
                    cid = phx.Spring.x2id(obj.Force, obj.ColorRange, [1 100]);
                    hL.ColorData = obj.CMap(cid, :)';
                    obj.hM.EdgeColorData = obj.CMap(cid, :)';
                end
            end
        end
    end

    methods (Static, Access = private)
        function len = totalLength(obj)
            pts = zeros(numel(obj.Parents), 3);
            for a = 1:numel(obj.Parents)
                pts(a, :) = phx.internal.transformPoint(obj.Parents{a}.Matrix, obj.Points(a, :));
            end
            len = sum(sqrt(sum(diff(pts, 1, 1).^2, 2)));
        end
    end

end
