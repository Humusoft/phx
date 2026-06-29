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
        LastLength = NaN
    end

    properties (SetAccess = private)
        % Current tension in the rope (N)
        Force = 0

        % Current total length of the rope (m)
        Length = 0
    end

    properties
        % Routing points in the local spaces of the bodies (one row per body)
        Points (:, 3) double

        % Unstretched rope length; NaN = measure the initial configuration
        InitialLength (1, 1) double = NaN

        % Free length change: positive pays out, negative winches in
        Displacement (1, 1) double = 0

        % Stiffness of the stretched rope (N/m)
        Stiffness (1, 1) double = 1000

        % Damping of the stretched rope (N*s/m)
        Damping (1, 1) double = 10

        % Name of the used colormap (tension coloring)
        Colormap (1, 1) string = "none"

        % Tension range mapped on the whole color map
        ColorRange (1, 2) double = [0 100]

        % Draw rope as overlay
        Overlay (1, 1) logical = false
    end

    properties (Dependent)
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

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parents(1).ParentAxes;

            % Process input arguments; a body may appear in the chain more
            % than once, but it must be registered as a parent only once
            for a = 1:numel(Parents)
                if ~any(Parents(1:a-1) == Parents(a))
                    addChild(Parents(a), obj);
                end
            end
            obj.Parents = num2cell(Parents);
            phx.internal.applyArguments(Options, obj);

            % Default routing points and their validation
            n = numel(Parents);
            if isempty(obj.Points)
                obj.Points = zeros(n, 3);
            elseif size(obj.Points, 1) ~= n
                error("phx:Rope:pointsSize", "Points must have one row per body.");
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

    methods (Access = protected)
        function valid = initObject(obj, world)
            obj.LastLength = NaN; % reset the damping memory on (re)builds
            valid = numel(obj.Parents) >= 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                P = obj.Parents;
                n = numel(P);
                handles = zeros(1, n, 'uint64');
                pts = zeros(n, 3);
                for a = 1:n
                    handles(a) = P{a}.ObjectHandle;
                    pts(a, :) = phx.internal.transformPoint(P{a}.Matrix, obj.Points(a, :));
                end

                % Total length and unit directions of the rope segments
                seg = diff(pts, 1, 1);
                lens = sqrt(sum(seg.^2, 2));
                dirs = seg./max(lens, 1e-9);
                len = sum(lens);

                % Single tension from the total stretch (frictionless pulleys)
                stretch = len - obj.InitialLength - obj.Displacement;
                if isnan(obj.LastLength)
                    rate = 0;
                else
                    rate = (len - obj.LastLength)/dt;
                end
                obj.LastLength = len;
                tension = obj.Stiffness*stretch + obj.Damping*rate;
                if stretch <= 0 || tension < 0
                    tension = 0;    % the rope cannot push
                end
                obj.Length = len;
                obj.Force = tension;

                if tension > 0
                    % Rope ends are pulled inward, every pulley feels the
                    % resultant of its two adjacent segments
                    F = zeros(n, 3);
                    F(1, :) = tension*dirs(1, :);
                    F(n, :) = -tension*dirs(n - 1, :);
                    for a = 2:n-1
                        F(a, :) = tension*(dirs(a, :) - dirs(a - 1, :));
                    end

                    F = F';
                    pl = obj.Points';
                    phx.engine.io('apply', world, handles, 'forces', F(:)', pl(:)', false, true);
                end
            end
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
