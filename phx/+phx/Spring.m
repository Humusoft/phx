classdef Spring < phx.base.Object
%phx.Spring Linear spring-damper between two bodies
%
%   Spring is a combined element that consists of an ideal spring, an ideal
%   damper and, with a nonzero Inertance, an ideal inerter in parallel.
%
%   phx.Spring(bodyA, bodyB) creates a spring between two bodies A and B
%   attached to their points of origin.
%   Custom connection points can be set using PointA and PointB properties.
%
%   phx.Spring(___, name, value, ...) creates a spring and sets properties values
%   according to given name-value pairs.
%
%   The engine solves the spring together with the joints and contacts, by
%   a step that keeps its energy and is stable at any stiffness and damping.
%   Force is the force applied over the last step; Length is read from the
%   current poses.
%
%   See also phx.Rope, phx.BushingJoint, phx.SphericalJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        hL
    end

    properties (Dependent, SetAccess = private)
        % Force acting on the first body (N); the second body gets its opposite
        Force (1, 3) double

        % Current length of the spring, the distance between both connecting
        % points (m); the extension is Length - FreeLength
        Length (1, 1) double

        % Elastic energy stored in the spring (J)
        Energy (1, 1) double
    end

    properties
        % Connecting point in the local space of the first body
        PointA (1, 3) double = [0 0 0]

        % Connecting point in the local space of the second body
        PointB (1, 3) double = [0 0 0]

        % Free length of the spring
        FreeLength (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Stiffness coefficient of the spring (N/m)
        Stiffness (1, 1) double {mustBeNonnegative, mustBeFinite} = 20

        % Damping coefficient of the damper (N*s/m)
        Damping (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Inertance of an inerter in parallel (kg): a force proportional to
        % the relative acceleration of the connecting points along the line
        Inertance (1, 1) double {mustBeNonnegative, mustBeFinite} = 0

        % Colormap (Nx3, RGB in 0-1) that colors the spring by the
        % magnitude of its force over ColorRange, e.g. parula(64); empty
        % for the fixed Color
        Colormap (:, 3) double {mustBeInRange(Colormap, 0, 1)} = zeros(0, 3)

        % Force range mapped on the whole color map
        ColorRange (1, 2) double = [0 100]
    end

    methods
        function obj = Spring(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.Spring
            end

            % Set default values; the engine does the work of the steps
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments; a failure leaves no half-built object
            % attached to the parents
            try
                obj.Parents = addChild([ParentA ParentB], obj);
                phx.internal.applyArguments(Options, obj);
            catch err
                obj.abandon([ParentA ParentB]);
                rethrow(err);
            end

            % Create graphics objects
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 2.0, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color*255 255]'), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            obj.applyColormap;
            phx.Spring.updateView({obj});
        end

        function value = get.Force(obj)
            value = [0 0 0];
            if ~isempty(obj.ObjectHandle)
                f = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'spring');
                [pa, pb] = obj.ends;
                d = pb - pa;
                n = norm(d);
                if n > 0
                    value = f*d/n;
                end
            end
        end

        function value = get.Length(obj)
            [pa, pb] = obj.ends;
            value = norm(pb - pa);
        end

        function value = get.Energy(obj)
            [pa, pb] = obj.ends;
            value = obj.Stiffness*(norm(pb - pa) - obj.FreeLength)^2/2;
        end

        function set.FreeLength(obj, value)
            obj.FreeLength = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'lengths', value);
            end
        end

        function set.Stiffness(obj, value)
            obj.Stiffness = value;
            obj.pushSpring;
        end

        function set.Damping(obj, value)
            obj.Damping = value;
            obj.pushSpring;
        end

        function set.Inertance(obj, value)
            obj.Inertance = value;
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'inertance', value);
            end
        end

        function set.Colormap(obj, map)
            obj.Colormap = map;
            obj.applyColormap;
        end

        function showCharacteristic(obj, x, vmul)
        %showCharacteristic Plots the force-travel characteristic of the spring.
        %
        %   showCharacteristic(obj) plots the spring force over one cycle of
        %   a sinusoidal travel of amplitude 1 m, at the velocity multiples
        %   0, 1 and 10, in a new figure.
        %
        %   showCharacteristic(obj, x) uses the travel amplitude x, in metres,
        %   or, for a vector x, the travel samples themselves.
        %
        %   showCharacteristic(obj, x, vmul) sets the velocity multiples;
        %   each adds vmul times the normalized travel velocity times Damping
        %   to the elastic force.
        %
        % See also phx.Spring

            arguments
                obj
                x (1, :) double = 1
                vmul (1, :) double = [0 1 10]
            end

            if isscalar(x)
                x = sind(0:360)*x;
            end

            ax = axes(figure, 'NextPlot', 'add', 'XAxisLocation', 'origin', 'YAxisLocation', 'origin');
            grid(ax, 'on');
            v = gradient(x);
            v = v./max(abs(v));

            for vk = vmul
                F = x*obj.Stiffness + vk*v*obj.Damping;
                plot(ax, x, F, 'DisplayName', "v * "+vk);
            end
            legend('show');
            xlabel('Travel (m)');
            ylabel('Force (N)');
        end
    end

    methods (Access = private)
        function [pa, pb] = ends(obj)
            pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
            pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.PointB);
        end

        function pushSpring(obj)
            if ~isempty(obj.ObjectHandle)
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'spring', obj.Stiffness, obj.Damping);
            end
        end

        function applyColormap(obj)
            % Keep Color aside while the colormap draws the line, or draw the
            % line in Color again; the next redraw applies the colormap
            if isempty(obj.hL) || ~isvalid(obj.hL)
                return
            end
            ph = phx.internal.PrimitiveHelper(obj.hL);
            if isempty(obj.Colormap)
                ph.releaseColor;
            else
                ph.holdColor;
            end
        end
    end

    methods (Access = protected)
        function valid = checkObject(obj)
            % A spring pulls two bodies together, so it needs both of them.
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if valid && isempty(obj.ObjectHandle)
                % Built once and kept across pipeline rebuilds; the setters
                % update it
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'spring', ...
                    [obj.Parents{1}.ObjectHandle obj.Parents{2}.ObjectHandle], ...
                    makehgtform('translate', obj.PointA), makehgtform('translate', obj.PointB), true, ...
                    obj.FreeLength, obj.Stiffness, obj.Damping);
                if obj.Inertance > 0
                    phxEngine('set', world, obj.ObjectHandle, 'inertance', obj.Inertance);
                end
            end
        end

        function destroyObject(obj)
            if ~isempty(obj.ObjectHandle)
                phxEngine('remove', obj.WorldHandle, obj.ObjectHandle);
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

                [pa, pb] = obj.ends;

                hL = obj.hL;
                hL.VertexData = single([pa', pb']);
                map = obj.Colormap;
                if ~isempty(map)
                    cid = phx.Spring.x2id(norm(obj.Force), obj.ColorRange, [1 size(map, 1)]);
                    hL.ColorData = uint8([map(cid, :) 1]'*255);
                end
            end
        end
    end

    methods (Static)
        % Real range to int range
        function id = x2id(x, xRange, idRange)
            x = (x - xRange(1))/(xRange(2) - xRange(1));
            id = round(idRange(1) + x*(idRange(2) - idRange(1)));
            if id < idRange(1)
                id = idRange(1);
            elseif id > idRange(2)
                id = idRange(2);
            end
        end
    end

end
