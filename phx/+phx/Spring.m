classdef Spring < phx.base.Object
%phx.Spring Spring
%
%   Spring is a combined element that consists of an ideal spring and an ideal
%   damper in parallel.
%
%   phx.Spring(bodyA, bodyB) creates a spring between two bodies A and B
%   attached to their points of origin.
%   Custom connection points can be set using PointA and PointB properties.
%
%   phx.Spring(___, name, value, ...) creates a spring and sets properties values
%   according to given name-value pairs.
%
%   See also phx.SphericalJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

% TODO inertia, hysteresis and progressiveness exponents
% TODO implementation of the spring calculation directly in the engine

    properties (Access = private)
        hL
        CMap
    end

    properties (SetAccess = private)
        Force = [0 0 0]
        Length = 0
    end

    properties (Dependent)
        % Elastic energy stored in the spring (J)
        Energy (1, 1) double
    end

    properties
        % Connecting point in the local space of the first body
        PointA (1, 3) double = [0 0 0]

        % Connecting point in the local space of the second body
        PointB (1, 3) double = [0 0 0]

        % Free length of the spring
        FreeLength (1, 1) double = 0

        % Stiffness coefficient of the spring (N/m)
        Stiffness (1, 1) double = 20

        % Damping coefficient of the damper (N*s/m)
        Damping (1, 1) double = 0

        % Name of the used colormap
        Colormap (1, 1) string = "none"

        % Force range mapped on the whole color map
        ColorRange (1, 2) double = [0 100]

        % Draw spring as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Spring(ParentA, ParentB, Options)
            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.Spring
            end

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";
            obj.ParentAxes = ParentA.ParentAxes;

            % Process input arguments
            obj.Parents = addChild([ParentA ParentB], obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 2.0, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color*255 255]'), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.Spring.updateView({obj});
        end

        function value = get.Energy(obj)
            pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
            pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.PointB);
            value = obj.Stiffness*(norm(pb - pa) - obj.FreeLength)^2/2;
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

        function showCharacteristic(obj, x, vmul)
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

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            nO = numel(cellObjs);
            allHandles = zeros(1, nO*2, 'uint64');
            allForces = zeros(1, nO*6);
            allPoints = zeros(1, nO*6);
            for i = 1:nO
                obj = cellObjs{i}; % much faster than obj = objs(i)

                A = obj.Parents{1};
                B = obj.Parents{2};
    
                pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
                pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.PointB);
                dp = (pb - pa)';
                nrm = sqrt(dp(1)*dp(1) + dp(2)*dp(2) + dp(3)*dp(3));
                dp = dp/nrm;
                len = nrm - obj.FreeLength;
                vel = (len - obj.Length)/dt;
                if nrm ~= 0
                    Force = (obj.Stiffness*len + obj.Damping*vel)*dp;
                else
                    Force = [0 0 0];
                end

                %phx.engine.io('apply', world, [A.ObjectHandle B.ObjectHandle], 'forces', [Force, -Force], [obj.PointA, obj.PointB], false, true);
                i2 = i*2;
                i6 = i*6;
                allHandles(i2 - 1) = A.ObjectHandle;
                allHandles(i2) = B.ObjectHandle;
                allForces(i6-5:i6) = [Force -Force];
                allPoints(i6-5:i6) = [obj.PointA obj.PointB];

                obj.Length = len;
                obj.Force = Force;
            end

            phx.engine.io('apply', world, allHandles, 'forces', allForces, allPoints, false, true);
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                pa = phx.internal.transformPoint(obj.Parents{1}.Matrix, obj.PointA);
                pb = phx.internal.transformPoint(obj.Parents{2}.Matrix, obj.PointB);

                hL = obj.hL;
                hL.VertexData = single([pa', pb']);
                if ~isempty(obj.CMap)
                    cid = phx.Spring.x2id(norm(obj.Force), obj.ColorRange, [1 100]);
                    hL.ColorData = obj.CMap(cid, :)';
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