classdef Dipole < phx.base.Object
%phx.Dipole Dipole interaction
%
%   Dipole object allows you to assign a dipole charge to selected bodies.
%   All bodies in the group formed by this object then interact with each
%   other via a reaction force given by the following equation:
%   
%               c1 * c2
%      F = A * ---------
%                 r^2
%
%   phx.Dipole(bodies) creates a Dipole object and assigns it to all given
%   bodies, thus creating a mutually interacting group.
%
%   phx.Dipole(___, name, value, ...) creates a Dipole object and sets
%   properties values according to given name-value pairs.
%
%   See also phx.Monopole

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        ehs
        hL
        GridPoints
    end

    properties
        % Dipole charge
        Charge (:, 1) double

        % Dipole axis
        % Specifies the dipole orientation (direction) and the pole offset
        % from the center (magnitude)
        Axis (:, 3) double = [1 0 0]

        % Attractivity
        % Specifies the polarity of attraction
        % -1: opposite poles attract each other
        % +1: opposite poles repel each other
        Attractivity (1, 1) double = -1

        % Position of vector field origin
        VectorFieldCenter (1, 3) double = [0 0 0]

        % Vector field size [x y z]
        VectorFieldSize (1, 3) double = [10 10 0]

        % Step of vector field grid
        VectorFieldStep (1, 1) double = 1

        % Number of segments of each vector
        VectorSegments (1, 1) double = 4

        % Length of vectors
        VectorLength (1, 1) double = 1

        % Draw vector field as overlay
        Overlay (1, 1) logical = false
    end

    properties (Dependent)
        % Potential energy of the pole pairs (J)
        Energy (1, 1) double
    end

    methods
        function value = get.Energy(obj)
            P = obj.Parents;
            n = numel(P);
            pos = zeros(2*n, 3);
            for a = 1:n
                pos(a, :) = phx.internal.transformPoint(P{a}.Matrix, obj.Axis(a, :));
                pos(n + a, :) = phx.internal.transformPoint(P{a}.Matrix, -obj.Axis(a, :));
            end
            charge = [obj.Charge; -obj.Charge];
            body = [1:n 1:n];
            value = 0;
            for a = 1:(2*n - 1)
                dp = pos(a+1:end, :) - pos(a, :);
                r = sqrt(sum(dp.^2, 2));
                cc = charge(a)*charge(a+1:end)./r;
                cc(body(a+1:end) == body(a)) = 0; % own poles do not count
                value = value + sum(cc);
            end
            value = -obj.Attractivity*value;
        end

        function obj = Dipole(Parents, Options)
            arguments
                Parents (1, :) {mustBeA(Parents, "phx.Body")}
                Options.?phx.Dipole
            end

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parents(1).ParentAxes;

            % Process input arguments
            obj.Parents = addChild(Parents, obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            p = obj.VectorFieldSize/2;
            c = obj.VectorFieldCenter;
            s = obj.VectorFieldStep;
            x = (-p(1):s:p(1)) + c(1);
            y = (-p(2):s:p(2)) + c(2);
            z = (-p(3):s:p(3)) + c(3);
            obj.GridPoints = combvec(x, y, z);
            count = size(obj.GridPoints, 2);
            seg = obj.VectorSegments + 2;
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 0.5, 'ColorBinding', 'object', 'ColorData', uint8([127 127 127 255]'), 'StripData', uint32(1:seg:(count*seg + 1)), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.Dipole.updateView({obj}, [], 0, []);
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            % Prepare constant data
            P = obj.Parents;
            n = numel(P);
            obj.ehs = zeros(1, n, 'uint64');
            for a = 1:n
                obj.ehs(a) = P{a}.ObjectHandle;
            end

            valid = true;
        end

        function destroyObject(obj)
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                P = obj.Parents;
                n = numel(P);

                % Collect both poles of every dipole: rows 1..n are the +Axis
                % poles, rows n+1..2n the -Axis poles of the same bodies
                pos = zeros(2*n, 3);
                for a = 1:n
                    pos(a, :) = phx.internal.transformPoint(P{a}.Matrix, obj.Axis(a, :));
                    pos(n + a, :) = phx.internal.transformPoint(P{a}.Matrix, -obj.Axis(a, :));
                end
                charge = [obj.Charge; -obj.Charge];
                body = [1:n 1:n]; % pole -> body index

                Fp = zeros(2*n, 3);
                for a = 1:(2*n - 1)
                    b = a + 1;
                    dp = pos(b:end, :) - pos(a, :);
                    l = sqrt(sum(dp.^2, 2));
                    l = l.*l.*l;
                    F = dp*charge(a).*charge(b:end)./l;
                    F(body(b:end) == body(a), :) = 0; % the two poles of the same body do not interact
                    Fp(a, :) = Fp(a, :) + sum(F, 1);
                    Fp(b:end, :) = Fp(b:end, :) - F;
                end

                Fp = obj.Attractivity*Fp;
                % Global forces applied at local points (the pole offsets):
                % the engine moment computation expects center-relative arms
                phx.engine.io('apply', world, [obj.ehs obj.ehs], 'forces', Fp', [obj.Axis; -obj.Axis]', false, true);
            end
        end
        
        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                hL = obj.hL;
                grid = obj.GridPoints;
                count = size(grid, 2);
                seg = obj.VectorSegments + 2;

                P = obj.Parents;
                n = numel(P);
                pos = zeros(3, n*2);

                for a = 1:n
                    pos(:, a) = phx.internal.transformPoint(obj.Parents{a}.Matrix, obj.Axis(a, :))';
                    pos(:, n + a) = phx.internal.transformPoint(obj.Parents{a}.Matrix, -obj.Axis(a, :))';
                end

                charge = [obj.Charge; -obj.Charge]';
                segLen = obj.VectorLength/obj.VectorSegments;

                % XYZ = zeros(3, count*seg);
                % si = 1;
                % for h = 1:count
                %     point = grid(:, h);
                %     XYZ(:, si) = point;
                %     for m = 1:(seg - 2)
                %         dp = pos - point;
                %         l = sqrt(sum(dp.^2));
                %         l = l.*l.*l;
                %         F = sum(dp.*charge./l, 2);
                %         Flen = sqrt(F(1)*F(1) + F(2)*F(2) + F(3)*F(3));
                %         F = segLen*F./Flen;
                %         point = point + F;
                %         si = si + 1;
                %         XYZ(:, si) = point;
                %     end
                %     arr = [-F(2); F(1); F(3)];
                %     XYZ(:, si + 1) = XYZ(:, si - 1) + 0.25*arr;
                %     si = si + 2;
                % end
                XYZ = phx.internal.computeFieldArrows(grid, pos, charge, count, seg, segLen);
                
                XYZ(isnan(XYZ)) = 0;
                hL.VertexData = single(XYZ);
            end
        end
    end

end