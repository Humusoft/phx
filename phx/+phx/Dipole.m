classdef Dipole < phx.base.Object
%phx.Dipole Dipole interaction between bodies
%
%   Dipole object allows you to assign a dipole charge to selected bodies.
%   Each body carries two opposite poles, +Charge and -Charge, placed at
%   +Axis and -Axis from its origin. The poles of different bodies in the
%   group interact in pairs, along the line joining them, via a force of the
%   magnitude
%
%                       c1 * c2
%      F = Attractivity ---------
%                         r^2
%
%   where c1, c2 are the pole charges and r is their distance; the two poles
%   of one body do not interact. A positive F pulls the poles together, a
%   negative one pushes them apart: with the default Attractivity of -1
%   opposite poles attract.
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
        % One value per body; a scalar is applied to all bodies (default 1)
        Charge (:, 1) double

        % Dipole axis
        % Specifies the dipole orientation (direction) and the pole offset
        % from the center (magnitude). One row per body; a single row is
        % applied to all bodies.
        Axis (:, 3) double = [1 0 0]

        % Attractivity
        % Signed coupling constant of the force law (see the class help):
        % negative values make opposite poles attract (-1 is the default),
        % positive values make like poles attract
        Attractivity (1, 1) double = -1

        % Position of vector field origin
        % Can be moved while the simulation runs, e.g. to follow a body
        VectorFieldCenter (1, 3) double = [0 0 0]

        % Vector field size [x y z]
        VectorFieldSize (1, 3) double = [10 10 0]

        % Step of vector field grid
        VectorFieldStep (1, 1) double = 1

        % Number of segments of each vector
        VectorSegments (1, 1) double = 4

        % Length of vectors
        VectorLength (1, 1) double = 1
    end

    properties (Dependent)
        % Potential energy of the pole pairs (J)
        Energy (1, 1) double
    end

    methods
        function value = get.Energy(obj)
            n = numel(obj.Parents);
            pos = polePositions(obj)';
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

        function set.VectorFieldCenter(obj, value)
            obj.VectorFieldCenter = value;
            buildGrid(obj);
        end

        function set.VectorFieldSize(obj, value)
            obj.VectorFieldSize = value;
            buildGrid(obj);
        end

        function set.VectorFieldStep(obj, value)
            obj.VectorFieldStep = value;
            buildGrid(obj);
        end

        function set.VectorSegments(obj, value)
            obj.VectorSegments = value;
            buildStrip(obj);
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
            obj.Color = [0.5 0.5 0.5];

            % Process input arguments; a failure leaves no half-built object
            % attached to the parents
            try
                obj.Parents = addChild(Parents, obj);
                phx.internal.applyArguments(Options, obj);
                n = numel(Parents);
                if isempty(obj.Charge)
                    obj.Charge = ones(n, 1);
                elseif isscalar(obj.Charge)
                    obj.Charge = repmat(obj.Charge, n, 1);
                elseif numel(obj.Charge) ~= n
                    error("phx:Dipole:chargeSize", ...
                        "Charge must have one value per body (%d), or be a scalar; it has %d.", n, numel(obj.Charge));
                end
                if size(obj.Axis, 1) == 1
                    obj.Axis = repmat(obj.Axis, n, 1);
                elseif size(obj.Axis, 1) ~= n
                    error("phx:Dipole:axisSize", ...
                        "Axis must have one row per body (%d), or a single row; it has %d.", n, size(obj.Axis, 1));
                end
            catch err
                obj.abandon(Parents);
                rethrow(err);
            end

            % Create graphics objects
            buildGrid(obj);
            count = size(obj.GridPoints, 2);
            seg = obj.VectorSegments + 2;
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 0.5, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color*255 255]'), 'StripData', uint32(1:seg:(count*seg + 1)), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
            phx.Dipole.updateView({obj}, [], 0, []);
        end
    end

    methods (Access = private)
        function buildGrid(obj)
            % Sample the vector field box; called from the constructor and
            % whenever one of the grid properties really changes
            p = obj.VectorFieldSize/2;
            c = obj.VectorFieldCenter;
            s = obj.VectorFieldStep;
            x = (-p(1):s:p(1)) + c(1);
            y = (-p(2):s:p(2)) + c(2);
            z = (-p(3):s:p(3)) + c(3);
            [gx, gy, gz] = ndgrid(x, y, z);
            obj.GridPoints = [gx(:)'; gy(:)'; gz(:)'];
            buildStrip(obj);
        end

        function buildStrip(obj)
            % Re-cut the line strip into one polyline per grid point
            if ~isempty(obj.hL)
                count = size(obj.GridPoints, 2);
                seg = obj.VectorSegments + 2;
                obj.hL.StripData = uint32(1:seg:(count*seg + 1));
            end
        end

        function pos = polePositions(obj)
            % Both poles of every dipole as a 3x2n array: columns 1..n are
            % the +Axis poles, n+1..2n the -Axis poles of the same bodies.
            % One property read per body, the transform is vectorised; the
            % sums run in the order of phx.internal.transformPoint
            P = obj.Parents;
            n = numel(P);
            M = zeros(16, n);
            for a = 1:n
                M(:, a) = P{a}.Matrix(:);
            end
            ax = obj.Axis';
            d = M(1:3, :).*ax(1, :) + M(5:7, :).*ax(2, :) + M(9:11, :).*ax(3, :);
            T = M(13:15, :);
            pos = [d + T, -d + T];
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

                pos = polePositions(obj);
                Fp = phx.internal.computeDipoleForces(pos, obj.Charge, obj.Attractivity);
                [~] = phx.engine.io('apply', world, [obj.ehs obj.ehs], 'forces', Fp, [obj.Axis; -obj.Axis]', false, true);
            end
        end
        
        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                hL = obj.hL;
                grid = obj.GridPoints;
                count = size(grid, 2);
                seg = obj.VectorSegments + 2;

                pos = polePositions(obj);
                charge = [obj.Charge; -obj.Charge]';
                segLen = obj.VectorLength/obj.VectorSegments;
                XYZ = phx.internal.computeFieldArrows(grid, pos, charge, count, seg, segLen);
                XYZ(isnan(XYZ)) = 0;
                hL.VertexData = single(XYZ);
            end
        end
    end

end