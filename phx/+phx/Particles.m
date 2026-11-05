classdef Particles < phx.base.Object
%phx.Particles Group of sphere particles simulated and drawn in bulk
%
%   Particles is a group of small spheres of one radius and density. They
%   collide with all bodies, and with each other, as bodies do, but the
%   group reads and draws them all at once: one array of positions and one
%   graphics object per redraw instead of a phx.Body each, so that many
%   thousands of them stay fast.
%
%   phx.Particles(axes, Positions=P) creates particles at the rows of P
%   (Nx3) and draws them into the given axes object; [] draws nothing.
%
%   phx.Particles(___, Name, Value, ...) sets properties values according
%   to given name-value pairs.
%
%   See also phx.Body, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Constant, Hidden)
        % Most particles a group may hold, for now: the engine world drops
        % contacts beyond some 10 000 to 15 000 particles touching each other
        MaxCount = 10000
    end

    properties (Access = private)
        hM

        % Positions and velocities (3xN) while the engine does not hold them
        StoredPositions = zeros(3, 0)
        StoredVelocities = zeros(3, 0)

        % Random number of each particle (1xN), drawn once, for its shade
        Shades = zeros(1, 0)
    end

    % Read when the simulation creates the particles
    properties
        % Radius of every particle (m)
        Radius (1, 1) double {mustBePositive, mustBeFinite} = 0.02

        % Density of the particles (kg/m^3)
        Density (1, 1) double {mustBePositive, mustBeFinite} = 1000

        % Rotation of the particles: false = they only slide, true = they also
        % rotate and roll
        Rotation (1, 1) logical = false
    end

    % Settings that apply at any time
    properties
        % Friction coefficient; a contact combines the values of both bodies
        Friction (1, 1) double {mustBeNonnegative} = 0.5

        % Restitution (bounciness) coefficient, 0 = no bounce
        Restitution (1, 1) double {mustBeNonnegative} = 0

        % Damping of the velocities (1/s): dv/dt = -Damping*v
        Damping (1, 1) double {mustBeNonnegative} = 0

        % Damping of the angular velocities (1/s) of rotating particles
        AngularDamping (1, 1) double {mustBeNonnegative} = 0

        % Collisions of the particles with each other
        SelfCollisions (1, 1) logical = true

        % Sleeping of particles at rest: a resting heap then costs next to
        % nothing until a body or a force touches it
        Sleeping (1, 1) logical = false

        % Size of the drawn points (points)
        MarkerSize (1, 1) double {mustBePositive} = 6

        % Random shade variation between the particles: each is drawn in
        % Color*(1 - rand*RandomTint), its random number kept for good
        RandomTint (1, 1) double {mustBeInRange(RandomTint, 0, 1)} = 0
    end

    properties (Dependent)
        % Positions of the particle centres in the global frame (Nx3)
        Positions (:, 3) double {mustBeFinite}

        % Velocities of the particles (Nx3, m/s)
        Velocities (:, 3) double {mustBeFinite}
    end

    properties (Dependent, SetAccess = private)
        % Number of particles
        Count (1, 1) double

        % Mass of one particle (kg)
        Mass (1, 1) double
    end

    methods
        function obj = Particles(ParentAxes, Options)
            arguments
                ParentAxes = gca
                Options.?phx.Particles
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.Color = phx.base.Shape.newColor;

            % Process input arguments
            obj.ParentAxes = ParentAxes;
            phx.internal.applyArguments(Options, obj);

            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, ...
                'VertexData', single(obj.StoredPositions), 'EdgeColorData', uint8([obj.Color*255 255]'), ...
                'Style', 'point', 'Size', obj.MarkerSize, ...
                'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1), 'PickableParts', 'none');
            obj.applyTint;
        end

        function add(obj, positions, velocities)
        %ADD Adds particles to the group.
        %
        %   add(particles, P) adds particles at the rows of P (Nx3), at rest.
        %
        %   add(particles, P, V) gives them the velocities V (Nx3).

            arguments
                obj (1, 1)
                positions (:, 3) double {mustBeFinite}
                velocities (:, 3) double {mustBeFinite} = zeros(size(positions))
            end

            if size(velocities, 1) ~= size(positions, 1)
                error("phx:Particles:sizeMismatch", "Give one velocity per particle.");
            end
            phx.Particles.checkCount(obj.Count + size(positions, 1));
            if obj.isLive
                phx.engine.io('add', obj.WorldHandle, obj.ObjectHandle, positions', velocities');
            else
                obj.StoredPositions = [obj.StoredPositions positions'];
                obj.StoredVelocities = [obj.StoredVelocities velocities'];
            end
            obj.Shades = [obj.Shades rand(1, size(positions, 1))];
            obj.redraw;
            obj.applyTint;
        end

        function remove(obj, index)
        %REMOVE Removes particles from the group.
        %
        %   remove(particles, index) removes the particles of the given
        %   indices or logical mask; the others keep their order.

            arguments
                obj (1, 1)
                index {mustBeVector}
            end

            n = obj.Count;
            if islogical(index)
                if numel(index) ~= n
                    error("phx:Particles:sizeMismatch", "The mask must have one element per particle.");
                end
                index = find(index);
            elseif ~all(index == fix(index) & index >= 1 & index <= n)
                error("phx:Particles:badIndex", "Particle indices must be integers from 1 to %d.", n);
            end
            if obj.isLive
                phx.engine.io('remove', obj.WorldHandle, obj.ObjectHandle, double(index));
            else
                obj.StoredPositions(:, index) = [];
                obj.StoredVelocities(:, index) = [];
            end
            obj.Shades(index) = [];
            obj.redraw;
            obj.applyTint;
        end

        function applyForces(obj, forces)
        %applyForces Applies forces to the particles for the next step.
        %
        %   applyForces(particles, F) applies the forces F (Nx3, one row per
        %   particle, or 1x3 for all) at the particle centres during the next
        %   simulation step.

            arguments
                obj (1, 1)
                forces (:, 3) double {mustBeFinite}
            end

            if ~obj.isLive
                error("phx:Particles:notSimulated", "Forces act only on particles in a running simulation.");
            end
            phx.engine.io('apply', obj.WorldHandle, obj.ObjectHandle, 'centralforces', forces', false);
        end

        function bbox = boundingBox(objs)
        %boundingBox Returns the axis-aligned bounding box of particle groups.
        %
        %   bbox = boundingBox(particles) returns the axis-aligned bounding
        %   box, in world coordinates, that encloses all particles of the
        %   given groups, their radius included. The result is a 2x3 matrix:
        %   - first row:  minimum corner [xmin ymin zmin]
        %   - second row: maximum corner [xmax ymax zmax]
        %   Without any particle, the corners are Inf and -Inf.
        %
        % See also phx.Body.boundingBox, phx.Particles.Positions

            arguments
                objs (1, :) phx.Particles
            end

            bbox = [Inf Inf Inf; -Inf -Inf -Inf];
            for obj = objs
                p = obj.Positions;
                if ~isempty(p)
                    [pmin, pmax] = bounds(p, 1);
                    bbox = [min(bbox(1, :), pmin - obj.Radius); max(bbox(2, :), pmax + obj.Radius)];
                end
            end
        end

        function value = get.Positions(obj)
            if obj.isLive
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'positions')';
            else
                value = obj.StoredPositions';
            end
        end

        function set.Positions(obj, value)
            if obj.isLive
                if size(value, 1) ~= obj.Count
                    error("phx:Particles:sizeMismatch", ...
                        "Give one position per particle; add or remove particles to change their number.");
                end
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'positions', value');
            else
                phx.Particles.checkCount(size(value, 1));
                % A new number of particles starts them at rest, in new shades
                if size(value, 1) ~= size(obj.StoredPositions, 2)
                    obj.StoredVelocities = zeros(3, size(value, 1));
                    obj.Shades = rand(1, size(value, 1));
                end
                obj.StoredPositions = value';
            end
            obj.redraw;
            obj.applyTint;
        end

        function value = get.Velocities(obj)
            if obj.isLive
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'velocities')';
            else
                value = obj.StoredVelocities';
            end
        end

        function set.Velocities(obj, value)
            if size(value, 1) ~= obj.Count
                error("phx:Particles:sizeMismatch", "Give one velocity per particle.");
            end
            if obj.isLive
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, 'velocities', value');
            else
                obj.StoredVelocities = value';
            end
        end

        function value = get.Count(obj)
            if obj.isLive
                value = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'count');
            else
                value = size(obj.StoredPositions, 2);
            end
        end

        function value = get.Mass(obj)
            value = obj.Density*4/3*pi*obj.Radius^3;
        end

        function set.Friction(obj, value)
            obj.Friction = value;
            obj.engineSet('friction', value);
        end

        function set.Restitution(obj, value)
            obj.Restitution = value;
            obj.engineSet('restitution', value);
        end

        function set.Damping(obj, value)
            obj.Damping = value;
            obj.engineSet('damping', value);
        end

        function set.AngularDamping(obj, value)
            obj.AngularDamping = value;
            obj.engineSet('angulardamping', value);
        end

        function set.SelfCollisions(obj, value)
            obj.SelfCollisions = value;
            obj.engineSet('selfcollisions', value);
        end

        function set.Sleeping(obj, value)
            obj.Sleeping = value;
            obj.engineSet('sleeping', value);
        end

        function set.MarkerSize(obj, value)
            obj.MarkerSize = value;
            if ~isempty(obj.hM)
                obj.hM.Size = value;
            end
        end

        function set.RandomTint(obj, value)
            obj.RandomTint = value;
            obj.applyTint;
        end
    end

    methods (Access = ?phx.base.Object)
        function state = stateTransfer(obj, state)
            % Positions and velocities move from the engine to the object
            % when the simulation ends, so that a new one starts from them
            arguments
                obj
                state = []
            end

            if isempty(state)
                state.Positions = obj.Positions;
                state.Velocities = obj.Velocities;
            else
                obj.StoredPositions = state.Positions';
                obj.StoredVelocities = state.Velocities';
            end
        end
    end

    methods (Access = private)
        function live = isLive(obj)
            live = ~isempty(obj.ObjectHandle);
        end

        function engineSet(obj, field, value)
            if obj.isLive
                phx.engine.io('set', obj.WorldHandle, obj.ObjectHandle, field, value);
            end
        end

        function applyTint(obj)
            % Each particle in its own shade of Color, or all in Color; the
            % marker keeps the shades so that a new Color keeps them too
            if isempty(obj.hM) || ~isvalid(obj.hM)
                return
            end
            color = obj.Color;
            if obj.RandomTint > 0
                setappdata(obj.hM, 'phxPointShade', 1 - obj.Shades*obj.RandomTint);
                obj.hM.EdgeColorBinding = 'interpolated';
            else
                for key = ["phxPointShade", "phxBaseColor"]
                    if isappdata(obj.hM, key)
                        rmappdata(obj.hM, key);
                    end
                end
                obj.hM.EdgeColorBinding = 'object';
            end
            ph = phx.internal.PrimitiveHelper(obj.hM);
            ph.Color = color;
        end

        function redraw(obj)
            % Positions drawn outside the simulation steps
            if ~isempty(obj.hM) && isvalid(obj.hM)
                if obj.isLive
                    obj.hM.VertexData = single(phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'positions'));
                else
                    obj.hM.VertexData = single(obj.StoredPositions);
                end
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            if isempty(obj.ObjectHandle)
                obj.WorldHandle = world;
                obj.ObjectHandle = phx.engine.io('add', world, 'particles', obj.StoredPositions, ...
                    obj.Radius, obj.Mass, obj.Rotation);
                if any(obj.StoredVelocities, "all")
                    phx.engine.io('set', world, obj.ObjectHandle, 'velocities', obj.StoredVelocities);
                end
                obj.Friction = obj.Friction;
                obj.Restitution = obj.Restitution;
                obj.Damping = obj.Damping;
                obj.AngularDamping = obj.AngularDamping;
                obj.SelfCollisions = obj.SelfCollisions;
                obj.Sleeping = obj.Sleeping;
            end
            valid = true;
        end

        function destroyObject(obj)
            if obj.isLive
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
                obj.hM.VertexData = single(phx.engine.io('get', world, obj.ObjectHandle, 'positions'));
            end
        end
    end

    methods (Static, Access = private)
        function checkCount(n)
            if n > phx.Particles.MaxCount
                error("phx:Particles:tooMany", "A particle group holds at most %d particles for now; %d were given.", ...
                    phx.Particles.MaxCount, n);
            end
        end
    end

end
