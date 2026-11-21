classdef (Abstract) ParticleGroup < phx.base.Object
%phx.base.ParticleGroup Superclass of groups of particles simulated in bulk
%
%   Abstract superclass of phx.Particles and phx.Liquid: a group of equal
%   spheres that the engine holds as one object, read as one array of
%   positions and drawn as one graphics object per redraw instead of a
%   phx.Body each. This class provides what the groups share:
%
%   - the particles themselves: Positions, Velocities and Count, add,
%     remove and applyForce, boundingBox;
%   - the contact properties Friction and Restitution and the drag
%     LinearDamping;
%   - the drawing as points: Color (from phx.base.Object), MarkerSize and
%     RandomTint.
%
%   A subclass creates the group in the engine (createGroup), tells its
%   particle radius (particleRadius) and may limit the number of particles
%   (checkCount).
%
%   See also phx.Particles, phx.Liquid, phx.base.Object

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = protected)
        % Point marker that draws the particles
        hM

        % Positions and velocities (3xN) while the engine does not hold them
        StoredPositions = zeros(3, 0)
        StoredVelocities = zeros(3, 0)

        % Random number of each particle (1xN), drawn once, for its shade
        Shades = zeros(1, 0)

        % The shape that fills the group until the simulation creates it
        % (or Positions replace it), and its lattice in the shape frame for
        % the spacing it was made with
        FillShape = []
        FillLocal = zeros(0, 3)
        FillSpacing = NaN

        % True while a constructor takes its arguments: the shape is filled
        % once they are all set
        Deferred = false
    end

    % Declared first, so that given to the constructor, the positions it
    % fills are in place before the views of the particles' mass
    properties (Dependent, GetAccess = private)
        % Shape filled with particles on a cubic lattice of the group's
        % spacing, which sets Positions: a phx.shape.* object that can
        % generate a mesh of tetrahedra (Box, Sphere, Cylinder, Cone,
        % Capsule, Tube), or a cell array of its name and constructor
        % arguments. It is placed at Position and Orientation and filled
        % again when they or the spacing change, until the simulation
        % creates the group; setting Positions replaces it.
        Shape
    end

    % Pose of the Shape, read when the group is filled
    properties
        % Position of the Shape that fills the group
        Position (1, 3) double {mustBeFinite} = [0 0 0]

        % Orientation of the Shape that fills the group (3x3)
        Orientation (3, 3) double {mustBeFinite} = eye(3)
    end

    % The orientation of the Shape in other forms, views of Orientation
    properties (Dependent)
        % Axis-angle rotation [x y z angle] of the Shape
        AxisAngle (1, 4) double {mustBeFinite}

        % Euler angles [x y z] of the Shape for z->y->x order
        EulerAngles (1, 3) double {mustBeFinite}

        % Quaternion [w x y z] of the Shape
        Quaternion (1, 4) double {mustBeFinite}
    end

    % Settings that apply at any time
    properties
        % Friction coefficient; a contact combines the values of both bodies
        % as their geometric mean, sqrt(mu1*mu2)
        Friction (1, 1) double {mustBeNonnegative} = 0.5

        % Restitution (bounciness) coefficient, 0 = no bounce; a contact
        % bounces with the smaller of the two bodies' values
        Restitution (1, 1) double {mustBeNonnegative} = 0

        % Damping of the linear velocities (1/s): dv/dt = -LinearDamping*v
        LinearDamping (1, 1) double {mustBeNonnegative} = 0

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
    end

    methods
        function add(obj, positions, velocities)
        %ADD Adds particles to the group.
        %
        %   add(group, P) adds particles at the rows of P (Nx3), at rest.
        %
        %   add(group, P, V) gives them the velocities V (Nx3).

            arguments
                obj (1, 1)
                positions (:, 3) double {mustBeFinite}
                velocities (:, 3) double {mustBeFinite} = zeros(size(positions))
            end

            if size(velocities, 1) ~= size(positions, 1)
                error(obj.errorId("sizeMismatch"), "Give one velocity per particle.");
            end
            obj.checkCount(obj.Count + size(positions, 1));
            if obj.isLive
                phxEngine('add', obj.WorldHandle, obj.ObjectHandle, positions', velocities');
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
        %   remove(group, index) removes the particles of the given indices
        %   or logical mask; the others keep their order.

            arguments
                obj (1, 1)
                index {mustBeVector}
            end

            n = obj.Count;
            if islogical(index)
                if numel(index) ~= n
                    error(obj.errorId("sizeMismatch"), "The mask must have one element per particle.");
                end
                index = find(index);
            elseif ~all(index == fix(index) & index >= 1 & index <= n)
                error(obj.errorId("badIndex"), "Particle indices must be integers from 1 to %d.", n);
            end
            if obj.isLive
                phxEngine('remove', obj.WorldHandle, obj.ObjectHandle, double(index));
            else
                obj.StoredPositions(:, index) = [];
                obj.StoredVelocities(:, index) = [];
            end
            obj.Shades(index) = [];
            obj.redraw;
            obj.applyTint;
        end

        function applyForce(obj, forces)
        %applyForce Applies forces to the particles for the next engine step.
        %
        %   applyForce(group, F) applies the force F (1x3, N) to every
        %   particle of the group, each particle getting the whole F.
        %
        %   applyForce(group, F) with F of N rows (Nx3, one row per particle,
        %   in the order of Positions) applies a force of its own to each
        %   particle.
        %
        %   The forces are given in the world frame and act at the particle
        %   centres during the next engine step only, which is one substep of
        %   phx.Simulation.step; call it from a phx.Function for a force in
        %   every substep. It works only in a running simulation.

            arguments
                obj (1, 1)
                forces (:, 3) double {mustBeFinite}
            end

            if ~obj.isLive
                error(obj.errorId("notSimulated"), "Forces act only on particles in a running simulation.");
            end
            phxEngine('apply', obj.WorldHandle, obj.ObjectHandle, 'centralforce', forces', false);
        end

        function bbox = boundingBox(objs)
        %boundingBox Returns the axis-aligned bounding box of particle groups.
        %
        %   bbox = boundingBox(group) returns the axis-aligned bounding box,
        %   in world coordinates, that encloses all particles of the given
        %   groups, their radius included. The result is a 2x3 matrix:
        %   - first row:  minimum corner [xmin ymin zmin]
        %   - second row: maximum corner [xmax ymax zmax]
        %   Without any particle, the corners are Inf and -Inf.
        %
        % See also phx.Body.boundingBox, phx.base.ParticleGroup.Positions

            arguments
                objs (1, :) phx.base.ParticleGroup
            end

            bbox = [Inf Inf Inf; -Inf -Inf -Inf];
            for obj = objs
                p = obj.Positions;
                if ~isempty(p)
                    [pmin, pmax] = bounds(p, 1);
                    r = obj.particleRadius;
                    bbox = [min(bbox(1, :), pmin - r); max(bbox(2, :), pmax + r)];
                end
            end
        end

        function value = get.Positions(obj)
            if obj.isLive
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'positions')';
            else
                value = obj.StoredPositions';
            end
        end

        function set.Positions(obj, value)
            if obj.isLive
                if size(value, 1) ~= obj.Count
                    error(obj.errorId("sizeMismatch"), ...
                        "Give one position per particle; add or remove particles to change their number.");
                end
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'positions', value');
            else
                % Positions given replace the shape that filled the group
                obj.FillShape = [];
                obj.checkCount(size(value, 1));
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
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'velocities')';
            else
                value = obj.StoredVelocities';
            end
        end

        function set.Velocities(obj, value)
            if size(value, 1) ~= obj.Count
                error(obj.errorId("sizeMismatch"), "Give one velocity per particle.");
            end
            if obj.isLive
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, 'velocities', value');
            else
                obj.StoredVelocities = value';
            end
        end

        function value = get.Count(obj)
            if obj.isLive
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'count');
            else
                value = size(obj.StoredPositions, 2);
            end
        end

        function set.Shape(obj, shape)
            if obj.isLive
                error(obj.errorId("running"), "The Shape fills the group before the simulation creates it.");
            end
            if iscell(shape)
                if endsWith(lower(shape{1}), [".stl", ".obj", ".ply"])
                    shape = phx.shape.Mesh("Source", shape{:});
                else
                    shape = feval("phx.shape." + shape{1}, shape{2:end});
                end
            end
            if ~isa(shape, "phx.base.Shape")
                error(obj.errorId("badShape"), ...
                    "The Shape must be a phx.shape.* object or a cell array of its name and arguments.");
            end
            if ~isnan(shape.Density)
                warning("phx:Shape:densityMoved", "The density of a shape is deprecated, and the " + ...
                    "group has its own Density: the density of its shape is ignored.");
            end
            obj.FillShape = shape;
            obj.FillSpacing = NaN;
            obj.refill;
        end

        function set.Position(obj, value)
            obj.Position = value;
            obj.refill;
        end

        function set.Orientation(obj, value)
            obj.Orientation = value;
            obj.refill;
        end

        function value = get.AxisAngle(obj)
            value = phx.internal.Math.decompAA(obj.Orientation);
        end

        function set.AxisAngle(obj, value)
            obj.Orientation = phx.internal.Math.rotAA(value(1:3), value(4));
        end

        function value = get.EulerAngles(obj)
            value = phx.internal.Math.decomp321(obj.Orientation);
        end

        function set.EulerAngles(obj, value)
            obj.Orientation = phx.internal.Math.rot321(value);
        end

        function value = get.Quaternion(obj)
            value = phx.internal.Math.decompQ(obj.Orientation);
        end

        function set.Quaternion(obj, value)
            obj.Orientation = phx.internal.Math.rotQ(value);
        end

        function set.Friction(obj, value)
            obj.Friction = value;
            obj.engineSet('friction', value);
        end

        function set.Restitution(obj, value)
            obj.Restitution = value;
            obj.engineSet('restitution', value);
        end

        function set.LinearDamping(obj, value)
            obj.LinearDamping = value;
            obj.engineSet('damping', value);
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

    methods (Abstract, Access = protected)
        % Creates the group in the engine world from StoredPositions and
        % returns its handle; the settings of the subclass go with it. The
        % velocities and the settings of this class follow.
        handle = createGroup(obj, world)

        % Radius of the particles (m), for boundingBox
        r = particleRadius(obj)

        % Spacing of the lattice that fills the Shape (m)
        d = fillSpacing(obj)
    end

    methods (Access = protected)
        function checkCount(obj, n)
            % Fails when the group cannot hold n particles; no limit here
        end

        function checkSources(obj, Options)
            % A constructor takes the particles from a Shape or from
            % Positions, not both
            if isfield(Options, "Shape") && isfield(Options, "Positions")
                error(obj.errorId("shapeAndPositions"), "Give the particles either a Shape or their Positions, not both.");
            end
        end

        function refill(obj)
            % Fills the Shape, placed at Position and Orientation, with a
            % cubic lattice of the group's spacing; the particles start at
            % rest, in new shades. Nothing to do without a shape, inside a
            % constructor or once the simulation holds the group.
            if obj.Deferred || isempty(obj.FillShape) || obj.isLive
                return
            end
            d = obj.fillSpacing;
            if d ~= obj.FillSpacing
                obj.FillLocal = phx.base.ParticleGroup.fillLattice(obj.FillShape, d, obj.errorId("badShape"));
                obj.FillSpacing = d;
            end
            P = obj.FillLocal*obj.Orientation' + obj.Position;
            obj.checkCount(size(P, 1));
            obj.StoredPositions = P';
            obj.StoredVelocities = zeros(3, size(P, 1));
            obj.Shades = rand(1, size(P, 1));
            obj.redraw;
            obj.applyTint;
        end

        function id = errorId(obj, mnemonic)
            % Error identifier in the namespace of the concrete class,
            % phx:<Class>:<mnemonic>
            name = string(class(obj));
            id = "phx:" + extractAfter(name, "phx.") + ":" + mnemonic;
        end

        function live = isLive(obj)
            live = ~isempty(obj.ObjectHandle);
        end

        function engineSet(obj, field, value)
            if obj.isLive
                phxEngine('set', obj.WorldHandle, obj.ObjectHandle, field, value);
            end
        end

        function createMarker(obj)
            % The point marker, made once the constructor has taken its
            % arguments
            obj.hM = matlab.graphics.primitive.world.Marker('Parent', obj.Graphics, ...
                'VertexData', single(obj.StoredPositions), 'EdgeColorData', uint8([obj.Color*255 255]'), ...
                'Style', 'point', 'Size', obj.MarkerSize, ...
                'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1), 'PickableParts', 'none');
            obj.applyTint;
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
                    obj.hM.VertexData = single(phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'positions'));
                else
                    obj.hM.VertexData = single(obj.StoredPositions);
                end
            end
        end

        function valid = initObject(obj, world)
            if isempty(obj.ObjectHandle)
                % The engine takes the particles from here on; the shape
                % that filled them is done
                obj.FillShape = [];
                obj.WorldHandle = world;
                obj.ObjectHandle = obj.createGroup(world);
                if any(obj.StoredVelocities, "all")
                    phxEngine('set', world, obj.ObjectHandle, 'velocities', obj.StoredVelocities);
                end
                obj.Friction = obj.Friction;
                obj.Restitution = obj.Restitution;
                obj.LinearDamping = obj.LinearDamping;
            end
            valid = true;
        end

        function destroyObject(obj)
            if obj.isLive
                phxEngine('remove', obj.WorldHandle, obj.ObjectHandle);
                obj.ObjectHandle = [];
            end
        end
    end

    methods (Static, Access = private)
        function P = fillLattice(shape, d, errorId)
            % The points of a cubic lattice of spacing d inside the shape
            % (Nx3, shape frame). The test uses the shape's mesh of
            % tetrahedra with cells about d across (a copy of the shape:
            % they are values), whose facets follow a curved surface within
            % about d^2/(8 r). Of the lattices centred on the bounding box
            % and those shifted by d/2 along any axes, the one whose count
            % comes nearest the volume of the shape over d^3; a flat face
            % then gets its outermost points d/2 inside.
            try
                [V, T] = phx.base.ParticleGroup.linearMesh(shape);
                if isprop(shape, "ElementResolution") && isscalar(shape.ElementResolution)
                    cellSize = (phx.base.ParticleGroup.meshVolume(V, T)/size(T, 1)*6)^(1/3);
                    shape.ElementResolution = max(2, ceil(shape.ElementResolution*cellSize/d));
                    [V, T] = phx.base.ParticleGroup.linearMesh(shape);
                end
            catch err
                if err.identifier == "phx:Shape:noVolumetricMesh"
                    error(errorId, "A %s cannot fill particles yet; give their Positions instead.", class(shape));
                end
                rethrow(err);
            end
            target = phx.base.ParticleGroup.meshVolume(V, T)/d^3;
            tetra = triangulation(T, V);
            [lo, hi] = bounds(V, 1);
            P = zeros(0, 3);
            best = Inf;
            for shift = 0:7
                axes = cell(1, 3);
                for a = 1:3
                    n = floor((hi(a) - lo(a))/d + 1e-9);
                    if bitget(shift, a)
                        n = n + 1;   % shifted by d/2: one point more fits
                    end
                    axes{a} = (lo(a) + hi(a))/2 + ((0:n - 1) - (n - 1)/2)*d;
                end
                [x, y, z] = ndgrid(axes{:});
                Q = [x(:) y(:) z(:)];
                Q = Q(~isnan(pointLocation(tetra, Q)), :);
                if abs(size(Q, 1) - target) < best
                    best = abs(size(Q, 1) - target);
                    P = Q;
                end
            end
        end

        function [V, T] = linearMesh(shape)
            % The shape's mesh of tetrahedra with their corners only
            % (quadratic ones have vertices in the middles of edges too)
            [V, T] = shape.generateVolumetricMesh;
            [used, ~, corner] = unique(T(:, 1:4));
            V = V(used, :);
            T = reshape(corner, [], 4);
        end

        function v = meshVolume(V, T)
            % Volume of a mesh of tetrahedra
            a = V(T(:, 2), :) - V(T(:, 1), :);
            b = V(T(:, 3), :) - V(T(:, 1), :);
            c = V(T(:, 4), :) - V(T(:, 1), :);
            v = sum(abs(dot(a, cross(b, c, 2), 2)))/6;
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                obj.hM.VertexData = single(phxEngine('get', world, obj.ObjectHandle, 'positions'));
            end
        end
    end

end
