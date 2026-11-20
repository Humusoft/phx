classdef Particles < phx.base.ParticleGroup
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
%   phx.Particles(axes, Shape=shape) fills a shape with particles
%   touching each other, on a cubic lattice of their diameter, the shape
%   placed at Position and Orientation:
%
%       grains = phx.Particles(gca, Shape={"Cylinder", "Radius", 0.1, ...
%           "Height", 0.3}, Position=[0 0 0.5], Radius=0.01);
%
%   phx.Particles(___, Name, Value, ...) sets properties values according
%   to given name-value pairs.
%
%   See also phx.Body, phx.Simulation, phx.base.ParticleGroup

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters

    properties (Constant, Hidden)
        % Most particles a group may hold, for now: the engine world drops
        % contacts beyond some 10 000 to 15 000 particles touching each other
        MaxCount = 10000
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
        % Damping of the angular velocities (1/s) of rotating particles
        AngularDamping (1, 1) double {mustBeNonnegative} = 0

        % Collisions of the particles with each other
        SelfCollisions (1, 1) logical = true

        % Sleeping of particles at rest: a resting heap then costs next to
        % nothing until a body or a force touches it
        Sleeping (1, 1) logical = false
    end

    % Views of Density, read when the simulation creates the particles.
    % Given to the constructor, they are taken after Positions and Radius
    % and override a Density given there.
    properties (Dependent)
        % Mass of one particle (kg); setting it sets the Density for the
        % present Radius
        ParticleMass (1, 1) double {mustBePositive, mustBeFinite}

        % Total mass of the particles (kg), Count times ParticleMass;
        % setting it sets the Density for the present Count and Radius,
        % so particles added or removed later change it
        Mass (1, 1) double {mustBePositive, mustBeFinite}
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

            % Process input arguments; a Shape is filled once they are all
            % set, and the views of Density need the particles and their
            % Radius, so they come last
            obj.ParentAxes = ParentAxes;
            obj.checkSources(Options);
            views = intersect(["ParticleMass" "Mass"], string(fieldnames(Options))', "stable");
            obj.Deferred = true;
            phx.internal.applyArguments(rmfield(Options, views), obj);
            obj.Deferred = false;
            obj.refill;
            for view = views
                obj.(view) = Options.(view);
            end

            obj.createMarker;
        end

        function set.Radius(obj, value)
            obj.Radius = value;
            obj.refill;
        end

        function value = get.ParticleMass(obj)
            value = obj.Density*4/3*pi*obj.Radius^3;
        end

        function set.ParticleMass(obj, value)
            obj.Density = value/(4/3*pi*obj.Radius^3);
        end

        function value = get.Mass(obj)
            value = obj.Count*obj.ParticleMass;
        end

        function set.Mass(obj, value)
            n = obj.Count;
            if n == 0
                error("phx:Particles:noParticles", "Mass needs the particles: give their Positions first.");
            end
            obj.ParticleMass = value/n;
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
    end

    methods (Access = protected)
        function handle = createGroup(obj, world)
            handle = phxEngine('add', world, 'particles', obj.StoredPositions, ...
                obj.Radius, obj.ParticleMass, obj.Rotation);
            phxEngine('set', world, handle, 'angulardamping', obj.AngularDamping);
            phxEngine('set', world, handle, 'selfcollisions', obj.SelfCollisions);
            phxEngine('set', world, handle, 'sleeping', obj.Sleeping);
        end

        function r = particleRadius(obj)
            r = obj.Radius;
        end

        function d = fillSpacing(obj)
            % A Shape is filled with particles touching each other
            d = 2*obj.Radius;
        end

        function checkCount(~, n)
            if n > phx.Particles.MaxCount
                error("phx:Particles:tooMany", "A particle group holds at most %d particles for now; %d were given.", ...
                    phx.Particles.MaxCount, n);
            end
        end
    end

end
