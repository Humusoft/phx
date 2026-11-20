classdef Liquid < phx.base.ParticleGroup
%phx.Liquid Liquid of particles (weakly compressible SPH)
%
%   Liquid is a body of liquid made of particles: they do not collide with
%   each other, the pressure of the liquid keeps them apart instead
%   (smoothed particle hydrodynamics, weakly compressible). The liquid
%   flows, splashes and fills containers; it collides with all bodies, and
%   a dynamic body in it floats and is pushed by it, its weight and the
%   forces of the liquid acting on each other.
%
%   phx.Liquid(axes, Shape=shape) fills a shape with the liquid, on a
%   cubic lattice of the Spacing, the shape placed at Position and
%   Orientation, and draws it into the given axes object; [] draws
%   nothing. A column of water 0.2 m wide and 0.4 m high on the floor:
%
%       water = phx.Liquid(gca, Shape={"Box", "Size", [0.2 0.4 0.4]}, ...
%           Position=[-0.5 0 0.2], Spacing=0.02, SoundSpeed=30);
%
%   phx.Liquid(axes, Positions=P) creates the liquid's particles at the
%   rows of P (Nx3) instead, best on a cubic lattice of the Spacing.
%
%   phx.Liquid(___, Name, Value, ...) sets properties values according to
%   given name-value pairs.
%
%   The liquid is explicit in time: the simulation step (interval over
%   substeps of phx.Simulation.step) must stay under
%   0.4*Spacing/(SoundSpeed + v), v the fastest flow, for example 0.25 ms
%   at a Spacing of 2 cm and a SoundSpeed of 30 m/s. The particles touch
%   the walls with a radius of Spacing/2: give the simulation an engine
%   LengthScale of the order of the Spacing (phx.engine.Settings), or the
%   default contact margin of 4 cm rounds thin walls and lets the liquid
%   into the corners of a tank.
%
%   See also phx.Particles, phx.Buoyancy, phx.Simulation,
%   phx.engine.Settings, phx.base.ParticleGroup

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters

    % Read when the simulation creates the liquid
    properties
        % Rest spacing of the particles (m): the particles of a liquid at
        % rest lie this far apart, their radius is half of it, and the
        % pressure reaches over twice it
        Spacing (1, 1) double {mustBePositive, mustBeFinite} = 0.02
    end

    % Settings that apply at any time
    properties
        % Density of the liquid at rest (kg/m^3)
        Density (1, 1) double {mustBePositive, mustBeFinite} = 1000

        % Speed of sound in the liquid (m/s): how stiff it is against
        % compression. Ten times the fastest flow keeps the density within
        % about 1 %; the step must be shorter the faster the sound.
        SoundSpeed (1, 1) double {mustBePositive, mustBeFinite} = 20

        % Viscosity coefficient (Monaghan's artificial viscosity, no unit):
        % 0.1 lets the liquid flow freely and calms its noise, 1 is a thick
        % liquid
        Viscosity (1, 1) double {mustBeNonnegative, mustBeFinite} = 0.1

        % How the liquid is drawn: "surface", its surface, smooth over the
        % particles (each redraw computes it, some 15 ms for 4000
        % particles), or "points", its particles
        Style {mustBeMember(Style, ["surface", "points"])} = "surface"
    end

    properties (Access = private)
        % The surface: a TriangleStrip in the Viewer, a patch elsewhere
        hS
        Strip = false
    end

    properties (Dependent, SetAccess = private)
        % Mass of one particle (kg), that of a cube of Spacing of the liquid
        % (0.990 Density*Spacing^3: a particle inside a cubic lattice of the
        % Spacing then reads exactly the Density)
        ParticleMass (1, 1) double

        % Total mass of the liquid (kg), Count times ParticleMass
        Mass (1, 1) double

        % Densities of the particles (Nx1, kg/m^3) at their present
        % positions; empty outside a running simulation
        Densities (:, 1) double

        % Pressures of the particles (Nx1, Pa) at their present positions;
        % empty outside a running simulation
        Pressures (:, 1) double
    end

    methods
        function obj = Liquid(ParentAxes, Options)
            arguments
                ParentAxes = gca
                Options.?phx.Liquid
            end

            % Set default values: a liquid slides along walls and does not
            % bounce off them
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.Color = [0.22 0.5 0.85];
            obj.Friction = 0;
            obj.Restitution = 0;

            % Process input arguments; a Shape is filled once they are all
            % set
            obj.ParentAxes = ParentAxes;
            obj.checkSources(Options);
            obj.Deferred = true;
            phx.internal.applyArguments(Options, obj);
            obj.Deferred = false;
            obj.refill;

            obj.createMarker;
            obj.createSurface;
            obj.applyStyle;
        end

        function set.Style(obj, value)
            obj.Style = string(value);
            obj.applyStyle;
        end

        function set.Spacing(obj, value)
            obj.Spacing = value;
            obj.refill;
        end

        function value = get.ParticleMass(obj)
            % rho0 over the poly6 kernel summed over a cubic lattice of
            % spacing d with support 2 d: 330 d^6 poly6, poly6 = 315/(64 pi (2d)^9)
            value = obj.Density*obj.Spacing^3*32768*pi/103950;
        end

        function value = get.Mass(obj)
            value = obj.Count*obj.ParticleMass;
        end

        function value = get.Densities(obj)
            value = obj.engineArray('density');
        end

        function value = get.Pressures(obj)
            value = obj.engineArray('pressure');
        end

        function set.Density(obj, value)
            obj.Density = value;
            obj.pushFluid;
        end

        function set.SoundSpeed(obj, value)
            obj.SoundSpeed = value;
            obj.pushFluid;
        end

        function set.Viscosity(obj, value)
            obj.Viscosity = value;
            obj.pushFluid;
        end
    end

    methods (Access = protected)
        function handle = createGroup(obj, world)
            handle = phxEngine('add', world, 'particles', obj.StoredPositions, ...
                obj.Spacing/2, obj.ParticleMass, false);
            phxEngine('set', world, handle, 'fluid', obj.fluidStruct);
        end

        function r = particleRadius(obj)
            r = obj.Spacing/2;
        end

        function d = fillSpacing(obj)
            d = obj.Spacing;
        end

        function redraw(obj)
            % Positions drawn outside the simulation steps, as points or as
            % the surface over them
            redraw@phx.base.ParticleGroup(obj);
            if obj.Style == "surface" && ~isempty(obj.hS) && isvalid(obj.hS)
                obj.drawSurface(obj.Positions');
            end
        end
    end

    methods (Access = private)
        function createSurface(obj)
            % In the Viewer a TriangleStrip, which redraws faster than a
            % patch, shinier than the plastic of the shapes there
            % (phx.base.Object recolors both with Color later)
            color = obj.Color;
            parent = obj.Graphics.Parent;
            obj.Strip = ~isempty(parent) && isappdata(parent, "phxAxes");
            if obj.Strip
                obj.hS = matlab.graphics.primitive.world.TriangleStrip("Parent", obj.Graphics, ...
                    "ColorData", uint8([color 1]*255)', ...
                    "ColorBinding", "object", "NormalBinding", "interpolated", "HitTest", "on", ...
                    "AmbientStrength", 0.3, "DiffuseStrength", 0.65, "SpecularStrength", 0.8, ...
                    "SpecularExponent", 25, "SpecularColorReflectance", 1);
            else
                obj.hS = patch(obj.Graphics, "Faces", zeros(0, 3), "Vertices", zeros(0, 3), ...
                    "FaceColor", color, ...
                    "EdgeColor", "none", "FaceLighting", "gouraud", "SpecularStrength", 0.8, ...
                    "SpecularExponent", 25, "BackFaceLighting", "reverselit");
            end
        end

        function applyStyle(obj)
            % The marker or the surface shown, the other hidden and empty
            if isempty(obj.hS) || ~isvalid(obj.hS)
                return
            end
            surface = obj.Style == "surface";
            obj.hM.Visible = ~surface;
            obj.hS.Visible = surface;
            if ~surface
                obj.drawSurface(zeros(3, 0));
            end
            obj.redraw;
        end

        function drawSurface(obj, P)
            % The surface over the particles at P (3xN)
            [V, F, N] = phx.internal.computeLiquidSurface(P, obj.Spacing);
            if obj.Strip
                set(obj.hS, "VertexData", single(V), "NormalData", single(N), ...
                    "VertexIndices", uint32(reshape(F', 1, [])));
            else
                set(obj.hS, "Faces", F, "Vertices", V', "VertexNormals", N');
            end
        end
        function s = fluidStruct(obj)
            s = struct("density", obj.Density, "soundspeed", obj.SoundSpeed, "viscosity", obj.Viscosity);
        end

        function pushFluid(obj)
            obj.engineSet('fluid', obj.fluidStruct);
        end

        function value = engineArray(obj, field)
            if obj.isLive
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, field)';
            else
                value = zeros(0, 1);
            end
        end
    end

    methods (Static, Access = protected)
        function updateView(cellObjs, dt, time, world) %#ok<INUSD>
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                P = phxEngine('get', world, obj.ObjectHandle, 'positions');
                if obj.Style == "surface"
                    obj.drawSurface(P);
                else
                    obj.hM.VertexData = single(P);
                end
            end
        end
    end

end
