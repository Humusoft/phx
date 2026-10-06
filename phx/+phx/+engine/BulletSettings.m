classdef BulletSettings
%phx.engine.BulletSettings Initial settings of the Bullet physics engine
%
%   s = phx.engine.BulletSettings(Name, Value, ...) creates the settings that
%   a simulation applies to its physics world when it creates it. Pass them
%   with the EngineSettings option of phx.Simulation:
%
%       sim = phx.Simulation(ax, "EngineSettings", ...
%           phx.engine.BulletSettings("Margin", 0.002));
%
%   The settings take effect once, as the world is created; changing the
%   object afterwards does not reach a running simulation.
%
%   Properties:
%     Solver        - Constraint solver of the engine
%     Iterations    - Iterations of the constraint solver per step
%     AutoActivated - Let resting bodies fall asleep
%     Margin        - Collision margin, in metres
%
%   See also phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties
        % Constraint solver: "sequentialimpulse" (the default), "nncg", or the
        % MLCP solvers "dantzig", "lemke" and "gaussseidelprojected"; the
        % choice matters in coupled systems such as joint chains and stacks
        Solver {mustBeMember(Solver, ["sequentialimpulse", "dantzig", "lemke", "gaussseidelprojected", "nncg"])} = "sequentialimpulse"

        % Iterations of the constraint solver per step; more iterations make
        % joint chains and stacks stiffer at a proportional cost, the direct
        % solvers "dantzig" and "lemke" do not use them
        Iterations (1, 1) double {mustBeInteger, mustBePositive} = 10

        % Let resting bodies fall asleep (true) or keep every body active
        % (false); a sleeping body is woken only by a direct interaction
        AutoActivated (1, 1) logical = false

        % Collision margin, in metres, added around convex hull, concave mesh
        % and cone collision shapes; boxes, spheres, cylinders and capsules
        % keep their exact size
        Margin (1, 1) double = 0.04
    end

    methods
        function obj = BulletSettings(Options)
            arguments
                Options.?phx.engine.BulletSettings
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
        end

        function apply(obj, objectHandle)
            phx.engine.io('set', objectHandle, uint64(0), 'solver', char(obj.Solver));
            phx.engine.io('set', objectHandle, uint64(0), 'numIterations', obj.Iterations);
            phx.engine.io('set', objectHandle, uint64(0), 'autoactivated', obj.AutoActivated);
            phx.engine.io('set', objectHandle, uint64(0), 'margin', obj.Margin);
        end
    end

end