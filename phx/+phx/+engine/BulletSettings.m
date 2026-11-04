classdef BulletSettings
%phx.engine.BulletSettings Initial settings of the Bullet physics engine
%
%   s = phx.engine.BulletSettings(Name, Value, ...) creates the settings that
%   a simulation applies to its physics world when it creates it. Pass them
%   with the EngineSettings option of phx.Simulation:
%
%       sim = phx.Simulation(ax, "EngineSettings", ...
%           phx.engine.BulletSettings("LengthScale", 0.01));
%
%   The settings take effect once, as the world is created; changing the
%   object afterwards does not reach a running simulation.
%
%   LengthScale is the typical size of the parts that touch each other, in
%   metres, as an order of magnitude: 1 for parts of about a metre, 0.1 for
%   decimetres, 0.01 for centimetres. The collision tolerances in automatic
%   mode follow from it (the margin is 4 % of it); setting a tolerance
%   switches it to manual mode, as with the limits of MATLAB axes:
%
%       s = phx.engine.BulletSettings("LengthScale", 0.01);
%       s.Margin        % 4e-4, MarginMode "auto"
%       s.Margin = 0;   % MarginMode "manual"
%
%   Properties:
%     Solver        - Constraint solver of the engine
%     Iterations    - Iterations of the constraint solver per step
%     AutoActivated - Let resting bodies fall asleep
%     LengthScale   - Typical size of the parts in contact, as an order of magnitude, in metres
%     Margin        - Collision margin, in metres
%     MarginMode    - Margin from LengthScale ("auto") or as set ("manual")
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

        % Typical size, in metres, of the parts that touch each other, as an
        % order of magnitude: 1 for parts of about a metre, 0.1 for
        % decimetres, 0.01 for centimetres. When the parts differ much, take
        % the smaller of those whose contacts matter. The tolerances in
        % automatic mode follow from it; the default 1 gives the values for
        % metre-sized scenes.
        LengthScale (1, 1) double {mustBePositive, mustBeFinite} = 1

        % Margin from LengthScale ("auto") or the value set ("manual");
        % setting Margin switches it to "manual"
        MarginMode (1, 1) string {mustBeMember(MarginMode, ["auto", "manual"])} = "auto"
    end

    properties (Dependent)
        % Collision margin, in metres, added around convex hull, concave mesh
        % and cone collision shapes; boxes, spheres, cylinders and capsules
        % keep their exact size. In automatic mode 4 % of LengthScale.
        Margin (1, 1) double
    end

    properties (Access = private)
        ManualMargin = 0.04
    end

    methods
        function obj = BulletSettings(Options)
            arguments
                Options.?phx.engine.BulletSettings
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
        end

        function value = get.Margin(obj)
            if obj.MarginMode == "auto"
                value = 0.04*obj.LengthScale;
            else
                value = obj.ManualMargin;
            end
        end

        function obj = set.Margin(obj, value)
            obj.ManualMargin = value;
            obj.MarginMode = "manual";
        end

        function apply(obj, objectHandle)
            phx.engine.io('set', objectHandle, uint64(0), 'solver', char(obj.Solver));
            phx.engine.io('set', objectHandle, uint64(0), 'numIterations', obj.Iterations);
            phx.engine.io('set', objectHandle, uint64(0), 'autoactivated', obj.AutoActivated);
            phx.engine.io('set', objectHandle, uint64(0), 'margin', obj.Margin);
        end
    end

end
