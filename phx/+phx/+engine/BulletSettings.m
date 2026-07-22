classdef BulletSettings
%phx.engine.BulletSettings Internal and specific settings for Bullet engine

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters

    properties
        % Engine solver
        Solver {mustBeMember(Solver, ["sequentialimpulse", "dantzig", "lemke", "gaussseidelprojected", "nncg"])} = "sequentialimpulse"
        
        % Maximal internal substep limit
        SubstepLimit (1, 1) double = 10

        % Automatic activation of body objects
        % (true = objects can be put to sleep, false = objects are always active)
        AutoActivated (1, 1) logical = false
        
        % Collision margin
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
            phx.engine.io('set', objectHandle, uint64(0), 'maxsubsteps', obj.SubstepLimit);
            phx.engine.io('set', objectHandle, uint64(0), 'autoactivated', obj.AutoActivated);
            phx.engine.io('set', objectHandle, uint64(0), 'margin', obj.Margin);
        end
    end

end