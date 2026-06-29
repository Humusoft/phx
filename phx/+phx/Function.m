classdef Function < phx.base.Object
%phx.Function Custom computation in the simulation pipeline
%
%   The Function object lets you insert an arbitrary computation into the
%   simulation pipeline without writing a custom phx.base.Object subclass.
%   It is the lightweight middle ground between phx.Script (declarative,
%   feedforward value profiles) and a full custom class: use it to apply
%   custom force laws, feedback control, filters or any per-step logic to
%   one or more bodies.
%
%   The supplied callback is called at every simulation step (including
%   substeps) with the signature:
%
%       fcn(obj, parents, dt, time)
%
%   where OBJ is this phx.Function instance (use its UserData property to
%   keep any internal state between steps), PARENTS is the cell array of
%   the bound objects (same as obj.Parents) and DT, TIME are the substep
%   size and the current simulation time. The callback has no return value
%   - it acts by modifying the bound objects directly.
%
%   phx.Function(parents, @fcn) creates the object, binds it to the given
%   objects (array or cell array) and assigns the callback.
%
%   See also phx.Script, phx.base.Object

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Callback executed each simulation step: fcn(obj, parents, dt, time)
        Callback = []
    end

    methods
        function obj = Function(Parents, Callback)
            arguments
                Parents (1, :) % can be a cell array of different objects (inherited from phx.base.Object)
                Callback (1, 1) function_handle
            end

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "none";
            obj.ParentAxes = [];

            % Process input arguments
            obj.Parents = addChild(Parents, obj, "phx.base.Object");
            obj.Callback = Callback;
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = ~isempty(obj.Callback) && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                obj.Callback(obj, obj.Parents, dt, time);
            end
        end

        function updateView(cellObjs, dt, time, world)
        end
    end

end
