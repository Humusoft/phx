classdef Logger < phx.base.Object
%phx.Logger Data logger
%
%   Logger allows you to record the progress of any property of any physical
%   object during the simulation.
% 
%   It is also possible to record the value of one property of multiple objects,
%   multiple properties of one object and even multiple properties of multiple
%   objects, but in this case all objects must contain these set of properties.
%
%   All recorded data can be read directly from the Data property, or by individual
%   channels using the getChannel method. The list of channels can be obtained
%   using the dispChannels method.
%
%   phx.Logger(parents, "Parameters", ["PropName1", "PropName2"])
%   creates a logger which will record the progress of the PropName1 and PropName2
%   property for all objects given by the array of parents.
%
%   phx.Logger(___, name, value, ...) creates a logger and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        AllData = []
        ChannSizes = []
        NextTime = 0
        Getters
    end

    properties (Dependent)
        % Time axis of recorded channels (time step may be irregular if
        % simulation step and recording frequency are not divisible)
        Time

        % Common matrix of values of all channels (use getChannel function
        % if you need to get data of selected channel)
        Data
    end

    properties
        % List of object properties that should be logged during the simulation
        Parameters (1, :) string

        % Desired recording frequency (the resulting frequency may be limited
        % by the simulation step)
        Frequency (1, 1) double = 10
    end

    methods
        function obj = Logger(Parents, Options)
            arguments
                Parents (1, :) % can be a cell array of different objects (inherited from phx.base.Object)
                Options.?phx.Logger
            end

            % Set default values            
            obj.SimulationOrder = "after";
            obj.RedrawOrder = "none";
            obj.ParentAxes = [];

            % Process input arguments
            obj.Parents = addChild(Parents, obj, "phx.base.Object");
            phx.internal.applyArguments(Options, obj);
        end

        function set.Parameters(obj, parameters)
            obj.ChannSizes = zeros(1, numel(obj.Parents)*numel(parameters));
            k = 1;
            for i = 1:numel(obj.Parents)
                for j = 1:numel(parameters)
                    obj.ChannSizes(k) = numel(obj.Parents{i}.(parameters(j)));
                    k = k + 1;
                end
            end
            obj.Parameters = parameters;
        end

        function time = get.Time(obj)
            time = obj.AllData(:, 1);
        end

        function data = get.Data(obj)
            data = obj.AllData(:, 2:end);
        end

        function data = getChannel(obj, chann)
        %getChannel Returns data of specified channel. Each channel corresponds
        % to a unique combination of one object and one parameter. The IDs
        % assigned to each channel can be found out using the dispChannels method.
        %
        %   y = getChannel(logger, channID)
        %
        % See also phx.Logger.dispChannels

            arguments
                obj
                chann (1, 1) double
            end

            c = (sum(obj.ChannSizes(1:chann-1))+1:sum(obj.ChannSizes(1:chann))) + 1;
            data = obj.AllData(:, c);
        end

        function dispChannels(obj)
        %dispChannels Displays list of channels and their IDs, which can then
        % be used in the getChannel method to read the recorded data as individual
        % channels.
        %
        %   dispChannels(logger)
        %
        % See also phx.Logger.getChannel

            disp(" ");
            disp("    Channels:");
            k = 1;
            for i = 1:numel(obj.Parents)
                for j = 1:numel(obj.Parameters)
                    disp("    "+k+" - Object"+i+"."+obj.Parameters(j));
                    k = k + 1;
                end
            end
            disp(" ");
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            parents = obj.Parents;
            parameters = obj.Parameters;
            obj.Getters = {};
            for j = 1:numel(parents)
                for k = 1:numel(parameters)
                    parent = parents{j}; %#ok<NASGU> used in eval
                    obj.Getters{end + 1} = eval("@parent."+parameters(k));
                end
            end

            valid = all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                if time >= obj.NextTime
                    getters = obj.Getters;
                    newData = time;
                    for j = 1:numel(getters)
                        data = getters{j}();
                        newData = vertcat(newData, data(:));
                    end
                    obj.AllData = vertcat(obj.AllData, newData');
                    obj.NextTime = obj.NextTime + 1/obj.Frequency;
                end
            end
        end

        function updateView(cellObjs, dt, time, world)
        end
    end

end