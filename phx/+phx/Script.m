classdef Script < phx.base.Object
%phx.Script Automation script
%
%   The Script object allows you to control the values of selected parameters
%   of other objects during simulation. The value profile for each parameter
%   can be defined using an interpolated curve or a time-dependent callback
%   function.
%
%   See also phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Interpolation curves {'ParamName', timeVector, valuesVector, 'interpolation_method', 'extrapolation_method'}
        Curves (:, 5) cell
    end

    methods
        function obj = Script(Parents, Curves)
            arguments
                Parents (1, :) % can be a cell array of different objects (inherited from phx.base.Object)
            end
            arguments (Repeating)
                Curves (1, :) cell
            end

            % Set default values            
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "none";
            obj.ParentAxes = [];

            % Process input arguments
            obj.Parents = addChild(Parents, obj, "phx.base.Object");

            % Add curves
            n = numel(Curves);
            obj.Curves(1:n, [4 5]) = {'linear'};
            for i = 1:n
                curve = Curves{i};
                if isnumeric(curve{2})
                    s = numel(curve);
                    obj.Curves(i, 1:s) = curve;
                    obj.Curves{i, 1} = char(obj.Curves{i, 1});
                    obj.Curves{i, 4} = char(obj.Curves{i, 4});
                    obj.Curves{i, 5} = char(obj.Curves{i, 5});
                else
                    obj.Curves{i, 1} = char(curve{1});
                    obj.Curves{i, 2} = eval(['@(t)' char(curve{2})]);
                    obj.Curves(i, 3:5) = {[]};
                end
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            valid = all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj) %#ok<MANU> function prototype
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                
                curves = obj.Curves;
                parents = obj.Parents;

                for p = 1:numel(parents)
                    parent = parents{p};
                    for j = 1:size(curves, 1)
                        if isnumeric(curves{j, 2})
                            if strcmp(curves{j, 5}, 'repeat')
                                modulo = curves{j, 2}(end);
                                stime = time - modulo*floor(time/modulo);
                                value = matlab.internal.math.interp1(curves{j, 2}, curves{j, 3}, curves{j, 4}, curves{j, 4}, stime);
                            else
                                value = matlab.internal.math.interp1(curves{j, 2}, curves{j, 3}, curves{j, 4}, curves{j, 5}, time);
                            end
                        else
                            value = curves{j, 2}(time);
                        end
                        parent.(curves{j, 1}) = value;
                    end
                end
            end
        end

        function updateView(cellObjs, dt, time, world)
        end
    end

end