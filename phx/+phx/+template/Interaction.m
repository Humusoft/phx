classdef Interaction < phx.base.Object
%phx.template.Interaction Interaction template
%
%   This template demonstrates how to write a class for custom physical
%   interactions.
%
%   See also phx.Spring

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties (Access = private)
        % Private properties - for internal use only
        hL
    end

    properties (SetAccess = private)
        % Read-only properties - cannot be changed by the user
    end

    properties
        % Public properties - fully user accessible
        Force (1, 1) double = 10
    end

    methods
        function obj = Interaction(ParentA, ParentB, Options)
        % This function shows a typical constructor for interaction between
        % two bodies

            arguments
                ParentA (1, 1) {mustBeA(ParentA, "phx.Body")}
                ParentB (1, 1) {mustBeA(ParentB, "phx.Body")}
                Options.?phx.template.Interaction
            end

            % You can specify when the computational and rendering methods
            % of this interaction will be executed in relation to the
            % engine's simulation step
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";

            % The interaction usually takes the rendering target from one
            % of the parent objects
            obj.ParentAxes = ParentA.ParentAxes;

            % Using the private method of the phx.base.Object superclass to
            % correctly bind this object to its parents
            obj.Parents = addChild([ParentA ParentB], obj);

            % Process optional input arguments passed as name-value pairs
            phx.internal.applyArguments(Options, obj);

            % If the interaction is to have a graphic form, it is advisable
            % to create all necessary graphic objects here and then just
            % update their state in the rendering method
            obj.hL = line(obj.Graphics, [NaN NaN], [NaN NaN], [NaN NaN], "LineWidth", 2.0);

            % Call the rendering method to draw the initial state of the
            % interaction
            phx.template.Interaction.updateView({obj});
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            % If needed, it is possible to initialize the default state of
            % the object here (e.g. create auxiliary lookup tables) or
            % communicate with the engine

            % The function should also return if the object state is
            % considered as valid
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj)
            % If needed, it is possible to perform tasks related to object
            % destruction here
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        % This is a typical form of the computational method. The input is
        % a cell array of all objects of the same class in the scene and
        % then the actual time step and time value.
        %
        % This method is called once per simulation step and must perform
        % the calculation for all passed objects.

            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                A = obj.Parents{1};
                B = obj.Parents{2};
    
                distanceVec = B.Position - A.Position;
                directionVec = distanceVec./norm(distanceVec);
                F = directionVec*obj.Force;

                A.applyForce(F, [0 0 0], false, true);
                B.applyForce(-F, [0 0 0], false, true);
            end
        end

        function updateView(cellObjs, dt, time, world)
        % This is a typical form of the rendering method. The input is
        % a cell array of all objects of the same class in the scene and
        % then the actual time step and time value.
        %
        % This method is called once per simulation step and must perform
        % the rendering for all passed objects.

            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                pa = obj.Parents{1}.Position;
                pb = obj.Parents{2}.Position;

                lineObj = obj.hL;
                lineObj.XData = [pa(1) pb(1)];
                lineObj.YData = [pa(2) pb(2)];
                lineObj.ZData = [pa(3) pb(3)];
            end
        end
    end

end