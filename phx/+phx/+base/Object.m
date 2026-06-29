classdef Object < matlab.mixin.SetGetExactNames
%phx.base.Object Superclass for custom simulable objects
%
%   Each object can have multiple children and multiple parents.
%
%   This superclass also defines four abstract methods which must be
%   implemented in the custom derived subclass:
%   - initObject(obj, world) this function is called each time when the execution
%     pipelines are created or changed. Is used to create the corresponding
%     object inside the physical engine or to initialize state of the
%     derived class.
%   - destroyObject(obj) this function is called once within the main destructor
%     of the object (delete). Is used to destroy the corresponding object
%     inside the physical engine.
%   - resolveState(cellObjs, dt, time, world) this function is called at each simulation
%     step (including substeps) in the order specified by the SimulationOrder
%     parameter. 
%   - updateView(cellObjs, dt, time, world) this function is called at each rendering
%     step in the order specified by the RedrawOrder parameter.
%   If certain function is not needed the body can stay empty.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters

    % properties (SetAccess = protected, WeakHandle)
    %     % Instance of a hgtransform object
    %     Graphics matlab.graphics.primitive.Transform
    % end
    % this seems to be more problematic and not faster
    
    properties (SetAccess = protected)
        % Instance of a hgtransform object
        Graphics

        % Cell array of any phx.* objects
        Parents = {}

        % Cell array of any phx.* objects
        Children = {}
    end

    properties (SetAccess = protected, Transient)
        % Internal handle to the related object in the engine
        ObjectHandle = []

        % Internal UID for sorting purposes
        UID = 0
    end

    properties (Dependent)
        % Shortcut to the Graphics/hgtransform.Parent property
        ParentAxes
        
        % Shortcut to the Graphics/hgtransform.Visible property
        Visible (1, 1) logical
    end

    properties
        % Global object color
        Color (1, 3) double

        % Defines when the resolveState method is called relative to the main simulation step
        SimulationOrder {mustBeMember(SimulationOrder, ["before", "after", "none"])} = "before"

        % Defines when the updateView method is called relative to the main simulation step
        RedrawOrder {mustBeMember(RedrawOrder, ["before", "after", "none"])} = "after"

        % User data
        UserData = []

        % Optional name (e.g. to identify objects in Simulink)
        Name (1, 1) string
    end

    methods
        function obj = Object()
            obj.Graphics = matlab.graphics.primitive.Transform('Parent', []);
            setappdata(obj.Graphics, 'phxObject', obj);
        end

        function delete(objs)
            for obj = objs
                parentSim = obj.getParentSim;

                % Perform class-specific destroy tasks
                obj.destroyObject;

                % Remove this object from parent's children
                removeChild(obj.Parents, obj);

                % Remove this object from children's parents and automatically
                % delete this children if it has no other parents
                removeParent(obj.Children, obj);

                % Delete graphics objects
                delete(obj.Graphics);

                % Rebuild pipelines in the associated simulation
                if ~isempty(parentSim)
                    parentSim.updatePipelines;
                end
            end
        end

        function dispStructure(obj, indent)
        %dispStructure Displays the object structure.
        %
        %   dispStructure(object)
        %
        % See also phx.base.Object.propagate

            arguments
                obj
                indent (1, 1) double = 0
            end

            if obj.Name == ""
                disp(repmat('    ', 1, indent)+"• "+class(obj));
            else
                disp(repmat('    ', 1, indent)+"• "+class(obj)+" ("+obj.Name+")");
            end

            for i = 1:numel(obj.Children)
                obj.Children{i}.dispStructure(indent + 1);
            end
        end

        function propagate(objs, propName, propValue)
        %PROPAGATE Propagates value through the object structure.
        %
        %   propagate(objects, propertyName, propertyValue) sets the given
        %   value to all given objects and all their children.
        %
        % See also phx.base.Object.findBy

            arguments
                objs (1, :)
                propName 
                propValue 
            end

            for obj = objs
                obj.(propName) = propValue;
                for i = 1:numel(obj.Children)
                    obj.Children{i}.propagate(propName, propValue);
                end
            end
        end

        function cobjs = findBy(objs, propName, propValue, className)
        %findBy Finds objects by the value of the given parameter.
        %
        %   c = findBy(objects, propertyName, propertyValue) returns a
        %   cell array of all objects of any class that have the given
        %   property with the given value.
        %
        %   c = findBy(___, className) searches only through objects of
        %   the given class (default "handle")
        %
        %   The array of found objects is sorted by hierarchy level
        %   (parents before children).
        %
        % See also phx.base.Object.propagate

            arguments
                objs (1, :)
                propName (1, 1) string
                propValue 
                className (1, 1) string = "handle"
            end

            cobjs = sortedBranchObjects(objs);
            id = false(size(cobjs));
            for i = 1:numel(cobjs)
                obj = cobjs{i};
                if ~isa(obj, className) || ~isprop(obj, propName) || ~isequal(obj.(propName), propValue)
                    id(i) = true;
                end
            end
            cobjs(id) = [];
        end

        function sim = getParentSim(obj)
        %getParentSim Returns the simulation the object belongs to.
        %
        %   sim = getParentSim(obj) walks up the parent hierarchy of the
        %   object and returns the phx.Simulation at its root. If the object
        %   is not part of any simulation, an empty value is returned.
        %
        % See also phx.Simulation

            sim = obj;
            while ~isempty(sim.Parents)
                if isvalid(sim.Parents{1})
                    sim = sim.Parents{1};
                end
            end
            if ~isa(sim, 'phx.Simulation') || obj == sim
                sim = [];
            end
        end

        function set.ParentAxes(obj, parent)
            obj.Graphics.Parent = parent;
        end

        function parent = get.ParentAxes(obj)
            parent = obj.Graphics.Parent;
        end

        function set.Visible(obj, visible)
            obj.Graphics.Visible = visible;
        end

        function visible = get.Visible(obj)
            if isvalid(obj.Graphics)
                visible = obj.Graphics.Visible;
            else
                visible = false;
            end
        end

        function set.Color(obj, color)
            obj.Color = color;
            for i = 1:numel(obj.Graphics.Children)
                ph = phx.internal.PrimitiveHelper(obj.Graphics.Children(i));
                ph.Color = color;
            end
        end

        function color = get.Color(obj)
            n = numel(obj.Graphics.Children);
            if n > 0
                for i = 1:n
                    ph = phx.internal.PrimitiveHelper(obj.Graphics.Children(i));
                    color = ph.Color;
                    if ~isempty(color)
                        return
                    end
                end
            else
                color = obj.Color;
            end
        end
    end

    methods (Access = protected)
        function objs = addParent(objs, parent, validationClass)
            arguments
                objs 
                parent
                validationClass (1, 1) string = ""
            end

            if ~iscell(objs)
                objs = num2cell(objs);
            end
            for i = 1:numel(objs)
                obj = objs{i};
                if validationClass ~= ""
                    mustBeA(obj, validationClass);
                end
                id = cellfun(@(c) c == parent, obj.Parents, "UniformOutput", 1);
                if any(id)
                    warning("phx:Object:duplicateParent", "This parent is already associated with the object.");
                else
                    obj.Parents{end + 1} = parent;
                end
            end
        end

        function objs = addChild(objs, child, validationClass)
            arguments
                objs 
                child 
                validationClass (1, 1) string = ""
            end

            if ~iscell(objs)
                objs = num2cell(objs);
            end
            for i = 1:numel(objs)
                obj = objs{i};
                if validationClass ~= ""
                    mustBeA(obj, validationClass);
                end
                id = cellfun(@(c) c == child, obj.Children, "UniformOutput", 1);
                if any(id)
                    warning("phx:Object:duplicateChild", "This child is already associated with the object.");
                else
                    obj.Children{end + 1} = child;
                end
            end
        end

        function removeChild(objs, child)
            for i = 1:numel(objs)
                obj = objs{i};
                id = cellfun(@(c) c == child, obj.Children, "UniformOutput", 1);
                obj.Children(id) = [];
            end
        end

        function removeParent(objs, parent)
            for i = 1:numel(objs)
                obj = objs{i};
                id = cellfun(@(c) c == parent, obj.Parents, "UniformOutput", 1);
                obj.Parents(id) = [];
            end
        end

        function cellObjs = sortedBranchObjects(objs)
            cellObjs = num2cell(objs);
            lev1 = cellObjs;
            while ~isempty(lev1)
                lev2 = [];
                for c = 1:numel(lev1)
                    lev2 = [lev2 lev1{c}.Children]; %#ok<AGROW> unknown size
                end
                cellObjs = [cellObjs lev2]; %#ok<AGROW> unknown size
                lev1 = lev2;
            end

            uid = zeros(size(cellObjs));
            for i = 1:numel(uid)
                cellObjs{i}.UID = i;
            end
            for i = 1:numel(uid)
                uid(i) = cellObjs{i}.UID;
            end
            [~, id] = unique(uid, "stable");
            cellObjs = cellObjs(id);
        end
    end

    methods (Abstract, Access = protected)
        % Initialize object or it's engine counterpart and return the state
        valid = initObject(obj, world)

        % Destroy object or it's engine counterpart
        destroyObject(obj)
    end

    methods (Abstract, Static, Access = protected)
        % Resolve the simulation step
        resolveState(cellObjs, dt, time, world)

        % Update graphic objects
        updateView(cellObjs, dt, time, world)
    end

end