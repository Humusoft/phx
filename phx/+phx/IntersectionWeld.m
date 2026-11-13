classdef IntersectionWeld < phx.base.Object
%phx.IntersectionWeld Rigid body welded into a soft body where they overlap
%
%   IntersectionWeld joins a phx.SoftBody or a phx.FlexibleBody to a
%   phx.Body that is placed so that it overlaps the soft (or flexible)
%   body: every vertex of the soft body that lies
%   inside the body's shape when the simulation starts is welded to the
%   body. The body then holds and loads the soft body like a metal insert
%   held in rubber: joints, motors, springs and forces act on the soft body
%   through it, and the soft body acts back on the body. The two do not
%   collide with each other. One body may be welded into several soft
%   bodies and so join them.
%
%   The body should overlap at least three vertices that do not lie on one
%   line, or it cannot pass on a turn, and it should not be much lighter
%   than the vertices it holds.
%
%   phx.IntersectionWeld(softBody, body) welds body into softBody.
%
%   phx.IntersectionWeld(___, Name, Value, ...) sets properties values
%   according to given name-value pairs.
%
%   See also phx.SoftBody, phx.FlexibleBody, phx.FixedJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Dependent, SetAccess = private)
        % Number of vertices of the soft body welded to the body; 0 before
        % the simulation starts
        VertexCount (1, 1) double
    end

    methods
        function obj = IntersectionWeld(SoftBody, Body, Options)
            arguments
                SoftBody (1, 1) {mustBeA(SoftBody, ["phx.SoftBody", "phx.FlexibleBody"])}
                Body (1, 1) {mustBeA(Body, "phx.Body")}
                Options.?phx.IntersectionWeld
            end

            % The engine does all the work
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "none";

            % Process input arguments; a failure leaves no half-built object
            % attached to the parents
            try
                obj.Parents = addChild({SoftBody, Body}, obj);
                phx.internal.applyArguments(Options, obj);
            catch err
                obj.abandon({SoftBody, Body});
                rethrow(err);
            end
        end

        function value = get.VertexCount(obj)
            value = 0;
            if ~isempty(obj.ObjectHandle)
                value = phxEngine('get', obj.WorldHandle, obj.ObjectHandle, 'vertexcount');
            end
        end
    end

    methods (Access = protected)
        function valid = checkObject(obj)
            % A weld joins exactly a soft body and a body
            valid = numel(obj.Parents) == 2 && all(cellfun(@isvalid, obj.Parents));
        end

        function valid = initObject(obj, world)
            valid = obj.checkObject;
            if ~valid
                return
            end
            if isempty(obj.ObjectHandle)
                % Both parents are initialized first; a parent left out of the
                % simulation leaves the weld out too
                [softBody, body] = obj.Parents{:};
                if isempty(softBody.ObjectHandle) || isempty(body.ObjectHandle)
                    valid = false;
                    return
                end
                obj.WorldHandle = world;
                obj.ObjectHandle = phxEngine('add', world, 'attachment', softBody.ObjectHandle, body.ObjectHandle);
                if obj.VertexCount < 3
                    warning("phx:IntersectionWeld:fewVertices", ...
                        "The body overlaps %d vertices of the soft body; at least 3 are needed to hold it.", ...
                        obj.VertexCount);
                end
            end
        end

        function destroyObject(obj)
            if ~isempty(obj.ObjectHandle)
                phxEngine('remove', obj.WorldHandle, obj.ObjectHandle);
                obj.ObjectHandle = [];
            end
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
        end
    end

end
