classdef Revolution < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Revolution Revolution shape
%
%   Visual appearance is based on a geometry of revolved profile defined by
%   the user.
%
%   Collision shape can be based on convex or concave envelope and mass
%   properties are calculated from the actual triangular mesh.
%   
%   phx.shape.Revolution() creates a shape with default parameters.
%
%   phx.shape.Revolution(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Extrusion

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Revolution profile along modeling axis
        Profile (:, 2) double

        % Modeling axis of the revolution
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of revolution segments
        Segments (1, 1) double = 45

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Envelope
        Envelope {mustBeMember(Envelope, ["convex", "concave"])} = "convex"
    end

    methods
        function obj = Revolution(Options)
            arguments
                Options.?phx.shape.Revolution
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);

            [V, N, F, T] = phx.internal.Geometry.revolution((obj.Profile), obj.Segments, false, false);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            [V, ~, F] = phx.internal.Geometry.revolution((obj.Profile), obj.Segments, false, false);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);

            [mass, I0] = phx.internal.Geometry.meshMass(V, F, obj.Density);
            inertia = I0([1 5 9]);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            switch obj.Envelope
                case "convex"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
                case {"concave"}
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    faces = int32(ph.LinearizedFaces - 1);
                    % faces = int32(fliplr(ph.LinearizedFaces) - 1);
                    sh_id = phx.engine.io('prepare', body.WorldHandle, uint64(0), 'concaveshape', vertices(:), numel(vertices)/3, faces, numel(faces)/3);
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'dynamictrimesh');
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'validation');
                    eh = phx.engine.io('add', body.WorldHandle, 'rigidbody', body.TypeID, sh_id, body.Transform, body.Mass, body.Inertia);
            end
        end

        function createComponent(obj, body)
        end
    end

end