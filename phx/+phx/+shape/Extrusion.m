classdef Extrusion < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Extrusion Extrusion shape
%
%   Visual appearance is based on a geometry of a profile extruded along a
%   spine curve.
%
%   Collision shape can be based on convex or concave envelope and mass
%   properties are calculated from the actual triangular mesh.
%   
%   phx.shape.Extrusion() creates a shape with default parameters.
%
%   phx.shape.Extrusion(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Revolution

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Extrusion spine
        % Matrix Nx3 where each row represents one point of the 3D spine curve
        Spine (:, 3)

        % Extrusion profile
        % Matrix Mx2 where each row represent one point of the 2D profile
        % curve. For closed profile the first and last point should be the same.
        Profile (:, 2) double

        % Profile scale
        % Can be 2-elements vector or matrix Nx2 with custom 2D scale
        % defined for each point of the spine.
        Scale (:, 2) = [1 1]

        % Modeling axis of the extrusion
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Envelope
        Envelope {mustBeMember(Envelope, ["convex", "concave"])} = "convex"
    end

    methods
        function obj = Extrusion(Options)
            arguments
                Options.?phx.shape.Extrusion
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            [V, N, F, T] = phx.internal.Geometry.extrusion(obj.Spine, obj.Scale, obj.Profile, true, true);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            [V, ~, F] = phx.internal.Geometry.extrusion(obj.Spine, obj.Scale, obj.Profile, true, true);
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