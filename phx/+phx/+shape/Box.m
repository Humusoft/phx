classdef Box < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Box Box shape
%
%   Visual appearance is based on a simple triangular mesh.
%
%   Collision shape and mass properties are modeled as an ideal block.
%   
%   phx.shape.Box() creates a shape with default parameters.
%
%   phx.shape.Box(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Sphere

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Box size (X Y Z)
        Size (1, 3) double = [1 1 1]

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000
    end

    methods
        function obj = Box(Options)
            arguments
                Options.?phx.shape.Box
            end

            % Defaults
            obj.Style = "flat";

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            if obj.ForcePatch
                [V, N, F] = phx.internal.Geometry.quadBox(obj.Size);
                primitive = obj.drawMesh(target, V, N, F, [], []);
            else
                [V, N, F, T] = phx.internal.Geometry.triBox(obj.Size);
                primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            end
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            s = obj.Size;
            mass = obj.Density*s(1)*s(2)*s(3);
            q = s.^2;
            inertia = [q(2) + q(3), q(1) + q(3), q(1) + q(2)]*mass/12;
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            bsize = obj.Size/2; % half-extents
            eh = phx.engine.io('add', body.WorldHandle, 'box', body.TypeID, bsize, body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end
    
end