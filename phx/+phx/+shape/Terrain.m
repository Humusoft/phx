classdef Terrain < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.Terrain Terrain heightfield
%
%   Visual appearance is a surface generated from the Height matrix sampled
%   over a rectangle of the given Size, optionally textured.
%
%   The collision shape is a heightfield built in the engine from the same
%   data. A terrain is typically used as a static body.
%
%   phx.shape.Terrain() creates a shape with default parameters.
%
%   phx.shape.Terrain(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Box

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
    end

    properties
        % Length and width of the terrain rectangle
        Size (1, 2) double = [10 10]

        % Height matrix of the terrain rectangle 
        Height (:, :) double = zeros(10)
    end

    methods
        function obj = Terrain(Options)
            arguments
                Options.?phx.shape.Terrain
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);

            % x = linspace(-0.5, 0.5, size(obj.Height, 2))*obj.Size(1);
            % y = linspace(-0.5, 0.5, size(obj.Height, 1))*obj.Size(2);
            c = (max(obj.Height(:)) + min(obj.Height(:)))/2;
            % primitive = surf(x, y, obj.Height - c, "Parent", target);
            % obj.applyStyle(primitive, obj.Style);
            [V, N, F, T] = phx.internal.Geometry.terrain([obj.Size 1], obj.Height - c, false);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            s = obj.Size;
            mass = 1000; %obj.Density*s(1)*s(2)*s(3);
            q = s.^2;
            inertia = 100; %[q(2) + q(3), q(1) + q(3), q(1) + q(2)]*mass/12;
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            h = obj.Height';
            nx = size(h, 1);
            ny = size(h, 2);
            eh = phx.engine.io('add', body.WorldHandle, 'terrain', body.TypeID, h(:), nx, ny, min(h(:)), max(h(:)), 'z', [obj.Size(1)/nx obj.Size(2)/ny 1], body.Transform, body.Mass, body.Inertia);
        end

        function createComponent(obj, body)
        end
    end

end