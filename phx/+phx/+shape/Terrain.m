classdef Terrain < phx.base.Shape
%phx.shape.Terrain Terrain heightfield
%
%   Visual appearance is a surface generated from the Height matrix sampled
%   over a rectangle of the given Size, optionally textured. The columns of
%   Height run along x and its rows along y, Height(1, 1) at the lowest x
%   and y. A texture is laid out the same way, image row 1 along the lowest
%   y, so an image of the same grid (or the Height matrix itself set as
%   Texture) lines up with the heights.
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

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Length and width of the terrain rectangle
        Size (1, 2) double = [10 10]

        % Height matrix of the terrain rectangle, at least 2-by-2 samples
        Height (:, :) double {mustBeFinite, mustBeHeightGrid} = zeros(10)
    end

    methods
        function obj = Terrain(Options)
            arguments
                Options.?phx.shape.Terrain
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function drawTo(obj, target)
            % The surface is centred between its lowest and highest point, as
            % the engine centres the heightfield
            c = (max(obj.Height(:)) + min(obj.Height(:)))/2;
            [V, N, F, T] = phx.internal.Geometry.terrain([obj.Size 1], obj.Height - c);

            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(~)
            % A terrain is meant to be static, where mass and inertia play no
            % part; these are fixed placeholders for the unusual dynamic case
            mass = 1000;
            inertia = [100 100 100];
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            h = obj.Height';
            nx = size(h, 1);
            ny = size(h, 2);
            % The scaling is the spacing between samples: n samples span
            % n - 1 intervals across the full Size, as the drawn surface does
            spacing = [obj.Size(1)/(nx - 1) obj.Size(2)/(ny - 1) 1];
            eh = phx.engine.io('add', body.WorldHandle, 'terrain', body.TypeID, h(:), nx, ny, min(h(:)), max(h(:)), 'z', spacing, body.Transform, body.Mass, body.Inertia);
        end
    end

end

function mustBeHeightGrid(h)
% The engine spaces the samples Size/(n - 1) apart, so it needs at least
% two of them along each side
    if size(h, 1) < 2 || size(h, 2) < 2
        error("phx:Terrain:heightSize", ...
            "Height must have at least 2-by-2 samples; it is %d-by-%d.", size(h, 1), size(h, 2));
    end
end
