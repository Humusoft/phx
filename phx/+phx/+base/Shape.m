classdef Shape
%phx.base.Shape Shape superclass
%
%   Abstract superclass for all shapes that define the visual and collision
%   geometry of a phx.Body. Concrete shapes (see phx.shape.*) derive from
%   this class and implement how they are drawn and how their mass and
%   moments of inertia are computed. The class also provides an optional
%   skeletal representation defined by the Skelet* properties.
%
%   See also phx.base.ShapeMesh, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties
        % Points defining the skeletal structure
        SkeletPoints (:, 3) double

        % Color of the skelet (if NaN then main color of the body will be
        % used)
        SkeletColor (1, 3) double = [NaN NaN NaN]

        % Style of the skelet
        SkeletStyle {mustBeMember(SkeletStyle, ["rays", "line", "chain", "points"])} = "rays"
    end

    methods (Abstract)
        % Draw the shape into the target (axes or hgtransform)
        drawTo(obj, target)

        % Compute mass and moments of inertia of the given shape
        [mass, inertia] = computeMass(obj)
    end

    methods (Abstract, Access = {?phx.base.Shape, ?phx.base.Object})
        % Create a new body of the given shape
        eh = createBody(obj, body)

        % Create a component of a compound body
        createComponent(obj, body)
    end

    methods (Access = protected)
        function drawSkelet(obj, target, Color)
            if ~isempty(obj.SkeletPoints)
                if ~isnan(obj.SkeletColor(1))
                    Color = obj.SkeletColor;
                end
                switch obj.SkeletStyle
                    case "rays"
                        rays = obj.SkeletPoints;
                        xyz = zeros(size(rays, 1)*3, 3);
                        xyz(1:3:end, :) = rays;
                        xyz(3:3:end, :) = NaN;        
                        line(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'LineWidth', 2.0, 'Color', Color, 'Marker', '.', 'MarkerSize', 20, 'Parent', target);
                    case "line"
                        xyz = obj.SkeletPoints;
                        line(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'LineWidth', 2.0, 'Color', Color, 'Parent', target);
                    case "chain"
                        xyz = obj.SkeletPoints;
                        line(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'LineWidth', 2.0, 'Color', Color, 'Marker', '.', 'MarkerSize', 20, 'Parent', target);
                    case "points"
                        xyz = obj.SkeletPoints;
                        line(xyz(:, 1), xyz(:, 2), xyz(:, 3), 'LineStyle', 'none', 'Color', Color, 'Marker', '.', 'MarkerSize', 20, 'Parent', target);
                end
            end
        end
    end

end