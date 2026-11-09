classdef PrimitiveHelper < handle
%phx.internal.PrimitiveHelper Uniform appearance/geometry facade over a graphics primitive

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties (Access = private)
        Primitive
        Type
    end

    properties (Dependent)
        Color (1, 3) double
        AmbientStrength (1, 1) double
        DiffuseStrength (1, 1) double
        SpecularStrength (1, 1) double
        SpecularExponent (1, 1) double
    end

    properties (Dependent, SetAccess = private)
        Vertices (:, 3) double
        LinearizedFaces (1, :) double
    end

    methods
        function obj = PrimitiveHelper(Primitive)
            obj.Primitive = Primitive;
            str = strsplit(class(Primitive), '.');
            obj.Type = lower(str{end});
        end

        function value = get.Color(obj)
            % A primitive drawn in other colors than its own (a colormap,
            % the shaded points of phx.Particles) keeps its color aside
            value = getappdata(obj.Primitive, 'phxBaseColor');
            if ~isempty(value)
                return
            end
            switch obj.Type
                case {'patch', 'surface'}
                    value = obj.Primitive.FaceColor;
                case {'trianglestrip', 'linestrip'}
                    if ~strcmp(obj.Primitive.ColorType, 'texturemapped')
                        value = double(obj.Primitive.ColorData(1:3, 1)')/255;
                    else
                        value = [0 0 0];
                    end
                case 'marker'
                    value = double(obj.Primitive.EdgeColorData(1:3, 1)')/255;
            end
        end

        function set.Color(obj, value)
            if isappdata(obj.Primitive, 'phxBaseColor') && ~isappdata(obj.Primitive, 'phxPointShade')
                % Drawn by a colormap: the color waits for releaseColor
                setappdata(obj.Primitive, 'phxBaseColor', value);
                return
            end
            switch obj.Type
                case {'patch', 'surface'}
                    obj.Primitive.FaceColor = value;
                case {'trianglestrip', 'linestrip'}
                    switch obj.Primitive.ColorType
                        case 'texturemapped'
                            % skip
                        case 'truecoloralpha'
                            % keep the transparency, change the color only
                            alpha = obj.Primitive.ColorData(4, 1);
                            obj.Primitive.ColorData = [uint8(value'*255); alpha];
                        otherwise
                            obj.Primitive.ColorData = uint8([value*255 255])';
                    end
                case 'marker'
                    shade = getappdata(obj.Primitive, 'phxPointShade');
                    if isempty(shade)
                        obj.Primitive.EdgeColorData = uint8([value*255 255])';
                    else
                        setappdata(obj.Primitive, 'phxBaseColor', value);
                        obj.Primitive.EdgeColorData = [uint8(value'*shade*255); repmat(uint8(255), 1, numel(shade))];
                    end
            end
        end

        function holdColor(obj)
        %holdColor Keeps the color aside while a colormap draws the primitive.
        %
        %   Until releaseColor, Color reads and writes the kept color and
        %   leaves the drawn colors to the colormap.
            if ~isappdata(obj.Primitive, 'phxBaseColor')
                setappdata(obj.Primitive, 'phxBaseColor', obj.Color);
            end
        end

        function releaseColor(obj)
        %releaseColor Draws the primitive in the color kept by holdColor again.
            if isappdata(obj.Primitive, 'phxBaseColor')
                color = getappdata(obj.Primitive, 'phxBaseColor');
                rmappdata(obj.Primitive, 'phxBaseColor');
                obj.Color = color;
            end
        end

        function value = get.AmbientStrength(obj)
             value = obj.Primitive.AmbientStrength;
        end

        function set.AmbientStrength(obj, value)
            obj.Primitive.AmbientStrength = value;
        end

        function value = get.DiffuseStrength(obj)
             value = obj.Primitive.DiffuseStrength;
        end

        function set.DiffuseStrength(obj, value)
            obj.Primitive.DiffuseStrength = value;
        end

        function value = get.SpecularStrength(obj)
             value = obj.Primitive.SpecularStrength;
        end

        function set.SpecularStrength(obj, value)
            obj.Primitive.SpecularStrength = value;
        end

        function value = get.SpecularExponent(obj)
             value = obj.Primitive.SpecularExponent;
        end

        function set.SpecularExponent(obj, value)
            obj.Primitive.SpecularExponent = value;
        end

        function value = get.Vertices(obj)
            value = [];
            switch obj.Type
                case 'patch'
                    value = obj.Primitive.Vertices;
                case 'surface'
                    [~, value, ~] = surf2patch(obj.Primitive);
                case {'trianglestrip', 'linestrip'}
                    value = double(obj.Primitive.VertexData');
            end
        end

        function value = get.LinearizedFaces(obj)
            value = [];
            switch obj.Type
                case 'patch'
                    value = obj.Primitive.Faces';
                    value = value(:)';
                case 'surface'
                    [value, ~, ~] = surf2patch(obj.Primitive);
                case {'trianglestrip', 'linestrip'}
                    value = double(obj.Primitive.VertexIndices);
            end
        end
    end

end