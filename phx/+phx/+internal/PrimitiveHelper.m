classdef PrimitiveHelper < handle
%phx.internal.PrimitiveHelper  Uniform appearance/geometry facade over a graphics primitive.

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
            value = [];
            switch obj.Type
                case {'patch', 'surface'}
                    value = obj.Primitive.FaceColor;
                case {'trianglestrip', 'linestrip'}
                    if ~strcmp(obj.Primitive.ColorType, 'texturemapped')
                        value = double(obj.Primitive.ColorData(1:3, 1)')/255;
                    else
                        value = [0 0 0];
                    end
            end
        end

        function set.Color(obj, value)
            switch obj.Type
                case {'patch', 'surface'}
                    obj.Primitive.FaceColor = value;
                case {'trianglestrip', 'linestrip'}
                    if ~strcmp(obj.Primitive.ColorType, 'texturemapped')
                        obj.Primitive.ColorData = uint8([value*255 255])';
                    else
                        % skip
                    end
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