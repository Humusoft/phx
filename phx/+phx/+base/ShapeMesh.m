classdef ShapeMesh
%phx.base.ShapeMesh Additional material parameters for shapes
%
%   Mix-in superclass that adds visual appearance parameters - surface
%   color, material, style and optional texture - shared by mesh-based
%   shapes. Concrete shapes (see phx.shape.*) combine it with phx.base.Shape
%   to define both their geometry and their appearance.
%
%   See also phx.base.Shape, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties (Access = protected, Transient)
        TextureData
    end

    properties
        % Surface color
        Color (1, 3) double = [NaN NaN NaN]

        % Surface material
        Material {mustBeMember(Material, ["default", "dull", "shiny", "metal", "matte"])} = "dull"

        % Surface style
        Style {mustBeMember(Style, ["smooth", "edged", "flat", "wireframe"])} = "smooth"

        % Use patch object for drawing
        ForcePatch (1, 1) logical = false
    end

    properties (Transient)
        % Texture file
        Texture (1, 1) string

        % Texture blending ratio (0-1)
        TextureBlend (1, 1) double = 1
    end

    methods
        function obj = set.Texture(obj, fileName)
            [img, ~, alpha] = imread(fileName);
            if ~isempty(alpha) && min(alpha(:)) < 255
                img(:, :, 4) = alpha;
            end
            obj.TextureData = permute(img, [3 1 2]);
        end

        function obj = colormapTexture(obj, cdata, cmap)
        %colormapTexture Generates a surface texture from scalar data.
        %
        %   obj = colormapTexture(obj, cdata) maps the scalar values in cdata
        %   to colors and stores the result as the surface texture of the
        %   shape. The default colormap is used.
        %
        %   obj = colormapTexture(obj, cdata, cmap) uses the given colormap,
        %   an Nx3 matrix of RGB rows. The values in cdata are normalized to
        %   the full range of the colormap.
        %
        % See also phx.base.ShapeMesh.Texture

            arguments
                obj 
                cdata (:, :) double
                cmap (:, 3) double = parula(10)
            end

            [a, b] = bounds(cdata(:));
            cdata = (cdata' - a)./(b - a);

            indices = round(cdata*(size(cmap, 1) - 1)) + 1;
            [M, N] = size(cdata);
            rgb_double = zeros(M, N, 3);
            rgb_double(:, :, 1) = reshape(cmap(indices, 1), M, N);
            rgb_double(:, :, 2) = reshape(cmap(indices, 2), M, N);
            rgb_double(:, :, 3) = reshape(cmap(indices, 3), M, N);
            img = uint8(round(rgb_double*255));

            obj.TextureData = permute(img, [3 1 2]);
        end

        function obj = nextColor(obj)
        %nextColor Assigns the next color from the color order to the shape.
        %
        %   obj = nextColor(obj) sets the surface color of the shape to the
        %   next color in the cyclic color order. Consecutive calls iterate
        %   through the palette, wrapping around to the beginning.
        %
        % See also phx.base.ShapeMesh.resetColorOrder, phx.base.ShapeMesh.Color

            obj.Color = phx.base.ShapeMesh.newColor;
        end
    end

    methods (Access = protected)
        function applyStyle(obj, patchObj, style)
            switch style
                case "smooth"
                    set(patchObj, 'FaceColor', obj.Color, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
                case "edged"
                    set(patchObj, 'FaceColor', obj.Color, 'EdgeColor', obj.Color/2, 'FaceLighting', 'flat');
                case "flat"
                    set(patchObj, 'FaceColor', obj.Color, 'EdgeColor', 'none', 'FaceLighting', 'flat');
                case "wireframe"
                    set(patchObj, 'FaceColor', 'none', 'EdgeColor', obj.Color, 'FaceLighting', 'flat');
            end
        end

        function applyMaterial(obj, primitive, material)
            switch material
                case "default"
                    primitive.AmbientStrength = 0.3;
                    primitive.DiffuseStrength = 0.6;
                    primitive.SpecularStrength = 0.9;
                    primitive.SpecularExponent = 10;
                    primitive.SpecularColorReflectance = 1.0; 

                case "dull"
                    primitive.AmbientStrength = 0.3;
                    primitive.DiffuseStrength = 0.8;
                    primitive.SpecularStrength = 0.0;
                    primitive.SpecularExponent = 10;
                    primitive.SpecularColorReflectance = 1.0; 
                case "shiny"
                    primitive.AmbientStrength = 0.3;
                    primitive.DiffuseStrength = 0.6;
                    primitive.SpecularStrength = 0.9;
                    primitive.SpecularExponent = 20;
                    primitive.SpecularColorReflectance = 1.0; 
                case "metal"
                    primitive.AmbientStrength = 0.3;
                    primitive.DiffuseStrength = 0.3;
                    primitive.SpecularStrength = 1.0;
                    primitive.SpecularExponent = 25;
                    primitive.SpecularColorReflectance = 0.5; 
                case "matte"
                    primitive.AmbientStrength = 0.5;
                    primitive.DiffuseStrength = 0.5;
                    primitive.SpecularStrength = 0.0;
                    primitive.SpecularExponent = 10;
                    primitive.SpecularColorReflectance = 0.0; 
            end
        end

        function go = drawMesh(obj, target, vertices, normals, faces, uvs, texture)
            if obj.ForcePatch || isempty(target.Parent) || ~isfield(target.Parent.ApplicationData, "phxAxes")
                if size(normals) == size(vertices)
                    go = patch(target, 'Faces', faces, 'Vertices', vertices, 'VertexNormals', normals);
                else
                    go = patch(target, 'Faces', faces, 'Vertices', vertices, 'FaceNormals', normals);
                end

                % Apply patch-only visual properties
                obj.applyStyle(go, obj.Style);
            else
                clr = uint8([obj.Color 1]*255)';
                vertices = single(vertices)';
                normals = single(normals)';
                faces = uint32(faces)';
                faces = faces(:)';
                if size(normals) == size(vertices)
                    normBinding = 'interpolated'; % vertex normals
                else
                    normBinding = 'discrete'; % face normals
                end
                if isempty(texture)
                    go = matlab.graphics.primitive.world.TriangleStrip('Parent', target, 'VertexIndices', faces, 'VertexData', vertices, 'NormalData', normals, 'NormalBinding', normBinding, 'ColorData', clr, 'ColorBinding', 'object', 'HitTest', 'on', 'FaceOffsetFactor', 0, 'FaceOffsetBias', 0);
                else
                    uvs = single(uvs)';
                    if size(texture, 1) == 4
                        tx = matlab.graphics.primitive.world.Texture('CData', texture, 'ColorType', 'truecoloralpha', 'SamplingFilter', 'bilinear');
                    else
                        texture(4, :, :) = uint8(255);
                        tx = matlab.graphics.primitive.world.Texture('CData', texture, 'SamplingFilter', 'bilinear');
                    end
                    go = matlab.graphics.primitive.world.TriangleStrip('Parent', target, 'VertexIndices', faces, 'VertexData', vertices, 'NormalData', normals, 'NormalBinding', normBinding, 'ColorData', uvs, 'ColorType', 'texturemapped', 'ColorBinding', 'interpolated', 'Texture', tx, 'HitTest', 'on', 'FaceOffsetFactor', 0, 'FaceOffsetBias', 0);
                end
            end

            % Apply common material properties
            obj.applyMaterial(go, obj.Material);
        end

        function textureData = getTexture(obj)
            b = obj.TextureBlend;
            if b < 1
                clr = uint8(obj.Color'*255*(1 - b));
                textureData = obj.TextureData(1:3, :, :)*b + clr;
            else
                textureData = obj.TextureData;
            end
        end
    end

    methods (Static)
        function resetColorOrder
        %resetColorOrder Resets the color order used by nextColor.
        %
        %   phx.base.ShapeMesh.resetColorOrder resets the internal counter so
        %   that the next call to nextColor starts again from the first color
        %   of the palette.
        %
        % See also phx.base.ShapeMesh.nextColor

            phx.base.ShapeMesh.newColor(1);
        end
    end

    methods (Static, Hidden)
        function clr = newColor(newValue)
            persistent counter colors

            if nargin == 1
                counter = newValue;
                colors = colororder;
                return
            end

            if isempty(counter)
                counter = 1;
                colors = colororder;
            end

            n = size(colors, 1);
            cid = mod(counter + n - 1, n) + 1;
            clr = colors(cid, :);

            counter = counter + 1;
        end
    end

end