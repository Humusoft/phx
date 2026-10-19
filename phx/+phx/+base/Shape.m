classdef Shape
%phx.base.Shape Shape superclass
%
%   Abstract superclass for all shapes that define the visual and collision
%   geometry of a phx.Body. Concrete shapes (see phx.shape.*) derive from
%   this class and implement how they are drawn and how their mass and
%   moments of inertia are computed. The class provides the appearance
%   parameters shared by all shapes - surface color, material, style and
%   optional texture.
%
%   Texture coordinates (u, v) address the image like a matrix: u runs
%   along the columns from left to right and v along the rows from top to
%   bottom, so (0, 0) is the top-left corner of the image. The shapes lay
%   the image out unmirrored, seen from outside; phx.shape.Terrain aligns
%   it with its Height matrix instead (row 1 at the lowest y).
%
%   See also phx.Body

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
        Material {mustBeMember(Material, ["unlit", "matte", "plastic", "glossy", "metal"])} = "plastic"

        % Surface style
        Style {mustBeMember(Style, ["smooth", "edged", "flat", "wireframe"])} = "smooth"

        % Use patch object for drawing
        ForcePatch (1, 1) logical = false
    end

    properties (Transient)
        % Texture file
        Texture (1, 1) string

        % Texture blending ratio (0-1)
        TextureBlend (1, 1) double {mustBeInRange(TextureBlend, 0, 1)} = 1
    end

    methods (Abstract)
        % Draw the shape into the target (axes or hgtransform)
        drawTo(obj, target)

        % Compute mass and moments of inertia of the given shape
        [mass, inertia] = computeMass(obj)
    end

    methods (Abstract, Access = {?phx.base.Shape, ?phx.base.Object})
        % Create the engine body of the given body, wearing this shape; the
        % primitive is the graphics object the shape drew for it
        eh = createBody(obj, body, primitive)
    end

    methods
        function obj = set.Texture(obj, fileName)
            % An empty name removes the texture
            obj.Texture = fileName;
            if fileName == ""
                obj.TextureData = [];
                return
            end

            % Resolve texture file
            if ~isfile(fileName)
                % Redirect to default textures
                testName = fullfile(fileparts(mfilename("fullpath")), "assets", lower(fileName)+".*");
                defTexture = dir(testName);
                if ~isempty(defTexture)
                    fileName = fullfile(defTexture(1).folder, defTexture(1).name);
                else
                    error("phx:Shape:fileNotFound", "File '%s' is not an existing file or name of a default texture.", fileName);
                end
            end

            obj.TextureData = phx.base.Shape.loadTexture(fileName);
        end

        function obj = colormapTexture(obj, cdata, cmap)
        %colormapTexture Generates a surface texture from scalar data.
        %
        %   obj = colormapTexture(obj, cdata) maps the scalar values in cdata
        %   to colors and stores the result as the surface texture of the
        %   shape. The default colormap is used. The matrix is mapped like
        %   an image file: its columns run along the texture coordinate u and
        %   its rows along v, row 1 at v = 0.
        %
        %   obj = colormapTexture(obj, cdata, cmap) uses the given colormap,
        %   an Nx3 matrix of RGB rows. The values in cdata are normalized to
        %   the full range of the colormap.
        %
        % See also phx.base.Shape.Texture

            arguments
                obj
                cdata (:, :) double
                cmap (:, 3) double = parula(10)
            end

            [a, b] = bounds(cdata(:));
            cdata = (cdata - a)./(b - a);

            indices = round(cdata*(size(cmap, 1) - 1)) + 1;
            [M, N] = size(cdata);
            rgb_double = zeros(M, N, 3);
            rgb_double(:, :, 1) = reshape(cmap(indices, 1), M, N);
            rgb_double(:, :, 2) = reshape(cmap(indices, 2), M, N);
            rgb_double(:, :, 3) = reshape(cmap(indices, 3), M, N);
            img = uint8(round(rgb_double*255));
            img(:, :, 4) = 255; % opaque RGBA, see loadTexture

            obj.Texture = ""; % the generated texture has no file
            obj.TextureData = permute(img, [3 2 1]);
        end

        function obj = nextColor(obj)
        %nextColor Assigns the next color from the color order to the shape.
        %
        %   obj = nextColor(obj) sets the surface color of the shape to the
        %   next color in the cyclic color order. Consecutive calls iterate
        %   through the palette, wrapping around to the beginning.
        %
        % See also phx.base.Shape.resetColorOrder, phx.base.Shape.Color

            obj.Color = phx.base.Shape.newColor;
        end
    end

    methods (Access = protected)
        function eh = createHullBody(~, body, vertices)
            % Engine body with the convex hull of the given vertices as its
            % collision shape, for the "convex" envelopes. The vertices come
            % as a 3-by-N matrix, one column per point.
            eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
        end

        function eh = createTrimeshBody(~, body, vertices, numVertices, faces, numFaces)
            % Engine body with a concave triangle-mesh collision shape, for
            % the "concave" envelopes. The vertices come as a column
            % [x1;y1;z1;x2;...], the faces as zero-based int32 indices.
            sh = phx.engine.io('prepare', body.WorldHandle, uint64(0), 'concaveshape', vertices, numVertices, faces, numFaces);
            phx.engine.io('prepare', body.WorldHandle, sh, 'dynamictrimesh');
            phx.engine.io('prepare', body.WorldHandle, sh, 'validation');
            eh = phx.engine.io('add', body.WorldHandle, 'rigidbody', body.TypeID, sh, body.Transform, body.Mass, body.Inertia);
        end

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
                case "unlit"
                    primitive.AmbientStrength = 1.0;
                    primitive.DiffuseStrength = 0.0;
                    primitive.SpecularStrength = 0.0;
                    primitive.SpecularExponent = 10;
                    primitive.SpecularColorReflectance = 1.0;
                case "matte"
                    primitive.AmbientStrength = 0.42;
                    primitive.DiffuseStrength = 0.58;
                    primitive.SpecularStrength = 0.0;
                    primitive.SpecularExponent = 10;
                    primitive.SpecularColorReflectance = 1.0;
                case "plastic"
                    primitive.AmbientStrength = 0.32;
                    primitive.DiffuseStrength = 0.68;
                    primitive.SpecularStrength = 0.40;
                    primitive.SpecularExponent = 10;
                    primitive.SpecularColorReflectance = 1.0;
                case "glossy"
                    primitive.AmbientStrength = 0.22;
                    primitive.DiffuseStrength = 0.78;
                    primitive.SpecularStrength = 0.85;
                    primitive.SpecularExponent = 45;
                    primitive.SpecularColorReflectance = 1.0;
                case "metal"
                    primitive.AmbientStrength = 0.12;
                    primitive.DiffuseStrength = 0.88;
                    primitive.SpecularStrength = 1.0;
                    primitive.SpecularExponent = 32;
                    primitive.SpecularColorReflectance = 0.1;
            end
        end

        function go = drawMesh(obj, target, vertices, normals, faces, uvs, texture)
            if obj.ForcePatch || (~isempty(target.Parent) && ~isfield(target.Parent.ApplicationData, "phxAxes"))
                vertices = double(single(vertices)); % keep the patch geometry identical to the world-primitive path
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
                    % Texture data is RGBA; use truecoloralpha only for real
                    % transparency, because it turns antialiasing off
                    if min(texture(4, :)) < 255
                        tx = matlab.graphics.primitive.world.Texture('CData', texture, 'ColorType', 'truecoloralpha', 'SamplingFilter', 'bilinear');
                    else
                        tx = matlab.graphics.primitive.world.Texture('CData', texture, 'SamplingFilter', 'bilinear');
                    end
                    go = matlab.graphics.primitive.world.TriangleStrip('Parent', target, 'VertexIndices', faces, 'VertexData', vertices, 'NormalData', normals, 'NormalBinding', normBinding, 'ColorData', uvs, 'ColorType', 'texturemapped', 'ColorBinding', 'interpolated', 'Texture', tx, 'HitTest', 'on', 'FaceOffsetFactor', 0, 'FaceOffsetBias', 0);
                end
            end

            % Apply common material properties
            obj.applyMaterial(go, obj.Material);
        end

        function textureData = getTexture(obj)
            % Tint the color channels towards the surface color, keeping alpha
            textureData = obj.TextureData;
            b = obj.TextureBlend;
            if b < 1 && ~isempty(textureData)
                clr = uint8(obj.Color*255*(1 - b));
                % One channel at a time: a scalar first index is a strided
                % loop, while (1:3, :, :) copies 3-byte blocks (4x slower)
                for c = 1:3
                    textureData(c, :, :) = textureData(c, :, :)*b + clr(c);
                end
            end
        end
    end

    methods (Static)
        function resetColorOrder
        %resetColorOrder Resets the color order used by nextColor.
        %
        %   phx.base.Shape.resetColorOrder resets the internal counter so
        %   that the next call to nextColor starts again from the first color
        %   of the palette.
        %
        % See also phx.base.Shape.nextColor

            phx.base.Shape.newColor(1);
        end
    end

    methods (Static, Hidden)
        function data = loadTexture(fileName)
        %loadTexture Read an image file into TriangleStrip texture data.
            % Always RGBA: the alpha plane is appended before permute, where
            % it is one contiguous block, so drawMesh never has to grow it
            [img, ~, alpha] = imread(fileName);
            if isempty(alpha)
                img(:, :, 4) = 255;
            else
                img(:, :, 4) = alpha;
            end
            % Texture coordinate u runs along the image columns and v along
            % the rows, so CData is channel x columns x rows
            data = permute(img, [3 2 1]);
        end

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
