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
%   TextureTransform rotates, mirrors or recolors the image before it is
%   blended with Color and laid out. The built-in textures are the default
%   outputs of static functions such as phx.base.Shape.wood, which can be
%   called with other parameters.
%
%   See also phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties (Access = protected, Transient)
        TextureData
    end

    properties (Constant, Hidden)
        % Names of the built-in textures, each a static generator method
        BuiltinTextures = ["arrows" "cells" "checker" "circles" "scratches" ...
            "stone" "stripes" "tiles" "wood" "xor"]
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
        % Texture: an image file, the name of a built-in texture, or an
        % image matrix - MxNx3 RGB (uint8, or 0-1) or MxN gray levels
        % mapped through TextureColormap: uint8 as they are (0-255), other
        % classes scaled to their full range; "" removes the texture
        Texture = ""

        % Colormap of an MxN texture matrix (Nx3), empty for grayscale;
        % uint8 0-255 and the range of other data span the whole colormap
        TextureColormap (:, 3) double {mustBeInRange(TextureColormap, 0, 1)} = zeros(0, 3)

        % Texture blending ratio (0-1)
        TextureBlend (1, 1) double {mustBeInRange(TextureBlend, 0, 1)} = 1

        % Texture image operations, applied from left to right before
        % TextureBlend: "rot90", "rot180" and "rot270" rotate the image
        % counterclockwise like rot90, "fliplr" and "flipud" mirror it,
        % "grayscale" keeps its luma (as rgb2gray), "invert" makes a
        % negative of the colors and "opaque" drops the alpha channel
        TextureTransform (1, :) string {mustBeMember(TextureTransform, ["rot90", "rot180", "rot270", "fliplr", "flipud", "grayscale", "invert", "opaque"])} = strings(1, 0)
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
        function obj = set.Texture(obj, texture)
            if isnumeric(texture) || islogical(texture)
                obj.TextureData = phx.base.Shape.matrixTexture(texture, obj.TextureColormap);
                obj.Texture = texture;
                return
            end

            % An empty name removes the texture
            mustBeTextScalar(texture);
            fileName = string(texture);
            obj.Texture = fileName;
            if fileName == ""
                obj.TextureData = [];
                return
            end

            name = lower(fileName);
            if name == "metal" && ~isfile(fileName)
                % Renamed built-in texture, accepted for a transition period
                warning("phx:Shape:renamedTexture", "The built-in texture ""metal"" was renamed to ""scratches"". The name ""metal"" will be removed in a future release.");
                name = "scratches";
            end

            if isfile(fileName)
                obj.TextureData = phx.base.Shape.loadTexture(fileName);
            elseif any(name == phx.base.Shape.BuiltinTextures)
                % A built-in texture is the default output of its generator
                obj.TextureData = phx.base.Shape.matrixTexture(feval("phx.base.Shape." + name), []);
            else
                error("phx:Shape:fileNotFound", "File '%s' is not an existing file or the name of a built-in texture (%s).", ...
                    fileName, join(phx.base.Shape.BuiltinTextures, ", "));
            end
        end

        function obj = set.TextureColormap(obj, cmap)
            % Remap a scalar texture matrix, whichever of the two was set first
            obj.TextureColormap = cmap;
            if ~isstring(obj.Texture) && ismatrix(obj.Texture)
                obj.TextureData = phx.base.Shape.matrixTexture(obj.Texture, cmap);
            end
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
            textureData = obj.TextureData;
            if isempty(textureData)
                return
            end

            % Image operations on data laid out channel x columns x rows
            % (see loadTexture); the color operations leave alpha alone
            for op = obj.TextureTransform
                switch op
                    case "rot90"
                        textureData = flip(permute(textureData, [1 3 2]), 3);
                    case "rot180"
                        textureData = flip(flip(textureData, 2), 3);
                    case "rot270"
                        textureData = flip(permute(textureData, [1 3 2]), 2);
                    case "fliplr"
                        textureData = flip(textureData, 2);
                    case "flipud"
                        textureData = flip(textureData, 3);
                    case "grayscale"
                        % ITU-R BT.601 luma, the weights of rgb2gray
                        luma = 0.298936021293775*double(textureData(1, :, :)) + ...
                            0.587043074451121*double(textureData(2, :, :)) + ...
                            0.114020904255103*double(textureData(3, :, :));
                        luma = uint8(luma);
                        for c = 1:3
                            textureData(c, :, :) = luma;
                        end
                    case "invert"
                        for c = 1:3
                            textureData(c, :, :) = 255 - textureData(c, :, :);
                        end
                    case "opaque"
                        textureData(4, :, :) = 255;
                end
            end

            % Tint the color channels towards the surface color, keeping alpha
            b = obj.TextureBlend;
            if b < 1
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

        function pattern = xor(resolution)
        %xor Generates a pattern of nested squares.
        %
        %   pattern = phx.base.Shape.xor() returns a 512-by-512 uint8 grayscale
        %   image of squares nested at all scales, like a recursive checkerboard
        %   (the bitwise XOR of the column and row indices). It is the built-in
        %   texture "xor".
        %
        %   pattern = phx.base.Shape.xor(resolution) sets the image size as
        %   [width height] in pixels.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.checker

            arguments
                resolution (1, 2) double = [512 512]
            end

            pattern = bitxor(uint16(0:resolution(1)-1), uint16((0:resolution(2)-1)'));
            pattern = uint8(double(pattern)*(255/double(max(pattern, [], "all"))));
        end

        function pattern = checker(resolution, count)
        %checker Generates a checkerboard.
        %
        %   pattern = phx.base.Shape.checker() returns a 512-by-512 uint8 image of
        %   a black and white checkerboard of 2-by-2 squares. It is the built-in
        %   texture "checker".
        %
        %   pattern = phx.base.Shape.checker(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.checker(resolution, count) sets the number of
        %   squares as [across down].
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.tiles

            arguments
                resolution (1, 2) double = [512 512]
                count (1, 2) double = [2 2]
            end

            cx = mod(floor(count(1)*(0:resolution(1)-1)/resolution(1)), 2);
            cy = mod(floor(count(2)*(0:resolution(2)-1)'/resolution(2)), 2);
            pattern = uint8(255*xor(cx, cy));
        end

        function pattern = stripes(resolution, width)
        %stripes Generates vertical stripes.
        %
        %   pattern = phx.base.Shape.stripes() returns a 512-by-512 uint8 image of
        %   black and white vertical stripes, each 128 pixels wide. It is the
        %   built-in texture "stripes".
        %
        %   pattern = phx.base.Shape.stripes(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.stripes(resolution, width) sets the width of
        %   one stripe in pixels.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.arrows

            arguments
                resolution (1, 2) double = [512 512]
                width (1, 1) double = 128
            end

            pattern = repmat(uint8(255*mod(floor((0:resolution(1)-1)/width), 2)), resolution(2), 1);
        end

        function pattern = circles(resolution, width)
        %circles Generates concentric rings.
        %
        %   pattern = phx.base.Shape.circles() returns a 512-by-512 uint8 image of
        %   black and white concentric rings with smooth edges, centered in the
        %   image. It is the built-in texture "circles".
        %
        %   pattern = phx.base.Shape.circles(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.circles(resolution, width) sets the width of
        %   one ring in pixels (default 32).
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.wood

            arguments
                resolution (1, 2) double = [512 512]
                width (1, 1) double = 32
            end

            rx = resolution(1);
            ry = resolution(2);
            r = hypot((0:rx-1) - rx/2 + 0.5, (0:ry-1)' - ry/2 + 0.5);
            % Distance to the nearest ring edge, smoothed over one pixel
            e = width/2 - abs(mod(r, 2*width) - width);
            pattern = uint8(255*min(max(0.5 + e, 0), 1));
        end

        function pattern = stone(resolution, smoothness, seed)
        %stone Generates a cloudy stone-like noise.
        %
        %   pattern = phx.base.Shape.stone() returns a 512-by-512 uint8 image of
        %   soft cloudy noise that tiles seamlessly in both directions. It is the
        %   built-in texture "stone".
        %
        %   pattern = phx.base.Shape.stone(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.stone(resolution, smoothness) sets how smooth
        %   the noise is: higher values give larger, softer blotches, lower values
        %   a finer grain (default 1.8).
        %
        %   pattern = phx.base.Shape.stone(resolution, smoothness, seed) selects
        %   another variant of the noise (default 0). The global random number
        %   generator is left untouched.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.scratches

            arguments
                resolution (1, 2) double = [512 512]
                smoothness (1, 1) double = 1.8
                seed (1, 1) double = 0
            end

            s = rng(seed);
            noise = randn(resolution(2), resolution(1));
            rng(s);
            pattern = uint8(rescale(phx.base.Shape.fractalNoise(noise, smoothness), 0, 255));
        end

        function pattern = tiles(resolution, count, base, seed)
        %tiles Generates square tiles.
        %
        %   pattern = phx.base.Shape.tiles() returns a 512-by-512 uint8 image of
        %   8-by-8 light square tiles, each in a slightly different shade and
        %   with a fine grain, separated by lighter joints. It is the built-in
        %   texture "tiles".
        %
        %   pattern = phx.base.Shape.tiles(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.tiles(resolution, count) sets the number of
        %   tiles as [across down]. The image size must be divisible by it.
        %
        %   pattern = phx.base.Shape.tiles(resolution, count, base) sets the
        %   lightness of the joints from 0 to 1 (default 0.8); the tiles are a
        %   little darker.
        %
        %   pattern = phx.base.Shape.tiles(resolution, count, base, seed) selects
        %   another variant of the shades and grain (default 0). The global
        %   random number generator is left untouched.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.checker

            arguments
                resolution (1, 2) double = [512 512]
                count (1, 2) double = [8 8]
                base (1, 1) double = 0.8
                seed (1, 1) double = 0
            end

            tw = resolution(1)/count(1);
            th = resolution(2)/count(2);
            tile = zeros(th, tw) + base;
            s = rng(seed);
            tile(2:end-1, 2:end-1) = base - 0.2 + rand(th-2, tw-2)*0.1;
            m = rand(count(2), count(1));
            rng(s);

            % Light tiles with lighter joints, each tile in its own shade
            pattern = zeros(resolution(2), resolution(1), "uint8");
            for i = 1:count(2)
                for j = 1:count(1)
                    r = (i - 1)*th + 1;
                    c = (j - 1)*tw + 1;
                    pattern(r:r+th-1, c:c+tw-1) = uint8(255*(tile + 0.2*m(i, j)));
                end
            end
        end

        function pattern = scratches(resolution, seed)
        %scratches Generates fine crossed scratches.
        %
        %   pattern = phx.base.Shape.scratches() returns a 512-by-512 uint8 image
        %   of fine scratches running along both image axes, which tiles
        %   seamlessly in both directions. It is the built-in texture
        %   "scratches".
        %
        %   pattern = phx.base.Shape.scratches(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.scratches(resolution, seed) selects another
        %   variant of the scratches (default 0). The global random number
        %   generator is left untouched.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.stone

            arguments
                resolution (1, 2) double = [512 512]
                seed (1, 1) double = 0
            end

            rx = resolution(1);
            ry = resolution(2);
            s = rng(seed);
            noise = rand(ry, rx);
            rng(s);

            % Gaussian blur (sigma 20, 63 px) along x and along y over
            % circularly padded noise, so the streaks continue across the seams
            k = 31;
            g = exp(-(-k:k).^2/(2*20^2));
            g = g/sum(g);
            px = conv2([noise(:, end-k+1:end), noise, noise(:, 1:k)], g, "valid");
            py = conv2([noise(end-k+1:end, :); noise; noise(1:k, :)], g', "valid");
            pattern = uint8(rescale(px + py, 0, 255));
        end

        function pattern = arrows(resolution, width)
        %arrows Generates chevrons pointing right.
        %
        %   pattern = phx.base.Shape.arrows() returns a 512-by-512 uint8 image of
        %   black and white chevrons pointing to the right, along the image rows.
        %   It is the built-in texture "arrows".
        %
        %   pattern = phx.base.Shape.arrows(resolution) sets the image size as
        %   [width height] in pixels. The height must be even.
        %
        %   pattern = phx.base.Shape.arrows(resolution, width) sets the width of
        %   one chevron band in pixels (default 64).
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.TextureTransform

            arguments
                resolution (1, 2) double = [512 512]
                width (1, 1) double = 64
            end

            pattern = uint8(255*mod(floor(((0:resolution(1)-1) + (0:resolution(2)/2-1)')/width), 2));
            pattern = [flipud(pattern); pattern];
        end

        function pattern = cells(resolution, density, seed)
        %cells Generates rounded cells.
        %
        %   pattern = phx.base.Shape.cells() returns a 512-by-512 uint8 image of
        %   40 irregular cells, light in their centers and darkening towards the
        %   borders with their neighbors. It is the built-in texture "cells".
        %
        %   pattern = phx.base.Shape.cells(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.cells(resolution, density) sets the number of
        %   cells.
        %
        %   pattern = phx.base.Shape.cells(resolution, density, seed) selects
        %   another layout of the cells (default 0). The global random number
        %   generator is left untouched.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.stone

            arguments
                resolution (1, 2) double = [512 512]
                density (1, 1) double = 40
                seed (1, 1) double = 0
            end

            rx = resolution(1);
            ry = resolution(2);
            s = rng(seed);
            P = rand(density, 2).*[rx ry];
            rng(s);

            % Squared distance to the nearest point, one point at a time
            x = 0:rx-1;
            y = (0:ry-1)';
            d2 = inf(ry, rx);
            for i = 1:density
                d2 = min(d2, (x - P(i, 1)).^2 + (y - P(i, 2)).^2);
            end
            pattern = uint8(rescale(-sqrt(d2), 0, 255));
        end

        function pattern = wood(resolution, rings, seed)
        %wood Generates a wood grain.
        %
        %   pattern = phx.base.Shape.wood() returns a 512-by-512 uint8 image of
        %   wood grain: soft growth rings curving around the pith off one corner
        %   and bending around a knot near the opposite one, with slightly
        %   darker edges that outline the faces of a box. It is the built-in
        %   texture "wood".
        %
        %   pattern = phx.base.Shape.wood(resolution) sets the image size as
        %   [width height] in pixels.
        %
        %   pattern = phx.base.Shape.wood(resolution, rings) sets the number of
        %   growth rings (default 7).
        %
        %   pattern = phx.base.Shape.wood(resolution, rings, seed) selects another
        %   variant, with the rings and the knot slightly moved (default 0). The
        %   global random number generator is left untouched.
        %
        % See also phx.base.Shape.Texture, phx.base.Shape.stone

            arguments
                resolution (1, 2) double = [512 512]
                rings (1, 1) double = 7
                seed (1, 1) double = 0
            end

            rx = resolution(1);
            ry = resolution(2);
            s = rng(seed);
            a = rand(1, 4);
            noise = randn(ry, rx);
            rng(s);
            n = phx.base.Shape.fractalNoise(noise, 2.5);
            n = n/max(abs(n), [], "all");

            % Coordinates scaled by the longer side, the image spans w x h
            sz = max(rx, ry);
            w = rx/sz;
            h = ry/sz;
            u = ((0:rx-1) + 0.5)/sz;
            v = ((0:ry-1)' + 0.5)/sz;

            % Rings around two piths past opposite corners (Cassini ovals),
            % meeting in a saddle like the grain around a knot
            c1 = [-0.35 - 0.15*a(1), -0.15 - 0.15*a(2)];
            c2 = [w + 0.05 + 0.08*a(3), h - 0.15 + 0.08*a(4)];
            r = hypot(u - c1(1), v - c1(2)).*sqrt(sqrt(hypot(u - c2(1), v - c2(2)) + 0.02));
            r = rescale(r) + 0.02*n;

            % Broad light early wood, narrow darker late wood, lighter towards
            % the pith and darker edges that frame each face of a box
            bands = (0.5 + 0.5*cos(2*pi*rings*r)).^0.6;
            frame = 1 - 0.35*max(exp(-min(u, w - u)/0.012), exp(-min(v, h - v)/0.012));
            pattern = uint8(rescale((0.3 + 0.25*bands + 0.45*(1 - r)).*frame, 0, 255));
        end
    end

    methods (Static, Hidden)
        function noise = fractalNoise(noise, factor)
        %fractalNoise Shape white noise into periodic 1/f^factor noise.
            [ry, rx] = size(noise);
            sz = max(rx, ry);
            fx = min(0:rx-1, rx:-1:1)*sz/rx;
            fy = min((0:ry-1)', (ry:-1:1)')*sz/ry;
            f2 = fx.^2 + fy.^2;
            f2(1) = 1;
            noise = ifft2(fft2(noise)./f2.^(factor/2), "symmetric");
        end

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

        function data = matrixTexture(texture, cmap)
        %matrixTexture Convert an image matrix into TriangleStrip texture data.
            if ismatrix(texture)
                % uint8 data are gray levels 0-255, other data span the full
                % range of gray levels or colormap
                if isempty(cmap)
                    if ~isa(texture, "uint8")
                        texture = uint8(255*rescale(texture));
                    end
                    img = repmat(texture, 1, 1, 3);
                else
                    if isa(texture, "uint8")
                        v = double(texture)/255;
                    else
                        v = rescale(texture);
                    end
                    img = uint8(255*ind2rgb(round(v*(size(cmap, 1) - 1)) + 1, cmap));
                end
            elseif ndims(texture) == 3 && size(texture, 3) == 3
                % RGB in uint8, or 0-1 with out-of-range values saturating
                if isa(texture, "uint8")
                    img = texture;
                elseif isfloat(texture) || islogical(texture)
                    img = uint8(255*texture);
                else
                    error("phx:Shape:invalidTexture", "An RGB texture must be uint8, or floating-point or logical with values from 0 to 1.");
                end
            else
                error("phx:Shape:invalidTexture", "Texture must be a file name, the name of a built-in texture, an MxN matrix or an MxNx3 RGB image.");
            end
            img(:, :, 4) = 255; % opaque RGBA, see loadTexture
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
