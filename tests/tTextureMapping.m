classdef tTextureMapping < PhxTestCase
%tTextureMapping Tests for the orientation of shape textures.
%
%   Every shape draws a texture set on it; a phx.shape.Mesh without texture
%   coordinates (an STL file, say) gets them by box projection.
%
%   Texture coordinates (u, v) address an image like a matrix: u runs along
%   the columns, v along the rows from the top, so (0, 0) is the top-left
%   corner. A world TriangleStrip indexes the first spatial dimension of its
%   Texture.CData by u, so the texture data is channel x columns x rows. The
%   generated shapes lay the image out unmirrored seen from outside, with
%   the image top at the end of the profile or spine; Terrain follows its
%   Height matrix instead. The layout and UV tests are pure, the tests that
%   read the drawn primitive back need a graphics session.
%
%   See also phx.base.Shape, phx.internal.Geometry, phx.shape.Terrain

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (TestParameter)
        generator = struct( ...
            "box", @() phx.internal.Geometry.triBox([1 2 3]), ...
            "sphere", @() phx.internal.Geometry.sphere(2, 16), ...
            "cylinder", @() phx.internal.Geometry.revolution([-0.5 0.5; 0.5 0.5], 16, true, true), ...
            "cone", @() phx.internal.Geometry.revolution([-0.5 0.5; 0.5 0], 16, true, false), ...
            "extrusion", @() phx.internal.Geometry.extrusion([0 0 0; 0.3 0.2 1; 0 1 2], 1, ...
                0.3*[-1 -1; 1 -1; 1 1; -1 1; -1 -1], true, true), ...
            "rock", @() tTextureMapping.rockPerVertex(1, 0.6), ...
            "boxProjection", @() tTextureMapping.boxProjectedSphere)

        % Every shape carries texture coordinates, a mesh without its own
        % by box projection
        texturedShape = struct( ...
            "mesh", {{"Mesh", "Vertices", [0 0 0; 1 0 0; 0 1 0; 0 0 1], ...
                "Faces", [1 3 2; 1 2 4; 1 4 3; 2 3 4]}}, ...
            "box", {{"Box"}}, "sphere", {{"Sphere"}}, "cylinder", {{"Cylinder"}}, ...
            "cone", {{"Cone"}}, "capsule", {{"Capsule"}}, "tube", {{"Tube"}}, ...
            "rock", {{"Rock"}}, "terrain", {{"Terrain"}}, ...
            "extrusion", {{"Extrusion", "Spine", [0 0 0; 0 0 1], ...
                "Profile", 0.3*[-1 -1; 1 -1; 1 1; -1 1; -1 -1]}}, ...
            "revolution", {{"Revolution", "Profile", [-0.5 0.3; 0 0.5; 0.5 0.3]}})
    end

    methods (Test)
        function imageFileLaysOutUAlongColumns(tc)
            % Texel (row r, column c) of the file is CData(:, c, r), with an
            % opaque alpha plane appended
            img = uint8(reshape(1:3*5*3, 3, 5, 3));
            file = fullfile(tc.tempFolder, "grid.png");
            imwrite(img, file);

            data = phx.base.Shape.loadTexture(file);

            tc.verifySize(data, [4 5 3]);
            tc.verifyEqual(permute(data(1:3, :, :), [3 2 1]), img);
            tc.verifyTrue(all(data(4, :) == 255));
        end

        function scalarMatrixWithoutColormapIsGrayscale(tc)
            % Scalar data span the gray levels, laid out like an image file
            data = phx.base.Shape.matrixTexture([0 1 2; 3 4 6], zeros(0, 3));

            gray = uint8(255*[0 1 2; 3 4 6]/6);
            tc.verifySize(data, [4 3 2]);
            for c = 1:3
                tc.verifyEqual(permute(data(c, :, :), [3 2 1]), gray);
            end
            tc.verifyTrue(all(data(4, :) == 255));
        end

        function uint8MatrixKeepsItsGrayLevels(tc)
            % uint8 data are absolute gray levels; through a colormap 0-255
            % spans the whole colormap
            g = uint8([100 150; 200 250]);
            data = phx.base.Shape.matrixTexture(g, zeros(0, 3));
            for c = 1:3
                tc.verifyEqual(permute(data(c, :, :), [3 2 1]), g);
            end

            data = phx.base.Shape.matrixTexture(uint8([0 255 128]), [1 0 0; 0 1 0; 0 0 1]);
            tc.verifyEqual(data(1:3, :)', uint8([255 0 0; 0 0 255; 0 255 0]));
        end

        function rgbMatrixMatchesUint8Image(tc)
            % An RGB matrix is used as is, in uint8 or scaled to 0-1
            img = uint8(reshape(10*(1:24), 2, 4, 3));

            fromUint8 = phx.base.Shape.matrixTexture(img, zeros(0, 3));
            fromDouble = phx.base.Shape.matrixTexture(double(img)/255, zeros(0, 3));

            tc.verifyEqual(permute(fromUint8(1:3, :, :), [3 2 1]), img);
            tc.verifyEqual(fromDouble, fromUint8);
        end

        function invalidTextureMatrixIsRejected(tc)
            tc.verifyError(@() phx.shape.Box("Texture", rand(2, 2, 2)), "phx:Shape:invalidTexture");
            tc.verifyError(@() phx.shape.Box("Texture", rand(2, 2, 3, 2)), "phx:Shape:invalidTexture");
            tc.verifyError(@() phx.shape.Box("Texture", uint16(ones(2, 2, 3))), "phx:Shape:invalidTexture");
        end

        function generatedShapesReadUnmirrored(tc, generator)
            % Seen from outside (against the vertex normals), image right
            % (+u) and image up (-v) form a right-handed pair on every
            % triangle that carries a texture area
            [V, N, F, T] = generator();
            [h, checked] = tTextureMapping.handedness(V, N, F, T);
            tc.assertGreaterThan(checked, 0);
            tc.verifyTrue(all(h(h ~= 0) > 0), ...
                sprintf("%d of %d triangles map the image mirrored.", sum(h < 0), checked));
        end

        function imageTopAtProfileEnd(tc)
            % A revolution profile and an extrusion spine run upward here;
            % the image top (v = 0) lies at their end
            [V, ~, ~, T] = phx.internal.Geometry.sphere(2, 16);
            tc.verifyEqual(T(V(:, 3) == max(V(:, 3)), 2), zeros(nnz(V(:, 3) == max(V(:, 3))), 1), "AbsTol", 1e-12);
            tc.verifyEqual(T(V(:, 3) == min(V(:, 3)), 2), ones(nnz(V(:, 3) == min(V(:, 3))), 1), "AbsTol", 1e-12);

            [V, ~, ~, T] = phx.internal.Geometry.extrusion([0 0 0; 0 0 1], 1, [-1 -1; 1 -1; 1 1; -1 1; -1 -1], false, false);
            tc.verifyEqual(T(V(:, 3) == 1, 2), zeros(5, 1));
            tc.verifyEqual(T(V(:, 3) == 0, 2), ones(5, 1));
        end

        function rockCoordinatesStayInsideTheImage(tc)
            % The projection is scaled by the diameter, which bounds the
            % deformed rock, so no coordinate leaves the image
            for roundness = [0.2 0.75 1]
                [~, ~, ~, T] = phx.internal.Geometry.rock(2, roundness);
                tc.verifyGreaterThanOrEqual(min(T(:)), 0);
                tc.verifyLessThanOrEqual(max(T(:)), 1);
            end
        end

        function terrainFollowsHeightMatrix(tc)
            % u runs along x (the columns of Height), v along y (its rows),
            % (0, 0) at the lowest x and y
            [V, ~, ~, T] = phx.internal.Geometry.terrain([4 2 1], zeros(3, 5));
            tc.verifyEqual(T, (V(:, 1:2) - min(V(:, 1:2)))./[4 2], "AbsTol", 1e-12);
        end

        function objCoordinatesFlipV(tc)
            % OBJ texture coordinates run up the image; PHX v runs down
            file = tc.writeTextFile("quad.obj", sprintf([ ...
                'v 0 0 0\nv 1 0 0\nv 1 1 0\n' ...
                'vt 0.1 0.2\nvt 0.7 0.4\nvt 0.9 1\n' ...
                'f 1/1 2/2 3/3\n']));
            mesh = phx.internal.readMesh(file);
            tc.verifyEqual(mesh.groups(1).submeshes(1).uv, [0.1 0.8; 0.7 0.6; 0.9 0], "AbsTol", 1e-12);
        end

        function plyCoordinatesFlipV(tc)
            % PLY s/t follow the OBJ convention
            file = tc.writeTextFile("tri.ply", sprintf([ ...
                'ply\nformat ascii 1.0\nelement vertex 3\n' ...
                'property float x\nproperty float y\nproperty float z\n' ...
                'property float s\nproperty float t\n' ...
                'element face 1\nproperty list uchar int vertex_indices\nend_header\n' ...
                '0 0 0 0.1 0.2\n1 0 0 0.7 0.4\n1 1 0 0.9 1\n3 0 1 2\n']));
            mesh = phx.internal.readMesh(file);
            tc.verifyEqual(mesh.groups(1).submeshes(1).uv, [0.1 0.8; 0.7 0.6; 0.9 0], "AbsTol", 1e-6);
        end
    end

    methods (Test, TestTags = {'Graphics'})
        function textureReachesTheDrawnShape(tc, texturedShape)
            % A texture set on a shape is drawn, never silently dropped
            ax = tc.prepareTexturedAxes;
            b = phx.Body(ax, "Type", "static", "Shape", [texturedShape, {"Texture", "checker"}]);
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));

            strips = findobj(b.Graphics.Children, "flat", "-isa", "matlab.graphics.primitive.world.TriangleStrip");
            tc.assertNotEmpty(strips);
            tc.verifyEqual(string({strips.ColorType}), repmat("texturemapped", 1, numel(strips)));
        end

        function builtinTextureIsItsGenerator(tc)
            % A built-in name, in any case, draws the default generator output
            ax = tc.prepareTexturedAxes;
            b = phx.Body(ax, "Shape", {"Box", "Texture", "Stone"});
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));

            tc.verifyEqual(b.Graphics.Children.Texture.CData, ...
                phx.base.Shape.matrixTexture(phx.base.Shape.stone, zeros(0, 3)));
            tc.verifyError(@() phx.shape.Box("Texture", "nosuchtexture"), "phx:Shape:fileNotFound");
        end

        function renamedMetalWarnsAndDrawsScratches(tc)
            % The old name of "scratches" still works, with a warning
            ax = tc.prepareTexturedAxes;
            shp = tc.verifyWarning(@() phx.shape.Box("Texture", "metal"), "phx:Shape:renamedTexture");
            b = phx.Body(ax, "Shape", shp);
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));

            tc.verifyEqual(b.Graphics.Children.Texture.CData, ...
                phx.base.Shape.matrixTexture(phx.base.Shape.scratches, zeros(0, 3)));
        end

        function colormappedMatrixMatchesImageFile(tc)
            % A scalar matrix lands on the shape as the image of its colors,
            % whichever of Texture and TextureColormap is set first
            cdata = [0 1 2; 3 4 5];
            cmap = [1 0 0; 0 1 0; 0 0 1; 1 1 0; 0 1 1; 1 0 1];
            img = uint8(255*reshape(cmap(cdata + 1, :), [size(cdata) 3]));
            file = fullfile(tc.tempFolder, "cmap.png");
            imwrite(img, file);

            ax = tc.prepareTexturedAxes;
            b1 = phx.Body(ax, "Shape", {"Box", "Texture", cdata, "TextureColormap", cmap});
            b2 = phx.Body(ax, "Shape", {"Box", "TextureColormap", cmap, "Texture", cdata});
            b3 = phx.Body(ax, "Shape", {"Box", "Texture", file});
            tc.addTeardown(@() PhxTestCase.deleteIfValid([b1 b2 b3]));

            tc.verifyEqual(b1.Graphics.Children.Texture.CData, b3.Graphics.Children.Texture.CData);
            tc.verifyEqual(b2.Graphics.Children.Texture.CData, b3.Graphics.Children.Texture.CData);
        end

        function sphereShowsImageTopAtNorthPole(tc)
            % End to end: a red-over-blue image, sampled where the drawn
            % primitive maps each vertex, is red on the upper hemisphere
            img = repmat(reshape(uint8([255 0 0]), 1, 1, 3), 8, 16);
            img(5:8, :, :) = repmat(reshape(uint8([0 0 255]), 1, 1, 3), 4, 16);
            file = fullfile(tc.tempFolder, "hemispheres.png");
            imwrite(img, file);

            ax = tc.prepareTexturedAxes;
            b = phx.Body(ax, "Shape", {"Sphere", "Segments", 16, "Texture", file});
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));

            ts = b.Graphics.Children;
            cdata = ts.Texture.CData;
            uv = double(ts.ColorData);
            col = round(uv(1, :)*(size(cdata, 2) - 1)) + 1;
            row = round(uv(2, :)*(size(cdata, 3) - 1)) + 1;
            red = cdata(sub2ind(size(cdata), ones(size(col)), col, row)) == 255;
            z = double(ts.VertexData(3, :));
            tc.verifyTrue(all(red(z > 0.05)) && ~any(red(z < -0.05)));
        end

        function stlMeshReadsUnmirrored(tc)
            % An STL file has no texture coordinates; the box projection
            % lays the image out unmirrored seen from outside on every face
            [V, F] = tTextureMapping.cuboid;
            file = fullfile(tc.tempFolder, "cuboid.stl");
            stlwrite(triangulation(F, V), file);

            ax = tc.prepareTexturedAxes;
            b = phx.Body(ax, "Type", "static", "Shape", {"Mesh", "Source", file, "Texture", "checker"});
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));

            [V, N, F, T] = tTextureMapping.drawnGeometry(b);
            [h, checked] = tTextureMapping.handedness(V, N, F, T);
            tc.verifyEqual(checked, size(F, 1));
            tc.verifyTrue(all(h > 0), ...
                sprintf("%d of %d triangles map the image mirrored.", sum(h < 0), checked));
        end

        function boxProjectionIgnoresWinding(tc)
            % The projection follows the normals the mesh is drawn with,
            % so reversing the faces leaves every vertex its coordinates
            [V, F] = tTextureMapping.cuboid;
            N = V./vecnorm(V, 2, 2);
            ax = tc.prepareTexturedAxes;
            b1 = phx.Body(ax, "Type", "static", "Shape", {"Mesh", "Vertices", V, "Normals", N, "Faces", F, "Texture", "checker"});
            b2 = phx.Body(ax, "Type", "static", "Shape", {"Mesh", "Vertices", V, "Normals", N, "Faces", fliplr(F), "Texture", "checker"});
            tc.addTeardown(@() PhxTestCase.deleteIfValid([b1 b2]));

            [V1, ~, ~, T1] = tTextureMapping.drawnGeometry(b1);
            [V2, ~, ~, T2] = tTextureMapping.drawnGeometry(b2);
            tc.verifyEqual(sortrows([V2 T2]), sortrows([V1 T1]));
        end

        function boxProjectionSpansTheLargestExtent(tc)
            % The image spans the largest extent of the mesh, wherever its
            % origin lies, so the coordinates fill the image without
            % leaving it
            [V, F] = tTextureMapping.cuboid;
            ax = tc.prepareTexturedAxes;
            b = phx.Body(ax, "Type", "static", "Shape", {"Mesh", "Vertices", V + [5 -3 1], ...
                "Faces", F, "Centered", false, "Texture", "checker"});
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));

            [~, ~, ~, T] = tTextureMapping.drawnGeometry(b);
            tc.verifyEqual([min(T(:)) max(T(:))], [0 1], "AbsTol", 1e-6);
        end
    end

    methods (Access = private)
        function ax = prepareTexturedAxes(tc)
            % Axes that take the world-primitive draw path, which carries
            % the texture, as the axes of phx.extra.Viewer do
            ax = tc.prepareAxes;
            setappdata(ax, "phxAxes", true);
        end
    end

    methods (Static, Access = private)
        function [V, N, F, T] = rockPerVertex(ASize, Roundness)
            % The rock comes with face normals; repeat them on the corners
            [V, N, F, T] = phx.internal.Geometry.rock(ASize, Roundness);
            N(F', :) = repelem(N, 3, 1);
        end

        function [V, N, F, T] = boxProjectedSphere
            % A curved surface, so the projection changes axis across it
            [V, N, F] = phx.internal.Geometry.sphere(2, 16);
            FN = N(F(:, 1), :) + N(F(:, 2), :) + N(F(:, 3), :);
            [V, F, T, idx] = phx.internal.Geometry.boxUV(V, F, 2, Normals = FN);
            N = N(idx, :);
        end

        function [V, F] = cuboid
            % A 2 x 1 x 0.5 box with shared corners, wound outward
            [V, ~, Q] = phx.internal.Geometry.quadBox([2 1 0.5]);
            F = fliplr([Q(:, [1 2 3]); Q(:, [1 3 4])]);
        end

        function [V, N, F, T] = drawnGeometry(body)
            % Geometry of the drawn world primitive, one row per vertex
            ts = body.Graphics.Children;
            V = double(ts.VertexData)';
            N = double(ts.NormalData)';
            F = reshape(double(ts.VertexIndices), 3, [])';
            T = double(ts.ColorData)';
        end

        function [h, checked] = handedness(V, N, F, T)
            % +1 where image right and image up form a right-handed pair
            % about the outward normal, -1 where mirrored, 0 where the
            % triangle has no texture or no surface area
            h = zeros(size(F, 1), 1);
            for i = 1:size(F, 1)
                p = V(F(i, :), :);
                t = T(F(i, :), :);
                dP = p(2:3, :) - p(1, :);
                dT = t(2:3, :) - t(1, :);
                if abs(det(dT)) < 1e-12 || norm(cross(dP(1, :), dP(2, :))) < 1e-12
                    continue
                end
                J = dT\dP; % rows: dP/du, dP/dv
                h(i) = sign(dot(cross(J(1, :), -J(2, :)), sum(N(F(i, :), :), 1)));
            end
            checked = nnz(h);
        end
    end
end
