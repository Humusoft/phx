classdef tReadPly < matlab.unittest.TestCase
%tReadPly Tests for the pure-MATLAB Stanford PLY reader.
%
%   phx.internal.readPly parses ASCII and binary (little- and big-endian)
%   PLY files without any toolbox dependency, returning the same unified
%   submesh structure as the other readMesh loaders. These tests write tiny
%   PLY files to a temporary folder, load them through phx.internal.readMesh
%   and check the geometry, the optional normals/color and the polygon
%   fan-triangulation. Being pure MATLAB, they need neither the engine nor a
%   graphics session.
%
%   See also phx.internal.readPly, phx.internal.readMesh, phx.shape.Mesh

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function asciiVerticesAndTriangulation(tc)
            % A single quad face fan-triangulates into two triangles, and the
            % optional per-vertex normals and color are read.
            file = tc.writeAsciiPly;
            mesh = phx.internal.readMesh(file);

            tc.assertNumElements(mesh.groups, 1);
            sub = mesh.groups.submeshes;
            tc.assertNumElements(sub, 1);

            tc.verifySize(sub.vertices, [4 3]);
            tc.verifyEqual(sub.vertices, [0 0 0; 1 0 0; 1 1 0; 0 1 0]);

            % Quad 0-1-2-3 -> triangles (0 1 2) and (0 2 3), 1-based
            tc.verifyEqual(sub.faces, [1 2 3; 1 3 4]);

            % Normals kept per-vertex, aligned with the vertices
            tc.verifyEqual(sub.normals, repmat([0 0 1], 4, 1));

            % The four corner colors average to mid-grey (0-255 -> 0-1)
            tc.verifyEqual(sub.color, [0.5 0.5 0.5], "AbsTol", 1e-6);

            % PLY winding is used as-is (unlike the flipped STL)
            tc.verifyFalse(sub.flip);
        end

        function binaryLittleEndianMatchesAscii(tc)
            file = tc.writeBinaryPly("ieee-le", "binary_little_endian");
            sub = tc.loadSingleSubmesh(file);
            tc.verifyEqual(sub.vertices, tc.binaryVertices, "AbsTol", 1e-5);
            tc.verifyEqual(sub.faces, [1 2 3; 1 3 4]);
            tc.verifyEmpty(sub.normals);
            tc.verifyEmpty(sub.color);
        end

        function binaryBigEndianMatchesAscii(tc)
            % Exercises the byte-swap path of the reader.
            file = tc.writeBinaryPly("ieee-be", "binary_big_endian");
            sub = tc.loadSingleSubmesh(file);
            tc.verifyEqual(sub.vertices, tc.binaryVertices, "AbsTol", 1e-5);
            tc.verifyEqual(sub.faces, [1 2 3; 1 3 4]);
        end

        function loadsThroughBodyShapeShorthand(tc)
            % The .ply extension routes through phx.shape.Mesh, so a bare file
            % name is accepted as a body shape (no engine needed to build it).
            file = tc.writeAsciiPly;
            shape = phx.shape.Mesh("Source", file, "Envelope", "convex");
            tc.verifyClass(shape, "phx.shape.Mesh");
        end
    end

    methods (Access = private)
        function sub = loadSingleSubmesh(tc, file)
            mesh = phx.internal.readMesh(file);
            tc.assertNumElements(mesh.groups, 1);
            sub = mesh.groups.submeshes;
            tc.assertNumElements(sub, 1);
        end

        function V = binaryVertices(~)
            V = [1 2 3; 4 5 6; 7 8 9; -1 -2 -3];
        end

        function folder = tempFolder(tc)
            import matlab.unittest.fixtures.TemporaryFolderFixture
            folder = tc.applyFixture(TemporaryFolderFixture).Folder;
        end

        function file = writeAsciiPly(tc)
            file = fullfile(tc.tempFolder, "quad.ply");
            fid = fopen(file, "w");
            tc.assertGreaterThan(fid, 0);
            closer = onCleanup(@() fclose(fid));
            fprintf(fid, ['ply\n' ...
                'format ascii 1.0\n' ...
                'comment a tiny test quad with normals and color\n' ...
                'element vertex 4\n' ...
                'property float x\nproperty float y\nproperty float z\n' ...
                'property float nx\nproperty float ny\nproperty float nz\n' ...
                'property uchar red\nproperty uchar green\nproperty uchar blue\n' ...
                'element face 1\n' ...
                'property list uchar int vertex_indices\n' ...
                'end_header\n' ...
                '0 0 0 0 0 1 255 0 0\n' ...
                '1 0 0 0 0 1 0 255 0\n' ...
                '1 1 0 0 0 1 0 0 255\n' ...
                '0 1 0 0 0 1 255 255 255\n' ...
                '4 0 1 2 3\n']);
        end

        function file = writeBinaryPly(tc, machinefmt, formatKeyword)
            file = fullfile(tc.tempFolder, "tris.ply");
            fid = fopen(file, "w", machinefmt);
            tc.assertGreaterThan(fid, 0);
            closer = onCleanup(@() fclose(fid));

            header = ['ply' newline ...
                'format ' char(formatKeyword) ' 1.0' newline ...
                'element vertex 4' newline ...
                'property float x' newline 'property float y' newline 'property float z' newline ...
                'element face 2' newline ...
                'property list uchar int vertex_indices' newline ...
                'end_header' newline];
            fwrite(fid, header, "char");

            V = tc.binaryVertices;
            for i = 1:size(V, 1)
                fwrite(fid, V(i, :), "single");
            end

            faces = {[0 1 2], [0 2 3]}; % 0-based triangles
            for i = 1:numel(faces)
                fwrite(fid, numel(faces{i}), "uint8");
                fwrite(fid, faces{i}, "int32");
            end
        end
    end

end
