classdef tInternals < matlab.unittest.TestCase
%tInternals Smoke tests for the undocumented MATLAB internals PHX relies on.
%
%   PHX deliberately calls a handful of matlab.*.internal.* APIs because
%   they skip the argument-validation overhead of their documented
%   counterparts in per-substep hot paths. That is an accepted trade-off;
%   these tests exist so that a MATLAB upgrade that changes or removes one
%   of them fails here, loudly, instead of inside a user's simulation.
%
%   Guarded internals and their call sites:
%     matlab.internal.math.ismemberhelper           - phx.Simulation/resolveState
%     matlab.internal.math.interp1                  - phx.Script/resolveState
%     matlab.internal.meshio.stlread                - phx.shape.STL/set.Source
%     matlab.io.internal.validators.validateFileName - phx.shape.STL/set.Source
%     matlab.graphics.internal.drawnow.startUpdate  - phx.Simulation/step
%     Matrix_I (hgtransform property)               - phx.Body/updateView
%
%   See also phx.Simulation, phx.Script, phx.shape.STL

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function ismemberhelperMatchesIsmember(tc)
            % Simulation/resolveState maps engine body IDs onto the sorted
            % SortedBodiesID column; every queried ID is present there.
            a = [30 10 50 20];
            b = [10 20 30 40 50];
            [tf, locb] = matlab.internal.math.ismemberhelper(a, b, true);
            [tfDoc, locbDoc] = ismember(a, b);
            tc.verifyEqual(logical(tf), tfDoc);
            tc.verifyEqual(double(locb), double(locbDoc));
        end

        function interp1InternalMatchesDocumented(tc)
            % Script/resolveState signature: (X, V, method, extrapolation, Xq).
            x = [0 1 2 4];
            v = [0 1 0 2];
            xq = [0.5 1.5 3 3.9];
            yi = matlab.internal.math.interp1(x, v, 'linear', 'linear', xq);
            tc.verifyEqual(yi, interp1(x, v, xq, 'linear'), "AbsTol", 1e-12);
        end

        function interp1InternalSupportsPchip(tc)
            x = 0:5;
            v = sin(x);
            xq = [0.4 2.7 4.1];
            yi = matlab.internal.math.interp1(x, v, 'pchip', 'pchip', xq);
            tc.verifyEqual(yi, interp1(x, v, xq, 'pchip'), "AbsTol", 1e-12);
        end

        function stlreadInternalReturnsMesh(tc)
            % shape.STL/set.Source reads .Faces, .Vertices and .Normals from
            % the returned mesh; lock in those fields on a one-triangle file.
            fileName = tc.writeTriangleSTL;
            stl = matlab.internal.meshio.stlread(fileName);
            tc.verifySize(stl.Faces, [1 3]);
            tc.verifySize(stl.Vertices, [3 3]);
            tc.verifySize(stl.Normals, [1 3]);
            tc.verifyEqual(sort(stl.Vertices(:, 1))', [0 0 1]);
        end

        function validateFileNameResolvesExisting(tc)
            % shape.STL/set.Source expects a cell array of resolved names.
            fileName = tc.writeTriangleSTL;
            cFileName = matlab.io.internal.validators.validateFileName(fileName);
            tc.verifyClass(cFileName, 'cell');
            tc.verifyTrue(isfile(cFileName{1}));
        end

        function startUpdateIsResolvable(tc)
            % Simulation/step calls it per redraw; calling it needs a live
            % graphics update in progress, so only pin down that it resolves.
            tc.verifyNotEmpty(which('matlab.graphics.internal.drawnow.startUpdate'));
        end
    end

    methods (Test, TestTags = {'Graphics'})
        function hgtransformHasMatrixI(tc)
            % Body/updateView writes Matrix_I (the validation-free twin of
            % Matrix) on hgtransforms; the set must land in Matrix.
            fig = figure("Visible", "off");
            tc.addTeardown(@() close(fig));
            t = hgtransform("Parent", axes(fig));
            M = eye(4);
            M(13:15) = [1 -2 3];
            t.Matrix_I = M;
            tc.verifyEqual(t.Matrix, M);
        end
    end

    methods (Access = private)
        function fileName = writeTriangleSTL(tc)
            % One-triangle ASCII STL in a temporary folder.
            import matlab.unittest.fixtures.TemporaryFolderFixture
            folder = tc.applyFixture(TemporaryFolderFixture).Folder;
            fileName = fullfile(folder, 'triangle.stl');
            fid = fopen(fileName, 'w');
            tc.assertGreaterThan(fid, 0);
            fprintf(fid, ['solid triangle\n' ...
                '  facet normal 0 0 1\n' ...
                '    outer loop\n' ...
                '      vertex 0 0 0\n' ...
                '      vertex 1 0 0\n' ...
                '      vertex 0 1 0\n' ...
                '    endloop\n' ...
                '  endfacet\n' ...
                'endsolid triangle\n']);
            fclose(fid);
        end
    end

end
