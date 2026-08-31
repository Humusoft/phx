classdef PhxTestCase < matlab.unittest.TestCase
%PhxTestCase Shared helpers for the PHX test suite.
%
%   Base class of the t* test classes. It carries the helpers that were
%   otherwise copied into almost every one of them: the engine assumption,
%   headless and on-axes body spawning, the shape lookup, the joint-anchor
%   check and the temporary-file writers.
%
%   It declares no Test methods, so it contributes nothing to the suite that
%   runtests_phx builds from this folder.
%
%   See also runtests_phx

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Access = private)
        CachedTempFolder string = string.empty
    end

    methods (Access = protected)
        function requireEngine(tc)
            % Assume the MEX away rather than fail when it is not installed.
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
        end

        function b = spawnBody(tc, position, varargin) %#ok<INUSD> tc unused by design
            % Headless body ([] axes) at the given position; any further
            % name-value pairs are passed to the constructor.
            b = phx.Body([], "Position", position, varargin{:});
        end

        function ax = prepareAxes(tc)
            % Axes in an invisible figure, made current for the builders that
            % default to gca. The figure is closed at teardown.
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            ax = axes(f);
        end

        function shape = bodyShape(~, body)
            % The phx.shape.* object a body wears, read off its primitives.
            shape = [];
            for ch = body.Graphics.Children'
                s = getappdata(ch, "phxShape");
                if ~isempty(s)
                    shape = s;
                    return
                end
            end
        end

        function verifyAnchorsCoincide(tc, joints, tol, point)
            % Both world-space anchor points of every given joint coincide
            % (and optionally sit at the given point). Accepts a single
            % joint, an array or cell array of them, or a struct of them.
            arguments
                tc
                joints
                tol (1, 1) double = 1e-9
                point double = []
            end
            for j = PhxTestCase.jointList(joints)
                j = j{1}; %#ok<FXSET> one joint of the list
                pa = phx.internal.transformPoint(j.Parents{1}.Transform, j.PointA);
                pb = phx.internal.transformPoint(j.Parents{2}.Transform, j.PointB);
                tc.verifyEqual(pa, pb, "AbsTol", tol, ...
                    "Anchor points of joint '" + j.Name + "' do not coincide.");
                if ~isempty(point)
                    tc.verifyEqual(pa, point, "AbsTol", tol, ...
                        "Joint '" + j.Name + "' does not sit at the expected point.");
                end
            end
        end

        function folder = tempFolder(tc)
            % One temporary folder per test, removed when the test finishes.
            % It is cached so that several files written by one test end up
            % side by side, which the mesh-path tests rely on.
            import matlab.unittest.fixtures.TemporaryFolderFixture
            if isempty(tc.CachedTempFolder) || ~isfolder(tc.CachedTempFolder)
                tc.CachedTempFolder = string(tc.applyFixture(TemporaryFolderFixture).Folder);
            end
            folder = tc.CachedTempFolder;
        end

        function file = writeTextFile(tc, name, content)
            % Write content into a file of that name in the temporary folder
            % of the test. The name may carry subfolders, which are created.
            file = fullfile(tc.tempFolder, name);
            folder = fileparts(file);
            if ~isfolder(folder)
                mkdir(folder);
            end
            fid = fopen(file, "w");
            tc.assertGreaterThan(fid, 0, "Cannot write " + file + ".");
            closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fwrite(fid, content);
        end

        function file = writeSTL(tc, name, V, F)
            % ASCII STL of the given vertices and faces in a temporary
            % folder. The facet normals are placeholders - every reader PHX
            % uses takes the geometry from the vertices.
            lines = "solid " + erase(name, ".stl");
            for f = F'
                lines(end + 1) = "  facet normal 0 0 1"; %#ok<AGROW>
                lines(end + 1) = "    outer loop"; %#ok<AGROW>
                for v = f'
                    lines(end + 1) = "      vertex " + ...
                        join(compose("%.17g", V(v, :)), " "); %#ok<AGROW>
                end
                lines(end + 1) = "    endloop"; %#ok<AGROW>
                lines(end + 1) = "  endfacet"; %#ok<AGROW>
            end
            lines(end + 1) = "endsolid";
            file = tc.writeTextFile(name, strjoin(lines, newline));
        end
    end

    methods (Static, Access = private)
        function list = jointList(joints)
            % Normalize the accepted joint containers into a cell row.
            if isstruct(joints)
                list = struct2cell(joints)';
            elseif iscell(joints)
                list = reshape(joints, 1, []);
            else
                list = num2cell(reshape(joints, 1, []));
            end
        end
    end

end
