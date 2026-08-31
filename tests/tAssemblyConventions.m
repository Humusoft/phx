classdef tAssemblyConventions < PhxTestCase
%tAssemblyConventions The conventions every phx.assembly builder shares.
%
%   All the builders - arena, chain, scatter, wall and the URDF import - obey
%   the same four rules, which used to be written out once per builder:
%
%     * an optional leading axes target is drawn into and leaves the current
%       axes alone;
%     * an explicit [] target follows the phx.Body([], ...) headless
%       convention: no parent axes, and no figure is created;
%     * the base-pose options Position/Orientation/EulerAngles move the whole
%       build rigidly, every part of it;
%     * Orientation together with EulerAngles is a conflict, reported with the
%       builder's own phx:<builder>:conflictingOptions identifier.
%
%   What is specific to one builder - the geometry it lays out and the inputs
%   it validates - stays in tAssembly and tImport.
%
%   See also phx.assembly.arena, phx.assembly.chain, phx.assembly.scatter,
%   phx.assembly.wall, phx.assembly.import, tAssembly, tImport

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        % Rigid base pose used by the base-pose test
        BaseTranslation = [0.5 -1 2]
        BaseEulerAngles = [0.2 -0.3 0.4]
    end

    properties (TestParameter)
        Builder = tAssemblyConventions.builders
    end

    methods (Test)
        function conflictingBaseRotationRaisesError(tc, Builder)
            tc.verifyError(@() tc.build(Builder, [], ...
                "Orientation", [0 -1 0; 1 0 0; 0 0 1], ...
                "EulerAngles", [0 0 pi/2]), ...
                "phx:" + Builder.Name + ":conflictingOptions");
        end
    end

    methods (Test, TestTags = {'Graphics'})
        function explicitAxesTargetIsHonored(tc, Builder)
            % Two plain axes rather than subplots: all this needs is a target
            % that is not the current one, and subplot is by far the slower
            % way to get it.
            f = figure("Visible", "off");
            tc.addTeardown(@() close(f));
            axTarget = axes(f);
            axCurrent = axes(f);
            axes(axCurrent);

            bodies = tc.build(Builder, axTarget);

            for b = bodies
                tc.verifyEqual(b.ParentAxes, axTarget, ...
                    "A body was not drawn into the requested axes.");
            end
            tc.verifyEqual(gca, axCurrent, "The current axes changed.");
        end

        function emptyTargetBuildsWithoutGraphics(tc, Builder)
            nFigures = numel(findall(groot, "Type", "figure"));

            bodies = tc.build(Builder, []);

            for b = bodies
                tc.verifyEmpty(b.ParentAxes, "A headless body got a parent axes.");
            end
            tc.verifyEqual(numel(findall(groot, "Type", "figure")), nFigures, ...
                "A headless build created a figure.");
        end

        function basePoseTransformsTheWholeBuild(tc, Builder)
            % The whole zero-pose build is moved rigidly, part by part. The
            % generator is reseeded before each call so that the builders
            % which place their parts at random lay out the same scene twice.
            TBase = eye(4);
            TBase(1:3, 1:3) = phx.internal.Math.rot321(tc.BaseEulerAngles);
            TBase(1:3, 4) = tc.BaseTranslation;

            rng(7);
            ref = tc.build(Builder, []);
            rng(7);
            moved = tc.build(Builder, [], "Position", tc.BaseTranslation, ...
                "EulerAngles", tc.BaseEulerAngles);
            rng(7);
            rotated = tc.build(Builder, [], "Orientation", TBase(1:3, 1:3));

            tc.assertNumElements(moved, numel(ref));
            tc.assertNumElements(rotated, numel(ref));
            for i = 1:numel(ref)
                tc.verifyEqual(moved(i).Transform, TBase*ref(i).Transform, ...
                    "AbsTol", 1e-12, ...
                    "Position/EulerAngles were not applied to part #" + i + ".");
                TExpected = ref(i).Transform;
                TExpected(1:3, :) = TBase(1:3, 1:3)*TExpected(1:3, :);
                tc.verifyEqual(rotated(i).Transform, TExpected, "AbsTol", 1e-12, ...
                    "Orientation alone was not applied to part #" + i + ".");
            end
        end
    end

    methods (Access = private)
        function bodies = build(tc, Builder, target, varargin)
            % Run one builder into the given target and return its parts as a
            % plain body array, whatever container the builder returns.
            ws = warning("off", "phx:import:substitutedJoint");
            tc.addTeardown(@() warning(ws));
            bodies = Builder.Build(target, varargin{:});
        end
    end

    methods (Static, Access = private)
        function s = builders()
            urdf = fullfile(fileparts(mfilename("fullpath")), "fixtures", ...
                "three_link_arm.urdf");

            s.arena = struct("Name", "arena", "Build", ...
                @(ax, varargin) tAssemblyConventions.parts( ...
                    phx.assembly.arena(ax, varargin{:})));
            s.chain = struct("Name", "chain", "Build", ...
                @(ax, varargin) tAssemblyConventions.parts( ...
                    phx.assembly.chain(ax, [0 0 0; 0.4 0 0; 0.8 0 0.2], ...
                        "Anchor", "both", varargin{:})));
            s.scatter = struct("Name", "scatter", "Build", ...
                @(ax, varargin) phx.assembly.scatter(ax, ...
                    {"Sphere", "Diameter", 0.2}, 3, varargin{:}));
            s.wall = struct("Name", "wall", "Build", ...
                @(ax, varargin) phx.assembly.wall(ax, "Rows", 2, "Columns", 3, ...
                    varargin{:}));
            s.import = struct("Name", "import", "Build", ...
                @(ax, varargin) tAssemblyConventions.parts( ...
                    phx.assembly.import(ax, urdf, varargin{:})));
        end

        function bodies = parts(built)
            % Flatten whatever a builder returns (a struct of arrays, or a
            % struct of named bodies) into one body row.
            bodies = phx.Body.empty(1, 0);
            for f = string(fieldnames(built))'
                value = built.(f);
                if isa(value, "phx.Body")
                    bodies = [bodies reshape(value, 1, [])]; %#ok<AGROW>
                end
            end
        end
    end

end
