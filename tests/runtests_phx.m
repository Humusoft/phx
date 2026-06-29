function results = runtests_phx(scope)
%runtests_phx Runs the PHX test suite.
%
%   runtests_phx() runs every test in the tests folder.
%
%   runtests_phx("pure") runs only tests that need neither the physics
%   engine nor a graphics session nor add-on toolboxes (tags Engine,
%   Graphics, Toolbox are excluded). Useful for headless CI.
%
%   runtests_phx("noengine") runs everything except the Engine-tagged
%   integration tests.
%
%   results = runtests_phx(___) returns the matlab.unittest.TestResult array.
%
%   See also tPhxMath, tShapeMass, tBodyKinematics, tSimulation

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        scope (1, 1) string {mustBeMember(scope, ["all", "pure", "noengine"])} = "all"
    end

    import matlab.unittest.TestSuite
    import matlab.unittest.selectors.HasTag

    here = fileparts(mfilename("fullpath"));
    root = fileparts(here);

    % Make the phx package visible without permanently touching the path.
    addpath(fullfile(root, "phx"));
    cleanup = onCleanup(@() rmpath(fullfile(root, "phx"))); %#ok<NASGU>

    suite = TestSuite.fromFolder(here);

    switch scope
        case "pure"
            suite = suite.selectIf(~HasTag("Engine") & ~HasTag("Graphics") & ~HasTag("Toolbox"));
        case "noengine"
            suite = suite.selectIf(~HasTag("Engine"));
    end

    results = run(suite);
    disp(table(results));
end
