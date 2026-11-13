classdef tEngineSettings < PhxTestCase
%tEngineSettings Contract of phx.engine.Settings: LengthScale, modes, convergence.
%
%   The collision tolerances follow LengthScale in automatic mode (Margin
%   4 %, SpeculativeDistance 2 %, PenetrationSlop 0.1 % of it) and keep a
%   value set to them in manual mode, as the limits of MATLAB axes do with
%   XLim and XLimMode. Settings is a value class, so these tests need
%   neither the physics engine nor a graphics session.
%
%   The velocity iterations of an island end at JointTolerance and
%   ContactTolerance, at the latest at JointIterations (an island with a
%   joint) or ContactIterations (any other); a tolerance of 0 runs every
%   island to its cap. The tests of that (tagged Engine) read the counts
%   from getDiagnosticReport of phx.Simulation.
%
%   See also phx.engine.Settings, phx.Simulation/getDiagnosticReport

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (TestParameter)
        % Each tolerance with its mode property and its share of LengthScale
        tolerance = struct( ...
            "Margin", {{"Margin", "MarginMode", 0.04}}, ...
            "SpeculativeDistance", {{"SpeculativeDistance", "SpeculativeDistanceMode", 0.02}}, ...
            "PenetrationSlop", {{"PenetrationSlop", "PenetrationSlopMode", 0.001}})
    end

    methods (Test)
        function defaultsAreTheMetreValues(tc)
            % The default LengthScale 1 gives the tolerances of a metre scene,
            % all in automatic mode
            s = phx.engine.Settings;
            tc.verifyEqual(s.LengthScale, 1);
            tc.verifyEqual([s.Margin, s.SpeculativeDistance, s.PenetrationSlop], [0.04 0.02 0.001]);
            tc.verifyEqual([s.MarginMode, s.SpeculativeDistanceMode, s.PenetrationSlopMode], ...
                ["auto" "auto" "auto"]);
        end

        function autoFollowsLengthScale(tc, tolerance)
            [name, ~, share] = tolerance{:};
            s = phx.engine.Settings("LengthScale", 0.01);
            tc.verifyEqual(s.(name), share*0.01, "RelTol", 1e-12);
            s.LengthScale = 0.1;
            tc.verifyEqual(s.(name), share*0.1, "RelTol", 1e-12);
        end

        function settingAValueSwitchesToManual(tc, tolerance)
            % A value set stays as it is when LengthScale changes
            [name, mode] = tolerance{1:2};
            s = phx.engine.Settings;
            s.(name) = 0.003;
            tc.verifyEqual(s.(mode), "manual");
            s.LengthScale = 0.01;
            tc.verifyEqual(s.(name), 0.003);
        end

        function autoModeRestoresTheDerivedValue(tc, tolerance)
            [name, mode, share] = tolerance{:};
            s = phx.engine.Settings("LengthScale", 0.1);
            s.(name) = 0;
            s.(mode) = "auto";
            tc.verifyEqual(s.(name), share*0.1, "RelTol", 1e-12);
        end

        function tolerancesAreIndependent(tc)
            % Setting one tolerance leaves the others automatic
            s = phx.engine.Settings("LengthScale", 0.01, "Margin", 0);
            tc.verifyEqual(s.Margin, 0);
            tc.verifyEqual(s.SpeculativeDistanceMode, "auto");
            tc.verifyEqual(s.SpeculativeDistance, 2e-4, "RelTol", 1e-12);
        end

        function constructorOrderDoesNotMatterForAValueSet(tc)
            % A value given at construction stays manual whether LengthScale
            % comes before or after it
            a = phx.engine.Settings("Margin", 0, "LengthScale", 0.1);
            b = phx.engine.Settings("LengthScale", 0.1, "Margin", 0);
            tc.verifyEqual([a.Margin, b.Margin], [0 0]);
            tc.verifyEqual([a.MarginMode, b.MarginMode], ["manual" "manual"]);
        end

        function invalidValuesAreRefused(tc, tolerance)
            [name, mode] = tolerance{1:2};
            s = phx.engine.Settings;
            tc.verifyError(@() setfield(s, name, -1), "MATLAB:validators:mustBeNonnegative"); %#ok<SFLD>
            tc.verifyError(@() setfield(s, mode, "fixed"), "MATLAB:validators:mustBeMember"); %#ok<SFLD>
        end

        function lengthScaleMustBePositiveAndFinite(tc)
            tc.verifyError(@() phx.engine.Settings("LengthScale", 0), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() phx.engine.Settings("LengthScale", Inf), "MATLAB:validators:mustBeFinite");
        end

        function convergenceDefaults(tc)
            s = phx.engine.Settings;
            tc.verifyEqual([s.JointTolerance, s.ContactTolerance], [1e-3 1e-3]);
            tc.verifyEqual([s.JointIterations, s.ContactIterations, s.PositionIterations], [50 20 4]);
        end

        function invalidConvergenceValuesAreRefused(tc)
            s = phx.engine.Settings;
            for name = ["JointTolerance" "ContactTolerance"]
                tc.verifyError(@() setfield(s, name, -1e-3), "MATLAB:validators:mustBeNonnegative", name); %#ok<SFLD>
                tc.verifyError(@() setfield(s, name, Inf), "MATLAB:validators:mustBeFinite", name); %#ok<SFLD>
            end
            tc.verifyError(@() setfield(s, "JointIterations", 256), "MATLAB:validators:mustBeInRange"); %#ok<SFLD>
            tc.verifyError(@() setfield(s, "JointIterations", 2.5), "MATLAB:validators:mustBeInteger"); %#ok<SFLD>
            tc.verifyError(@() setfield(s, "ContactIterations", 0), "MATLAB:validators:mustBePositive"); %#ok<SFLD>
            tc.verifyError(@() setfield(s, "PositionIterations", 0), "MATLAB:validators:mustBePositive"); %#ok<SFLD>
        end
    end

    methods (Test, TestTags = {'Engine'})
        function applyHandsTheValuesToTheEngine(tc)
            % ContactIterations is the engine's iteration count; the angular
            % velocities enter the tolerances over a tenth of LengthScale
            tc.requireEngine;
            w = phxEngine('setup');
            tc.addTeardown(@() phxEngine('clear', w));
            phx.engine.Settings("JointTolerance", 2e-3, "ContactTolerance", 5e-3, "JointIterations", 30, ...
                "ContactIterations", 12, "PositionIterations", 3, "LengthScale", 0.01).apply(w);
            s = phxEngine('get', w, uint64(0), 'settings');
            tc.verifyEqual([s.jointtolerance, s.contacttolerance], [2e-3 5e-3], "RelTol", 1e-6);
            tc.verifyEqual(s.tolerancelength, 1e-3, "RelTol", 1e-6);
            tc.verifyEqual([s.iterations, s.jointiterations, s.positioniterations], [12 30 3]);
        end

        function zeroJointToleranceRunsToJointIterations(tc)
            % A swinging pendulum changes its joint impulse in every
            % iteration, so it runs to the cap in every step
            s = phx.engine.Settings("JointTolerance", 0, "JointIterations", 30, "ContactIterations", 12);
            r = tc.runScene(s, "pendulum");
            tc.verifyEqual([r.Iterations, r.MaxIterations], [30 30]);
        end

        function islandsAtTheCapAreCountedByKind(tc)
            % An unmet tolerance runs a jointed island to JointIterations and
            % a contact island to ContactIterations, each counted apart
            s = phx.engine.Settings("JointTolerance", 1e-9, "ContactTolerance", 1e-9, ...
                "JointIterations", 30, "ContactIterations", 12);
            r = tc.runScene(s, "pendulum");
            tc.verifyEqual(r.MaxIterations, 30);
            tc.verifyGreaterThan(r.JointIslandsAtCap, 0);
            tc.verifyEqual(r.ContactIslandsAtCap, 0);
            tc.verifyTrue(isnan(r.ContactResidual), "No contact island, no contact residual");
            r = tc.runScene(s, "box");
            tc.verifyEqual(r.MaxIterations, 12);
            tc.verifyEqual(r.JointIslandsAtCap, 0);
            tc.verifyGreaterThan(r.ContactIslandsAtCap, 0);
            tc.verifyTrue(isnan(r.JointResidual), "No joint island, no joint residual");
        end

        function convergedIslandsEndBeforeTheCap(tc)
            % At the default tolerances a swinging pendulum and a box sliding
            % to rest converge long before their caps
            r = tc.runScene(phx.engine.Settings, "both");
            tc.verifyEqual(r.Islands, 2, "A pendulum and a box");
            tc.verifyLessThan(r.MaxIterations, 50);
            tc.verifyLessThan(r.Iterations, 20);
            tc.verifyEqual([r.JointIslandsAtCap, r.ContactIslandsAtCap], [0 0]);
            tc.verifyLessThanOrEqual([r.JointResidual, r.ContactResidual], 1e-3);
        end
    end

    methods (Access = private)
        function r = runScene(tc, settings, scene)
            % Diagnostics of 0.2 s of a pendulum swinging from 60 degrees,
            % of a box sliding on a static floor at 1 m/s, or of both apart
            tc.requireEngine;
            bodies = phx.Body.empty;
            if scene ~= "box"
                anchor = phx.Body([], "Type", "static", "Position", [0 0 2], "Shape", {"Sphere", "Radius", 0.05});
                bob = phx.Body([], "Position", [sind(60) 0 2 - cosd(60)], "Shape", {"Sphere", "Radius", 0.1});
                phx.RevoluteJoint(anchor, bob, "PointA", [0 0 0], "PointB", [-sind(60) 0 cosd(60)], ...
                    "AxisA", [0 1 0], "AxisB", [0 1 0]);
                bodies = [bodies anchor bob];
            end
            if scene ~= "pendulum"
                floor = phx.Body([], "Type", "static", "Position", [3 0 -0.5], "Shape", {"Box", "Size", [2 2 1]});
                box = phx.Body([], "Position", [3 0 0.25], "LinearVelocity", [1 0 0], ...
                    "Shape", {"Box", "Size", [0.5 0.5 0.5]});
                bodies = [bodies floor box];
            end
            sim = phx.Simulation(bodies, "EngineSettings", settings, "DiagnosticInfo", "warnings");
            tc.addTeardown(@() delete(sim));
            sim.step(0.2, 20, -1);
            r = sim.getDiagnosticReport;
        end
    end

end
