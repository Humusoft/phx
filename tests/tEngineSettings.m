classdef tEngineSettings < PhxTestCase
%tEngineSettings Contract of phx.engine.Settings: LengthScale and the modes.
%
%   The collision tolerances follow LengthScale in automatic mode (Margin
%   4 %, SpeculativeDistance 2 %, PenetrationSlop 0.1 % of it) and keep a
%   value set to them in manual mode, as the limits of MATLAB axes do with
%   XLim and XLimMode. Settings is a value class, so these tests need
%   neither the physics engine nor a graphics session.
%
%   See also phx.engine.Settings

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
    end

end
