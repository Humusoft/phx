classdef tResistance < PhxTestCase
%tResistance Tests for the property names of phx.Resistance.
%
%   EnvironmentVelocity and EnvironmentTwist are the former names of
%   EnvironmentLinearVelocity and EnvironmentAngularVelocity. They still
%   read and write the new properties, with the warning
%   phx:Resistance:renamedProperty, until they are removed. Headless: no
%   engine, no graphics.
%
%   See also phx.Resistance

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (TestParameter)
        renamed = struct( ...
            "velocity", {{"EnvironmentVelocity", "EnvironmentLinearVelocity"}}, ...
            "twist", {{"EnvironmentTwist", "EnvironmentAngularVelocity"}})
    end

    methods (Test)
        function formerNameWarnsAndWritesTheNewProperty(tc, renamed)
            [oldName, newName] = renamed{:};
            r = tc.makeResistance;
            tc.verifyWarning(@() set(r, oldName, [1 2 3]), "phx:Resistance:renamedProperty");
            tc.verifyEqual(r.(newName), [1 2 3]);
        end

        function formerNameWarnsAndReadsTheNewProperty(tc, renamed)
            [oldName, newName] = renamed{:};
            r = tc.makeResistance;
            r.(newName) = [4 5 6];
            value = tc.verifyWarning(@() r.(oldName), "phx:Resistance:renamedProperty");
            tc.verifyEqual(value, [4 5 6]);
        end

        function formerNameWarnsInTheConstructor(tc, renamed)
            [oldName, newName] = renamed{:};
            b = tc.spawnBody([0 0 0]);
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));
            r = tc.verifyWarning(@() phx.Resistance(b, oldName, [0 0 1]), ...
                "phx:Resistance:renamedProperty");
            tc.verifyEqual(r.(newName), [0 0 1]);
        end

        function newNamesDoNotWarn(tc)
            b = tc.spawnBody([0 0 0]);
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));
            tc.verifyWarningFree(@() phx.Resistance(b, ...
                "EnvironmentLinearVelocity", [1 0 0], "EnvironmentAngularVelocity", [0 0 1]));
        end
    end

    methods (Access = private)
        function r = makeResistance(tc)
            b = tc.spawnBody([0 0 0]);
            tc.addTeardown(@() PhxTestCase.deleteIfValid(b));
            r = phx.Resistance(b);
        end
    end
end
