classdef tLogger < PhxTestCase
%tLogger Tests of when phx.Logger takes its samples.
%
%   A Logger samples on the grid n/Frequency, each sample at the simulation
%   step nearest to its grid point. These tests pin that contract down for
%   a recording frequency equal to, below, above and incommensurate with
%   the step rate, and for irregular steps. The bodies are headless, so
%   only the engine is needed.
%
%   See also phx.Logger

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant, Access = private)
        Dt = 1/200      % simulation step of the regular runs
    end

    methods (TestClassSetup)
        function engineIsPresent(tc)
            tc.requireEngine;
        end
    end

    methods (Test, TestTags = {'Engine'})
        function oneRowPerStepAtStepRate(tc)
            % The step time and the sample time are separate float sums;
            % they must not have to match bit for bit
            lg = tc.logSubsteps(1/tc.Dt);
            tc.verifyNumElements(lg.Time, 300);
            tc.verifyEqual(diff(lg.Time), repmat(tc.Dt, 299, 1), "AbsTol", 1e-9);
        end

        function divisorFrequencyIsEvenlySpaced(tc)
            lg = tc.logSubsteps(1/(2*tc.Dt));
            tc.verifyNumElements(lg.Time, 150);
            tc.verifyEqual(diff(lg.Time), repmat(2*tc.Dt, 149, 1), "AbsTol", 1e-9);
        end

        function frequencyAboveStepRateLogsEveryStep(tc)
            lg = tc.logSubsteps(5/tc.Dt);
            tc.verifyNumElements(lg.Time, 300);
        end

        function longStepLeavesNoBacklog(tc)
            % After a step far longer than the recording period, sampling
            % goes on at the recording frequency instead of catching up
            [lg, sim] = tc.makeLogger(10);
            sim.step(0.01);
            sim.step(1);
            for i = 1:20
                sim.step(0.01);
            end
            after = lg.Time(lg.Time > 1);
            tc.verifyGreaterThanOrEqual(diff(after), 0.09);
        end

        function midStepGridPointIsTakenOnce(tc)
            % At 10 Hz steps and 20/3 Hz recording every other grid point
            % (0.15, 0.45, ...) lies exactly mid-step; each is taken once,
            % so 52 steps (last at 5.1 s) give the grid points 0 to 34
            [lg, sim] = tc.makeLogger(20/3);
            for i = 1:52
                sim.step(0.1);
            end
            tc.verifyNumElements(lg.Time, 35);
        end
    end

    methods (Access = private)
        function [lg, sim] = makeLogger(tc, frequency)
            % Falling body logged at the given frequency, in a simulation
            % deleted at teardown
            b = tc.spawnBody([0 0 50]);
            lg = phx.Logger(b, "Parameters", "Position", "Frequency", frequency);
            sim = phx.Simulation(b);
            tc.addTeardown(@() delete(sim));
        end

        function lg = logSubsteps(tc, frequency)
            % 300 steps of Dt, taken as 30 steps of 10 substeps each
            [lg, sim] = tc.makeLogger(frequency);
            for i = 1:30
                sim.step(10*tc.Dt, 10, -1);
            end
        end
    end
end
