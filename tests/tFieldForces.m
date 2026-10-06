classdef tFieldForces < PhxTestCase
%tFieldForces Pairwise forces of the monopole and dipole field kernels.
%
%   phx.Monopole and phx.Dipole compute their per-step forces in
%   phx.internal.computeMonopoleForces and phx.internal.computeDipoleForces
%   (MEX builds, with .m fallbacks). These tests check the kernels against
%   hand-computed values and against an independent sum over all ordered
%   pairs; the kernels visit each pair once and use Newton's third law.
%   No engine and no graphics are needed.
%
%   See also phx.Monopole, phx.Dipole

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function likeChargesRepelByCoulomb(tc)
            % F1 = Attractivity*q1*q2*(p2 - p1)/r^3; the default
            % Attractivity of -1 pushes like charges apart
            F = phx.internal.computeMonopoleForces([0 2; 0 0; 0 0], [3; 5], -1);
            tc.verifyEqual(F, [-15/4 15/4; 0 0; 0 0], "AbsTol", 1e-14);
        end

        function oppositeChargesAttract(tc)
            F = phx.internal.computeMonopoleForces([0 2; 0 0; 0 0], [3; -5], -1);
            tc.verifyEqual(F, [15/4 -15/4; 0 0; 0 0], "AbsTol", 1e-14);
        end

        function monopoleForceFallsWithDistanceSquared(tc)
            pos = [0 1; 0 1; 0 1]/sqrt(3);
            F1 = phx.internal.computeMonopoleForces(pos, [1; 1], -1);
            F2 = phx.internal.computeMonopoleForces(2*pos, [1; 1], -1);
            tc.verifyEqual(vecnorm(F1(:, 1)), 1, "RelTol", 1e-14);
            tc.verifyEqual(F2, F1/4, "RelTol", 1e-14);
        end

        function monopoleScalesWithAttractivity(tc)
            [pos, q] = tc.randomConfiguration(6);
            F1 = phx.internal.computeMonopoleForces(pos, q, 1);
            F2 = phx.internal.computeMonopoleForces(pos, q, -2.5);
            tc.verifyEqual(F2, -2.5*F1, "RelTol", 1e-14);
        end

        function monopoleMatchesPairSum(tc)
            [pos, q] = tc.randomConfiguration(40);
            F = phx.internal.computeMonopoleForces(pos, q, -1);
            Fref = tFieldForces.pairSum(pos, q, 1:40, -1);
            tc.verifyEqual(F, Fref, "AbsTol", 1e-12*max(abs(Fref), [], "all"));
        end

        function monopoleForcesSumToZero(tc)
            [pos, q] = tc.randomConfiguration(40);
            F = phx.internal.computeMonopoleForces(pos, q, -1);
            tc.verifyEqual(sum(F, 2), [0; 0; 0], "AbsTol", 1e-12*max(abs(F), [], "all"));
        end

        function monopoleSingleAndEmptyGroup(tc)
            tc.verifyEqual(phx.internal.computeMonopoleForces([1; 2; 3], 4, -1), zeros(3, 1));
            tc.verifySize(phx.internal.computeMonopoleForces(zeros(3, 0), zeros(0, 1), -1), [3 0]);
        end

        function ownPolesDoNotInteract(tc)
            % A lone dipole feels nothing: its two poles belong to one body
            F = phx.internal.computeDipoleForces([1 -1; 0 0; 0 0], 2, -1);
            tc.verifyEqual(F, zeros(3, 2));
        end

        function twoCollinearDipoles(tc)
            % Poles on the x axis: body 1 has +1 at 0 and -1 at -1, body 2
            % has +1 at 3 and -1 at 2 (columns: +poles first, then -poles)
            pos = [0 3 -1 2; 0 0 0 0; 0 0 0 0];
            F = phx.internal.computeDipoleForces(pos, [1; 1], -1);
            % e.g. the +pole of body 1: pushed by +3 (-3/27), pulled by -2 (2/8)
            Fx = [-3/27 + 2/8, ...                 % + of body 1
                  3/27 - 1/16, ...                 % + of body 2
                  4/64 - 3/27, ...                 % - of body 1
                  -2/8 + 3/27];                    % - of body 2
            tc.verifyEqual(F, [Fx; zeros(2, 4)], "AbsTol", 1e-14);
        end

        function dipoleMatchesPairSum(tc)
            n = 20;
            [pos, q] = tc.randomConfiguration(2*n);
            q = q(1:n);
            F = phx.internal.computeDipoleForces(pos, q, -1);
            Fref = tFieldForces.pairSum(pos, [q; -q], [1:n 1:n], -1);
            tc.verifyEqual(F, Fref, "AbsTol", 1e-12*max(abs(Fref), [], "all"));
        end

        function dipoleForcesSumToZero(tc)
            n = 20;
            [pos, q] = tc.randomConfiguration(2*n);
            F = phx.internal.computeDipoleForces(pos, q(1:n), -1);
            tc.verifyEqual(sum(F, 2), [0; 0; 0], "AbsTol", 1e-12*max(abs(F), [], "all"));
        end
    end

    methods (Access = private)
        function [pos, q] = randomConfiguration(tc, n)
            % Reproducible scatter with no two points closer than 0.1
            s = RandStream("twister", "Seed", n);
            pos = zeros(3, 0);
            while size(pos, 2) < n
                p = 4*rand(s, 3, 1) - 2;
                if isempty(pos) || min(vecnorm(pos - p)) > 0.1
                    pos(:, end + 1) = p; %#ok<AGROW> small test set
                end
            end
            q = 2*rand(s, n, 1) - 1;
            tc.assertSize(pos, [3 n]);
        end
    end

    methods (Static, Access = private)
        function F = pairSum(pos, q, body, k)
            % Reference: every point against every point of another body
            n = size(pos, 2);
            F = zeros(3, n);
            for i = 1:n
                for j = 1:n
                    if body(i) ~= body(j)
                        d = pos(:, j) - pos(:, i);
                        F(:, i) = F(:, i) + k*q(i)*q(j)*d/norm(d)^3;
                    end
                end
            end
        end
    end

end
