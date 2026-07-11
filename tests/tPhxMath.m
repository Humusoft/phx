classdef tPhxMath < matlab.unittest.TestCase
%tPhxMath Unit tests for phx.internal.Math and related pure helpers.
%
%   These tests are completely engine-free and toolbox-free; they exercise
%   only deterministic numeric helpers and can run in any MATLAB session.
%
%   See also phx.internal.Math, phx.internal.transformPoint

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function rotAAIsOrthonormal(tc)
            R = phx.internal.Math.rotAA([0.3 -0.7 0.5], 1.2);
            tc.verifyEqual(R*R', eye(3), "AbsTol", 1e-12);
            tc.verifyEqual(det(R), 1, "AbsTol", 1e-12);
        end

        function rotAAKnownZRotation(tc)
            a = pi/3;
            R = phx.internal.Math.rotAA([0 0 1], a);
            expected = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
            tc.verifyEqual(R, expected, "AbsTol", 1e-12);
        end

        function rotAAZeroAxisReturnsIdentity(tc)
            % Degenerate (zero-length) axis must not produce NaNs.
            tc.verifyEqual(phx.internal.Math.rotAA([0 0 0], 1.0), eye(3));
        end

        function rotAAIgnoresAxisLength(tc)
            % The axis is normalized internally, so scaling it must not matter.
            R1 = phx.internal.Math.rotAA([0 0 1], 0.7);
            R2 = phx.internal.Math.rotAA([0 0 5], 0.7);
            tc.verifyEqual(R1, R2, "AbsTol", 1e-12);
        end

        function rot321IsOrthonormal(tc)
            R = phx.internal.Math.rot321([0.2 -0.4 0.9]);
            tc.verifyEqual(R*R', eye(3), "AbsTol", 1e-12);
            tc.verifyEqual(det(R), 1, "AbsTol", 1e-12);
        end

        function rot321MatchesElementaryXRotation(tc)
            ax = 0.5;
            R = phx.internal.Math.rot321([ax 0 0]);
            expected = [1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
            tc.verifyEqual(R, expected, "AbsTol", 1e-12);
        end

        function transformPointMatchesMatrixProduct(tc)
            M = eye(4);
            M(1:3, 1:3) = phx.internal.Math.rot321([0.1 0.2 0.3]);
            M(13:15) = [1 2 3];
            p = [0.5 -0.4 0.9];
            P = phx.internal.transformPoint(M, p);
            expected = (M(1:3, 1:3)*p' + [1; 2; 3])';
            tc.verifyEqual(P, expected, "AbsTol", 1e-12);
        end

        function euler321RoundTrip(tc)
            xyz = [0.2 -0.3 0.5];
            R = phx.internal.Math.rot321(xyz);
            tc.verifyEqual(phx.internal.Math.decomp321(R), xyz, "AbsTol", 1e-9);
        end

        function axisAngleRoundTrip(tc)
            R = phx.internal.Math.rotAA([0.3 0.2 0.9], 0.8);
            aa = phx.internal.Math.decompAA(R);
            R2 = phx.internal.Math.rotAA(aa(1:3), aa(4));
            tc.verifyEqual(R2, R, "AbsTol", 1e-9);
        end

        function euler321GimbalLockRoundTrip(tc)
            % Y = pi/2 makes X and Z coupled; any decomposition must still
            % reproduce the original matrix.
            R = phx.internal.Math.rot321([0.4 pi/2 -0.3]);
            xyz = phx.internal.Math.decomp321(R);
            tc.verifyEqual(phx.internal.Math.rot321(xyz), R, "AbsTol", 1e-9);
        end

        function axisAngleHalfTurnRoundTrip(tc)
            % A 180-degree rotation is the singular case of the extraction.
            R = phx.internal.Math.rotAA([0.3 -0.5 0.8], pi);
            aa = phx.internal.Math.decompAA(R);
            tc.verifyEqual(phx.internal.Math.rotAA(aa(1:3), aa(4)), R, "AbsTol", 1e-9);
        end

        function axisAngleIdentityConvention(tc)
            tc.verifyEqual(phx.internal.Math.decompAA(eye(3)), [0 0 1 0]);
        end

        function quaternionRoundTrip(tc)
            R = phx.internal.Math.rotAA([0.3 0.2 0.9], 0.8);
            q = phx.internal.Math.decompQ(R);
            tc.verifyEqual(norm(q), 1, "AbsTol", 1e-12);
            tc.verifyGreaterThanOrEqual(q(1), 0);
            tc.verifyEqual(phx.internal.Math.rotQ(q), R, "AbsTol", 1e-12);
        end

        function quaternionKnownZRotation(tc)
            a = pi/3;
            R = phx.internal.Math.rotAA([0 0 1], a);
            expected = [cos(a/2) 0 0 sin(a/2)];
            tc.verifyEqual(phx.internal.Math.decompQ(R), expected, "AbsTol", 1e-12);
        end
    end

end
