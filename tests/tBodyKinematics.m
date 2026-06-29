classdef tBodyKinematics < matlab.unittest.TestCase
%tBodyKinematics Round-trip tests for phx.Body pose properties.
%
%   A body is created in an invisible figure and is never added to a
%   simulation, so no physics engine is involved. The pose setters/getters
%   are pure transform-matrix algebra. A graphics-capable MATLAB session is
%   required because a body owns an hgtransform (tag "Graphics").
%
%   See also phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.

    properties
        Fig
        Body
    end

    methods (TestMethodSetup)
        function makeBody(tc)
            tc.Fig = figure("Visible", "off");
            tc.addTeardown(@() close(tc.Fig));
            tc.Body = phx.Body(axes(tc.Fig));
            tc.addTeardown(@() delete(tc.Body));
        end
    end

    methods (Test, TestTags = {'Graphics'})
        function defaultPoseIsIdentity(tc)
            tc.verifyEqual(tc.Body.Transform, eye(4), "AbsTol", 1e-12);
            tc.verifyEqual(tc.Body.Position, [0 0 0], "AbsTol", 1e-12);
        end

        function positionRoundTrip(tc)
            tc.Body.Position = [1 -2 3];
            tc.verifyEqual(tc.Body.Position, [1 -2 3], "AbsTol", 1e-12);
        end

        function transformRoundTripAndPosition(tc)
            M = eye(4);
            M(1:3, 1:3) = phx.internal.Math.rot321([0.1 0.2 0.3]);
            M(13:15) = [4 5 6];
            tc.Body.Transform = M;
            tc.verifyEqual(tc.Body.Transform, M, "AbsTol", 1e-12);
            tc.verifyEqual(tc.Body.Position, [4 5 6], "AbsTol", 1e-12);
        end

        function orientationRoundTrip(tc)
            R = phx.internal.Math.rotAA([0 0 1], pi/4);
            tc.Body.Orientation = R;
            tc.verifyEqual(tc.Body.Orientation, R, "AbsTol", 1e-12);
        end

        function positionDoesNotDisturbOrientation(tc)
            R = phx.internal.Math.rotAA([0.2 0.5 0.1], 0.6);
            tc.Body.Orientation = R;
            tc.Body.Position = [7 8 9];
            tc.verifyEqual(tc.Body.Orientation, R, "AbsTol", 1e-12);
            tc.verifyEqual(tc.Body.Position, [7 8 9], "AbsTol", 1e-12);
        end
    end

    methods (Test, TestTags = {'Graphics', 'Toolbox'})
        % Euler/Quaternion conversions route through helpers that may need
        % add-on toolboxes (Robotics/Navigation).
        function eulerAnglesRoundTrip(tc)
            ang = [0.2 -0.3 0.5];
            tc.Body.EulerAngles = ang;
            tc.verifyEqual(tc.Body.EulerAngles, ang, "AbsTol", 1e-9);
        end

        function quaternionRoundTrip(tc)
            R = phx.internal.Math.rotAA([0.3 0.2 0.9], 0.8);
            tc.Body.Orientation = R;
            q = tc.Body.Quaternion;
            tc.Body.Orientation = eye(3);   % wipe it
            tc.Body.Quaternion = q;          % restore from quaternion
            tc.verifyEqual(tc.Body.Orientation, R, "AbsTol", 1e-9);
        end
    end

    methods (Test, TestTags = {'Graphics'})
        % Validation itself is engine-free, but the shared setup builds a
        % body (hence a figure), so these stay under the Graphics tag.
        function badTypeIsRejected(tc)
            tc.verifyError(@() set(tc.Body, "Type", "bogus"), ?MException);
        end

        function frictionOutOfRangeIsRejected(tc)
            tc.verifyError(@() set(tc.Body, "Friction", [2 0 0]), ?MException);
        end
    end

end
