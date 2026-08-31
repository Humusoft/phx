classdef tBodyKinematics < PhxTestCase
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
        Body
    end

    methods (TestMethodSetup)
        function makeBody(tc)
            tc.Body = phx.Body(tc.prepareAxes);
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

        % Validation itself is engine-free, but the shared setup builds a
        % body (hence a figure), so these stay under the Graphics tag too.
        function badTypeIsRejected(tc)
            tc.verifyError(@() set(tc.Body, "Type", "bogus"), ?MException);
        end

        function frictionOutOfRangeIsRejected(tc)
            % Only the lower bound is enforced; coefficients above 1 are
            % physically meaningful (e.g. rubber) and deliberately allowed.
            tc.verifyError(@() set(tc.Body, "Friction", [-1 0 0]), ?MException);
        end

        function nonFinitePoseIsRejected(tc)
            % A NaN/Inf pose must never reach the engine: it either crashes
            % the renderer at the next redraw or hangs the whole session.
            poses = {"Position", [NaN 0 0]; "Transform", inf(4); ...
                     "Orientation", nan(3); "AxisAngle", [0 0 1 NaN]; ...
                     "EulerAngles", [0 Inf 0]; "Quaternion", [NaN 0 0 1]};
            for i = 1:size(poses, 1)
                tc.verifyError(@() set(tc.Body, poses{i, 1}, poses{i, 2}), ...
                    "MATLAB:validators:mustBeFinite", poses{i, 1});
            end
        end

        function rejectedPoseLeavesBodyIntact(tc)
            % The setters write Matrix before notifying the engine, so the
            % rejection has to happen in the property validator (which keeps
            % the old value) rather than downstream of the write.
            R = phx.internal.Math.rotAA([0 0 1], pi/3);
            tc.Body.Orientation = R;
            tc.Body.Position = [1 2 3];
            tc.verifyError(@() set(tc.Body, "Position", [NaN 0 0]), ?MException);
            tc.verifyError(@() set(tc.Body, "Orientation", nan(3)), ?MException);
            tc.verifyEqual(tc.Body.Position, [1 2 3], "AbsTol", 1e-12);
            tc.verifyEqual(tc.Body.Orientation, R, "AbsTol", 1e-12);
            tc.verifyTrue(all(isfinite(tc.Body.Transform(:))));
        end

        function nonFiniteGroupTransformIsRejected(tc)
            other = phx.Body(tc.Body.ParentAxes, "Position", [1 0 0]);
            tc.addTeardown(@() delete(other));
            group = [tc.Body other];
            tc.verifyError(@() groupTransform(group, "Translation", [NaN 0 0]), ?MException);
            tc.verifyError(@() groupTransform(group, "EulerAngles", [0 Inf 0]), ?MException);
            tc.verifyTrue(all(isfinite(tc.Body.Transform(:))));
            tc.verifyTrue(all(isfinite(other.Transform(:))));
            groupTransform(group, "Translation", [0.5 0 0]);   % still usable
            tc.verifyEqual(other.Position, [1.5 0 0], "AbsTol", 1e-12);
        end
    end

end
