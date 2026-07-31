classdef Joint < phx.base.Object
%phx.base.Joint Superclass for custom kinematic joints
%
%   Abstract superclass for kinematic constraints connecting two bodies
%   (A and B). Concrete joints (see below) derive from it and define the
%   actual constraint; this class provides what all of them share:
%
%   - the joint coordinate systems TransformA and TransformB, given in the
%     local space of the respective body, together with the PointA, PointB,
%     AxisA, AxisB, EulerAnglesA, EulerAnglesB, AxisAngleA and AxisAngleB
%     helpers that read and write parts of them;
%   - the reaction force and torque feedback ForceA, TorqueA, ForceB and
%     TorqueB;
%   - the MutualCollisions switch.
%
%   A joint whose constraint has a single distinguished axis (one it rotates
%   about or slides along) should take it as the Z axis of the joint coordinate
%   systems.
%
%   A joint keeps both joint coordinate systems in coincidence, except for the
%   degrees of freedom it leaves free (e.g. the rotation of a revolute joint).
%   If there is a mismatch of coordinate systems in the initial pose (except
%   the free direction), the solver simply pulls the bodies together over the
%   following steps.
%
%   See also phx.RevoluteJoint, phx.PrismaticJoint, phx.CylindricalJoint,
%   phx.SphericalJoint, phx.FixedJoint, phx.GearJoint, phx.GenericJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

% TODO motorized joints

    properties (Access = protected)
        WorldHandle = []
    end

    properties
        % Enable mutual collisions of connected bodies
        MutualCollisions (1, 1) logical = false

        % Transformation matrix relative to the first body
        TransformA (4, 4) double = eye(4)

        % Transformation matrix relative to the second body
        TransformB (4, 4) double = eye(4)
    end

    properties (Dependent)
        % Position of the connecting point in the local space of the first body
        PointA (1, 3) double

        % Position of the connecting point in the local space of the second body
        PointB (1, 3) double

        % Normalized direction of the joint axis in the local space of the first body
        % It only explicitly sets the direction of the Z axis of TransformA,
        % but does not guarantee the overall alignment of both bodies.
        AxisA (1, 3) double

        % Normalized direction of the joint axis in the local space of the second body
        % It only explicitly sets the direction of the Z axis of TransformB,
        % but does not guarantee the overall alignment of both bodies.
        AxisB (1, 3) double

        % Rotation of the connecting point of the first body (Euler angles in z->y->x order)
        EulerAnglesA (1, 3) double

        % Rotation of the connecting point of the second body (Euler angles in z->y->x order)
        EulerAnglesB (1, 3) double

        % Rotation of the connecting point of the first body (axis and angle)
        AxisAngleA (1, 4) double
        
        % Rotation of the connecting point of the second body (axis and angle)
        AxisAngleB (1, 4) double

        % Force acting on the body A
        ForceA (1, 3) double

        % Torque acting on the body A
        TorqueA (1, 3) double

        % Force acting on the body B
        ForceB (1, 3) double

        % Torque acting on the body B
        TorqueB (1, 3) double
    end

    methods
        function set.PointA(obj, value)
            obj.TransformA(13:15) = value;
        end

        function value = get.PointA(obj)
            value = obj.TransformA(13:15);
        end

        function set.PointB(obj, value)
            obj.TransformB(13:15) = value;
        end

        function value = get.PointB(obj)
            value = obj.TransformB(13:15);
        end

        function set.AxisA(obj, value)
            if ~any(value)
                error("phx:Joint:invalidAxis", "AxisA must be a nonzero direction vector.");
            end
            obj.TransformA(1:3, 1:3) = phx.internal.Math.alignZ(obj.TransformA(1:3, 1:3), value);
        end

        function value = get.AxisA(obj)
            value = obj.TransformA(9:11);
        end

        function set.AxisB(obj, value)
            if ~any(value)
                error("phx:Joint:invalidAxis", "AxisB must be a nonzero direction vector.");
            end
            obj.TransformB(1:3, 1:3) = phx.internal.Math.alignZ(obj.TransformB(1:3, 1:3), value);
        end

        function value = get.AxisB(obj)
            value = obj.TransformB(9:11);
        end

        function set.EulerAnglesA(obj, value)
            obj.TransformA(1:3, 1:3) = phx.internal.Math.rot321(value);
        end

        function value = get.EulerAnglesA(obj)
            value = phx.internal.Math.decomp321(obj.TransformA(1:3, 1:3));
        end

        function set.EulerAnglesB(obj, value)
            obj.TransformB(1:3, 1:3) = phx.internal.Math.rot321(value);
        end

        function value = get.EulerAnglesB(obj)
            value = phx.internal.Math.decomp321(obj.TransformB(1:3, 1:3));
        end

        function value = get.AxisAngleA(obj)
            value = phx.internal.Math.decompAA(obj.TransformA(1:3, 1:3));
        end

        function set.AxisAngleA(obj, value)
            obj.TransformA(1:3, 1:3) = phx.internal.Math.rotAA(value(1:3), value(4));
        end

        function value = get.AxisAngleB(obj)
            value = phx.internal.Math.decompAA(obj.TransformB(1:3, 1:3));
        end

        function set.AxisAngleB(obj, value)
            obj.TransformB(1:3, 1:3) = phx.internal.Math.rotAA(value(1:3), value(4));
        end

        function value = get.ForceA(obj)
            if ~isempty(obj.ObjectHandle)
                f = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'feedback')';
                value = f{1}';
            else
                value = [NaN NaN NaN];
            end
        end

        function value = get.TorqueA(obj)
            if ~isempty(obj.ObjectHandle)
                f = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'feedback')';
                value = f{2}';
            else
                value = [NaN NaN NaN];
            end
        end

        function value = get.ForceB(obj)
            if ~isempty(obj.ObjectHandle)
                f = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'feedback')';
                value = f{3}';
            else
                value = [NaN NaN NaN];
            end
        end

        function value = get.TorqueB(obj)
            if ~isempty(obj.ObjectHandle)
                f = phx.engine.io('get', obj.WorldHandle, obj.ObjectHandle, 'feedback')';
                value = f{4}';
            else
                value = [NaN NaN NaN];
            end
        end        
    end

end