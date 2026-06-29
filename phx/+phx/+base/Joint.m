classdef Joint < phx.base.Object
%phx.base.Joint Superclass for custom kinematic joints
%
%   Abstract superclass for kinematic constraints connecting two bodies
%   (A and B). Concrete joints (see below) derive from it and define the
%   actual constraint; this class provides the reaction force and torque
%   feedback common to all of them, exposed through the ForceA, TorqueA,
%   ForceB and TorqueB properties.
%
%   See also phx.RevoluteJoint, phx.SphericalJoint, phx.FixedJoint,
%   phx.GearJoint, phx.GenericJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

% TODO slider constraint
% TODO cylindrical constraint
% TODO general constraint

    properties (Access = protected)
        WorldHandle = []
    end

    properties (Dependent)
        % Force acting on the body A
        ForceA

        % Torque acting on the body A
        TorqueA

        % Force acting on the body B
        ForceB

        % Torque acting on the body B
        TorqueB
    end

    methods
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