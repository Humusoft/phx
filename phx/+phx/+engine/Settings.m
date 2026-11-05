classdef Settings
%phx.engine.Settings Initial settings of the physics engine
%
%   s = phx.engine.Settings(Name, Value, ...) creates the settings that a
%   simulation applies to its physics world when it creates it. Pass them
%   with the EngineSettings option of phx.Simulation:
%
%       sim = phx.Simulation(ax, "EngineSettings", ...
%           phx.engine.Settings("LengthScale", 0.01));
%
%   The settings take effect once, as the world is created; changing the
%   object afterwards does not reach a running simulation. Bodies do not
%   fall asleep (phx.Simulation keeps every body active).
%
%   LengthScale is the typical size of the parts that touch each other, in
%   metres, as an order of magnitude: 1 for parts of about a metre, 0.1 for
%   decimetres, 0.01 for centimetres. The collision tolerances in automatic
%   mode follow from it (Margin 4 %, SpeculativeDistance 2 %,
%   PenetrationSlop 0.1 % of it); setting a tolerance switches it to manual
%   mode, as with the limits of MATLAB axes:
%
%       s = phx.engine.Settings("LengthScale", 0.01);
%       s.SpeculativeDistance        % 2e-4, SpeculativeDistanceMode "auto"
%       s.SpeculativeDistance = 0;   % SpeculativeDistanceMode "manual"
%
%   Properties:
%     Iterations              - Velocity iterations of the constraint solver per step
%     JointIterations         - Velocity iterations of the groups of bodies held by joints
%     PositionIterations      - Position iterations of the constraint solver per step
%     LengthScale             - Typical size of the parts in contact, as an order of magnitude, in metres
%     Margin                  - Rounding of convex shapes and contact distance of meshes, in metres
%     MarginMode              - Margin from LengthScale ("auto") or as set ("manual")
%     PenetrationSlop         - Depth to which resting bodies may sink into each other, in metres
%     PenetrationSlopMode     - PenetrationSlop from LengthScale ("auto") or as set ("manual")
%     SpeculativeDistance     - Distance ahead of a touch at which contacts begin, in metres
%     SpeculativeDistanceMode - SpeculativeDistance from LengthScale ("auto") or as set ("manual")
%     Threads                 - Threads of the step
%     ContactCache            - Reuse the contacts of body pairs that barely moved (off)
%
%   See also phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties
        % Velocity iterations of the constraint solver per step; more
        % iterations make joint chains and stacks stiffer at a proportional
        % cost
        Iterations (1, 1) double {mustBeInteger, mustBePositive} = 10

        % Velocity iterations per step of every group of bodies held together
        % by a joint (and of whatever touches them); 0 leaves them at
        % Iterations. Only jointed mechanisms pay for them: motors hold
        % loads and servoed arms track without ringing (a Kinova arm on a
        % floating pontoon needs 50, a servo rings 0.010 / 0.001 rad at
        % 20 / 50) and loaded chains get stiffer (ten links holding 100 kg
        % at 5 ms steps stretch 9.8 / 3.9 / 1.9 mm at 20 / 50 / 100) at
        % 1.5 / 2.4 times the cost of the mechanism at 50 / 100 compared
        % with 20.
        JointIterations (1, 1) double {mustBeInteger, mustBeInRange(JointIterations, 0, 255)} = 50

        % Position iterations of the constraint solver per step, which push
        % penetrating bodies and drifted joints back after the velocity
        % iterations
        PositionIterations (1, 1) double {mustBeInteger, mustBePositive} = 2

        % Typical size, in metres, of the parts that touch each other, as an
        % order of magnitude: 1 for parts of about a metre, 0.1 for
        % decimetres, 0.01 for centimetres. When the parts differ much, take
        % the smaller of those whose contacts matter. The tolerances in
        % automatic mode follow from it; the default 1 gives the values for
        % metre-sized scenes.
        LengthScale (1, 1) double {mustBePositive, mustBeFinite} = 1

        % Margin from LengthScale ("auto") or the value set ("manual");
        % setting Margin switches it to "manual"
        MarginMode (1, 1) string {mustBeMember(MarginMode, ["auto", "manual"])} = "auto"

        % PenetrationSlop from LengthScale ("auto") or the value set
        % ("manual"); setting PenetrationSlop switches it to "manual"
        PenetrationSlopMode (1, 1) string {mustBeMember(PenetrationSlopMode, ["auto", "manual"])} = "auto"

        % SpeculativeDistance from LengthScale ("auto") or the value set
        % ("manual"); setting SpeculativeDistance switches it to "manual"
        SpeculativeDistanceMode (1, 1) string {mustBeMember(SpeculativeDistanceMode, ["auto", "manual"])} = "auto"

        % Threads of the step; 1 runs it on the MATLAB thread alone, more
        % pay off in scenes of hundreds of bodies
        Threads (1, 1) double {mustBeInteger, mustBePositive} = 1

        % Reuse the contacts of body pairs that moved by less than 1 mm and
        % 2 degrees in a step instead of finding them anew. Faster for
        % large piles at rest (a pile of 500 boxes about 25 % on one thread,
        % 1000 boxes 8 % on eight), but the reused contacts stay glued to
        % both bodies: slowly rolling or turning bodies gain speed (a ball
        % rolling at 0.15 m/s sped up by a third in 10 s) and sliding mesh
        % teeth can wedge.
        ContactCache (1, 1) logical = false
    end

    properties (Dependent)
        % Rounding radius, in metres, of boxes, cylinders, cones and convex
        % hulls, taken inside the shape (it keeps its size) and limited by
        % its dimensions; also the distance at which concave meshes begin to
        % touch each other. In automatic mode 4 % of LengthScale.
        Margin (1, 1) double {mustBeNonnegative}

        % Depth, in metres, to which resting bodies may sink into each other
        % before the position iterations push them apart. In automatic mode
        % 0.1 % of LengthScale.
        PenetrationSlop (1, 1) double {mustBeNonnegative}

        % Distance, in metres, ahead of a touch at which contacts begin, so
        % that bodies closing faster do not pass through each other within
        % a step; a larger distance gives more contacts to solve. In
        % automatic mode 2 % of LengthScale.
        SpeculativeDistance (1, 1) double {mustBeNonnegative}
    end

    properties (Access = private)
        ManualMargin = 0.04
        ManualPenetrationSlop = 0.001
        ManualSpeculativeDistance = 0.02
    end

    methods
        function obj = Settings(Options)
            arguments
                Options.?phx.engine.Settings
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
        end

        function value = get.Margin(obj)
            if obj.MarginMode == "auto"
                value = 0.04*obj.LengthScale;
            else
                value = obj.ManualMargin;
            end
        end

        function obj = set.Margin(obj, value)
            obj.ManualMargin = value;
            obj.MarginMode = "manual";
        end

        function value = get.PenetrationSlop(obj)
            if obj.PenetrationSlopMode == "auto"
                value = 0.001*obj.LengthScale;
            else
                value = obj.ManualPenetrationSlop;
            end
        end

        function obj = set.PenetrationSlop(obj, value)
            obj.ManualPenetrationSlop = value;
            obj.PenetrationSlopMode = "manual";
        end

        function value = get.SpeculativeDistance(obj)
            if obj.SpeculativeDistanceMode == "auto"
                value = 0.02*obj.LengthScale;
            else
                value = obj.ManualSpeculativeDistance;
            end
        end

        function obj = set.SpeculativeDistance(obj, value)
            obj.ManualSpeculativeDistance = value;
            obj.SpeculativeDistanceMode = "manual";
        end

        function apply(obj, objectHandle)
            s.iterations = obj.Iterations;
            s.jointiterations = obj.JointIterations;
            s.positioniterations = obj.PositionIterations;
            s.margin = obj.Margin;
            s.penetrationslop = obj.PenetrationSlop;
            s.speculativedistance = obj.SpeculativeDistance;
            s.threads = obj.Threads;
            s.contactcache = obj.ContactCache;
            phx.engine.io('set', objectHandle, uint64(0), 'settings', s);
        end
    end

end
