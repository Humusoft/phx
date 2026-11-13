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
%   The velocity iterations of a group of bodies (an island: the bodies
%   that touch or are joined) end when its joints and its contacts have
%   converged, each to its own tolerance, and at the latest after
%   JointIterations (a group with a joint) or ContactIterations (any other).
%   The joints of a group (fixed, revolute, prismatic and spherical joints,
%   generic joints with all rotations or all translations locked, motors
%   below their limit, closed loops included) are solved together, directly,
%   in every iteration; the iterations serve the contacts and what the
%   joints share with them. A tolerance is the largest change of a body's velocity that one joint
%   or contact may still make in an iteration, as a fraction of that body's
%   velocity; it is the quantity that getDiagnosticReport of phx.Simulation
%   reports as JointResidual and ContactResidual.
%
%   Properties:
%     JointTolerance          - Velocity tolerance of the joints
%     ContactTolerance        - Velocity tolerance of the contacts
%     JointIterations         - Most velocity iterations of a group of bodies held by a joint
%     ContactIterations       - Most velocity iterations of any other group of bodies
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
        % Velocity tolerance of the joints: the joints of a group of bodies
        % have converged when none of them changed the velocity of one of
        % its bodies in an iteration by more than this fraction of that
        % body's velocity (of at least |g| dt). Smaller is more accurate and
        % takes more iterations; 0 runs every group with a joint to
        % JointIterations, a fixed count.
        JointTolerance (1, 1) double {mustBeNonnegative, mustBeFinite} = 1e-3

        % Velocity tolerance of the contacts, as JointTolerance; 0 runs
        % every group to its iteration count. Heaps rarely meet it and run
        % to ContactIterations; a looser tolerance lets small rolling or
        % tipping bodies drift off their course.
        ContactTolerance (1, 1) double {mustBeNonnegative, mustBeFinite} = 1e-3

        % Most velocity iterations per step of a group of bodies held
        % together by a joint (and of whatever touches them); fewer once it
        % has converged. Raise it when such groups run to it without
        % meeting their tolerance (JointIslandsAtCap of the diagnostics, a
        % JointResidual above JointTolerance) and the result matters: a
        % hard mechanism halves its joint gap with each doubling, one that
        % does not converge at all gains nothing. 0 leaves such groups at
        % ContactIterations.
        JointIterations (1, 1) double {mustBeInteger, mustBeInRange(JointIterations, 0, 255)} = 50

        % Most velocity iterations per step of a group of bodies without a
        % joint (contacts, springs, ropes); fewer once it has converged.
        % Heaps rarely converge and run to it, so it sets their cost.
        ContactIterations (1, 1) double {mustBeInteger, mustBePositive} = 20

        % Position iterations of the constraint solver per step, which push
        % penetrating bodies and drifted joints back after the velocity
        % iterations; the drift of a joint halves with every doubling, for
        % a few per cent of the step
        PositionIterations (1, 1) double {mustBeInteger, mustBePositive} = 4

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

        % Threads of the step; 1 runs it on the MATLAB thread alone and
        % suits scenes of up to about a hundred bodies, about 8 suit
        % hundreds to thousands of bodies, particles or mesh contacts.
        % Results do not depend on the number of threads.
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
        % Rounding radius, in metres, of boxes, cylinders, cones (also those
        % of a Revolution with Envelope "cones") and convex hulls, taken
        % inside the shape (it keeps its size) and limited by
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
            s.jointtolerance = obj.JointTolerance;
            s.contacttolerance = obj.ContactTolerance;
            s.tolerancelength = 0.1*obj.LengthScale;   % turns an angular velocity into a linear one
            s.iterations = obj.ContactIterations;
            s.jointiterations = obj.JointIterations;
            s.positioniterations = obj.PositionIterations;
            s.margin = obj.Margin;
            s.penetrationslop = obj.PenetrationSlop;
            s.speculativedistance = obj.SpeculativeDistance;
            s.threads = obj.Threads;
            s.contactcache = obj.ContactCache;
            phxEngine('set', objectHandle, uint64(0), 'settings', s);
        end
    end

end
