classdef tZone < matlab.unittest.TestCase
%tZone Tests for phx.Zone spatial detection.
%
%   The detection-logic tests drive body positions by hand and call the
%   public update() method, so they need only graphics (a body owns an
%   hgtransform) and no physics engine. The pipeline tests build a real
%   phx.Simulation - watch-all enumeration, the static-anchor rule, passive
%   zones and seeding on rebuild - and therefore carry the "Engine" tag.
%
%   See also phx.Zone, phx.Measure

%   Copyright 2026 HUMUSOFT s.r.o.

    properties
        Fig
        Ax
        EnterLog
        ExitLog
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.Fig = figure("Visible", "off");
            tc.addTeardown(@() close(tc.Fig));
            tc.Ax = axes(tc.Fig);
            tc.EnterLog = phx.Body.empty;
            tc.ExitLog = phx.Body.empty;
        end
    end

    % --- Detection logic (engine-free, driven via update()) --------------
    methods (Test, TestTags = {'Graphics'})
        function emptyWatchIsCountZero(tc)
            % Watch-all with no simulation resolves to an empty watch set.
            anchor = tc.mkBody([0 0 0], "static");
            z = phx.Zone(anchor, "Size", [2 2 2]);
            z.update();
            tc.verifyEqual(z.Count, 0);
            tc.verifyEmpty(z.Contents);
        end

        function detectsEntryAndExitViaUpdate(tc)
            anchor = tc.mkBody([0 0 0], "static");
            target = tc.mkBody([5 0 0], "dynamic");   % starts outside
            z = phx.Zone(anchor, "Size", [2 2 2], "Bodies", target, ...
                "EnteredFcn", @(zn, b) tc.onEnter(b), ...
                "ExitedFcn",  @(zn, b) tc.onExit(b));

            z.update();                               % still outside
            tc.verifyEqual(z.Count, 0);
            tc.verifyEmpty(tc.EnterLog);

            target.Position = [0 0 0];                % move inside
            z.update();
            tc.verifyEqual(z.Count, 1);
            tc.verifyEqual(z.EnteredCount, 1);
            tc.verifyEqual(z.Contents, target);
            tc.verifyNumElements(tc.EnterLog, 1);
            tc.verifyTrue(tc.EnterLog(1) == target);

            target.Position = [5 0 0];                % move back out
            z.update();
            tc.verifyEqual(z.Count, 0);
            tc.verifyEqual(z.ExitedCount, 1);
            tc.verifyNumElements(tc.ExitLog, 1);
        end

        function anchorExcludedFromWatchSet(tc)
            % The anchor is never its own content, even if explicitly listed.
            anchor = tc.mkBody([0 0 0], "static");
            b = tc.mkBody([0 0 0], "dynamic");
            z = phx.Zone(anchor, "Size", [4 4 4], "Bodies", [anchor b]);
            z.update();
            tc.verifyEqual(z.Count, 1);
            tc.verifyEqual(z.Contents, b);
        end

        function boxBoundaryRespectsSize(tc)
            anchor = tc.mkBody([0 0 0], "static");
            b = tc.mkBody([0 0 0], "dynamic");
            z = phx.Zone(anchor, "Size", [2 2 2], "Bodies", b);   % box [-1,1]^3

            b.Position = [0.99 0 0]; z.update();
            tc.verifyEqual(z.Count, 1);
            b.Position = [1.01 0 0]; z.update();
            tc.verifyEqual(z.Count, 0);
        end

        function positionOffsetsTheZone(tc)
            anchor = tc.mkBody([0 0 0], "static");
            b = tc.mkBody([0 0 0], "dynamic");
            z = phx.Zone(anchor, "Position", [5 0 0], "Size", [2 2 2], "Bodies", b);

            b.Position = [5 0 0]; z.update();
            tc.verifyEqual(z.Count, 1);
            b.Position = [0 0 0]; z.update();
            tc.verifyEqual(z.Count, 0);
        end

        function zoneMovesWithTheAnchor(tc)
            % The zone frame is relative to the anchor, so moving the anchor
            % moves the zone; a fixed body drops out of it.
            anchor = tc.mkBody([0 0 0], "kinematic");
            b = tc.mkBody([0 0 0], "static");
            z = phx.Zone(anchor, "Size", [2 2 2], "Bodies", b);

            z.update();
            tc.verifyEqual(z.Count, 1);               % b at the anchor origin
            anchor.Position = [10 0 0];               % zone travels away from b
            z.update();
            tc.verifyEqual(z.Count, 0);
        end
    end

    % --- Pipeline integration (needs the engine) -------------------------
    methods (Test, TestTags = {'Engine', 'Graphics'})
        function watchAllSkipsStaticUnderStaticAnchor(tc)
            tc.requireEngine;
            floor = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [10 10 1]});
            kin = phx.Body(tc.Ax, "Type", "kinematic", "Position", [0 0 3], ...
                "Shape", {"Sphere", "Diameter", 1});
            stat = phx.Body(tc.Ax, "Type", "static", "Position", [2 0 3], ...
                "Shape", {"Sphere", "Diameter", 1});
            z = phx.Zone(floor, "Position", [0 0 3], "Size", [8 8 4]);
            sim = phx.Simulation([floor kin stat]);
            tc.addTeardown(@() delete(sim));

            sim.step(0.01, 1, -1);
            tc.verifyEqual(z.Count, 1);               % static scenery skipped
            tc.verifyEqual(z.Contents, kin);
        end

        function watchAllKeepsStaticUnderMovingAnchor(tc)
            tc.requireEngine;
            mover = phx.Body(tc.Ax, "Type", "kinematic", "Position", [0 0 3], ...
                "Shape", {"Box", "Size", [1 1 1]});
            stat = phx.Body(tc.Ax, "Type", "static", "Position", [1 0 3], ...
                "Shape", {"Sphere", "Diameter", 1});
            z = phx.Zone(mover, "Size", [8 8 8]);      % anchor moves -> keep static
            sim = phx.Simulation([mover stat]);
            tc.addTeardown(@() delete(sim));

            sim.step(0.01, 1, -1);
            tc.verifyEqual(z.Count, 1);               % the static body stays detectable
            tc.verifyEqual(z.Contents, stat);
        end

        function passiveZoneTalliedByUpdate(tc)
            tc.requireEngine;
            floor = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [10 10 1]});
            movers = phx.Body.empty;
            for i = 1:3
                movers(i) = phx.Body(tc.Ax, "Type", "kinematic", ...
                    "Position", [i-2 0 3], "Shape", {"Sphere", "Diameter", 1}); %#ok<AGROW>
            end
            z = phx.Zone(floor, "Position", [0 0 3], "Size", [8 8 4], ...
                "SimulationOrder", "none");
            sim = phx.Simulation([floor movers]);
            tc.addTeardown(@() delete(sim));

            sim.step(0.01, 1, -1);
            tc.verifyEqual(z.Count, 0);               % passive: not evaluated while stepping
            z.update();
            tc.verifyEqual(z.Count, 3);               % tallied on demand
        end

        function seedSuppressesSpuriousEnter(tc)
            tc.requireEngine;
            floor = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [10 10 1]});
            resident = phx.Body(tc.Ax, "Type", "kinematic", "Position", [0 0 3], ...
                "Shape", {"Sphere", "Diameter", 1});
            z = phx.Zone(floor, "Position", [0 0 3], "Size", [8 8 4], ...
                "EnteredFcn", @(zn, b) tc.onEnter(b)); %#ok<NASGU>
            sim = phx.Simulation([floor resident]);
            tc.addTeardown(@() delete(sim));

            sim.step(0.5, 50, -1);
            tc.verifyEmpty(tc.EnterLog);              % already inside at build -> no enter
            tc.verifyEqual(z.Count, 1);
        end

        function fallingBodyFiresEnterAndExit(tc)
            tc.requireEngine;
            floor = phx.Body(tc.Ax, "Type", "static", "Position", [0 0 0], ...
                "Shape", {"Box", "Size", [10 10 1]});
            faller = phx.Body(tc.Ax, "Position", [0 0 10], ...
                "Shape", {"Sphere", "Diameter", 0.5});
            z = phx.Zone(floor, "Position", [0 0 5], "Size", [4 4 2]); % z in [4,6]
            sim = phx.Simulation([floor faller]);
            tc.addTeardown(@() delete(sim));

            sim.step(3, 600, -1);                     % falls through and lands below
            tc.verifyEqual(z.EnteredCount, 1);
            tc.verifyEqual(z.ExitedCount, 1);
            tc.verifyEqual(z.Count, 0);
        end
    end

    methods (Access = private)
        function requireEngine(tc)
            tc.assumeNotEmpty(which("phx.engine.io"), ...
                "Physics engine (phx.engine.io) is not on the path.");
        end

        function b = mkBody(tc, pos, type)
            arguments
                tc
                pos (1, 3) double
                type (1, 1) string = "dynamic"
            end
            b = phx.Body(tc.Ax, "Position", pos, "Type", type);
        end

        function onEnter(tc, b)
            tc.EnterLog(end + 1) = b;
        end

        function onExit(tc, b)
            tc.ExitLog(end + 1) = b;
        end
    end

end
