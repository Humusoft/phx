function phxex_grip(mu, squeeze)
% PHXEX_GRIP Picking up an object by friction grip
%
% Two parallel jaws (kinematic bodies) close on a free object and then lift
% it. Nothing connects the object to the jaws - it is held up purely by the
% friction in the two contact patches balancing gravity. Whether the object
% is lifted or slips out of the grip depends only on the clamping force
% (how hard the jaws squeeze) and the friction coefficient of the contacts.
%
% This is a classic robotic grasping problem that has no meaning without a
% contact/collision solver: the grip force is transmitted entirely through
% body-to-body contact, and slipping is decided by the Coulomb friction
% condition at those contacts.
%
% The object's vertical position is measured against the (rising) jaws with
% phx.Measure and logged, so a slip shows up directly as the object falling
% behind the jaws during the lift.
%
% Input Arguments:
%     mu      - friction coefficient between jaws and object
%     squeeze - jaw closing overlap that sets the clamping force
%
% Example:
%     phxex_grip            % firm grip, object is lifted
%     phxex_grip(0.2)       % slippery contact -> object slips out
%     phxex_grip(0.6, 0.0)  % barely touching -> not enough force, slips

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        mu (1, 1) double {mustBeInRange(mu, 0, 1)} = 0.6
        squeeze (1, 1) double = 0.0035
    end

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.4], ...
        "DefaultCameraPosition", [1.2 -1.4 0.8]);

    % Geometry
    objW = 0.2;            % width of the object to be grasped
    objH = 0.4;            % height of the object
    jawW = 0.04;           % thickness of each jaw
    jawGap0 = objW + 0.12; % initial open gap between inner jaw faces

    % Static floor / drop catcher
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [4 4 0.1], "Color", 1});

    % Static table the object initially rests on
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], ...
        "Shape", {"Box", "Size", [1 1 0.1], "Color", 0.9});

    % The object to be grasped (free dynamic body, friction on all faces)
    obj = phx.Body(ax, "Position", [0 0 objH/2], ...
        "Shape", {"Cylinder", "Diameter", objW, "Height", objH, "Style", "edged", ...
                  "Color", [0.4 0.6 1]}, ...
        "Mass", 0.5, "Friction", [mu 0 0]);

    % Two kinematic jaws - we drive their position directly. They hold the
    % object only through contact friction, there is no joint to the object.
    jawL = phx.Body(ax, "Type", "kinematic", "Position", [-(jawGap0/2 + jawW/2) 0 objH/2], ...
        "Shape", {"Box", "Size", [jawW objW*1.5 objH*0.8], "Color", [0.9 0.5 0.3]}, ...
        "Friction", [mu 0 0]);
    jawR = phx.Body(ax, "Type", "kinematic", "Position", [ (jawGap0/2 + jawW/2) 0 objH/2], ...
        "Shape", {"Box", "Size", [jawW objW*1.5 objH*0.8], "Color", [0.9 0.5 0.3]}, ...
        "Friction", [mu 0 0]);

    % Measure the object's height relative to the left jaw. If the grip holds
    % this stays roughly constant during the lift; on slip the object drops.
    slip = phx.Measure(jawL, obj, "PointA", [0 0 0], "PointB", [0 0 0], "Overlay", true);

    % Log jaw height and object height to compare them afterwards.
    logJaw = phx.Logger(jawL, "Frequency", 50, "Parameters", "Position");
    logObj = phx.Logger(obj,  "Frequency", 50, "Parameters", "Position");

    sim = phx.Simulation;

    % Phase 1 - let the object settle on the table
    sim.step(0.4, 20, 1);

    % Target inner-face position when closed: jaw faces overlap the object
    % faces by "squeeze", which sets how hard the object is clamped.
    closedCenter = objW/2 + jawW/2 - squeeze;

    dt = 0.02;

    % Phase 2 - close the jaws symmetrically onto the object
    viewer.displayText("Phase: close");
    openCenter = jawGap0/2 + jawW/2;
    for k = 1:40
        a = k/40;                                   % 0 -> 1
        c = openCenter + (closedCenter - openCenter)*a;
        jawL.Position = [-c 0 objH/2];
        jawR.Position = [ c 0 objH/2];
        sim.step(dt, 10, 1);
        pause(0);
    end

    % Phase 3 - lift both jaws straight up, carrying the object by friction
    viewer.displayText("Phase: lift");
    liftH = 0.6;
    for k = 1:80
        z = objH/2 + liftH*(k/80);
        jawL.Position = [-closedCenter 0 z];
        jawR.Position = [ closedCenter 0 z];
        sim.step(dt, 10, 1);
        viewer.displayText(sprintf("Phase: lift   |   jaw z = %.2f   obj z = %.2f", ...
            z, obj.Position(3)));
        pause(0);
    end

    % Phase 4 - hold and see whether the object stays in the grip
    viewer.displayText("Phase: hold");
    sim.step(0.6, 30, 1);

    delete(sim);

    % Verdict: compare final object height with the jaws.
    jawZ = jawL.Position(3);
    objZ = obj.Position(3);
    if objZ > jawZ - objH      % object roughly level with the jaws
        fprintf("Grip held: object lifted to z = %.2f (mu = %.2f, squeeze = %.3f).\n", ...
            objZ, mu, squeeze);
    else
        fprintf("Grip slipped: object fell to z = %.2f while jaws reached %.2f (mu = %.2f, squeeze = %.3f).\n", ...
            objZ, jawZ, mu, squeeze);
    end

    % Plot jaw vs object height to visualise any slip
    figure(2);
    jp = logJaw.getChannel(1);     % jaw Position [x y z]
    op = logObj.getChannel(1);     % object Position [x y z]
    plot(logJaw.Time, jp(:, 3), "LineWidth", 1.5); hold on;
    plot(logObj.Time, op(:, 3), "LineWidth", 1.5);
    grid on; xlabel("time [s]"); ylabel("height z [m]");
    legend("jaw", "object", "Location", "northwest");
    title(sprintf("Friction grip (mu = %.2f, squeeze = %.3f)", mu, squeeze));

end