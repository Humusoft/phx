function phxex_grip(mu, squeeze)
% PHXEX_GRIP Picking up an object by friction grip
%
% Two parallel jaws (kinematic bodies) close on a free object until they
% overlap its faces by squeeze, and then lift it. Nothing connects the
% object to the jaws - it is held up purely by the friction in the two
% contact patches. The clamping force comes from the overlap: the engine
% pushes the object out of both rigid jaws at once, so a fraction of a
% millimetre already clamps it hard and even a low friction carries it.
% Without an overlap, or without friction, the object stays on the table.
%
% The heights of a jaw and of the object are logged, so a slip shows up as
% the object falling behind the jaws during the lift.
%
% Input Arguments:
%     mu      - friction coefficient of the jaws and the object (default 0.6)
%     squeeze - overlap of each closed jaw with the object in m (default 0.0035)
%
% Example:
%     phxex_grip                % firm grip, object is lifted
%     phxex_grip(0.05, 0.0002)  % low friction, light clamp -> slips during the lift
%     phxex_grip(0.6, 0)        % jaws only touch -> no clamping force, stays down
%     phxex_grip(0, 0.001)      % no friction -> the jaws slide up along it

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

    % Static table the object initially rests on
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], ...
        "Shape", {"Box", "Size", [1 1 0.1], "Color", 0.9});

    % The object to be grasped (free dynamic body, friction on all faces)
    obj = phx.Body(ax, "Position", [0 0 objH/2], ...
        "Shape", {"Cylinder", "Diameter", objW, "Height", objH, ...
                  "Color", [0.4 0.6 1]}, ...
        "Mass", 0.5, "Friction", mu);

    % Two kinematic jaws - we drive their velocity directly. They hold the
    % object only through contact friction, there is no joint to the object.
    jawL = phx.Body(ax, "Type", "kinematic", "Position", [-(jawGap0/2 + jawW/2) 0 objH/2], ...
        "Shape", {"Box", "Size", [jawW objW*1.5 objH*0.8], "Color", [0.9 0.5 0.3]}, ...
        "Friction", mu);
    jawR = phx.Body(ax, "Type", "kinematic", "Position", [ (jawGap0/2 + jawW/2) 0 objH/2], ...
        "Shape", {"Box", "Size", [jawW objW*1.5 objH*0.8], "Color", [0.9 0.5 0.3]}, ...
        "Friction", mu);

    % Log jaw height and object height to compare them afterwards.
    logJaw = phx.Logger(jawL, "Frequency", 50, "Parameters", "Position");
    logObj = phx.Logger(obj,  "Frequency", 50, "Parameters", "Position");

    sim = phx.Simulation;

    % Phase 1 - let the object settle on the table
    sim.step(0.4, 20);

    % Target inner-face position when closed: jaw faces overlap the object
    % faces by "squeeze", which sets how hard the object is clamped.
    closedCenter = objW/2 + jawW/2 - squeeze;

    dt = 0.02;

    % Phase 2 - close the jaws symmetrically onto the object in 0.8 s; a
    % kinematic body keeps its velocity until told otherwise
    viewer.displayText("Phase: close");
    openCenter = jawGap0/2 + jawW/2;
    vClose = (openCenter - closedCenter)/0.8;
    jawL.LinearVelocity = [ vClose 0 0];
    jawR.LinearVelocity = [-vClose 0 0];
    sim.step(0.8, 400);

    % Phase 3 - lift both jaws straight up in 1.6 s, carrying the object by
    % friction
    viewer.displayText("Phase: lift");
    liftH = 0.6;
    jawL.LinearVelocity = [0 0 liftH/1.6];
    jawR.LinearVelocity = [0 0 liftH/1.6];
    for k = 1:80
        sim.step(dt, 10);
        viewer.displayText(sprintf("Phase: lift   |   jaw z = %.2f   obj z = %.2f", ...
            jawL.Position(3), obj.Position(3)));
    end

    % Phase 4 - stop the jaws, hold and see whether the object stays in the
    % grip
    viewer.displayText("Phase: hold");
    jawL.LinearVelocity = [0 0 0];
    jawR.LinearVelocity = [0 0 0];
    sim.step(0.6, 30);
    delete(sim);

    % Plot jaw vs object height to visualise any slip
    clf(figure(2));
    jp = logJaw.getChannel(1);     % jaw Position [x y z]
    op = logObj.getChannel(1);     % object Position [x y z]
    plot(logJaw.Time, jp(:, 3), "LineWidth", 1.5); hold on;
    plot(logObj.Time, op(:, 3), "LineWidth", 1.5);
    grid on; xlabel("time [s]"); ylabel("height z [m]");
    legend("jaw", "object", "Location", "northwest");
    title(sprintf("Friction grip (mu = %.2f, squeeze = %.3f)", mu, squeeze));

end