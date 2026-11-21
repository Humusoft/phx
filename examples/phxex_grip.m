function phxex_grip(mu, squeeze, stiffness)
% PHXEX_GRIP Picking up an object by friction grip
%
% Two parallel jaws (kinematic bodies) with compliant rubber pads close on a
% free object until they press into its faces by squeeze, and then lift
% it. Nothing connects the object to the jaws - it is held up purely by the
% friction in the two contact patches. The pads are springs on the contact
% (ContactStiffness), so each jaw clamps with stiffness*squeeze, and the
% grip holds while the friction of both jaws, 2*mu*stiffness*squeeze,
% exceeds the weight and the force that accelerates the object. Without an
% overlap, or without friction, the object stays on the table.
%
% The heights of a jaw and of the object and the contact forces on the
% jaws are logged: a slip shows up as the object falling behind the jaws,
% and the plot of the forces shows the clamping force against
% stiffness*squeeze and the friction carrying the object against its limit.
%
% Input Arguments:
%     mu        - friction coefficient of the jaws and the object (default 0.6)
%     squeeze   - overlap of each closed jaw with the object in m (default 0.0035)
%     stiffness - contact stiffness of the jaw pads in N/m (default 1e4); Inf
%                 makes them rigid, and the clamping force is then whatever
%                 pushes the object out of both jaws at once
%
% Example:
%     phxex_grip                    % firm grip (35 N per jaw), object is lifted
%     phxex_grip(0.6, 0.0003)       % 3 N per jaw: friction 3.6 N < weight 4.9 N, slips
%     phxex_grip(0.6, 0)            % jaws only touch -> no clamping force, stays down
%     phxex_grip(0, 0.001)          % no friction -> the jaws slide up along it
%     phxex_grip(0.6, 0.0035, Inf)  % rigid jaws

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        mu (1, 1) double {mustBeInRange(mu, 0, 1)} = 0.6
        squeeze (1, 1) double = 0.0035
        stiffness (1, 1) double {mustBePositive} = 1e4
    end

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.4], ...
        "DefaultCameraPosition", [1.2 -1.4 0.8]);

    % Geometry
    objW = 0.2;            % width of the object to be grasped
    objH = 0.4;            % height of the object
    objM = 0.5;            % mass of the object
    jawW = 0.04;           % thickness of each jaw
    jawGap0 = objW + 0.12; % initial open gap between inner jaw faces

    % Static table the object initially rests on
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], ...
        "Shape", {"Box", "Size", [1 1 0.1], "Color", 0.9});

    % The object to be grasped (free dynamic body, friction on all faces)
    obj = phx.Body(ax, "Position", [0 0 objH/2], ...
        "Shape", {"Cylinder", "Diameter", objW, "Height", objH, ...
                  "Color", [0.4 0.6 1]}, ...
        "Mass", objM, "Friction", mu);

    % Two kinematic jaws - we drive their velocity directly. They hold the
    % object only through contact friction, there is no joint to the object.
    % Their pads are damped at about half the critical damping of the object
    % between them.
    damping = 0;
    if isfinite(stiffness)
        damping = sqrt(stiffness*objM);
    end
    jaw = {"Type", "kinematic", "Friction", mu, ...
        "ContactStiffness", stiffness, "ContactDamping", damping, ...
        "Shape", {"Box", "Size", [jawW objW*1.5 objH*0.8], "Color", [0.9 0.5 0.3]}};
    jawL = phx.Body(ax, "Position", [-(jawGap0/2 + jawW/2) 0 objH/2], jaw{:});
    jawR = phx.Body(ax, "Position", [ (jawGap0/2 + jawW/2) 0 objH/2], jaw{:});

    % Log the heights of a jaw and of the object, and the contact forces on
    % the jaws (a kinematic body's TotalForce is the sum of its contact forces)
    logJaw = phx.Logger(jawL, "Frequency", 50, "Parameters", "Position");
    logObj = phx.Logger(obj,  "Frequency", 50, "Parameters", "Position");
    logForce = phx.Logger([jawL jawR], "Frequency", 50, "Parameters", "TotalForce");

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
        viewer.displayText(sprintf("Phase: lift   |   jaw z = %.2f   obj z = %.2f   clamp = %.1f N", ...
            jawL.Position(3), obj.Position(3), -jawL.TotalForce(1)));
    end

    % Phase 4 - stop the jaws, hold and see whether the object stays in the
    % grip
    viewer.displayText("Phase: hold");
    jawL.LinearVelocity = [0 0 0];
    jawR.LinearVelocity = [0 0 0];
    sim.step(0.6, 30);
    delete(sim);

    % Plot jaw vs object height to visualise any slip, and the contact forces
    clf(figure(2));
    tiledlayout(2, 1);
    nexttile;
    jp = logJaw.getChannel(1);     % jaw Position [x y z]
    op = logObj.getChannel(1);     % object Position [x y z]
    plot(logJaw.Time, jp(:, 3), "LineWidth", 1.5); hold on;
    plot(logObj.Time, op(:, 3), "LineWidth", 1.5);
    grid on; ylabel("height z [m]");
    legend("jaw", "object", "Location", "northwest");
    title(sprintf("Friction grip (mu = %.2f, squeeze = %.4f m, stiffness = %g N/m)", ...
        mu, squeeze, stiffness));

    nexttile;
    fL = logForce.getChannel(1);   % force of the object on the left jaw
    fR = logForce.getChannel(2);   % ... and on the right jaw
    clamp = -fL(:, 1);             % pushes the left jaw outwards, along -x
    carried = -(fL(:, 3) + fR(:, 3));
    plot(logForce.Time, clamp, "LineWidth", 1.5); hold on;
    plot(logForce.Time, carried, "LineWidth", 1.5);
    plot(logForce.Time, 2*mu*clamp, ":", "LineWidth", 1.5);
    if isfinite(stiffness)
        yline(stiffness*squeeze, "--", "stiffness*squeeze");
    end
    yline(objM*9.81, "--", "weight");
    grid on; xlabel("time [s]"); ylabel("force [N]");
    legend("clamping force of a jaw", "friction carrying the object", ...
        "friction limit 2*mu*clamping", "Location", "northwest");
end
