function phxex_droptest(dropH, stiffness, damping)
% PHXEX_DROPTEST Package drop test - shock transmitted through an impact
%
% A package is dropped onto a hard floor. Inside the outer shell, a
% "product" mass is suspended on a spring-damper that represents the
% cushioning of the packaging. When the shell hits the floor, the impact
% is resolved as a contact between the falling body and the static floor,
% and the resulting shock travels through the cushioning spring into the
% product.
%
% The event that drives the whole experiment - the collision of the shell
% with the floor - cannot be modelled without a contact/collision solver:
% it is a sudden, non-penetrating impact that abruptly changes the shell's
% velocity, and only then does the spring transmit a (hopefully reduced)
% force to the protected mass.
%
% phx.Logger records the force in the cushioning spring and the height of
% both bodies, so the peak load delivered to the product can be read off
% directly and compared for different cushioning parameters.
%
% Input Arguments:
%     dropH     - drop height of the package above the floor
%     stiffness - cushioning spring stiffness in N/m
%     damping   - cushioning damping in N*s/m
%
% Example:
%     phxex_droptest                 % default cushioning
%     phxex_droptest(2, 5e4, 80)     % stiff packaging -> high peak load
%     phxex_droptest(2, 2e3, 200)    % soft, well-damped -> lower peak load

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        dropH (1, 1) double = 2
        stiffness (1, 1) double = 5e3
        damping (1, 1) double = 80
    end

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 1], ...
        "DefaultCameraPosition", [4 -5 2.5], "ViewMode", "plain");

    % Hard static floor that the package will hit
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.25], ...
        "Shape", {"Box", "Size", [6 6 0.5], "Color", 1}, "Friction", [0.8 0 0]);

    % Outer shell of the package (the body that actually collides)
    shellSize = [1 1 0.6];
    shellMass = 0.5;
    shellZ = dropH + shellSize(3)/2;
    shell = phx.Body(ax, "Position", [0 0 shellZ], ...
        "Shape", {"Box", "Size", shellSize, "Style", "edged", "Color", [0.8 0.6 0.4]}, ...
        "Mass", shellMass, "Friction", [0.8 0 0]);

    % Protected product mass suspended inside the shell. It does NOT collide
    % with the shell or floor - it is held only by the cushioning spring, so
    % the only way a load reaches it is through that spring after the impact.
    prodSize = [0.4 0.4 0.4];
    prodZ = shellZ + 0.5;          % sits above, spring pre-stretched a little
    product = phx.Body(ax, "Position", [0 0 prodZ], ...
        "Shape", {"Box", "Size", prodSize, "Style", "edged", "Color", [0.4 0.6 1]}, ...
        "Mass", 1, "Collisions", false);

    % Cushioning element between shell and product (spring + damper).
    cushion = phx.Spring(shell, product, "Stiffness", stiffness, "Damping", damping, ...
        "FreeLength", 0.5, "PointA", [0 0 shellSize(3)/2], "PointB", [0 0 -prodSize(3)/2], ...
        "Visible", true, "Colormap", "jet", "ColorRange", [0 2e3]);

    % Data logging: force in the cushioning spring and heights of both bodies
    logForce  = phx.Logger(cushion,  "Frequency", 500, "Parameters", "Force");
    logHeight = phx.Logger([shell product], "Frequency", 500, "Parameters", "Position");

    % On-screen readout
    label = uilabel(gcf, "FontSize", 14, "FontColor", [1 0.5 0], ...
        "Position", [20 20 360 40], "Text", "Dropping...");

    % Simulation - fine time step to capture the short impact event
    sim = phx.Simulation;
    dt = 0.001;
    for k = 1:1200
        sim.step(dt, 1, mod(k, 10));   % redraw every 10 steps
        if mod(k, 25) == 0
            label.Text = sprintf("t = %.2f s   |   shell z = %.2f   product z = %.2f", ...
                sim.Time, shell.Position(3), product.Position(3));
            pause(0);
        end
    end
    delete(sim);

    % Peak force transmitted into the product through the cushioning
    F = logForce.getChannel(1);            % spring force [Fx Fy Fz]
    Fmag = sqrt(sum(F.^2, 2));
    [peakF, iPeak] = max(Fmag);
    peakAcc = peakF / 1;                   % product mass = 1 kg -> a = F/m
    fprintf("Peak load on product: %.1f N (~%.1f g) at t = %.3f s.\n", ...
        peakF, peakAcc/9.81, logForce.Time(iPeak));
    fprintf("Cushioning: stiffness = %.0f N/m, damping = %.0f N*s/m.\n", ...
        stiffness, damping);

    % Plot heights and the transmitted force
    figure(2);
    h = logHeight.getChannel(1);           % shell Position
    p = logHeight.getChannel(2);           % product Position
    subplot(2, 1, 1);
    plot(logHeight.Time, h(:, 3), logHeight.Time, p(:, 3), "LineWidth", 1.2);
    grid on; ylabel("height z [m]"); legend("shell", "product");
    title(sprintf("Drop test (k = %.0f N/m, c = %.0f N*s/m)", stiffness, damping));
    subplot(2, 1, 2);
    plot(logForce.Time, Fmag, "LineWidth", 1.2); hold on;
    plot(logForce.Time(iPeak), peakF, "ro");
    grid on; xlabel("time [s]"); ylabel("cushion force [N]");
    title(sprintf("Transmitted load, peak = %.0f N", peakF));

end