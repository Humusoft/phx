function phxex_stack(n, mu, tiltRate)
% PHXEX_STACK Stability of a box stack on a tilting pallet
%
% This demo stacks several boxes on a pallet and then slowly tilts the
% pallet (a kinematic body) until the stack slides or topples. The whole
% behaviour - whether the boxes hold together, slip layer by layer, or
% collapse - emerges purely from the contact and friction between the
% individual bodies, which is exactly what a rigid-body collision solver
% is needed for.
%
% The horizontal drift of the top box relative to the pallet is measured
% with phx.Measure and logged, giving an engineering criterion for the
% maximum safe tilt angle (e.g. during transport or palletizing).
%
% Input Arguments:
%     n        - number of stacked boxes (default 6)
%     mu       - friction coefficient between boxes and pallet (default 0.5)
%     tiltRate - pallet tilt rate in deg/s (default 5)
%
% Example:
%     phxex_stack            % default stack, moderate friction
%     phxex_stack(8, 0.3)    % taller stack, slippery surfaces -> slides sooner

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        n (1, 1) double = 6
        mu (1, 1) double {mustBeInRange(mu, 0, 1)} = 0.5
        tiltRate (1, 1) double = 5
    end

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 3], ...
        "DefaultCameraPosition", [18 -18 8]);

    % Box geometry (width depth height) and stacking parameters
    s = [3 3 1];           % size of a single box
    gap = 0.02;            % tiny gap to seat boxes without initial overlap

    % Static ground far below to catch anything that falls off
    phx.Body(ax, "Type", "static", "Position", [0 0 -5], ...
        "Shape", {"Box", "Size", [60 60 1], "Color", [1 1 1]});

    % Kinematic pallet - we drive its tilt directly, the boxes only respond
    % to it through contact (no joints, no constraints holding them).
    pallet = phx.Body(ax, "Type", "kinematic", "Position", [0 0 0], ...
        "Shape", {"Box", "Size", [8 8 0.5], "Color", [0.6 0.4 0.2]}, ...
        "Friction", [mu 0 0]);

    % Build the stack. A small random horizontal offset per box makes the
    % outcome realistic rather than a perfectly balanced ideal column.
    clr = (jet(n) + 1)/2;
    for i = 1:n
        z = 0.25 + (i - 0.5)*s(3) + i*gap;     % center height of i-th box
        off = (rand(1, 2) - 0.5)*0.1;          % small placement imperfection
        box(i) = phx.Body(ax, "Position", [off, z], ...
            "Shape", {"Box", "Size", s, "Style", "edged", "Color", clr(i, :)}, ...
            "Friction", [mu 0 0], "Mass", 1, "Inertia", 0.1);
    end

    % Measure the horizontal drift of the top box w.r.t. the pallet origin.
    drift = phx.Measure(pallet, box(n), "PointA", [0 0 0], "PointB", [0 0 0], ...
        "Overlay", true);

    % Log the pallet tilt angle and the measured drift over time.
    logTilt = phx.Logger(pallet, "Frequency", 50, "Parameters", "EulerAngles");

    % On-screen readout
    label = uilabel(gcf, "FontSize", 18, "FontColor", [1 1 1], ...
        "Position", [20 20 320 40], "Text", "Tilt: 0.0 deg");

    % Simulation: let the stack settle, then tilt the pallet about the X axis.
    sim = phx.Simulation;

    % Phase 1 - settle under gravity so contacts are properly seated
    sim.step(0.5, 25, 1);

    % Phase 2 - progressively tilt the pallet and watch the stack react
    dt = 0.02;                       % time step
    nSteps = 300;                    % total tilt steps
    collapseAngle = NaN;             % tilt angle at which the stack lets go
    driftThreshold = s(1);          % drift of one box width counts as collapse
    for k = 1:nSteps
        tiltDeg = tiltRate * (k*dt);
        tiltRad = tiltDeg * pi/180;

        % Drive the kinematic pallet orientation; boxes follow only via contact
        pallet.EulerAngles = [tiltRad 0 0];

        sim.step(dt, 5, 1);

        % Detect the onset of collapse from the measured top-box drift
        if isnan(collapseAngle) && abs(drift.Distance) > driftThreshold
            collapseAngle = tiltDeg;
        end

        label.Text = sprintf("Tilt: %.1f deg   |   top drift: %.2f", ...
            tiltDeg, drift.Distance);
        pause(0);
    end

    delete(sim);

    % Report and plot the result
    if isnan(collapseAngle)
        fprintf("Stack survived up to %.1f deg tilt (mu = %.2f, n = %d).\n", ...
            tiltDeg, mu, n);
    else
        fprintf("Stack collapsed at ~%.1f deg tilt (mu = %.2f, n = %d).\n", ...
            collapseAngle, mu, n);
    end

    figure(2);
    tiltX = logTilt.getChannel(1);          % EulerAngles channel -> [X Y Z]
    plot(logTilt.Time, tiltX(:, 1)*180/pi, "LineWidth", 1.5);
    grid on; xlabel("time [s]"); ylabel("pallet tilt [deg]");
    title(sprintf("Box stack on tilting pallet (mu = %.2f, n = %d)", mu, n));
    if ~isnan(collapseAngle)
        yline(collapseAngle*180/pi, "--y", "collapse");
    end

end