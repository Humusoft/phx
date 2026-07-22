function phxex_camvalve
% PHXEX_CAMVALVE Cam-operated engine valve: when does the valve float?
%
% A kinematic cam (a phx.shape.Extrusion of an egg-shaped lobe) presses down
% on a poppet valve (a phx.shape.Revolution) that slides in its guide on a
% phx.PrismaticJoint and is held up against the cam by a phx.Spring. As the
% cam speeds up the spring struggles to keep the follower on the closing
% flank of the lobe; past a critical speed the valve separates from the cam
% and bounces - "valve float". The demo ramps the cam speed up and reports
% the rpm at which the follower first lifts off, then plots the valve lift
% against the low-speed cam profile and the follower gap versus rpm.
%
% See also phx.shape.Extrusion, phx.shape.Revolution, phx.PrismaticJoint, phx.Spring

%   Copyright 2026 HUMUSOFT s.r.o.

    % --- Mechanism parameters -------------------------------------------
    rBase   = 0.045;             % cam base-circle radius (m)
    lift    = 0.020;             % cam nose lift over the base circle (m)
    noseSpan = 2.5;              % angular width of the lobe (rad)
    camThick = 0.030;            % cam width along its axis (m)
    zc      = 0.300;             % cam rotation-axis height (m)

    Lvalve  = 0.140;             % overall valve length, head base to tappet top (m)
    mValve  = 0.080;             % valve mass (kg)

    kSpring = 1500;              % valve-spring stiffness (N/m)
    cSpring = 3;                 % valve-spring damping (N*s/m)
    freeLen = 0.123;             % spring free length -> sets the preload (m)

    % --- Run schedule ---------------------------------------------------
    dt       = 1.0e-4;           % physics timestep (s)
    subFrame = 10;               % substeps between redraws
    tSettle  = 0.01;             % seat the valve on the cam
    rpmCal   = 240;              % calibration speed (low, follower stays seated)
    rpmLo    = 1600;             % ramp start
    rpmHi    = 2600;             % ramp end
    tRamp    = 0.4;              % ramp duration (s)
    gapThresh = 1.5e-3;          % lift-off gap that counts as float (m)

    % --- Scene ----------------------------------------------------------
    clf; view(35, 12); axis("equal"); grid("on"); camlight("headlight");
    axis([-0.12 0.12, -0.12 0.12, 0 0.38]);
    xlabel("X"); ylabel("Y"); zlabel("Z");

    ax = gca;
    [cam, valve, info] = buildScene(ax, rBase, lift, noseSpan, camThick, ...
        zc, Lvalve, mValve, kSpring, cSpring, freeLen);

    % Zero collision margin - the default 40 mm margin dwarfs these
    % centimetre-scale parts and blows the cam/tappet contact apart.
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.BulletSettings("Margin", 0));

    capTop = @() valve.Position(3) + Lvalve;   % tappet-face height (m)

    % --- Settle: hold the cam nose up, let the spring seat the follower --
    cam.EulerAngles = [0 0 0];
    runPhase(sim, cam, 0, 0, tSettle, dt, subFrame);

    % --- Calibration: one slow revolution captures the true lift profile -
    nbins = 720;
    Lref = nan(1, nbins);
    theta = 0;
    wCal = 2*pi*rpmCal/60;
    tCal = 2*pi/wCal * 1.25;                   % a bit more than one rev
    steps = round(tCal/dt);
    for i = 1:steps
        theta = theta + wCal*dt;
        cam.EulerAngles = [theta 0 0];         % spin about world X (the cam axis)
        sim.step(dt, 1, mod(i, subFrame*2) == 0);
        b = mod(floor(mod(theta, 2*pi)/(2*pi)*nbins), nbins) + 1;
        Lref(b) = capTop();
    end
    Lref = fillbins(Lref);
    liftRest = max(Lref);                       % base-circle contact height

    % --- Ramp: speed the cam up and watch for follower lift-off ----------
    % theta runs on continuously from calibration - resetting it would snap
    % the kinematic cam back a full turn and kick the seated valve.
    log = struct("t", [], "rpm", [], "lift", [], "ref", [], "gap", []);
    t = 0; onsetRpm = NaN;
    nStep = round(tRamp/dt);
    for i = 1:nStep
        rpm = rpmLo + (rpmHi - rpmLo)*t/tRamp;
        w = 2*pi*rpm/60;
        theta = theta + w*dt;
        cam.EulerAngles = [theta 0 0];         % spin about world X (the cam axis)
        sim.step(dt, 1, mod(i, subFrame) == 0);
        t = t + dt;

        if mod(i, subFrame) == 0
            b = mod(floor(mod(theta, 2*pi)/(2*pi)*nbins), nbins) + 1;
            ref = Lref(b);
            ct = capTop();
            gap = ref - ct;                     % >0 : follower below the cam
            log.t(end+1) = t; log.rpm(end+1) = rpm;
            log.lift(end+1) = liftRest - ct;    % valve opening (down is positive)
            log.ref(end+1) = liftRest - ref;
            log.gap(end+1) = gap;
            if isnan(onsetRpm) && gap > gapThresh
                onsetRpm = rpm;
            end
            title(ax, sprintf("cam %.0f rpm   valve lift %.1f mm", rpm, log.lift(end)*1e3));
        end
    end
    delete(sim);

    % --- Report ---------------------------------------------------------
    if isnan(onsetRpm)
        fprintf("Valve stayed on the cam up to %.0f rpm (no float detected).\n", rpmHi);
    else
        fprintf("Valve float onset at %.0f rpm (follower lifted off by > %.1f mm).\n", ...
            onsetRpm, gapThresh*1e3);
    end
    fprintf("Cam lift %.1f mm, valve mass %.0f g, spring preload %.0f N.\n", ...
        (liftRest - min(Lref))*1e3, mValve*1e3, kSpring*(freeLen - info.springLen0));

    % --- Plots ----------------------------------------------------------
    figure;
    subplot(2, 1, 1);
    plot(log.t, log.ref*1e3, "--", log.t, log.lift*1e3, "LineWidth", 1.3);
    grid on; ylabel("valve lift [mm]");
    legend("cam profile (low speed)", "actual", "Location", "northwest");
    title("Valve follows the cam until it starts to float");
    subplot(2, 1, 2);
    plot(log.rpm, log.gap*1e3, "LineWidth", 1.3); hold on;
    yline(gapThresh*1e3, ":", "float threshold");
    if ~isnan(onsetRpm)
        xline(onsetRpm, "r", sprintf("%.0f rpm", onsetRpm));
    end
    grid on; xlabel("cam speed [rpm]"); ylabel("follower gap [mm]");
    title("Follower lift-off grows with cam speed");
end

% ---------------------------------------------------------------------------
function [cam, valve, info] = buildScene(ax, rBase, lift, noseSpan, camThick, ...
        zc, Lvalve, mValve, kSpring, cSpring, freeLen)
% Build the cam, the valve on its prismatic guide, and the spring

    % Cam: egg-shaped lobe extruded to a disc, rotating about world X.
    % The 2D profile spans world Y-Z; the straight spine gives the width.
    phi = linspace(0, 2*pi, 145)';
    d = mod(phi - pi/2 + pi, 2*pi) - pi;        % nose centred pointing +Z
    bump = zeros(size(phi));
    m = abs(d) < noseSpan/2;
    bump(m) = 0.5*(1 + cos(pi*d(m)/(noseSpan/2)));
    r = rBase + lift*bump;
    profile = [r.*cos(phi) r.*sin(phi)];
    spine = [-camThick/2 0 0; camThick/2 0 0];
    camShape = phx.shape.Extrusion("Spine", spine, "Profile", profile, ...
        "Envelope", "convex", "Material", "metal", "Color", [0.55 0.57 0.62]);
    cam = phx.Body(ax, "Type", "kinematic", "Position", [0 0 zc], ...
        "Shape", camShape, "Friction", [0.2 0 0]);

    % Poppet valve: revolved head + stem + bucket tappet, sliding along Z.
    % rStem kept >~7 mm so the revolved stem clears Geometry.revolution's
    % absolute 1 mm degenerate-face cull (a thinner stem draws invisible).
    rHead = 0.030; hHead = 0.012; rStem = 0.010; rCap = 0.030; hCap = 0.008;
    vProfile = [0 0; 0 rHead; hHead rHead; hHead rStem; ...
                Lvalve-hCap rStem; Lvalve-hCap rCap; Lvalve rCap; Lvalve 0];
    valveShape = phx.shape.Revolution("Axis", "z", "Profile", vProfile, ...
        "Envelope", "concave", "Style", "flat", "Material", "metal", "Color", [0.8 0.3 0.25]);
    zv = zc - rBase - Lvalve;                    % tappet just touching the base circle
    valve = phx.Body(ax, "Position", [0 0 zv], "Shape", valveShape, "Friction", [0.2 0 0]);
    valve.Mass = mValve;
    valve.Inertia = [1e-4 1e-4 1e-4];            % rotation is locked by the joint

    % Cylinder-head deck: static, both the guide partner and the spring seat.
    zDeck = zv + 0.045;
    deck = phx.Body(ax, "Type", "static", "Position", [0 0 zDeck], ...
        "Shape", {"Cylinder", "Diameter", 0.04 "Height", 0.04, "Color", [0.9 0.7 0.5]});

    % Prismatic guide: sliding axis is the joint-frame local X -> world Z.
    phx.PrismaticJoint(deck, valve, "PointA", [0 0 0], "PointB", [0 0 0], ...
        "EulerAnglesA", [0 -pi/2 0], "EulerAnglesB", [0 -pi/2 0]);

    % Valve spring: seats on the deck, pushes the retainer (and the valve) up.
    pRet = Lvalve - hCap;                         % retainer height on the valve
    phx.Spring(deck, valve, "PointA", [0 0 -0.005], "PointB", [0 0 pRet], ...
        "Stiffness", kSpring, "Damping", cSpring, "FreeLength", freeLen, ...
        "Visible", true, "Colormap", "jet", "ColorRange", [0 80], "Overlay", true);

    info.springLen0 = (zv + pRet) - (zDeck - 0.005);   % nominal spring length
end

% ---------------------------------------------------------------------------
function runPhase(sim, cam, theta0, w, tPhase, dt, subFrame)
% Advance the sim for tPhase seconds with the cam at constant speed w
    n = round(tPhase/dt);
    theta = theta0;
    for i = 1:n
        theta = theta + w*dt;
        cam.EulerAngles = [theta 0 0];
        sim.step(dt, 1, mod(i, subFrame) == 0);
    end
end

% ---------------------------------------------------------------------------
function L = fillbins(L)
% Fill empty angle bins by nearest-neighbour wrap interpolation
    idx = find(~isnan(L));
    if numel(idx) < 2, return; end
    ang = (0:numel(L)-1);
    L = interp1([ang(idx)-numel(L), ang(idx), ang(idx)+numel(L)], ...
                [L(idx), L(idx), L(idx)], ang, "linear");
end
