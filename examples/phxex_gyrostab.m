function phxex_gyrostab(duration, spinRate, maxAmplitude)
% PHXEX_GYROSTAB Gyroscopic roll stabilizer - who keeps the cargo in growing seas?
%
% Two identical floating platforms ride the same beam waves side by side,
% each loaded with four cargo crates on the corners of its deck, held only
% by friction. The left platform is bare; the right one carries a passive
% gyroscopic stabilizer of the kind used on ships: a fast-spinning steel
% disk whose shaft is gimballed to the platform by two revolute joints.
% The disk gets only an initial angular velocity and is never driven
% afterwards.
%
% When a wave rolls the platform, the roll rate crosses the disk's angular
% momentum and the gyroscopic torque swings the gimbal; the reaction of
% that precession opposes the roll. Everything emerges from rigid-body
% dynamics alone, without a single line of control code.
%
% The sea builds up: the wave amplitude ramps from calm to maxAmplitude
% over the run. A crate slides once the roll of its platform exceeds its
% friction angle.
%
% Input Arguments:
%     duration     - simulated time in seconds (default 30)
%     spinRate     - initial disk angular velocity in rad/s (default 150)
%     maxAmplitude - wave amplitude reached at the end of the run (default 0.1)
%
% Example:
%     phxex_gyrostab            % default run
%     phxex_gyrostab(30, 0)     % disk parked, no stabilization

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        duration (1, 1) double {mustBePositive} = 30
        spinRate (1, 1) double {mustBeNonnegative} = 150
        maxAmplitude (1, 1) double {mustBePositive} = 0.1
    end

    platSize = [2 2 0.4];       % platform, density 500 -> floats half draft
    gyroZ = 0.5;                % gimbal center above the platform center
    diskR = 0.4;
    offset = 1.5;               % platform distance from the scene center

    % Cargo: four crates on the deck corners, same relative positions and
    % friction on both platforms (clear of the spinning disk in the middle)
    crateSize = 0.4;
    crateMu = 0.15;
    cratePos = [-0.65 -0.65; -0.65 0.65; 0.65 0.65; 0.65 -0.65];

    % Figure setup
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0 0 0.3], "DefaultCameraPosition", [1 -7 3]);

    % Two identical platforms; the waves travel along y, so both see the
    % same wave phase and roll about their x axis
    platA = phx.Body(ax, "Position", [-offset 0 0], "Shape", {"Box", "Size", platSize, "Density", 500, "Color", 1});
    platB = phx.Body(ax, "Position", [offset 0 0], "Shape", {"Box", "Size", platSize, "Density", 500, "Color", 1});
    plats = [platA platB];

    % Struts carrying the gimbal bearings
    line(platB.Graphics, [0 0 0 0], [-0.6 -0.6 0.6 0.6], [gyroZ 0 0 gyroZ], "LineWidth", 2, "Color", [1 1 1], "Marker", ".", "MarkerSize", 20);

    crates = phx.Body.empty;
    crateShape = phx.shape.Box("Size", crateSize*[1 1 1], "Density", 400, ...
        "Color", [0.85 0.7 0.5], "Texture", "wood", "TextureBlend", 0.3);
    for p = 1:2
        for i = 1:4
            crates(p, i) = phx.Body(ax, "Position", plats(p).Position + [cratePos(i, :), platSize(3)/2 + crateSize/2], "Shape", crateShape, "Friction", [crateMu 0 0]);
        end
    end

    % Gyro stabilizer on platform B: shaft hinged to the platform about
    % the transverse y axis (gimbal), spinning disk hinged to the shaft
    % about its z axis
    shaft = phx.Body(ax, "Position", [offset 0 gyroZ], "Shape", {"Cylinder", "Radius", 0.04, "Height", 0.55, "Density", 2000, "Color", 0.4});
    disk = phx.Body(ax, "Position", [offset 0 gyroZ], "Shape", {"Cylinder", "Radius", diskR, "Height", 0.06, "Density", 7800, "Color", [0.95 0.55 0.5], "Texture", "stripes", "TextureBlend", 0.5});
    line(shaft.Graphics, [0 0], [-0.6 0.6], [0 0], "LineWidth", 2, "Color", [0.4 0.4 0.4]);

    gimbal = phx.RevoluteJoint(platB, shaft, "PointA", [0 0 gyroZ], "PointB", [0 0 0], "AxisA", [0 1 0], "AxisB", [0 1 0]);
    phx.RevoluteJoint(shaft, disk, "PointA", [0 0 0], "PointB", [0 0 0]);

    % The flywheel is only spun up initially, never driven again
    disk.AngularVelocity = [0 0 spinRate];

    % Beam sea along y with a linearly growing amplitude; fallen crates
    % float, so they get the same buoyancy as the platforms
    kw = 4*pi/6;
    ww = sqrt(9.81*kw);
    rampRate = maxAmplitude/duration;
    amp = @(t) min(rampRate*t, maxAmplitude);
    wave = @(x, y, t) amp(t).*sin(kw*y - ww*t);
    phx.Buoyancy([plats crates(:)'], "LevelFunction", wave, ...
        "LinearDamping", 400, "AngularDamping", 20, ...
        "SurfaceSize", [8 8], "SurfaceStep", 0.25);

    % Fine time step: the fast-spinning disk inside a joint chain needs it
    dt = 0.002;
    subSteps = 25;

    % Platform attitudes and the gimbal angle, once per frame
    logRoll = phx.Logger(plats, "Parameters", "EulerAngles", "Frequency", 1/(dt*subSteps));
    logGimbal = phx.Logger(gimbal, "Parameters", "Angle", "Frequency", 1/(dt*subSteps));

    sim = phx.Simulation(ax);

    while sim.Time < duration
        sim.step(dt*subSteps, subSteps, subSteps);
        t = sim.Time;
        viewer.displayText(sprintf(...
            "t = %4.1f s   amplitude %.2f m   roll: bare %+5.1f deg / gyro %+5.1f deg", ...
            t, amp(t), rad2deg(platA.EulerAngles(1)), rad2deg(platB.EulerAngles(1))));
    end
    delete(sim);

    % Roll is the first Euler angle; the log holds those of platA, then platB
    roll = logRoll.Data(:, [1 4]);

    % Roll history of both platforms and the gimbal precession
    clf(figure(2));
    subplot(2, 1, 1);
    plot(logRoll.Time, rad2deg(roll), "LineWidth", 1.2);
    grid on; ylabel("roll [deg]");
    legend("bare platform", "gyro platform");
    title(sprintf("Platform roll in growing seas (spin %.0f rad/s)", spinRate));
    subplot(2, 1, 2);
    plot(logGimbal.Time, rad2deg(logGimbal.Data), "LineWidth", 1.2);
    grid on; xlabel("time [s]"); ylabel("gimbal angle [deg]");
    title("Gimbal precession absorbing the wave roll moment");

end