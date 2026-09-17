function phxex_bowlfeeder(Options)
% PHXEX_BOWLFEEDER Parts climbing the spiral track of a vibratory bowl feeder
%
% A bowl feeder moves parts uphill with nothing but vibration. The bowl shakes
% in a torsional mode - a twist about its axis and a lift of the same frequency,
% in phase - so every point of the helical track runs along a short line
% inclined to the track surface. Parts tipped onto the floor at random find the
% track by themselves and ride it up to the rim, where a phx.Zone counts them
% and a flap deflects them back onto the floor, so the bowl never runs dry.
%
% Whether they ride at all is set by the throw number Gamma = Az*omega^2/g, the
% peak vertical acceleration of the track in units of g. Below one the parts
% only shuffle in place and the rise of the track cancels the feed; above one
% the track drops away from under them once per cycle and they climb.
%
% Options:
%     Gamma     - throw number of the drive
%     Parts     - number of parts in the bowl
%     Frequency - drive frequency in Hz
%     Duration  - length of the run
%     Seed      - random seed of the initial charge
%
% Example:
%     phxex_bowlfeeder
%     phxex_bowlfeeder(Gamma = 0.8)     % below the threshold, nothing climbs
%
% See also phxex_drum, phxex_screwconv, phxex_conveyors

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        Options.Gamma (1, 1) double {mustBePositive} = 2.5
        Options.Parts (1, 1) double {mustBePositive, mustBeInteger} = 32
        Options.Frequency (1, 1) double {mustBePositive} = 15
        Options.Duration (1, 1) double {mustBePositive} = 20
        Options.Seed (1, 1) double = 7
    end

    b = bowlDimensions;
    f = Options.Frequency;
    Az = Options.Gamma*9.81/(2*pi*f)^2;             % lift amplitude for this Gamma

    fprintf("bowl radius %.0f mm, track %.0f mm wide, pitch %.0f mm over %.1f turns\n", ...
        1000*b.R, 1000*b.Width, 1000*b.Pitch, b.Turns);
    fprintf("drive %g Hz, Gamma %.1f -> lift %.2f mm, twist %.2f mm at the track\n", ...
        f, Options.Gamma, 1000*Az, 1000*Az/tand(b.Beta));

    rng(Options.Seed);
    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "Texture", "gradient", "DefaultCameraPosition", [-0.4 0.8 0.9], "DefaultCameraTarget", [0 0 0.1]);
    
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 b.Base], "Shape", {"Box", "Size", [2 2 0.02], "Texture", "tiles"});
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.20], "Shape", {"Cylinder", "Diameter", 0.34, "Height", 0.16, "Color", [0.30 0.33 0.37]});

    [~, outlet] = buildFeeder(ax, b, f, Az, Options.Parts, ground);

    sim = phx.Simulation(ax, "EngineSettings", phx.engine.BulletSettings("Margin", 0.0005));

    dt = 1/(f*100);                                 % 100 substeps per drive period
    window = 0.2;
    sim.step(0.3, round(0.3/dt), -1);               % settle the charge before shaking

    % The flap sends every part that reaches the top back to the floor, so the
    % zone keeps counting passes for as long as the machine runs.
    for time = window:window:Options.Duration
        sim.step(window, round(window/dt), 20);
        viewer.displayText(sprintf("Gamma = %.1f   |   %d parts through the outlet   |   %.0f/min", ...
            Options.Gamma, outlet.EnteredCount, 60*outlet.EnteredCount/time));
    end

    fprintf("%d parts through the outlet in %g s = %.0f/min\n", ...
        outlet.EnteredCount, Options.Duration, 60*outlet.EnteredCount/Options.Duration);
    delete(sim);
end

function b = bowlDimensions
% Every dimension of the machine, in metres.
    b.R = 0.25;             % inner radius of the wall, also the track's outer edge
    b.Height = 0.27;        % wall height
    b.Thickness = 0.008;    % sheet thickness of the bowl
    b.Cone = 0.06;          % height of the floor cone, which sheds parts to the wall
    b.Width = 0.055;        % track width
    b.Pitch = 0.09;         % track rise per turn
    b.Turns = 2.8;
    b.Bank = deg2rad(2);    % track tilted down towards the wall
    b.Ledge = 0.010;        % track plate thickness
    b.Ramp = 0.35;          % turns over which the track grows to full width
    b.Beta = 25;            % angle of the drive line to the track surface, degrees
    b.Part = [0.036 0.018 0.007];
    b.Mass = 0.02;
    b.Base = -0.30;         % height of the floor the machine stands on
end

function [parts, outlet] = buildFeeder(ax, b, f, Az, nParts, ground)
% Floor cone, wall and track are three kinematic bodies shaken by one
% phx.Function. The charge is tipped onto the floor at random.

    floorCone = phx.Body(ax, "Type", "kinematic", "Friction", 0.5, "Position", [0 0 b.Cone/2], ...
        "Shape", {"Cone", "Diameter", 2*b.R, "Height", b.Cone, "Segments", 64, ...
                  "Color", [0.62 0.60 0.36]});
    bowlWall = phx.Body(ax, "Type", "kinematic", "Friction", 0.5, "Position", [0 0 b.Height/2], ...
        "Shape", {"Tube", "InnerDiameter", 2*b.R, "WallThickness", b.Thickness, ...
                  "Height", b.Height, "Segments", 64, "Color", 0.9});

    % The track is the profile below swept along the helix of the spine
    th = linspace(0, 2*pi*b.Turns, round(72*b.Turns) + 1)';
    spine = [b.R*cos(th) b.R*sin(th) b.Pitch*th/(2*pi)];
    drop = b.Width/2*tan(b.Bank);                   % outer edge sits this much lower
    profile = [0 -drop; b.Width drop; b.Width drop-b.Ledge; 0 -drop-b.Ledge; 0 -drop];
    widen = min(1, th/(2*pi*b.Ramp));

    track = phx.Body(ax, "Type", "kinematic", "Friction", 0.5, ...
        "Shape", {"Extrusion", "Spine", spine, "Profile", profile, ...
                  "Scale", [widen ones(size(widen))], "Axis", "z", ...
                  "Envelope", "concave", "Color", 0.3, "Material", "matte"});

    bowl = [floorCone bowlWall track];

    inertia = b.Mass/12*[b.Part(2)^2 + b.Part(3)^2, b.Part(1)^2 + b.Part(3)^2, ...
        b.Part(1)^2 + b.Part(2)^2];
    parts = phx.Body.empty;
    for i = 1:nParts
        parts(i) = phx.Body(ax, "Position", poolPose(b), "EulerAngles", 2*pi*rand(1, 3), ...
            "Mass", b.Mass, "Inertia", inertia, "Friction", 0.5, ...
            "Shape", {"Box", "Size", b.Part, "Color", [0.85 0.5 0.15]});
    end

    % Counting zone
    ang = 2*pi*(b.Turns - 0.08);
    rc = b.R - b.Width/2;
    outlet = phx.Zone(ground, "Bodies", parts, "Size", 0.1, "Position", [rc*cos(ang) rc*sin(ang) b.Pitch*ang/(2*pi) - b.Base + 0.04]);

    % Deflection flap
    ang = 2*pi*(b.Turns + 0.01);
    phx.Body("Type", "static", "Position", [rc*cos(ang) rc*sin(ang) b.Pitch*ang/(2*pi) + 0.02], "EulerAngles", [0 0 1], "Shape", {"Box", "Size", [0.09 0.005 0.03], "Color", [0.9 0.2 0.2]});

    drive = phx.Function(num2cell(bowl), @shake);
    drive.UserData = struct("Az", Az, "Aphi", Az/tand(b.Beta)/b.R, "Omega", 2*pi*f, ...
        "Start", 0.3, "Rest", {arrayfun(@(o) o.Transform, bowl, "UniformOutput", false)});
end

function shake(obj, parents, ~, time)
% Twist about the bowl axis and lift, peaking together, which tilts the motion
% of every point of the track by Beta out of the track surface.
    u = obj.UserData;
    if time < u.Start
        return
    end
    phi = u.Aphi*sin(u.Omega*(time - u.Start));
    T = [cos(phi) -sin(phi) 0 0; sin(phi) cos(phi) 0 0; ...
         0 0 1 u.Az*sin(u.Omega*(time - u.Start)); 0 0 0 1];
    for i = 1:numel(parents)
        parents{i}.Transform = T*u.Rest{i};
    end
end

function p = poolPose(b)
    radius = 0.04 + 0.12*sqrt(rand);
    angle = 2*pi*rand;
    p = [radius*cos(angle) radius*sin(angle) b.Cone*(1 - radius/b.R) + 0.03 + 0.03*rand];
end