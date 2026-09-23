function phxex_gears
% PHXEX_GEARS Meshing involute gears driven by an applied torque
%
% Gear bodies extruded from a parametric involute profile spin on parallel
% shafts; a torque applied to one drives the rest purely through collision
% contact.
%
% See also phx.shape.Extrusion, phx.RevoluteJoint

%   Copyright 2026 HUMUSOFT s.r.o.

    % Clear the current axes and set up the viewer
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0 1.2 1.6], "DefaultCameraTarget", [0 0 0.3], "Texture", "Gradient");

    % Create static base body
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.1], "Shape", {"Box", "Size", [2 1 0.2], "Color", [1 1 1]});

    % Define gear parameters. Both gears share one module, which puts their
    % pitch circles tangent midway between the shafts. The 25 deg pressure
    % angle keeps the 7-tooth pinion free of undercut.
    z1 = 7; z2 = 11;         % tooth counts
    d = 0.30;                % half the shaft distance
    m = 4*d/(z1 + z2);       % module
    alpha = 25*pi/180;       % pressure angle
    w = 0.12;                % gear width

    % Offset the wheel by half a pitch when its tooth count is even, so that a
    % tooth gap - not a tooth - faces the pinion
    ph = pi/z2*mod(z2 + 1, 2);

    % Create right static cylinder and dynamic gear body
    b1 = phx.Body(ax, "Type", "static", "Position", [-d 0 0.2], "Shape", {"Cylinder", "Radius", 0.1, "Height", 0.4, "Color", 0.6});
    g1 = phx.Body(ax, "Position", [-d 0 0.1], "Shape", gearShape(m, z1, alpha, 0, w, [0.4 0.5 0.6]));
    phx.RevoluteJoint(b1, g1);

    % Create left static cylinder and dynamic gear body
    b2 = phx.Body(ax, "Type", "static", "Position", [d 0 0.2], "Shape", {"Cylinder", "Radius", 0.1, "Height", 0.4, "Color", 0.6});
    g2 = phx.Body(ax, "Position", [d 0 0.1], "Shape", gearShape(m, z2, alpha, ph, w, [0.5 0.6 0.4]));
    phx.RevoluteJoint(b2, g2, "TargetVelocity", 5, "MaxTorque", 10);

    % Initialize the simulation with specified settings
    opt = phx.engine.BulletSettings("Margin", 0);
    sim = phx.Simulation("EngineSettings", opt);

    % Run the simulation for 500 steps
    for i = 1:500
        sim.step(0.005, 1, 1);
    end
    delete(sim); % Clean up the simulation object

end

% ---------------------------------------------------------------------------
function shape = gearShape(m, z, alpha, phase, width, color)
    % Spur gear as a profile extruded along the shaft axis
    shape = phx.shape.Extrusion("Profile", gearProfile(m, z, alpha, phase), ...
        "Spine", [-width/2 0 0; width/2 0 0], "Axis", "y", "Envelope", "concave", "Color", color);
end

% ---------------------------------------------------------------------------
function P = gearProfile(m, z, alpha, phase)
% Closed outline of an involute spur gear, first tooth centred on +X

    r = m*z/2;                                   % pitch radius
    rb = r*cos(alpha);                           % base radius
    ra = r + m;                                  % tip radius
    rf = r - 1.25*m;                             % root radius
    inv = @(a) tan(a) - a;                       % involute function
    psi = @(R) pi/(2*z) + inv(alpha) - inv(acos(min(rb./R, 1)));   % half tooth angle

    % Sample one flank at an even spacing along the involute itself, which runs
    % R = rb*sqrt(1 + t^2) with arc length s = rb*t^2/2. Spacing the radius
    % instead would crowd the points near the root, where the curve turns
    % fastest. Coarse is deliberate: a finer flank makes the mesh-to-mesh
    % contact worse, not better.
    t0 = sqrt(max((max(rb, rf)/rb)^2 - 1, 0));
    t1 = sqrt((ra/rb)^2 - 1);
    s = linspace(rb*t0^2/2, rb*t1^2/2, 4)';
    R = rb*sqrt(1 + 2*s/rb);
    p = psi(R);
    if rf < rb
        R = [rf; R];
        p = [psi(rb); p];
    end

    % One tooth: rising flank, tip land, falling flank, root land. Both lands
    % drop their end points, which the flanks already carry - repeating a point
    % would put a zero-area triangle into the collision mesh.
    tip = linspace(-p(end), p(end), 3)';
    root = linspace(p(1), 2*pi/z - p(1), 4)';
    one = [R.*cos(-p) R.*sin(-p); ...
           ra*cos(tip(2:end-1)) ra*sin(tip(2:end-1)); ...
           flipud([R.*cos(p) R.*sin(p)]); ...
           rf*cos(root(2:end-1)) rf*sin(root(2:end-1))];

    % Repeat the tooth around the circle and close the contour
    n = size(one, 1);
    P = zeros(z*n + 1, 2);
    for k = 0:z-1
        a = 2*pi*k/z + phase;
        P(k*n + (1:n), :) = one*[cos(a) sin(a); -sin(a) cos(a)];
    end
    P(end, :) = P(1, :);
end
