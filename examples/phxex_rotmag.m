function phxex_rotmag
% PHXEX_ROTMAG Rotating monopole field dragging a jointed linkage
%
% Static poles set in a ring carry sinusoidally modulated phx.Monopole charges,
% producing a rotating force field that drives a linkage built from revolute
% and spherical joints.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [1, 0, 80], "ViewMode", "plain"); % Set camera position and view mode

    % Physical model: create static and dynamic bodies
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Cylinder", "Diameter", 1 "Height", 7, "Color", 0.9}, "Collisions", false);

    % Create static poles
    r = 16;
    N = 6;
    for i = 1:N
        a = (i - 1)*2*pi/N;
        poles(i) = phx.Body(ax, "Type", "static", "Position", [cos(a)*r sin(a)*r 0], "EulerAngles", [0 pi/2 a], "Shape", {"Cylinder", "Diameter", 5});
    end
    poles(end + 1) = phx.Body(ax, "Position", [-10 0 0], "Shape", {"Box", "Size", 5, "Color", [0.5 0.5 1], "Density", 100});
    poles(end + 1) = phx.Body(ax, "Position", [+10 0 0], "Shape", {"Box", "Size", 5, "Color", [1 0.5 0.5], "Density", 100});
    
    % Add kinematic joints
    phx.RevoluteJoint(ground, poles(end - 1), "PointB", [+10 0 0]);
    phx.RevoluteJoint(ground, poles(end), "PointB", [-10 0 0]);
    phx.SphericalJoint(poles(end - 1), poles(end), "PointA", [10 2 0], "PointB", [-10 2 0]);

    % Add monopole interaction
    m = phx.Monopole(poles, "Attractivity", -1, "Charge", zeros(size(poles)), ...
        "VectorFieldSize", [60 60 0], "VectorFieldStep", 3, "VectorLength", 6, ...
        "VectorSegments", 9, "Color", 0.5);
    
    % Simulation
    sim = phx.Simulation;
    phases = 0:360/N:359;
    for i = 1:300
        a = poles(end).EulerAngles(3)*180/pi;
        m.Charge = [sind(phases - a), +1 -1]*1e3;
        %m.Charge = [1, -1, 1, -1, 1, -1, -cosd([0 180] + a*3)]*1e3;
        sim.step(0.1, 10, 10); % Step the simulation
    end
    delete(sim); % Clean up the simulation object

end