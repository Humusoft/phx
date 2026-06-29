function phxex_rotmagdip
% PHXEX_ROTMAGDIP Rotating dipole field spinning a bar magnet
%
% Static poles carry sinusoidally modulated phx.Dipole charges, creating a
% rotating field that spins a freely pivoting bar (a phx.RevoluteJoint) much
% like the rotor of a synchronous motor.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [1, 0, 80], "ViewMode", "plain"); % Set camera position and view mode

    % Physical model: create static and dynamic bodies
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"Cylinder", "Diameter", 1 "Height", 7, "Color", 0.9}, "Collisions", false);
    poles(1) = phx.Body(ax, "Type", "static", "Position", [-15 0 0], "EulerAngles", [0 pi/2 0], "Shape", {"Cylinder", "Diameter", 5, "Height", 2}, "Collisions", false);
    poles(2) = phx.Body(ax, "Type", "static", "Position", [7.5 -13 0], "EulerAngles", [0 pi/2 2*pi/3], "Shape", {"Cylinder", "Diameter", 5, "Height", 2}, "Collisions", false);
    poles(3) = phx.Body(ax, "Type", "static", "Position", [7.5 13 0], "EulerAngles", [0 pi/2 4*pi/3], "Shape", {"Cylinder", "Diameter", 5, "Height", 2}, "Collisions", false);
    poles(4) = phx.Body(ax, "Position", [0 0 0], "Shape", {"Box", "Size", [20 5 5], "Density", 30, "Texture", resdir+"arrows.png", "TextureBlend", 0.5, "Color", 0.5}, "Collisions", false);
    
    % Add kinematic joints
    phx.RevoluteJoint(ground, poles(4));

    % Add dipole interaction
    m = phx.Dipole(poles, "Attractivity", -1, "Charge", zeros(1, 4), "Axis", [0 0 1; 0 0 1; 0 0 1; 10 0 0], ...
        "VectorFieldSize", [60 60 0], "VectorFieldStep", 3, "VectorLength", 6, "VectorSegments", 9, "Color", 0.5);
    
    % Simulation
    sim = phx.Simulation;
    phases = [0 120 240];
    for i = 1:300
        a = poles(end).EulerAngles(3)*180/pi;
        m.Charge = [sind(phases - a), 1]*1e3;
        sim.step(0.1, 10, 10); % Step the simulation
    end
    delete(sim); % Clean up the simulation object

end