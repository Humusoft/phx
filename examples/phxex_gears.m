function phxex_gears
% PHXEX_GEARS Meshing STL gears driven by an applied torque
%
% Gear bodies imported from STL spin on parallel shafts; a torque applied to
% one drives the rest purely through collision contact.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Clear the current axes and set up the viewer
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0 1.2 1.6], "DefaultCameraTarget", [0 0 0.3], "Texture", "defaultGradient");

    % Create static base body
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.1], "Shape", {"Box", "Size", [2 1 0.2], "Color", [1 1 1]});

    % Define parameters for gear shapes
    s = 0.004; % Scale factor for STL models
    d = 0.28;  % Distance from the center for gear positioning

    % Create right static cylinder and dynamic gear body from STL file
    phx.Body(ax, "Type", "static", "Position", [-d 0 0.2], "Shape", {"Cylinder", "Radius", 0.09, "Height", 0.4, "Color", 0.6}, "Friction", 0);
    g1 = phx.Body(ax, "Position", [-d 0 0.1], "Shape", {resdir+"Z7.stl", "Scale", s, "Color", [0.4 0.5 0.6], "Envelope", "concavef", "Details", 0.25, "Style", "flat"});

    % Create left static cylinder and dynamic gear body from STL file
    phx.Body(ax, "Type", "static", "Position", [d 0 0.2], "Shape", {"Cylinder", "Radius", 0.09, "Height", 0.4, "Color", 0.6});
    phx.Body(ax, "Position", [d 0 0.1], "Shape", {resdir+"Z11.stl", "Scale", s, "Color", [0.5 0.6 0.4], "Envelope", "concavef", "Details", 0.25, "Style", "flat"});

    % Initialize the simulation with specified settings
    opt = phx.engine.BulletSettings("Margin", 0.001);
    sim = phx.Simulation("EngineSettings", opt);

    % Run the simulation for 500 steps
    for i = 1:500
        g1.applyTorque([0 0 60]); % Apply torque to the gear
        sim.step(0.005, 1, 1);
    end
    delete(sim); % Clean up the simulation object

end