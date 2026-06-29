function phxex_wankel(showGraphs)
% PHXEX_WANKEL Wankel engine driven through revolute and gear joints
%
% Housing, shaft and rotor (STL) are coupled by phx.RevoluteJoint and
% phx.GearJoint constraints; a torque spins the shaft and phx.Logger records
% forces and angles.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true
    end

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Generate custom equirectangular background
    % imwrite(phx.extra.equirectHall('Text', {'HUMUSOFT', 'PHX', '^..^', '2026'}, 'Width', 4096), resdir+"HumusoftHall.png");
    
    % Clear the current axes and set up the viewer
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0.4 -0.8 0.4], "DefaultCameraTarget", [0 0 0.15], "Texture", resdir+"HumusoftHall.png");
    
    % Create static base body
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], "Shape", {"Box", "Size", [1 1 0.1], "Color", [1 1 0.9]});
    
    % Create housing body with specified properties
    h = phx.Body(ax, "Position", [0 0 0], "Shape", {resdir+"housing.stl", "Scale", 0.01, "Centered", false, "Color", 0.4, "Style", "flat", "Material", "shiny", "Density", 2700}, "Collisions", true);
    
    % Create shaft body with specified properties
    s = phx.Body(ax, "Position", [0 0 0], "Shape", {resdir+"shaft.stl", "Scale", 0.01, "Centered", false, "Color", [1 0.8 0.5], "Style", "flat", "Density", 7850}, "AxisAngle", [0 0 1 pi/2], "Collisions", false);
    
    % Create rotor body with specified properties
    r = phx.Body(ax, "Position", [0 0.015 0], "Shape", {resdir+"rotor.stl", "Scale", 0.01, "Centered", false, "Color", 1, "Style", "flat", "Density", 7850}, "Collisions", false);
    
    % Create cat body with specified properties
    phx.Body(ax, "Position", [0.2 0 0.2], "Shape", {resdir+"cat.stl", "Scale", 0.005, "Color", [0.9 0.9 1]});
    
    % Group the engine components
    engine = [h s r];
    engine.groupTransform("Translation", [0 0 0.15], "AxisAngle", [1 0 0 pi/2]);
    
    % Create joints between the bodies
    bearing1 = phx.RevoluteJoint(h, s, "Visible", false);
    bearing2 = phx.RevoluteJoint(s, r, "PointA", [0.015 0 0], "Visible", false);
    phx.GearJoint(s, r, "Ratio", -3, "Visible", false);
    log1 = phx.Logger(bearing1, "Frequency", 1000, "Parameters", "ForceA");
    log2 = phx.Logger([bearing1 bearing2], "Frequency", 1000, "Parameters", "Angle");

    % Initialize the simulation
    sim = phx.Simulation;
    
    % Run the simulation for a specified number of steps
    for i = 1:3000
        % Apply torque to the shaft body
        s.applyTorque([0 0 0.1]);
        
        % Step the simulation and update the display every 10 iterations
        sim.step(0.001, 1, mod(i, 10));
    end
    
    % Clean up the simulation object
    delete(sim);

    % Plot recorded data
    if showGraphs
        figure(2);
        subplot(2, 1, 1);
        plot(log1.Time, log1.Data);
        subplot(2, 1, 2);
        plot(log2.Time, log2.Data*180/pi);
    end

end