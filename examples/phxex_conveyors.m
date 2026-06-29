function phxex_conveyors
% PHXEX_CONVEYORS Conveyor line driven by phx.Script automation
%
% A kinematic piston, carousel and belt are animated with phx.Script (time
% expressions and interpolated motion) to move imported STL bottles along a line.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    viewer = phx.extra.Viewer("clear", "DefaultCameraPosition", [0.5 -3.8 1.8], "DefaultCameraTarget", [0.5 -1 0], "Texture", resdir+"HumusoftHall.png");

    % Add a static box shape to the simulation
    phx.Body("Type", "static", "Position", [0 0 -0.5], "Shape", {"Box", "Size", [4 4 0.1], "Texture", resdir+"tiles.jpg"});
    phx.Body("Type", "static", "Position", [0 0 0], "Shape", {"Box", "Size", [2 0.4 0.1], "Color", 0.9});
    phx.Body("Type", "static", "Position", [0 0.22 0.1], "Shape", {"Box", "Size", [2 0.04 0.2], "Color", 1});
    phx.Body("Type", "static", "Position", [0 -0.22 0.1], "Shape", {"Box", "Size", [2 0.04 0.2], "Color", [1 1 1], "Style", "wireframe"});
    piston = phx.Body("Type", "kinematic", "Position", [-1 0 0.2], "Shape", {"Cylinder", "Diameter", 0.3, "Height", 0.1, "Axis", "x", "Color", [0.8 1 0.7]});
    carousel = phx.Body("Type", "kinematic", "Position", [1.5 0 0], "Shape", {"Cylinder", "Diameter", 1, "Height", 0.1, "Axis", "z", "Color", [1 0.5 1], "Texture", resdir+"checker2.png", "TextureBlend", 0.2});
    phx.Body("Type", "static", "Position", [1.5 -0.0 0.15], "EulerAngles", [0 0 0.4], "Shape", {"Box", "Size", [0.05 1 0.2], "Color", [1 0.5 0.5]});
    phx.Body("Type", "static", "Position", [1.5 -0.7 -0.16], "EulerAngles", [pi/4 0 0], "Shape", {"Box", "Size", [0.8 0.5 0.01], "Color", [1 0.7 0.6]});
    belt = phx.Body("Type", "kinematic", "Position", [1, -1.3, -0.4], "Shape", {"Box", "Size", [2 0.8 0.05], "Texture", resdir+"arrows.png"});

    % Automation scripts
    phx.Script(carousel, {"EulerAngles", "[0 0 2*pi*t*0.2]"}); % time dependent expression
    phx.Script(piston, {"Position", [0; 4; 5], [-1 0 0.2; 0.6 0 0.2; -1 0 0.2], 'linear', 'repeat'}); % interpolated curve
    phx.Script(belt, {"Position", [0; 0.01; 0.02;], [1 -1.3 -0.4; 0.998 -1.3 -0.4; 1 -1.3 -0.4], 'nearest', 'repeat'}, {"Friction", [0; 0.01; 0.02], [0; 1; 0], 'nearest', 'repeat'});

    % Import an STL model as a shape
    stl = phx.shape.STL("Source", resdir+"bottle.stl", "Scale", 0.01, "Color", [0.4 0.6 1], "Material", "shiny");

    % Create the simulation object
    sim = phx.Simulation;

    % Add bottles and run the simulation repeatedly
    for i = 1:6
        for j = 1:10
            stl.Color = [0.4 0.6 0.8] + sin(j)*0.2;
            newBottles(j) = phx.Body("Position", [-0.9+j*0.1 sin(j)*0.05 0.3], "Shape", stl);
        end
        viewer.displayText("Added "+numel(newBottles)+" bottles", "below");
        sim.addObjects(newBottles);
        sim.step(5, 500, 5);
    end

    % Clean up by deleting the simulation object
    delete(sim);

end