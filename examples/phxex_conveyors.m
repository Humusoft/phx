function phxex_conveyors
% PHXEX_CONVEYORS Conveyor line driven by phx.Script automation
%
% Batches of imported STL bottles are carried along two kinematic belts
% and funnelled by a pair of static guide flaps into a bin at the end of
% the line. The belts do not move as bodies - each one is a kinematic box
% that phx.Script shifts forward by one timestep of travel and snaps back
% again, with friction scripted on for the forward move only, so contact
% drags the bottles along at the requested belt speed.
%
% See also phx.Script, phx.shape.Mesh, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.

    % Default values
    nBatches = 6;
    nBottles = 12;
    vBelt1 = 0.1;
    vBelt2 = 0.2;
    flapAngle = 38;
    bottleSizePerc = 100;

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    viewer = phx.extra.Viewer("clear", "DefaultCameraPosition", [3.8 -2.6 2.4], "DefaultCameraTarget", [1 -0.2 0], "Texture", resdir+"HumusoftHall.png");

    % Static bodies
    phx.Body("Type", "static", "Position", [-0.5 0 -0.05], "Shape", {"Box", "Size", [4 4 0.1], "Texture", "tiles"});
    phx.assembly.arena("Position", [2 0 -0.6], "Size", [1 1 0.5]);
    flapAngle = flapAngle*pi/180;
    phx.Body("Type", "static", "Position", [-0.3 0.3 0.15], "EulerAngles", [0 0 -flapAngle], "Shape", {"Box", "Size", [0.6 0.05 0.15], "Color", 1}, "Friction", 0.1);
    phx.Body("Type", "static", "Position", [-0.3 -0.3 0.15], "EulerAngles", [0 0 flapAngle], "Shape", {"Box", "Size", [0.6 0.05 0.15], "Color", 1}, "Friction", 0.1);

    % Kinematic bodies
    belt1 = phx.Body("Type", "kinematic", "Position", [-1 0 0.02], "EulerAngles", [0 0 pi], "Shape", {"Box", "Size", [2 0.8 0.04], "Color", [0 0 1], "Texture", resdir+"arrows.png", "TextureBlend", 0.7});
    belt2 = phx.Body("Type", "kinematic", "Position", [1 0 0.02], "EulerAngles", [0 0 pi], "Shape", {"Box", "Size", [2 0.8 0.03], "Color", [1 0 0], "Texture", resdir+"arrows.png", "TextureBlend", 0.7});
    piston = phx.Body("Type", "kinematic", "Position", [1 0.9 0.2], "Shape", {"Cylinder", "Diameter", 0.3, "Height", 1, "Axis", "y", "Color", [0.8 1 0.7]});

    % Automation scripts
    dt = 0.01;
    dx1 = vBelt1*dt;
    dx2 = vBelt2*dt;
    phx.Script(belt1, {"Position", [0; 0.01; 0.02;], [-1 0 0.02; -1+dx1 0 0.02; -1 0 0.02], 'nearest', 'repeat'}, {"Friction", [0; 0.01; 0.02], [0; 1; 0], 'nearest', 'repeat'});
    phx.Script(belt2, {"Position", [0; 0.01; 0.02;], [1 0 0.02; 1+dx2 0 0.02; 1 0 0.02], 'nearest', 'repeat'}, {"Friction", [0; 0.01; 0.02], [0; 1; 0], 'nearest', 'repeat'});
    phx.Script(piston, {"Position", [0; 6; 8; 10], [1 0.9 0.2; 1 0.9 0.2; 1 0.1 0.2; 1 0.9 0.2], 'linear', 'repeat'}); % interpolated curve

    % Import an STL model as a shape
    stl = phx.shape.Mesh("Source", resdir+"bottle.stl", "Scale", bottleSizePerc*1e-4, "Color", [0.4 0.6 1], "Material", "glossy", "Envelope", "cylinder");

    % Create the simulation object
    sim = phx.Simulation("EngineSettings", phx.engine.BulletSettings("Margin", 0.004));

    % Add bottles and run the simulation repeatedly
    total = 0;
    for i = 1:nBatches
        for j = 1:nBottles
            stl.Color = [0.4 0.6 0.8] + sin(j)*0.2;
            newBottles(j) = phx.Body("Position", [-1.9+j*0.05 sind(j*60)*0.3 0.1], "Shape", stl);
        end
        total = total + nBottles;
        viewer.displayText("Batch "+i+": "+total+" bottles", "below");
        sim.addObjects(newBottles);
        sim.step(10, 10/dt, 10);
    end

    % Clean up by deleting the simulation object
    delete(sim);

end