function phxex_conveyors
% PHXEX_CONVEYORS Conveyor line driven by phx.Script automation
%
% Batches of bottles, each a body of revolution, are carried along two belts
% and funnelled by a pair of static guide flaps into a bin at the end of
% the line. The belts do not move as bodies - each one is a kinematic box
% that phx.Script shifts forward by one timestep of travel and snaps back
% again, with friction scripted on for the forward move only, so contact
% drags the bottles along at the requested belt speed.
%
% See also phx.Script, phx.shape.Revolution, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.

    % Default values
    nBatches = 6;
    nBottles = 12;
    vBelt1 = 0.1;
    vBelt2 = 0.2;
    flapAngle = 35;
    bottleSizePerc = 100;

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Figure setup
    viewer = phx.extra.Viewer("clear", "DefaultCameraPosition", [3.8 -2.6 2.4], "DefaultCameraTarget", [1 -0.2 0], "Texture", resdir+"HumusoftHall.png");

    % Ground and basket
    phx.Body("Type", "static", "Position", [-0.5 0 -0.05], "Shape", {"Box", "Size", [4 4 0.1], "Texture", "tiles"});
    phx.assembly.arena("Position", [2 0 -0.6], "Size", [1 1 0.5]);
    
    % Belts
    phx.Body("Type", "static", "Position", [-1 0 0.02], "SurfaceVelocity", [vBelt1 0 0], "Shape", {"Box", "Size", [2 0.8 0.04], "Color", [0 0 1], "Texture", resdir+"arrows.png", "TextureBlend", 0.7}, "Friction", [1 0.05 0]);
    phx.Body("Type", "static", "Position", [1 0 0.02], "SurfaceVelocity", [vBelt2 0 0], "Shape", {"Box", "Size", [2 0.8 0.03], "Color", [1 0 0], "Texture", resdir+"arrows.png", "TextureBlend", 0.7}, "Friction", [1 0.05 0]);
    
    % Flaps
    flapAngle = flapAngle*pi/180;
    phx.Body("Type", "static", "Position", [-0.3 0.4 0.15], "EulerAngles", [0 0 -flapAngle], "Shape", {"Box", "Size", [0.6 0.05 0.15], "Color", 1}, "Friction", 0.1);
    phx.Body("Type", "static", "Position", [-0.3 -0.4 0.15], "EulerAngles", [0 0 flapAngle], "Shape", {"Box", "Size", [0.6 0.05 0.15], "Color", 1}, "Friction", 0.1);

    % Piston
    piston = phx.Body("Type", "kinematic", "Position", [1 0.9 0.2], "Shape", {"Cylinder", "Diameter", 0.3, "Height", 1, "Axis", "y", "Color", [0.8 1 0.7]});

    % Automation scripts
    phx.Script(piston, {"Position", [0; 7; 9; 10], [1 0.9 0.2; 1 0.9 0.2; 1 0.1 0.2; 1 0.9 0.2], 'linear', 'repeat'}); % interpolated curve

    % Bottle as a body of revolution
    prof = bottleSizePerc/100*[-0.222 0; -0.222 0.057; 0.056 0.057; 0.078 0.047; ...
                                0.100 0.029; 0.122 0.024; 0.190 0.024; 0.190 0.027; ...
                                0.222 0.027; 0.222 0];
    bottle = phx.shape.Revolution("Profile", prof, "Segments", 24, "Color", [0.4 0.6 1], "Material", "glossy", "Envelope", "cylinder", "Texture", "checker", "TextureBlend", 0.2);

    % Create the simulation object
    sim = phx.Simulation("EngineSettings", phx.engine.BulletSettings("Margin", 0.001));

    % Add bottles and run the simulation repeatedly
    total = 0;
    dt = 0.01;
    t = 10;
    for i = 1:nBatches
        for j = 1:nBottles
            bottle.Color = [0.4 0.6 0.8] + sin(j)*0.2;
            newBottles(j) = phx.Body("Position", [-1.9+j*0.06, mod(j, 3)*0.3 - 0.3, 0.3], "Shape", bottle);
        end
        total = total + nBottles;
        viewer.displayText("Batch "+i+": "+total+" bottles", "below");
        sim.addObjects(newBottles);
        sim.step(t, t/dt, 10);
    end

    % Clean up by deleting the simulation object
    delete(sim);

end