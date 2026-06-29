function phxex_multisim2
% PHXEX_MULTISIM2 Measuring distance between independent simulations
%
% Independent phx.Simulation objects run in the same axes; a phx.Measure reports
% the distance between a body from each as they fall.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Clear the current figure, set 3D view, and configure lighting
    clf; view(3); axis("equal"); camlight("headlight"); grid("on");
    
    % Create two sets of bodies
    A = phx.Body("Position", [0.6 0 2], "Shape", {"Cylinder", "Color", [1 0.5 0.5]}, "Mass", 100);
    B = phx.Body("Position", [0.6 0 2], "Shape", {"Cylinder", "Color", [0.5 0.5 1]}, "Mass", 200);
    S1 = phx.Body("Type", "static", "Shape", {"Box", "Color", 0.8});
    S2 = phx.Body("Type", "static", "Shape", {"Box", "Color", 0.8});

    % Create a measure between physical objects of two independent simulations
    M = phx.Measure(A, B, "Color", 1, "Overlay", true);

    % Create simulation objects with different set of objects
    sim1 = phx.Simulation([A S1]);
    sim2 = phx.Simulation([B S2]);
    
    % Run the simulation for 200 steps
    for i = 1:200
        sim1.step(0.01); % just compute
        sim2.step(0.01, 1, 1); % compute and redraw
        title("Distance: "+M.Distance);
    end

    % Clean up the simulation objects
    delete(sim1);
    delete(sim2);

end