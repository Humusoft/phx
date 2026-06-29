function phxex_noview
% PHXEX_NOVIEW Running a simulation headless, with no graphics
%
% Bodies are created without parent axes and stepped without rendering; the
% scene is only drawn afterwards to read out a single body's final position.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Set random seed for reproducibility
    rng(0);

    % Create a static ground body with a box shape
    ground = phx.Body([], "Type", "static", "Position", [0 0 -1], "Shape", {"Box", "Style", "edged", "Size", [50 50 1], "Color", [0.8 0.8 0.8]});

    % Create multiple box bodies positioned above the ground
    for i = 1:40
        boxes(i) = phx.Body([], "Position", [0 0 5+i*2], "Shape", {"Box", "Style", "edged", "Size", rand(1, 3)*5});
    end

    % Combine ground and boxes into a single array for simulation
    simulatedBodies = [ground boxes];

    % Run the simulation for 5 seconds with 1000 time steps
    sim = phx.Simulation(simulatedBodies);
    sim.step(5, 1000);
    delete(sim);

    % Optional view of the model state after the simulation
    ax = cla(clf); view(3); axis("equal");
    set([ground boxes], "ParentAxes", ax);

    % Display the position of the 7th box after the simulation
    title("Box #7 position: "+mat2str(boxes(7).Position));

end