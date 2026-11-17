function phxex_noview
% PHXEX_NOVIEW Running a simulation headless, with no graphics
%
% Bodies are created without parent axes and stepped without rendering; the
% scene is only drawn afterwards to read out a single body's final position.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Set random seed for reproducibility
    rng(0);

    % Create a static ground body with a box shape
    ground = phx.Body([], "Type", "static", "Position", [0 0 -1], "Shape", {"Box", "Style", "edged", "Size", [150 150 1], "Color", [0.8 0.8 0.8]});

    % Create multiple boxes of random sizes stacked above the ground, half a
    % metre apart
    bottom = 5;
    for i = 1:40
        sz = rand(1, 3)*5;
        boxes(i) = phx.Body([], "Position", [0 0 bottom + sz(3)/2], "Shape", {"Box", "Style", "edged", "Size", sz});
        bottom = bottom + sz(3) + 0.5;
    end

    % Combine ground and boxes into a single array for simulation
    simulatedBodies = [ground boxes];

    % Run the simulation for 8 seconds with 1600 time steps
    sim = phx.Simulation(simulatedBodies);
    sim.step(8, 1600, -1);
    delete(sim);

    % Optional view of the model state after the simulation
    ax = cla(clf); view(3); axis("equal");
    set([ground boxes], "ParentAxes", ax);

    % Edged boxes are drawn by primitives that the automatic axes limits do
    % not see: fit the limits to the boxes, down to the top of the ground
    bb = boundingBox(boxes);
    bb(1, 3) = -0.5;
    axis(ax, reshape(bb + [-1; 1], 1, []));

    % Display the position of the 7th box after the simulation
    title("Box #7 position: "+mat2str(boxes(7).Position));

end