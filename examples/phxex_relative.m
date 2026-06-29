function phxex_relative
% PHXEX_RELATIVE Demonstration of creating bodies in relative positions
% and measuring relative kinematic values during simulation.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Figure setup
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [30 30 30], "ViewMode", "axis");
    xlim([-20 20]); ylim([-20 20]); zlim([0 10]);

    % Create a static physical body
    body = phx.Body(ax, "Type", "static", "Shape", {"Box", "Size", [40 40 1], "Color", [1 1 1]});

    % Create additional dynamic bodies in relative positions
    for i = 1:12
        body(i + 1) = phx.Body(ax, "Transform", body(i).offset([4 0 1.5], [0 0 pi/6]), "Shape", {"Box", "Size", [6 2 1]});
    end

    % Create a measuring object between two points on specified bodies
    meas39 = phx.Measure(body(3), body(9), "PointA", [-3 1 0.5], "PointB", [0 0 0], "Overlay", true);

    % Simulation loop
    sim = phx.Simulation(ax);
    for i = 1:10
        sim.step(0.2, 20, 5); % Step the simulation
        viewer.displayText(num2str(sim.Time, '%0.1f')+": "+meas39.Distance, "below");
    end

    % Clean up the simulation object
    delete(sim);

end