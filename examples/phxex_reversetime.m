function phxex_reversetime
% PHXEX_REVERSETIME Demonstration of a negative time step
%
% This function sets up a physical model of springs and boxes, simulates
% their interactions.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Figure setup
    ax = cla(clf(figure(1)));
    viewer = phx.extra.Viewer(ax, "DefaultCameraPosition", [0, -150, 0], "DefaultCameraTarget", [20 0 -30], "ViewMode", "plain");

    % Physical model
    n = 11;
    boxes(1) = phx.Body(ax, "Type", "static", "Position", [0 0 10], "Shape", {"Box", "Size", [4 2 1], "Color", [1 0.5 0.5]});
    for i = 2:n
        boxes(i) = phx.Body(ax, "Position", [(i - 1)*6 0 10], "Shape", {"Box", "Size", [4 2 1], "Color", 0.8});
        phx.Spring(boxes(i - 1), boxes(i), "FreeLength", 2, "Stiffness", 1e6, "Damping", 0, "PointA", [2 0 0], "PointB", [-2 0 0], "Visible", true, "Colormap", "jet", "ColorRange", [0 2e6]);
        phx.Trace(boxes(i), "TracePoints", 200, "Color", [0.5 0.5 0.5]);
    end

    % Simulation
    sim = phx.Simulation;

    % Forward time
    for i = 1:100
        sim.step(0.05, 5, 5);
        viewer.displayText("Time: "+round(sim.Time, 2));
    end

    % Backward time
    for i = 1:100
        sim.step(-0.05, 5, 5);
        viewer.displayText("Time: "+round(sim.Time, 2));
    end

    % Delete simulation object
    delete(sim);

end