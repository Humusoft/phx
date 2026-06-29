function phxex_springs(showGraphs)
% PHXEX_SPRINGS Demonstration of a spring system simulation
%
% This function sets up a physical model of springs and boxes, simulates
% their interactions, and logs the data for analysis.
%
% The simulation is divided into individual phases in which some simulation 
% objects are added and deleted.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        showGraphs (1, 1) logical = true
    end

    % Figure setup
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0, -70, 0], "ViewMode", "plain");

    % Physical model
    n = 11;
    colors = ones(n, 3)*0.8;
    colors(4, :) = [0.5 0.5 1]; % logged box;
    colors(7, :) = [1 0.5 0.5]; % box to be deleted
    cx = (n*6 - 6)/2;
    boxes(1) = phx.Body(ax, "Type", "static", "Position", [-cx 0 10], "Shape", {"Box", "Size", [4 2 1], "Color", colors(1, :)});
    for i = 2:n
        boxes(i) = phx.Body(ax, "Position", [(i - 1)*6 - cx 0 10], "Shape", {"Box", "Size", [4 2 1], "Color", colors(i, :)});
        phx.Spring(boxes(i - 1), boxes(i), "Stiffness", 3e5, "Damping", 2e3, "PointA", [2 0 0], "PointB", [-2 0 0], "Visible", true, "Colormap", "jet", "ColorRange", [0 2e6]);
        phx.Trace(boxes(i), "TracePoints", 50, "Color", [0.5 0.5 0.5]);
    end
    boxes(end).Type = "static";
    newBox = phx.Body(ax, "Position", [0 0 15], "Shape", {"Box", "Size", [4 4 4], "Color", [0.5 1 0.5]}); % box to be added

    % Data logging
    log1 = phx.Logger(boxes(4), "Frequency", 20, "Parameters", "Position");
    log2 = phx.Logger(boxes(4).Children([1 3]), "Frequency", 10, "Parameters", "Force");
    %log2.dispChannels;

    % Simulation
    sim = phx.Simulation(boxes, "HideInvalid", true);
    sim.step(5, 500, 5);

    % Add new object to the simulation
    sim.addObjects(newBox);
    sim.step(10, 1000, 5);

    % Delete one of the simulation objects
    delete(boxes(7));
    sim.step(5, 500, 5);
    delete(sim);

    % Display logged data
    if showGraphs
        figure(2);
        subplot(2, 1, 1);
        plot(log1.Time, log1.Data);
        title("Box #4 position"); legend("x", "y", "z");
        subplot(2, 1, 2);
        mag = @(x) sqrt(sum(x.^2, 2));
        plot(log2.Time, mag(log2.getChannel(1)), log2.Time, mag(log2.getChannel(2)));
        title("Box #4 springs forces"); legend("left", "right");
    end

end