function phxex_multisim
% PHXEX_MULTISIM Independent simulations side by side in subplots
%
% Separate phx.Simulation objects run in their own axes; a phx.Measure spans
% both simulations and the distance between their bodies is plotted live.

%   Copyright 2026 HUMUSOFT s.r.o.

    clf;

    % Create the first subplot for the first simulation
    ax1 = subplot(2, 2, 1); view(3); axis("equal"); camlight("headlight"); grid("on");
    set(ax1, "Xlim", [-1 2], "YLim", [-1 1], "ZLim", [-0.5 2.5]);
    title("Situation 1");

    % Create the second subplot for the second simulation
    ax2 = subplot(2, 2, 2); view(3); axis("equal"); camlight("headlight"); grid("on");
    set(ax2, "Xlim", [-1 2], "YLim", [-1 1], "ZLim", [-0.5 2.5]);
    title("Situation 2");

    % Create a plot for the measured distance
    subplot(2, 2, [3 4]);
    L = plot(nan(1, 100), "Color", [1 0 1]);
    title("Distance between the bodies"); xlim([1 100]); ylim([0 1.5]); grid("on");

    % Initialize the bodies in the first and second axes
    A = phx.Body(ax1, "Position", [0.6 0 2], "Shape", {"Cylinder", "Color", [1 0.5 0.5]});
    phx.Body(ax1, "Type", "static", "Shape", {"Box", "Color", 0.8});
    B = phx.Body(ax2, "Position", [0.6 0 1], "Shape", {"Cylinder", "Color", [0.5 0.5 1]});
    phx.Body(ax2, "Type", "static", "Shape", {"Box", "Color", 0.8});

    % Create a measure between physical objects of two independent simulations
    M = phx.Measure(A, B, "Color", [1 0 1]);

    % Create simulation objects for both axes
    sim1 = phx.Simulation(ax1);
    sim2 = phx.Simulation(ax2);

    % Run the simulation for 100 steps
    for i = 1:100
        sim1.step(0.01); % just compute
        sim2.step(0.01, 1, 1); % compute and redraw
        L.YData(i) = M.Distance;
    end

    % Clean up the simulation objects
    delete(sim1);
    delete(sim2);

end
