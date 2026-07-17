function phxex_multiview
% PHXEX_MULTIVIEW One simulation shown in 3D and 2D subplots at once
%
% Falling boxes are rendered in a 3D subplot and mirrored into a 2D view via
% shadowCopy, so a single simulation drives both synchronized views.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Create the first subplot for 3D view
    rng(0);
    ax1 = subplot(1, 2, 1);
    cla(ax1); view(3); axis("equal"); xlim([-25 25]); ylim([-25 25]); zlim([-10 40]); camlight; grid("on");

    % Create the second subplot for 2D view
    ax2 = subplot(1, 2, 2);
    cla(ax2); view(2); axis("equal"); xlim([-25 25]); ylim([-25 25]); camlight;

    % Create a static ground body
    ground = phx.Body(ax1, "Type", "static", "Position", [0 0 -1], "Shape", {"Box", "Size", [50 50 1], "Color", [0.8 0.8 0.8]});

    % Loop to create multiple boxes with random colors and sizes
    for i = 1:40
        clr = rand(1, 3); % Generate a random color
        scl = rand(1, 3)*5; % Generate a random size
        boxes(i) = phx.Body(ax1, "Position", [0 0 10+i*2], "Shape", {"Box", "Size", scl, "Color", clr});
        
        % Trace the box with specified parameters
        phx.Trace(boxes(i), "TracePoints", 50, "Point", scl/2, "Color", clr);
    end

    % Create a shadow copy of the ground and boxes in the 2D view
    shadowCopy([ground boxes], ax2);

    % Initialize and run the simulation
    sim = phx.Simulation([ground boxes]);
    sim.step(10, 1000, 10); % Step the simulation for 10 seconds with 1000 steps
    delete(sim); % Clean up the simulation object

end