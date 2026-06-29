function phxex_multiview2(closeViewers)
% PHXEX_MULTIVIEW2 One simulation shown in separate viewer windows
%
% Same idea as phxex_multiview1, but the 3D and 2D views live in independent
% phx.extra.Viewer windows.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        closeViewers (1, 1) logical = true
    end

    rng(0);

    % Create two independent viewers
    [viewer1, ax1] = phx.extra.Viewer("newfigure", "DefaultCameraPosition", [-30 30 30]);
    [viewer2, ax2] = phx.extra.Viewer("newfigure", "DefaultCameraPosition", [100 0 20], "DefaultCameraTarget", [0 0 20]);
    viewer2.Position(1) = viewer1.Position(1) + viewer1.Position(3) + 10;
    drawnow;

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

    % Close viewers
    if closeViewers
        close([viewer1 viewer2]);
    end

end