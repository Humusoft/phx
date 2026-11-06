function phxex_isolation
% PHXEX_ISOLATION Roller base isolation against an earthquake
%
% Two identical dry-stacked towers (wall blocks and floor slabs held by
% friction only) stand on a ground plate shaken with a ramping amplitude.
% The left tower sits directly on the ground, the right one on a base plate
% carried by two free rollers. The roof motion of both towers is plotted,
% with the moment the left one collapses marked.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Parameters
    storeys = 4;         % storeys of each tower
    mu      = 0.8;       % friction between the stacked blocks
    rollerR = 0.15;      % radius of the isolation rollers (m)
    ampMax  = 0.16;      % peak ground displacement (m)
    freq    = 1.1;       % ground shaking frequency (Hz)
    tRamp   = 8;         % time for the amplitude to ramp up to ampMax (s)
    tShake  = 10;        % duration of the shaking (s)
    dt      = 0.002;     % time step (s)

    % Viewer
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0 -10 3.2], "DefaultCameraTarget",   [0 0 1.9]);

    % Scene
    % shaking ground
    ground = phx.Body(ax, "Type", "kinematic", "Position", [0 0 -0.25], "Friction", [1.2 0 0], ...
        "Shape", {"Box", "Size", [12 6 0.5], "Color", [0.45 0.35 0.30], "Texture", "cells", "TextureBlend", 0.2});

    % reference tower: straight on the ground
    roofA = buildTower(ax, -2.6, 0, storeys, mu);

    % isolation layer: a base plate on two free rollers
    rollerShape = phx.shape.Cylinder("Radius", rollerR, "Height", 1.6, "Axis", "y", "Color", [0.80 0.62 0.20], "Texture", "stripes", "TextureBlend", 0.25);
    for x = 2.6 + [-1.1 1.1]
        phx.Body(ax, "Position", [x, 0, rollerR], "Friction", [1.2 0.0015 0.0015], "Shape", rollerShape);
    end

    plateT = 0.12;       % base plate thickness
    phx.Body(ax, "Position", [2.6 0 2*rollerR + plateT/2], "Friction", [1.2 0 0], "Shape", {"Box", "Size", [4 1.7 plateT], "Color", [0.58 0.62 0.68]});

    roofB = buildTower(ax, 2.6, 2*rollerR + plateT, storeys, mu);

    % Data logging: positions of the ground and of both roofs
    logger = phx.Logger([ground roofA roofB], "Frequency", 100, "Parameters", "Position");

    % Simulation
    sim = phx.Simulation(ax);

    for k = 1:round(tShake/dt)
        tk  = k*dt;
        amp = ampMax*min(1, tk/tRamp);
        ground.Position = [amp*sin(2*pi*freq*tk) 0 -0.25];

        sim.step(dt, 1, 1);
    end

    delete(sim);

    % Collapse: the first moment the reference tower's roof has dropped by 25 cm
    t  = logger.Time;
    pg = logger.getChannel(1);
    pA = logger.getChannel(2);
    pB = logger.getChannel(3);
    collTime = t(find(pA(:, 3) < pA(1, 3) - 0.25, 1));

    % Plot results
    clf(figure(2));

    hold on; grid on;
    plot(t, 1000*pg(:, 1), "Color", [.6 .6 .6], "LineWidth", 0.8);
    plot(t, 1000*(pA(:, 1)-pA(1, 1)), "Color", [.85 .25 .15], "LineWidth", 1.1);
    plot(t, 1000*(pB(:, 1)-pB(1, 1)), "Color", [.10 .40 .75], "LineWidth", 1.1);
    if ~isempty(collTime)
        xline(collTime, "k--", "collapse", "LineWidth", 1.2);
    end
    xlabel("time [s]");
    ylabel("horizontal displacement [mm]");
    legend("ground", "roof, un-isolated", "roof, roller-isolated", "Location", "southwest");
    title("Roof motion, dry-stacked towers under a ramped earthquake");
    
end

% =========================================================================
function roof = buildTower(ax, x0, zBase, storeys, mu)
% Storeys of two wall blocks under a floor slab; the top slab is the roof
    wall = [0.25 1.2 0.7];   % wall block: thickness along the shaking, depth, height
    slab = [1.5 1.4 0.15];   % floor slab
    h = wall(3) + slab(3);
    for s = 0:storeys-1
        zw = zBase + s*h + wall(3)/2;
        for x = x0 + [-0.5 0.5]     % the two walls 1 m apart
            phx.Body(ax, "Position", [x, 0, zw], "Friction", [mu 0 0], ...
                "Shape", {"Box", "Size", wall, "Color", [1 1 1], "Texture", "stone", "TextureBlend", 0.5});
        end
        zs = zBase + s*h + wall(3) + slab(3)/2;
        roof = phx.Body(ax, "Position", [x0 0 zs], "Friction", [mu 0 0], ...
            "Shape", {"Box", "Size", slab, "Color", [0.3 0.3 0.3], "Texture", "stone", "TextureBlend", 0.5});
    end
end