function phxex_raycast
% PHXEX_RAYCAST Fan of rays sweeping a rotating cube
%
% A small sphere orbits a spinning cube and fires a fan of ten rays at it.
% The fan is defined once in the local frame of the sphere, so it follows
% the sphere around the orbit; the rays end at the surface they hit and run
% on into the distance where they miss.

%   Copyright 2026 HUMUSOFT s.r.o.

    nRays = 10;
    orbitRadius = 12;
    cubeRate = 0.5;         % cube spin, rad/s
    orbitRate = -0.25;       % sphere orbit, rad/s
    duration = 25;
    dt = 0.02;

    % --- Scene ----------------------------------------------------------
    figure(1);
    viewer = phx.extra.Viewer("clear", "DefaultCameraPosition", [-20 -20 20], "Texture", "tiles");

    % Two kinematic bodies
    cube = phx.Body("Type", "kinematic", "Shape", {"Box", "Size", [8 8 8], "Color", [0.35 0.55 0.75]});
    sphere = phx.Body("Type", "kinematic", "Position", [orbitRadius 0 0], "Shape", {"Sphere", "Diameter", 1.2, "Color", [0.95 0.75 0.2]});

    % Ten rays spread over +-25 deg about the local +X axis of the sphere,
    % which the pose below keeps pointing at the cube.
    fan = linspace(-1, 1, nRays)*deg2rad(25);
    raycast = phx.Raycast(sphere, "Ends", 26*[cos(fan); sin(fan); zeros(1, nRays)]);

    ax2 = uiaxes(viewer.Figure, "XLim", [1 nRays], "YLim", [0 orbitRadius], "Position", [10 10 200 200]);
    ln = plot(ax2, 1:nRays, zeros(1, nRays), '.', "MarkerSize", 10);

    sim = phx.Simulation;
    for t = 0:dt:duration
        cube.EulerAngles = [0 0 cubeRate*t];
        sphere.Transform = orbitPose(orbitRate*t, orbitRadius);
        sim.step(dt, 1, 1);
        ln.YData = raycast.Distances;
    end
    delete(sim);

end

function T = orbitPose(a, r)
% Pose on the circle of radius r at angle a, with local +X aimed at the origin.
    inward = [-cos(a); -sin(a); 0];
    T = [inward, [sin(a); -cos(a); 0], [0; 0; 1], r*[cos(a); sin(a); 0]
         0 0 0 1];
end