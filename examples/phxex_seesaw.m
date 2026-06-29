function phxex_seesaw
% PHXEX_SEESAW  A cat falls onto one end of a seesaw and launches a box
%
% The scene contains:
%   - a static sphere in the middle of the scene (serves as the seesaw fulcrum)
%   - the seesaw plank resting freely on the sphere (collision contact only)
%   - a box placed on the right end of the seesaw
%   - a cat (STL model) suspended above the left end - released after a moment
 
%   Copyright 2026 HUMUSOFT s.r.o.
 
    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Window and camera
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0 -25 8], "DefaultCameraTarget", [0  0  1]);
    xlim([-8 8]); ylim([-4 4]); zlim([-1 6]);
    title("Seesaw: cat vs. box", "FontSize", 14);
 
    % Dimensions
    plankLen  = 12;    % length of the plank [m]
    plankH    = 0.25;  % thickness of the plank [m]
    ballR     = 0.6;   % radius of the sphere [m]
    pivotH    = ballR * 2 + plankH/2;  % height of the plank center above the ground
    armLen    = plankLen/2 - 0.5;   % distance of the weights from the center
 
    % Floor
    phx.Body(ax, "Type", "static", ...
        "Position", [0 0 -0.5], ...
        "Shape", {"Box", "Size", [20 8 1], "Color", [0.85 0.80 0.70]});
 
    % Sphere as fulcrum (seesaw support)
    phx.Body(ax, "Type", "static", ...
        "Position", [0 0 ballR], ...
        "Shape", {"Sphere", "Radius", ballR, "Color", [0.55 0.45 0.35]});
 
    % Seesaw plank (rests freely on the sphere)
    phx.Body(ax, ...
        "Position", [0 0 pivotH], ...
        "Shape", {"Box", "Size", [plankLen 1.2 plankH], ...
                  "Color", [0.70 0.55 0.35]}, ...
        "Mass", 8, "Inertia", 10);
 
    % Box on the right end
    box = phx.Body(ax, ...
        "Position", [armLen 0 pivotH + plankH/2 + 0.5], ...
        "Shape", {"Box", "Size", [1 1 1], ...
                  "Color", [0.3 0.5 0.9]}, ...
        "Mass", 2);
 
    % Cat above the left end (will be released)
    catHeight = pivotH + plankH/2 + 4.5;   % height from which it falls
    cat = phx.Body(ax, ...
        "Type", "kinematic", ...            % held fixed for now
        "Position", [-armLen 0 catHeight], ...
        "Shape", {"STL", "Source", resdir+"cat.stl", ...
                  "Scale", 0.030, ...
                  "Details", 0.25, ...
                  "Color", [0.85 0.70 0.55]}, ...
        "Mass", 60);
 
    % Trace for the box
    phx.Trace(box, "TracePoints", 120, "Color", [0.3 0.5 0.9]);
 
    % Create simulation object
    sim = phx.Simulation;
 
    % Phase 1: briefly stabilize the scene, the cat hangs still (kinematic)
    sim.step(0.4, 40, 10);
 
    % Release the cat - switch to dynamic, physics lets it fall freely
    cat.Type = "dynamic";
 
    % Phase 2: the cat lands on the seesaw and launches the box
    sim.step(10, 2000, 1);
 
    % Delete simulation object
    delete(sim);

end