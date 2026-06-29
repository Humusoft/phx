function phxex_textures2
% PHXEX_TEXTURES2 Larger textured scene with cottage, palms and a buggy
%
% Extends phxex_textures1 with palm trees and a buggy loaded from a saved scene,
% all textured, while boxes rain onto the ground.

%   Copyright 2026 HUMUSOFT s.r.o.

    rng(0); % Set random seed for reproducibility

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Set up the viewer with a default camera position and view mode
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [60.0, 90.7, 35.0], "DefaultCameraTarget", [-13.4 -7.7 7.7]);

    % Create a static ground body
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], "Shape", {"Box", "Texture", resdir+"tiles.jpg", "Size", [100 100 1]});
    
    % Cottage
    cottageShape = phx.shape.OBJ("Source", resdir+"cottage.obj", "Texture", resdir+"cottage_diffuse.jpg", "Envelope", "convex", "Scale", 2);
    phx.Body(ax, "Type", "static", "Position", [0 -20 0], "Shape", cottageShape);

    % Trees
    treeShape = phx.shape.OBJ("Source", resdir+"palm_tree.obj", "Texture", resdir+"palm_atlas.jpg", "Material", "matte", "Envelope", "convex", "Scale", 4);
    for y = -20:8:20
        phx.Body(ax, "Type", "static", "Position", [19+rand*2 y 0]*2, "EulerAngles", [0 0 y], "Shape", treeShape);
    end

    % Create dynamic boxes with random colors and sizes
    boxShape = phx.shape.Box("Texture", resdir+"companion_cube.jpg", "Density", 0.5);
    for i = 1:16
        boxShape.Size = rand*[10 10 10]; % Generate a random size
        phx.Body(ax, "Position", [(rand(1, 2)-0.5)*50 20+i*2]*2, "EulerAngles", rand(1, 3), "Shape", boxShape);
    end

    % Import buggy
    buggy = load("saved_buggy.mat");
    car = cell2mat(struct2cell(buggy));
    propagate(car, "ParentAxes", ax);
    car.groupTransform("Translation", [0 30 0], "EulerAngles", [0 0 pi]);

    drawnow;
    
    % Initialize the simulation based on all objects from a current scene
    sim = phx.Simulation;
    sim.step(9, 900, 1);

    % Clean up the simulation object
    delete(sim); 
        
end