function phxex_textures
% PHXEX_TEXTURES Larger textured scene with cottage, palms and a buggy
%
% Composed scene with cottage, palm trees and a buggy loaded from a saved 
% model, all textured, while boxes rain onto the ground.

%   Copyright 2026 HUMUSOFT s.r.o.

    rng(0); % Set random seed for reproducibility

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Set up the viewer with a default camera position and view mode
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [6.0, 9.07, 3.5], "DefaultCameraTarget", [-1.34 -0.77 0.77]);

    % Create a static ground body
    phx.Body(ax, "Type", "static", "Position", [0 0 -0.05], "Shape", {"Box", "Texture", "tiles", "Size", [10 10 0.1]});
    
    % Cottage
    cottageShape = phx.shape.Mesh("Source", resdir+"cottage.obj", "Group", "Cube_Cube.002", "Texture", resdir+"cottage_diffuse.jpg", "Envelope", "convex", "Centered", false, "Scale", 0.2);
    phx.Body(ax, "Type", "static", "Position", [0 -2 0], "Shape", cottageShape);

    % Trees
    treeShape = phx.shape.Mesh("Source", resdir+"palm_tree.obj", "Material", "matte", "Envelope", "convex", "Centered", false, "Scale", 0.4);
    for y = -20:8:20
        phx.Body(ax, "Type", "static", "Position", [19+rand*2 y 0]*0.2, "EulerAngles", [0 0 y], "Shape", treeShape);
    end

    % Create dynamic boxes with random colors and sizes
    boxShape = phx.shape.Box("Texture", peaks(-1:0.01:1), "TextureColormap", gray(12), "TextureBlend", 0.75, "Density", 30);
    for i = 1:16
        boxShape.Size = rand*[1 1 1]; % Generate a random size
        phx.Body(ax, "Position", [(rand(1, 2)-0.5)*50 20+i*2]*0.2, "EulerAngles", rand(1, 3), "Shape", boxShape.nextColor);
    end

    % Import buggy
    buggy = load("saved_buggy.mat");
    car = cell2mat(struct2cell(buggy));
    propagate(car, "ParentAxes", ax);
    car.groupTransform("Translation", [0 3 0], "EulerAngles", [0 0 pi]);

    drawnow;
    
    % Initialize the simulation based on all objects from a current scene;
    % contact tolerances for gaps of about 10 cm keep the buggy's wheels clear
    sim = phx.Simulation("EngineSettings", phx.engine.BulletSettings("LengthScale", 0.1));
    sim.step(9, 900, 1);

    % Clean up the simulation object
    delete(sim); 
        
end