function phxex_textures1
% PHXEX_TEXTURES1 Textured boxes dropping around a textured OBJ cottage
%
% Demonstrates image textures on bodies and meshes: companion-cube boxes fall
% onto a tiled ground beside an OBJ cottage model.

%   Copyright 2026 HUMUSOFT s.r.o.

    rng(0); % Set random seed for reproducibility

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Set up the viewer with a default camera position and view mode
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [48, 74, 24], "DefaultCameraTarget", [-5 0 10]);

    % Create a static ground body
    ground = phx.Body(ax, "Type", "static", "Position", [0 0 -0.5], "Shape", {"Box", "Texture", resdir+"tiles.jpg", "Size", [50 50 1]});
    cottage = phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", {"OBJ", "Source", resdir+"cottage.obj", "Texture", resdir+"cottage_diffuse.jpg", "Envelope", "convex"});

    % Create dynamic boxes with random colors and sizes
    boxShape = phx.shape.Box("Texture", resdir+"companion_cube.jpg");
    for i = 1:10
        boxShape.Size = rand*[5 5 5]; % Generate a random size
        boxes(i) = phx.Body(ax, "Position", [(rand(1, 2)-0.5)*20 20+i*2], "EulerAngles", rand(1, 3), "Shape", boxShape, "Name", "CompanionCube"+i);
    end

    % Attach camera
    % phx.Camera(boxes(4), cottage, "PointA", [-10 -20 20]);

    % Attach camera and record video
    % phx.Camera(boxes(4), cottage, "PointA", [-10 -20 20], "RecordFile", "videos/phxex_textures_cam.mp4");

    drawnow;
    
    % Initialize the simulation with the ground, boxes, and cottage
    sim = phx.Simulation([ground cottage boxes]);

    % Step the simulation forward and update the figure at each iteration
    sim.step(9, 900, 1);

    % Clean up the simulation object
    delete(sim); 
        
end