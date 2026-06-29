function phxex_terrain
% PHXEX_TERRAIN Balls rolling over a height-map terrain
%
% A phx.shape.Terrain built from peaks() carries a grid of spheres that roll
% downhill; the sphere colours spell out the PHX logo.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Figure setup
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [-30 -75 30], "Texture", "defaultChecker");
    
    % Generate terrain using the peaks function
    h = peaks(64);
    trn = phx.shape.Terrain("Size", [50 40], "Height", h, "Material", "matte");

    % Compute colormap data and apply as texture
    colormap = (parula(256) + repmat(1-gray(16), [16 1])*0.5 + 0.5)/2;
    trn = trn.colormapTexture(peaks(1024), colormap);
    
    % Create a static body representing the terrain
    phx.Body(ax, "Type", "static", "Position", [0 0 0], "Shape", trn);
    
    % Define color matrix for PHX logo
    phxc = ones(16, 20);
    phxc(2:15, [1:6 8:13 15:20]) = 0;
    phxc(10:15, [4:6 10:11 17:18]) = 1;
    phxc(2:7, [10:11 17:18]) = 1;
    phxc(4:7, 3:4) = 1;
    phxc(8:9, [15 16 19 20]) = 1;
    phxc = flipud(phxc);

    % Create dynamic bodies (spheres) at specified positions
    shp = phx.shape.Sphere("Diameter", 1.1, "Material", "shiny");
    i = 0;
    for x = -19:2:19
        for y = -15:2:15
            i = i + 1;
            shp.Color = phxc(i)*0.8 + 0.2; %mod(i, 2)*0.8 + 0.2;
            balls(i) = phx.Body(ax, "Type", "dynamic", "Position", [x y 5 + (15 + y)/3], "Shape", shp);
        end
    end

    % Add wind
    % phx.Resistance(balls, "VelocityFactors", [0 100], "ForceVectorSize", 0.005, "EnvironmentVelocity", [10 0 0]);

    % Add tornado!
    % phx.Resistance(balls, "VelocityFactors", [0 100], "ForceVectorSize", 0.005, "EnvironmentTwist", [0 0 2]);

    % Run the simulation for a specified duration and steps
    sim = phx.Simulation;
    sim.step(5, 500, 5);
    delete(sim);

end