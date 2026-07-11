function model_sorter
% MODEL_SORTER  Scene for the vision color-sorter Simulink demo.
%
%   A kinematic carousel disk carries coloured parts past a fixed deflector
%   that scrapes them onto a sloped chute. A kinematic trapdoor in the chute
%   sorts each part: closed -> the part rides over into bin B (end of chute);
%   open (dropped in z) -> the part falls through into bin A below. A camera
%   over the chute classifies parts by colour; Simulink opens/closes the
%   trapdoor accordingly (see phxex_sim_sorter).
%
%   Simulink inputs:  Disk.EulerAngles(3) (carousel rotation, a ramp),
%                     Trap.Position(3)    (trapdoor: closed ~ -2.9, open ~ -5.9)
%   Builds the bodies and saves them to saved_sorter.mat.
%
%   See also phxex_sim_sorter, PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Viewer setup
    ax = cla(clf);
    phx.extra.Viewer(ax, "ViewMode", "plain");

    zc = @(x) 0.25 - tand(20)*(x - 8);     % chute surface line

    Disk = phx.Body("Type", "kinematic", "Position", [0 0 0.75], "Shape", {"Cylinder", "Diameter", 16, "Height", 0.5, "Axis", "z", "Color", [0.80 0.80 0.85], "Texture", resdir+"checker4.png", "TextureBlend", 0.25}, "Friction", [0.9 0 0], "Name", "Disk"); %#ok<NASGU>
    Defl = phx.Body("Type", "kinematic", "Position", [6.5 0 1.8], "EulerAngles", [0 0 deg2rad(30)], "Shape", {"Box", "Size", [7 0.4 1.6], "Color", [0.90 0.80 0.30]}, "Friction", [0.4 0 0], "Name", "Defl"); %#ok<NASGU>
    ChuteUp = phx.Body("Type", "static", "Position", [10.75 0 zc(10.75)], "EulerAngles", [0 deg2rad(20) 0], "Shape", {"Box", "Size", [5.5 9 0.4], "Color", [0.92 0.92 0.92], "Density", 100}, "Friction", [0 0 0], "Name", "ChuteUp"); %#ok<NASGU>
    ChuteDn = phx.Body("Type", "static", "Position", [21.00 0 zc(21.00)-0.3], "EulerAngles", [0 deg2rad(20) 0], "Shape", {"Box", "Size", [5.0 9 0.4], "Color", [0.92 0.92 0.92]}, "Friction", [0.1 0 0], "Name", "ChuteDn"); %#ok<NASGU>
    Trap = phx.Body("Type", "kinematic", "Position", [16.00 0 zc(16.00)], "EulerAngles", [0 deg2rad(20) 0], "Shape", {"Box", "Size", [5.5 9 0.4], "Color", [0.20 0.80 0.40]}, "Friction", [0.1 0 0], "Name", "Trap"); %#ok<NASGU>

    SideU  = phx.Body("Type", "static", "Position", [16 4.7 -3], "Shape", {"Box", "Size", [22 0.4 9], "Color", [0.85 0.85 0.85]}, "Name", "SideU"); %#ok<NASGU>
    SideL  = phx.Body("Type", "static", "Position", [16 -4.7 -3], "Shape", {"Box", "Size", [22 0.4 9], "Color", [0.85 0.85 0.85]}, "Name", "SideL"); %#ok<NASGU>
    EndW   = phx.Body("Type", "static", "Position", [24.0 0 zc(24)+1], "EulerAngles", [0 deg2rad(20) 0], "Shape", {"Box", "Size", [0.4 9 3], "Color", [0.85 0.85 0.85]}, "Name", "EndW"); %#ok<NASGU>   % bin B back wall
    BinA   = phx.Body("Type", "static", "Position", [21 0 -10], "EulerAngles", [0 0.2 0], "Shape", {"Box", "Size", [20 11 0.5], "Color", [1 1 1]*0.6}, "Friction", [0.2 0 0], "Name", "BinA"); %#ok<NASGU>
    BinAbk = phx.Body("Type", "static", "Position", [31 0 -9.8], "EulerAngles", [0 0.2 0], "Shape", {"Box", "Size", [0.4 11 4], "Color", [1 1 1]*0.6}, "Name", "BinAbk"); %#ok<NASGU>

    % No pre-placed parts: a PHX Action block spawns coloured cubes onto the
    % carousel at regular angular steps during simulation (see phxex_sim_sorter).
    save("saved_sorter.mat", "Disk", "Defl", "ChuteUp", "ChuteDn", "Trap", "SideU", "SideL", "EndW", "BinA", "BinAbk");
end
