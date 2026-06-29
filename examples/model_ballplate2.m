function model_ballplate2
% MODEL_BALLPLATE2  Scene for the two-ball "ball & bowl" Simulink demo.
%
%   A kinematic concave bowl (tilted from Simulink via its EulerAngles) holds
%   two dynamic balls. The bowl passively keeps the balls together near its
%   bottom, where they collide; tilting the bowl steers the pair. Only the
%   centroid of the two balls is controllable from the 2-DOF tilt (the balls
%   cannot be placed independently) - see phxex_sim_ballplate2.
%
%   Builds the bodies and saves them to saved_ballplate2.mat.
%
%   See also phxex_sim_ballplate2, PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");
    
    % Viewer setup
    ax = cla(clf);
    phx.extra.Viewer(ax, "ViewMode", "plain");

    % Wide shallow bowl - kinematic, driven from Simulink (Bowl.EulerAngles).
    % Revolution profile columns are [Z axial-height, X radius].
    prof = [0 0; 0.2 2; 0.7 4; 1.5 6; 2.6 8];
    Bowl = phx.Body("Type", "kinematic", "Position", [0 0 0], ...
        "Shape", {"Revolution", "Profile", prof, "Axis", "z", "Envelope", "concave", "Color", [0.82 0.85 0.90], "Texture", resdir+"checker4.png", "TextureBlend", 0.5}, ...
        "Friction", [0.6 0 0], "Name", "Bowl"); %#ok<NASGU>

    % Two dynamic balls (density-based mass/inertia; a little rolling friction
    % damps the in-bowl oscillation so a simple proportional controller suffices)
    Ball1 = phx.Body("Position", [ 2 0 2], "Shape", {"Globe", "Diameter", 1.8, "Density", 1, "Color", [0.90 0.30 0.30], "Texture", resdir+"checker4.png", "TextureBlend", 0.5}, "Friction", [0.6 0.05 0], "Name", "Ball1"); %#ok<NASGU>
    Ball2 = phx.Body("Position", [-4 0 4], "Shape", {"Globe", "Diameter", 2.3, "Density", 2, "Color", [0.30 0.50 0.90], "Texture", resdir+"checker4.png", "TextureBlend", 0.5}, "Friction", [0.6 0.05 0], "Name", "Ball2"); %#ok<NASGU>

    save("saved_ballplate2.mat", "Bowl", "Ball1", "Ball2");
end
