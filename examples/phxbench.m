function phxbench(group)
% PHXBENCH Function to run a series of demo functions for benchmarking
%
% This function measures the time taken to execute various demo functions
% related to different simulations or visualizations.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        group {mustBeMember(group, ["all", "viewer", "native", "classic", "new"])} = "classic" 
    end

    g1 = group == "all" || group == "classic" || group == "viewer";
    g2 = group == "all" || group == "classic" || group == "native";
    g3 = group == "all" || group == "new";

    t = tic; % Start timer

    % Execute various demo functions
    if g3, phxex_antisway; end
    if g3, phxex_balance; end
    if g1, phxex_buggy(false); end
    if g3, phxex_buoyancy; end
    if g3, phxex_camvalve; end
    if g1, phxex_conveyors; end
    if g3, phxex_crystal; end
    if g1, phxex_determinism(200); end
    if g3, phxex_drone; end
    if g1, phxex_gears; end
    if g1, phxex_gravity(100, 128, false); end
    if g3, phxex_grip; end
    if g3, phxex_gyrostab; end
    if g1, phxex_charges(50); end
    if g2, phxex_joints(false); end
    if g3, phxex_maglev; end
    if g2, phxex_multisim; end
    if g2, phxex_multiview; end
    if g2, phxex_noview; end
    if g3, phxex_optimize; end
    if g3, phxex_rocket; end
    if g1, phxex_rotmagdip; end
    if g3, phxex_segway; end
    if g2, phxex_shapes; end
    if g1, phxex_slide; end
    if g1, phxex_soil; end
    if g1, phxex_springs(false); end
    if g3, phxex_stairfall; end
    if g3, phxex_tackle; end
    if g1, phxex_terrain; end
    if g1, phxex_textures; end
    if g2, phxex_trainwheel; end
    if g3, phxex_vengine; end
    if g1, phxex_wankel(false); end

    toc(t) % Display elapsed time

end