function phxbench(group)
% PHXBENCH Function to run a series of demo functions for benchmarking
%
% This function measures the time taken to execute various demo functions
% related to different simulations or visualizations.

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        group {mustBeMember(group, ["all", "viewer", "native"])} = "all" 
    end

    g1 = group == "all" || group == "viewer";
    g2 = group == "all" || group == "native";

    t = tic; % Start timer

    % Execute various demo functions
    if g1, phxex_buggy(false); end
    if g1, phxex_catastrophy; end
    if g1, phxex_conveyors; end
    if g1, phxex_determinism(200); end
    if g1, phxex_gears; end
    if g1, phxex_gravity(100, 128, false); end
    if g1, phxex_charges(50); end
    if g2, phxex_joints(false); end
    if g2, phxex_multisim; end
    if g2, phxex_multiview; end
    if g2, phxex_noview; end
    if g1, phxex_rotmagdip; end
    if g2, phxex_shapes; end
    if g1, phxex_slide; end
    if g1, phxex_soil; end
    if g1, phxex_springs(false); end
    if g1, phxex_terrain; end
    if g1, phxex_textures; end
    if g2, phxex_trainwheel; end
    if g1, phxex_wankel(false); end

    toc(t) % Display elapsed time

end