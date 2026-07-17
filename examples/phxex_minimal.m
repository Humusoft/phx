function phxex_minimal
% PHXEX_MINIMAL Example of minimal physics simulation code

%   Copyright 2026 HUMUSOFT s.r.o.

    clf; view(3); axis("equal"); grid("on"); camlight("headlight");

    phx.Body("Type", "static");
    phx.Body("Position", [0.6 -0.5 2]);

    sim = phx.Simulation;
    sim.step(1, 100, 1);

end