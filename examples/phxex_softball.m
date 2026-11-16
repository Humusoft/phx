function phxex_softball
% PHXEX_SOFTBALL Soft ball dropped on the edge of a rigid cube

%   Copyright 2026 HUMUSOFT s.r.o.

    clf; view(3); axis("equal"); grid("on"); camlight("headlight");

    phx.Body("Type", "static");
    phx.Body("Type", "static", "Position", [3 0 -2], "Shape", {"Box", "Size", [3 2 0.5]}, "EulerAngles", [0 -0.5 0]);
    phx.Body("Type", "static", "Position", [0 0 -4], "ZAxis", [0.3 0 1]);

    xlim([-1 5]); ylim(ylim); zlim([-5 2]);

    phx.SoftBody("Shape", {"Sphere", "Diameter", 1, "ElementResolution", 24}, "Position", [0.5 0 2], ...
        "Mass", 1, "Compliance", 1e-3, "Pressure", 15, "Iterations", 10);

    sim = phx.Simulation("PacingRate", 1);
    sim.step(4, 400);
    delete(sim);

end
