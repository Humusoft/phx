function phxex_softball
% PHXEX_SOFTBALL Soft ball dropped on the edge of a rigid cube

%   Copyright 2026 HUMUSOFT s.r.o.

    clf; view(3); axis("equal"); grid("on"); camlight("headlight");

    phx.Body("Type", "static");
    phx.Body("Type", "static", "Position", [3 0 -2], "Shape", {"Box", "Size", [3 2 0.5]}, "EulerAngles", [0 -0.5 0]);
    phx.Body("Type", "static", "Position", [0 0 -4]);

    xlim([-1 5]); ylim(ylim); zlim([-5 2]);

    % The subdivided icosahedron repeats the shared vertices and winds its
    % faces inwards; a soft body needs them merged and wound outwards
    [V, ~, F] = phx.internal.Geometry.icosphere(5);
    [V, ~, id] = uniquetol(V, 1e-9, "ByRows", true);
    F = id(F(:, [1 3 2]));
    phx.SoftBody("Vertices", 0.5*V.*[1 1 1], "Faces", F, "Position", [0.5 0 2], ...
        "Mass", 1, "Compliance", 1e-3, "Pressure", 15, "Iterations", 10);

    sim = phx.Simulation;
    sim.step(4, 400);
    delete(sim);

end
