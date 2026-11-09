function phxex_cantilever
% PHXEX_CANTILEVER Flexible cantilever - a ball dropped on a rubber beam
%
% A rubber beam (phx.FlexibleBody of quadratic tetrahedra, E = 20 MPa) is
% set into a wall at one end and carries a plate at the other; both are
% welded into it with phx.IntersectionWeld. The beam sags under its own
% weight and the plate, then a ball falls onto the plate, rides the beam
% down and rolls off as it swings back. The beam is coloured by its von
% Mises stress, 0 to 400 kPa.

%   Copyright 2026 HUMUSOFT s.r.o.

    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraTarget", [0.5 0 1.0], ...
        "DefaultCameraPosition", [1.6 -2.9 2.0]);

    phx.Body(ax, "Type", "static", "Position", [2 0 -0.1], ...
        "Shape", {"Box", "Size", [6 3 0.2], "Color", [0.7 0.7 0.7], "Texture", "tiles"});
    wall = phx.Body(ax, "Type", "static", "Position", [-0.15 0 0.7], ...
        "Shape", {"Box", "Size", [0.3 0.8 1.4], "Color", [0.55 0.5 0.45]});

    % The beam, 1 m out of the wall, its root 6 cm deep in it
    L = 1; embed = 0.06;
    [V, T] = boxTetrahedra([L + embed, 0.12, 0.1], 0.06);
    V(:, 1) = V(:, 1) + (L + embed)/2 - embed;
    beam = phx.FlexibleBody(ax, "Vertices", V, "Tetrahedra", T, "Position", [0 0 1], ...
        "Density", 1100, "YoungsModulus", 20e6, "PoissonsRatio", 0.45, "Damping", 3, "Colormap", turbo(64), "ColorRange", [0 6e5]);
    plate = phx.Body(ax, "Position", [L - 0.12 0 1.05], ...
        "Shape", {"Box", "Size", [0.24 0.3 0.04], "Color", [0.85 0.65 0.2]}, ...
        "Mass", 2, "Inertia", 2*[0.0916 0.0592 0.1476]/12);
    phx.IntersectionWeld(beam, wall);
    phx.IntersectionWeld(beam, plate);

    ball = phx.Body(ax, "Position", [L - 0.12 0 2.4], "Shape", {"Sphere", "Radius", 0.1}, ...
        "Mass", 4, "Inertia", 0.4*4*0.07^2*[1 1 1]);

    sim = phx.Simulation(ax);
    tip = abs(V(:, 1) - max(V(:, 1))) < 1e-9;
    lowest = 0;
    while sim.Time < 4
        sim.step(0.02, 4);
        X = beam.CurrentVertices;
        sag = 1 - mean(X(tip, 3));
        lowest = max(lowest, sag);
        viewer.displayText(sprintf("t = %4.2f s   tip sag %3.0f mm", sim.Time, 1e3*sag));
    end
    fprintf("The tip sagged by up to %.0f mm; the ball ended at z = %.2f m.\n", 1e3*lowest, ball.Position(3));

end

function [V, T] = boxTetrahedra(sz, cell)
% Box centred at the origin, cut into cells of about the given size, each
% split into six tetrahedra
    n = max(1, round(sz/cell));
    [x, y, z] = ndgrid(linspace(-sz(1)/2, sz(1)/2, n(1) + 1), linspace(-sz(2)/2, sz(2)/2, n(2) + 1), ...
        linspace(-sz(3)/2, sz(3)/2, n(3) + 1));
    V = [x(:) y(:) z(:)];
    corner = @(i, j, k) sub2ind(n + 1, i, j, k);
    split = [1 2 4 8; 1 2 6 8; 1 3 4 8; 1 3 7 8; 1 5 6 8; 1 5 7 8];
    T = zeros(6*prod(n), 4);
    m = 0;
    for i = 1:n(1)
        for j = 1:n(2)
            for k = 1:n(3)
                c = [corner(i, j, k) corner(i + 1, j, k) corner(i, j + 1, k) corner(i + 1, j + 1, k) ...
                    corner(i, j, k + 1) corner(i + 1, j, k + 1) corner(i, j + 1, k + 1) corner(i + 1, j + 1, k + 1)];
                T(m + (1:6), :) = c(split);
                m = m + 6;
            end
        end
    end
end
