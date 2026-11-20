function phxex_airbag(withAirbag)
% PHXEX_AIRBAG Sled crash test - a belted dummy thrown into an airbag
%
% A URDF dummy (res/human.urdf) stands in front of the seat on a sled;
% the retractor spring of its lap belt pulls it into the seat. The sled,
% carrying the seat and a dashboard, is then pushed along the floor into a
% wall at about 30 km/h. Its bumper rides on a slide braked by a damper,
% the crumple zone. On contact the airbag, a soft body welded into the
% dashboard by phx.IntersectionWeld, inflates within 25 ms; the elastic
% belt holds the dummy back at the waist while its torso and head swing
% into the bag.
%
% Input Arguments:
%     withAirbag - false runs the same crash without the airbag
%
% Example:
%     phxex_airbag          % with the airbag
%     phxex_airbag(false)   % the head meets the dashboard

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        withAirbag (1, 1) logical = true
    end

    resdir = fullfile(fileparts(mfilename("fullpath")), "res");
    boxInertia = @(m, s) m*(sum(s.^2) - s.^2)/12;

    figure(1);
    [viewer, ax] = phx.extra.Viewer("clear");

    % Floor and wall; the floor is slippery, the sled's deck is not (the
    % friction of a contact is the geometric mean of the two bodies' values)
    wallX = 8;
    phx.Body(ax, "Type", "static", "Position", [4 0 -0.1], "Friction", 0.02, ...
        "Shape", {"Box", "Size", [20 4 0.2], "Color", [0.7 0.7 0.7], "Texture", "tiles"});
    phx.Body(ax, "Type", "static", "Position", [wallX + 0.25 0 1], ...
        "Shape", {"Box", "Size", [0.5 3 2], "Color", [0.55 0.5 0.45]});

    % Sled with a crumple zone: the bumper slides back against a damper
    sledSize = [2.4 1.2 0.15];
    z0 = sledSize(3)/2 + 0.002;
    sled = phx.Body(ax, "Position", [0 0 z0], "Shape", {"Box", "Size", sledSize, "Color", [0.3 0.3 0.35]}, ...
        "Mass", 300, "Inertia", boxInertia(300, sledSize));
    bumperSize = [0.1 1.2 0.15];
    bumper = phx.Body(ax, "Position", [1.75 0 z0], "Shape", {"Box", "Size", bumperSize, "Color", [0.8 0.2 0.2]}, ...
        "Mass", 20, "Inertia", boxInertia(20, bumperSize));
    phx.PrismaticJoint(sled, bumper, "AxisA", [1 0 0], "AxisB", [1 0 0], "MutualCollisions", true);
    phx.Spring(sled, bumper, "PointA", [1.2 0 0], "PointB", [-0.05 0 0], "Stiffness", 0, "Damping", 6000);

    % Seat and dashboard, fixed to the sled
    top = sledSize(3) + 0.002;
    parts = {
        "cushion",   [0.55 0.5 0.1],  [-0.42 0 top + 0.37]
        "backrest",  [0.1 0.5 1.0],  [-0.75 0 top + 0.9]
        "dashboard", [0.4 1.1 0.95],  [0.5 0 top + 0.475]
        };
    for i = 1:size(parts, 1)
        [name, s, p] = parts{i, :};
        part = phx.Body(ax, "Position", p, "Shape", {"Box", "Size", s, "Color", [0.25 0.3 0.45]}, ...
            "Mass", 20, "Inertia", boxInertia(20, s));
        phx.FixedJoint(sled, part, "PointA", p - [0 0 z0]);
        if name == "dashboard"
            dashboard = part;
        end
    end

    % The dummy, standing in front of the seat
    dummy = phx.assembly.import(ax, fullfile(resdir, "human.urdf"), "Position", [0.0 0 top + 1.002]);

    % Lap belt from anchors behind the seat over the waist (the bottom of
    % the torso). Its retractor spring winds it in (a free length of zero),
    % pulls the dummy into the seat and holds it back in the crash
    phx.Rope([sled, dummy.torso, sled], "Points", [-0.85 0.3 0.45; 0.12 0 -0.05; -0.85 -0.3 0.45], ...
        "InitialLength", 0, "Stiffness", 250, "Damping", 300);

    % The airbag, welded into the back of the dashboard and kept slack until
    % the bumper touches the wall
    if withAirbag
        [V, F] = phx.shape.Sphere("Diameter", 2, "ElementResolution", 30).generateSurfaceMesh;
        bag = phx.SoftBody(ax, "Vertices", V.*[0.16 0.3 0.3], "Faces", F, "Position", [0.27 0 1.15], ...
            "Mass", 0.3, "Compliance", 1e-5, "Pressure", 0.3, "Iterations", 10, "VertexRadius", 0.01, ...
            "Color", [0.95 0.95 0.9]);
        phx.IntersectionWeld(bag, dashboard);
        phx.Function({bag, bumper}, @(o, p, dt, t) inflate(o, p, t, wallX));
    end

    % Once the dummy sits, push the sled up to speed, then let it run into
    % the wall
    phx.Function(sled, @(o, p, dt, t) drive(o, p, t, wallX));

    % On-board camera and the head's velocity record
    phx.Camera(sled, sled, "PointA", [-2.4 3.5 2.4], "PointB", [-0.1 0 0.9]);
    head = phx.Logger(dummy.head, "Frequency", 500, "Parameters", "LinearVelocity");
    rideLog = phx.Logger(sled, "Frequency", 500, "Parameters", "LinearVelocity");

    % Simulation - 2 ms steps for the impact
    sim = phx.Simulation(ax);
    while sim.Time < 4.5
        sim.step(0.02, 10);
        viewer.displayText(sprintf("t = %4.2f s   %3.0f km/h", sim.Time, 3.6*sled.LinearVelocity(1)));
    end
    delete(sim);

    % Crash figures from the velocity records
    t = rideLog.Time(:);
    v = rideLog.getChannel(1);
    [vMax, iMax] = max(v(:, 1));
    sledDecel = max((v(1:end - 5, 1) - v(6:end, 1))./(t(6:end) - t(1:end - 5)));
    % Head deceleration averaged over 10 ms (5 samples), as crash tests
    % filter the contact spikes
    vh = head.getChannel(1);
    th = head.Time(:);
    headDecel = max(vecnorm(vh(6:end, :) - vh(1:end - 5, :), 2, 2)./(th(6:end) - th(1:end - 5)));
    fprintf("Impact at %.0f km/h, t = %.2f s; peak sled deceleration %.0f g, peak head deceleration %.0f g (10 ms).\n", ...
        3.6*vMax, t(iMax), sledDecel/9.81, headDecel/9.81);

end

function drive(~, parents, t, wallX)
% Pushes the sled with 2500 N from 1.5 s on, until 2 m before the wall
    sled = parents{1};
    if t < 1.5
        return
    end
    sled.AppliedForce = [2500*(sled.Position(1) < wallX - 2) 0 0];
end

function inflate(obj, parents, t, wallX)
% Fires the airbag when the bumper touches the wall: the gas fills it in
% 25 ms and then escapes through the vents, so that the bag absorbs the
% occupant instead of throwing him back
    [bag, bumper] = parents{:};
    if isempty(obj.UserData) && bumper.Position(1) > wallX - 0.07
        obj.UserData = t;
    end
    if ~isempty(obj.UserData)
        s = t - obj.UserData;
        bag.Pressure = 10 + 790*min(s/0.025, 1)*exp(-max(s - 0.025, 0)/0.08);
    end
end
