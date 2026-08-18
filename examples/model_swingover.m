function model_swingover
% MODEL_SWINGOVER  Scene for the "swing the load over a wall" Simulink demo.
%
%   EXPERIMENTAL - not part of the documented example set.
%
%   The same mechanism as phxex_swingover: a trolley on a rail carries a
%   rigid arm with a heavy ball, and a wall of dry-stacked bricks stands in
%   the middle of the travel, higher than the hanging ball. Here the trolley
%   is kinematic and its x position is prescribed from Simulink
%   (Trolley.Position(1)), so the pendulum and the wall are simulated while
%   the trolley follows the command; the prismatic joint of the MATLAB demo
%   is therefore left out. The swing angle (Arm.EulerAngles(2)) and the ball
%   pose (Ball.Position) go back to Simulink as outputs.
%
%   Two numbers differ from phxex_swingover, both because a kinematic
%   trolley has no force limit: the travel is shorter (the trolley parks
%   1.4 m from the wall instead of 2.2 m) and the wall is 0.85 m instead of
%   0.90 m high. With the long travel of the MATLAB demo the prescribed run
%   across pumps the pendulum right over the top of its hinge.
%
%   Builds the bodies and saves them to saved_swingover.mat.
%
%   See also phxex_sim_swingover, phxex_swingover, PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.

    % Viewer setup
    ax = cla(clf);
    phx.extra.Viewer(ax, "ViewMode", "plain");

    zPivot = 1.75;               % hinge height above the ground (m)
    armLen = 1.2;                % hinge to ball centre (m)
    rBall = 0.13;                % ball radius (m)
    mBall = 14;                  % ball mass (kg)
    xStart = -1.4;               % parking position (m)

    % Ground
    Ground = phx.Body("Type", "static", "Position", [0 0 -0.15], ...
        "Shape", {"Box", "Size", [12 6 0.3], "Color", 0.85, "Texture", "tiles"}, ...
        "Name", "Ground");

    % Rail beam on two posts
    zRail = zPivot + 0.23;
    Posts = phx.Body.empty;
    for xPost = [-3.5 3.5]
        Posts(end + 1) = phx.Body("Type", "static", ...
            "Position", [xPost 0 (zRail - 0.07)/2], ...
            "Shape", {"Box", "Size", [0.14 0.14 zRail - 0.07], ...
                      "Color", [0.45 0.47 0.5]});                  %#ok<AGROW>
    end
    Rail = phx.Body("Type", "static", "Position", [0 0 zRail], ...
        "Shape", {"Box", "Size", [7.2 0.14 0.14], "Color", [0.45 0.47 0.5]}, ...
        "Name", "Rail");

    % The trolley: kinematic, its x position comes from Simulink
    Trolley = phx.Body("Type", "kinematic", "Position", [xStart 0 zPivot + 0.08], ...
        "Shape", {"Box", "Size", [0.3 0.2 0.16], "Color", [0.85 0.5 0.15]}, ...
        "Name", "Trolley");

    % Arm and ball, hinged under the trolley and starting a few degrees off
    % vertical so that the pumping has something to grow
    beta = -15*pi/180;                      % rotation about y, ball towards +x
    dir = [-sin(beta) 0 -cos(beta)];        % from the hinge towards the ball
    pivot = [xStart 0 zPivot];
    Arm = phx.Body("Position", pivot + armLen/2*dir, "EulerAngles", [0 beta 0], ...
        "Shape", {"Cylinder", "Axis", "z", "Radius", 0.025, "Height", armLen, ...
                  "Color", [0.35 0.37 0.4], "Material", "metal"}, ...
        "Name", "Arm");
    Ball = phx.Body("Position", pivot + armLen*dir, ...
        "Shape", {"Sphere", "Radius", rBall, "Color", 0.3, "Material", "metal"}, ...
        "Mass", mBall, "Inertia", 2/5*mBall*rBall^2, "Name", "Ball");
    Hinge = phx.RevoluteJoint(Trolley, Arm, "PointA", [0 0 -0.08], ...
        "PointB", [0 0 armLen/2], "AxisA", [0 1 0], "AxisB", [0 1 0], ...
        "Visible", false, "Name", "Hinge");
    Weld = phx.FixedJoint(Arm, Ball, "PointA", [0 0 -armLen/2], ...
        "Visible", false, "Name", "Weld");
    Trace = phx.Trace(Ball, "TracePoints", 900, "Color", [1 0.8 0.2], ...
        "Overlay", true, "Name", "Trace");

    % The wall, laid across the travel in the middle of the rail
    Bricks = phx.assembly.wall("Size", [1.5 0.1 0.85], "EulerAngles", [0 0 pi/2], ...
        "Rows", 6, "Columns", 5, "Color", [0.72 0.4 0.32]);

    save("saved_swingover.mat", "Ground", "Posts", "Rail", "Trolley", ...
        "Arm", "Ball", "Hinge", "Weld", "Trace", "Bricks");
end
