function phxex_cartpendulum
% PHXEX_CARTPENDULUM Pendulum on a cart held by a spring
%
% A cart slides along a static rail on a phx.PrismaticJoint and is tied to
% the end of the rail by a damped phx.Spring. A pendulum rod hangs under
% the cart on a phx.RevoluteJoint, and a sleeve slides along the rod on a
% second phx.PrismaticJoint, hung from the top of the rod by another
% spring. The cart starts off its rest position; the motion passes between
% the cart, the pendulum and the sleeve and slowly dies out. A phx.Trace
% draws the path of the sleeve.
%
% See also phx.PrismaticJoint, phx.RevoluteJoint, phx.Spring, phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.

    % Clear the current figure, view the XZ plane, and configure lighting
    clf; view(0, 0); axis("equal"); camlight("headlight"); grid("on");
    xlim([-1.5 1.5]); ylim([-0.2 0.2]); zlim([0 1.5]);

    % Bodies: a static rail, a cart on it, a pendulum rod under the cart and
    % a sleeve on the rod
    rail = phx.Body("Type", "static", "Position", [0 0 1.2], "Shape", {"Box", "Size", [2.4 0.05 0.05], "Color", [0.6 0.6 0.6]});
    cart = phx.Body("Position", [0.5 0 1.2], "Shape", {"Box", "Size", [0.2 0.15 0.1], "Color", [0.1 0.2 0.3]});
    rod = phx.Body("Position", [0.5 0 0.72], "Shape", {"Box", "Size", [0.04 0.04 0.8], "Color", [0.3 0.3 0.3]});
    sleeve = phx.Body("Position", [0.5 0 0.82],"Shape", {"Cylinder", "Diameter", 0.2, "Height", 0.1, "Color", [0.8 0 1]});

    % Slider: the cart moves along the joint axis (local Z of the joint
    % frames), turned here to the world X axis
    phx.PrismaticJoint(rail, cart, "EulerAnglesA", [0 pi/2 0], "EulerAnglesB", [0 pi/2 0]);

    % Hinge about the world Y axis, joining the bottom of the cart to the
    % top of the rod
    phx.RevoluteJoint(cart, rod, "PointA", [0 0 -0.08], "PointB", [0 0 0.4], "AxisA", [0 1 0], "AxisB", [0 1 0]);

    % Slider along the rod: the default joint axis is the local Z of both
    % bodies, the long axis of the rod
    phx.PrismaticJoint(rod, sleeve);

    % Damped spring from the end of the rail to the cart, relaxed with the
    % cart at x = 0
    phx.Spring(rail, cart, "PointA", [-1.2 0 0.04], "PointB", [-0.1 0 0.04], "FreeLength", 1.1, ...
        "Stiffness", 60, "Damping", 1,"Color", [0.9 0.1 0.1]);

    % Spring from the top of the rod to the sleeve, drawn just in front of
    % the rod
    phx.Spring(rod, sleeve, "PointA", [0 -0.03 0.4], "PointB", [0 -0.03 0], "FreeLength", 0.065, ...
        "Stiffness", 60, "Damping", 0.2,"Color", [0.9 0.1 0.1], "ColorRange", [80 -20], "Colormap", hsv(64));

    % Path of the sleeve
    phx.Trace(sleeve, "TracePoints", 100, "Color", [0.2 0.4 0.9]);

    sim = phx.Simulation("PacingRate", 1);    % run in real time
    sim.step(10, 1000);

end
