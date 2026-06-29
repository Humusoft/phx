function model_buggy
% MODEL_BUGGY

%   Copyright 2026 HUMUSOFT s.r.o.

    % Viewer setup
    ax = cla(clf);
    phx.extra.Viewer(ax, "ViewMode", "plain");

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Create a buggy chassis from an STL model
    chassis = phx.Body("Position", [0 0 10], "Shape", {"STL", "Source", resdir+"BuggyBody.stl", "Details", 0.2, "Scale", 0.01, "Color", [0.9 0.9 0.9], "Envelope", "convex"}, "Mass", 300, "Inertia", [100 100 100], "Name", "Chassis");

    % Create wheels with marks
    shpWheel = phx.shape.STL("Source", resdir+"BuggyWheelFL.stl", "Details", 0.2, "Scale", 0.01, "Color", [0.3 0.3 0.3], "Envelope", "cylinder", "SkeletPoints", [1 0 1.6; 1 0 2.2], "SkeletStyle", "line", "SkeletColor", [1 1 1]);
    wheelFL = phx.Body("Position", [-9 -7 4], "EulerAngles", [0 0 -pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelFL");
    wheelFR = phx.Body("Position", [-9 7 4], "EulerAngles", [0 0 pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelFR");
    wheelRL = phx.Body("Position", [11 -7 4], "EulerAngles", [0 0 -pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelRL");
    wheelRR = phx.Body("Position", [11 7 4], "EulerAngles", [0 0 pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelRR");

    % Create arms
    shpArm = phx.shape.Box("Size", [0.5 4 0.5], "Color", [0.4 0.8 1]);
    armFL = phx.Body("Position", [-9 -4 4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmFL");
    armFR = phx.Body("Position", [-9 4 4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmFR");
    armRL = phx.Body("Position", [11 -4 4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmRL");
    armRR = phx.Body("Position", [11 4 4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmRR");
    
    % Create joints connecting arms with chassis and wheels
    s = 0; % -0.5 "steering angle"
    phx.RevoluteJoint(chassis, armFL, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [-9 -1.5 -6], "PointB", [0 2.5 0], "Visible", false);
    phx.RevoluteJoint(armFL, wheelFL, "AxisA", [s 1 0], "AxisB", [-1 0 0], "PointA", [0 -3 0], "PointB", [0 0 0], "Visible", false);
    phx.RevoluteJoint(chassis, armFR, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [-9 1.5 -6], "PointB", [0 -2.5 0], "Visible", false);
    phx.RevoluteJoint(armFR, wheelFR, "AxisA", [s 1 0], "AxisB", [1 0 0], "PointA", [0 3 0], "PointB", [0 0 0], "Visible", false);
    phx.RevoluteJoint(chassis, armRL, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [11 -1.5 -6], "PointB", [0 2.5 0], "Visible", false);
    phx.RevoluteJoint(armRL, wheelRL, "AxisA", [0 1 0], "AxisB", [-1 0 0], "PointA", [0 -3 0], "PointB", [0 0 0], "Visible", false);
    phx.RevoluteJoint(chassis, armRR, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [11 1.5 -6], "PointB", [0 -2.5 0], "Visible", false);
    phx.RevoluteJoint(armRR, wheelRR, "AxisA", [0 1 0], "AxisB", [1 0 0], "PointA", [0 3 0], "PointB", [0 0 0], "Visible", false);

    % Create springs between arms and chassis
    phx.Spring(chassis, armFL, "Stiffness", 8e3, "Damping", 1000, "FreeLength", 4.5, "PointA", [-9 -1.5 -3], "Color", [1 0.5 0], "Name", "SpringFL");
    phx.Spring(chassis, armFR, "Stiffness", 8e3, "Damping", 1000, "FreeLength", 4.5, "PointA", [-9 1.5 -3], "Color", [1 0.5 0], "Name", "SpringFR");
    phx.Spring(chassis, armRL, "Stiffness", 8e3, "Damping", 1000, "FreeLength", 4.5, "PointA", [11 -1.5 -3], "Color", [1 0.5 0], "Name", "SpringRL");
    phx.Spring(chassis, armRR, "Stiffness", 8e3, "Damping", 1000, "FreeLength", 4.5, "PointA", [11 1.5 -3], "Color", [1 0.5 0], "Name", "SpringRR");

    save("saved_buggy.mat", "chassis", "wheelFL", "wheelFR", "wheelRL", "wheelRR", "armFL", "armFR", "armRL", "armRR");

end