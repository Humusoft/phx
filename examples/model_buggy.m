function model_buggy
% MODEL_BUGGY Build the buggy scene and save it to saved_buggy.mat

%   Copyright 2026 HUMUSOFT s.r.o.

    % Viewer setup
    ax = cla(clf);
    phx.extra.Viewer(ax, "ViewMode", "plain");

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res") + filesep;

    % Create a buggy chassis from an STL model (in millimetres)
    chassis = phx.Body("Position", [0 0 1], "Shape", {"Mesh", "Source", resdir+"BuggyBody.stl", "Details", 0.2, "Scale", 0.001, "Color", [0.9 0.9 0.9], "Envelope", "convex", "Material", "metal", "Texture", "circles", "Color", 1, "TextureBlend", 0.25}, "Mass", 300, "Inertia", [100 100 100], "Name", "Chassis");

    % Create wheels with marks
    shpWheel = phx.shape.Mesh("Source", resdir+"BuggyWheelFL.stl", "Details", 0.2, "Scale", 0.001, "Color", [0.3 0.3 0.3], "Envelope", "cylinder", "Axis", "x");
    wheelFL = phx.Body("Position", [-0.9 -0.7 0.4], "EulerAngles", [0 0 -pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelFL");
    wheelFR = phx.Body("Position", [-0.9 0.7 0.4], "EulerAngles", [0 0 pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelFR");
    wheelRL = phx.Body("Position", [1.1 -0.7 0.4], "EulerAngles", [0 0 -pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelRL");
    wheelRR = phx.Body("Position", [1.1 0.7 0.4], "EulerAngles", [0 0 pi/2], "Shape", shpWheel, "Mass", 30, "Inertia", [2 2 2], "Name", "WheelRR");
    for wheel = [wheelFL wheelFR wheelRL wheelRR]
        line([0.1 0.1], [0 0], [0.16 0.22], "LineWidth", 2, "Color", [1 1 1], "Parent", wheel.Graphics);
    end

    % Create arms
    shpArm = phx.shape.Box("Size", [0.05 0.4 0.05], "Color", [0.4 0.8 1]);
    armFL = phx.Body("Position", [-0.9 -0.4 0.4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmFL");
    armFR = phx.Body("Position", [-0.9 0.4 0.4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmFR");
    armRL = phx.Body("Position", [1.1 -0.4 0.4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmRL");
    armRR = phx.Body("Position", [1.1 0.4 0.4], "Shape", shpArm, "Collisions", false, "Mass", 20, "Inertia", 1, "Name", "ArmRR");

    % Create joints connecting arms with chassis and wheels
    s = 0; % -0.5 "steering angle"
    phx.RevoluteJoint(chassis, armFL, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [-0.9 -0.15 -0.6], "PointB", [0 0.25 0], "Visible", false);
    phx.RevoluteJoint(armFL, wheelFL, "AxisA", [s 1 0], "AxisB", [-1 0 0], "PointA", [0 -0.3 0], "PointB", [0 0 0], "Visible", false);
    phx.RevoluteJoint(chassis, armFR, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [-0.9 0.15 -0.6], "PointB", [0 -0.25 0], "Visible", false);
    phx.RevoluteJoint(armFR, wheelFR, "AxisA", [s 1 0], "AxisB", [1 0 0], "PointA", [0 0.3 0], "PointB", [0 0 0], "Visible", false);
    phx.RevoluteJoint(chassis, armRL, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [1.1 -0.15 -0.6], "PointB", [0 0.25 0], "Visible", false);
    phx.RevoluteJoint(armRL, wheelRL, "AxisA", [0 1 0], "AxisB", [-1 0 0], "PointA", [0 -0.3 0], "PointB", [0 0 0], "Visible", false);
    phx.RevoluteJoint(chassis, armRR, "AxisA", [1 0 0], "AxisB", [1 0 0], "PointA", [1.1 0.15 -0.6], "PointB", [0 -0.25 0], "Visible", false);
    phx.RevoluteJoint(armRR, wheelRR, "AxisA", [0 1 0], "AxisB", [1 0 0], "PointA", [0 0.3 0], "PointB", [0 0 0], "Visible", false);

    % Create springs between arms and chassis
    phx.Spring(chassis, armFL, "Stiffness", 28e3, "Damping", 1900, "FreeLength", 0.45, "PointA", [-0.9 -0.15 -0.3], "Color", [1 0.5 0], "Name", "SpringFL");
    phx.Spring(chassis, armFR, "Stiffness", 28e3, "Damping", 1900, "FreeLength", 0.45, "PointA", [-0.9 0.15 -0.3], "Color", [1 0.5 0], "Name", "SpringFR");
    phx.Spring(chassis, armRL, "Stiffness", 28e3, "Damping", 1900, "FreeLength", 0.45, "PointA", [1.1 -0.15 -0.3], "Color", [1 0.5 0], "Name", "SpringRL");
    phx.Spring(chassis, armRR, "Stiffness", 28e3, "Damping", 1900, "FreeLength", 0.45, "PointA", [1.1 0.15 -0.3], "Color", [1 0.5 0], "Name", "SpringRR");

    save("saved_buggy.mat", "chassis", "wheelFL", "wheelFR", "wheelRL", "wheelRR", "armFL", "armFR", "armRL", "armRR");

end
