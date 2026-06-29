function model_stand
% MODEL_STAND

%   Copyright 2026 HUMUSOFT s.r.o.

    % Viewer setup
    ax = cla(clf);
    phx.extra.Viewer(ax, "ViewMode", "plain");

    % Load buggy
    model = load("saved_buggy.mat");
    propagate(cell2mat(struct2cell(model)), "ParentAxes", ax);

    % Add stands to the model structure
    model.stands(1) = phx.Body(ax, "Type", "kinematic", "Position", [-9 -7 0], "Shape", {"Cylinder", "Diameter", 10, "Height", 2, "Color", [1 1 1]}, "Name", "StandFL");
    model.stands(2) = phx.Body(ax, "Type", "kinematic", "Position", [-9  7 0], "Shape", {"Cylinder", "Diameter", 10, "Height", 2, "Color", [1 1 1]}, "Name", "StandFR");
    model.stands(3) = phx.Body(ax, "Type", "kinematic", "Position", [11 -7 0], "Shape", {"Cylinder", "Diameter", 10, "Height", 2, "Color", [1 1 1]}, "Name", "StandRL");
    model.stands(4) = phx.Body(ax, "Type", "kinematic", "Position", [11  7 0], "Shape", {"Cylinder", "Diameter", 10, "Height", 2, "Color", [1 1 1]}, "Name", "StandRR");

    save("saved_stand.mat", "-struct", "model");

end