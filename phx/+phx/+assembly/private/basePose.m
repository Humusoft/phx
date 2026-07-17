function T = basePose(Options, component)
%basePose Resolve the base-pose options shared by the phx.assembly builders
%
%   T = basePose(Options, component) turns the Position, Orientation and
%   EulerAngles fields of an options struct into a single 4x4 base
%   transformation matrix. Orientation (a 3x3 rotation matrix) and
%   EulerAngles (z->y->x order) are alternatives; when both request a
%   rotation the error phx:<component>:conflictingOptions is raised.
%
%   See also phx.assembly.import, phx.assembly.arena

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    if ~isequal(Options.Orientation, eye(3)) && any(Options.EulerAngles)
        error("phx:" + component + ":conflictingOptions", "Specify the base rotation either as Orientation or as EulerAngles, not both.");
    end

    T = eye(4);
    if any(Options.EulerAngles)
        T(1:3, 1:3) = phx.internal.Math.rot321(Options.EulerAngles);
    else
        T(1:3, 1:3) = Options.Orientation;
    end
    T(1:3, 4) = Options.Position;
end
