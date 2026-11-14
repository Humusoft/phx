function mustBeAngularLimits(value)
%phx.internal.mustBeAngularLimits Validate joint limits about an axis
%
%   phx.internal.mustBeAngularLimits(value) throws an error unless every
%   row of the N-by-2 value is a pair [lower upper] of limits about an axis
%   (rad): either [-Inf Inf], which leaves the rotation free, or two finite
%   angles within (-pi, pi) with lower <= upper.
%
%   The engine measures the angle wrapped to [-pi, pi], so a stop on one
%   side alone is not possible: the rest of the circle would close the
%   other side at the back point. Angles of pi and beyond are reserved for
%   limits over several turns.
%
%   See also phx.internal.mustBeLinearLimits, phx.RevoluteJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    phx.internal.mustBeLinearLimits(value);
    free = value(:, 1) == -Inf & value(:, 2) == Inf;
    inside = all(abs(value) < pi, 2);
    if ~all(free | inside)
        error("phx:Joint:invalidLimits", ...
            "Angular joint limits must both lie within (-pi, pi), or be [-Inf Inf] to leave the rotation free.");
    end

end
