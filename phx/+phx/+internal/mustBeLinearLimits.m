function mustBeLinearLimits(value)
%phx.internal.mustBeLinearLimits Validate joint limits along an axis
%
%   phx.internal.mustBeLinearLimits(value) throws an error unless every row
%   of the N-by-2 value is a pair [lower upper] of limits along an axis:
%   not NaN and lower <= upper. Infinite limits are allowed: -Inf or Inf is
%   no stop on that side, [-Inf Inf] leaves the axis free.
%
%   See also phx.internal.mustBeAngularLimits, phx.PrismaticJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    if any(isnan(value), "all")
        error("phx:Joint:invalidLimits", "Joint limits must not be NaN.");
    end
    if any(value(:, 1) > value(:, 2))
        error("phx:Joint:invalidLimits", ...
            "Joint limits must be pairs [lower upper] with lower <= upper; use [-Inf Inf] to leave the axis free.");
    end

end
