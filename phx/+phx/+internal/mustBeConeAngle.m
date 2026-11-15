function mustBeConeAngle(value)
%phx.internal.mustBeConeAngle Validate the half-angles of a joint cone
%
%   phx.internal.mustBeConeAngle(value) throws an error unless every
%   element of value is a half-angle of a cone (rad) within [0, pi]: 0 locks
%   the swing in that direction, pi leaves it free.
%
%   See also phx.internal.mustBeAngularLimits, phx.SphericalJoint

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    if ~all(value >= 0 & value <= pi, "all")
        error("phx:Joint:invalidLimits", ...
            "The cone half-angles must lie within [0, pi]; pi leaves that direction free.");
    end

end
