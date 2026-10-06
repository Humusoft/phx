function XY = computeBuoyancyXY(M, P, counts)
%phx.internal.computeBuoyancyXY World x, y coordinates of the buoyancy sampling points
% M: 16xN array, the 4x4 transforms of N bodies as columns
% P: 3xT array of local sampling points of all bodies, body after body
% counts: vector of N point counts per body (sum T)
% XY: 2xT array of world x, y coordinates
%
% The MEX build of this function is used where it exists; this file is
% the fallback with the same results.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    XY = zeros(2, size(P, 2));
    j = 0;
    for a = 1:numel(counts)
        c = j + (1:counts(a));
        j = j + counts(a);
        m = reshape(M(:, a), 4, 4);
        XY(:, c) = m(1:2, 1:3)*P(:, c) + m(1:2, 4);
    end

end
