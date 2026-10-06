function [Vsub, Vtot, C] = computeBuoyancy(M, P, dV, counts, level)
%phx.internal.computeBuoyancy Submerged volumes and centers of buoyancy of sampled bodies
% M: 16xN array, the 4x4 transforms of N bodies as columns
% P: 3xT array of local sampling points of all bodies, body after body
% dV: vector of T volumes of the sampling points
% counts: vector of N point counts per body (sum T)
% level: liquid level, a scalar or a vector of T levels, one per point
% Vsub: 1xN submerged volumes (points whose world z is below the level)
% Vtot: 1xN total sampled volumes
% C: 3xN local centers of buoyancy (centroids of the submerged points),
%    zero where Vsub is zero
%
% The MEX build of this function is used where it exists; this file is
% the fallback with the same results.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    n = numel(counts);
    Vsub = zeros(1, n);
    Vtot = zeros(1, n);
    C = zeros(3, n);
    dV = reshape(dV, 1, []);
    flat = isscalar(level);
    level = reshape(level, 1, []);

    j = 0;
    for a = 1:n
        c = j + (1:counts(a));
        j = j + counts(a);
        wz = M([3 7 11], a)'*P(:, c) + M(15, a);
        if flat
            sub = wz < level;
        else
            sub = wz < level(c);
        end
        v = dV(c);
        Vtot(a) = sum(v);
        vs = v(sub);
        Vsub(a) = sum(vs);
        if Vsub(a) > 0
            C(:, a) = P(:, c(sub))*vs'/Vsub(a);
        end
    end

end
