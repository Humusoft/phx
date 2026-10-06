function F = computeMonopoleForces(pos, charge, attractivity)
%phx.internal.computeMonopoleForces Pairwise central forces of a group of point charges
% pos: 3xN array of body positions
% charge: vector of N charges
% attractivity: scalar
% F: 3xN array of central forces
%
% The MEX build of this function is used where it exists; this file is
% the fallback with the same results.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    pos = pos';
    charge = charge(:);
    n = size(pos, 1);
    F = zeros(n, 3);

    for a = 1:(n - 1)
        b = a + 1;
        dp = pos(b:end, :) - pos(a, :);
        l = sqrt(sum(dp.^2, 2));
        l = l.*l.*l;
        Fab = dp.*charge(a).*charge(b:end)./l;
        F(a, :) = F(a, :) + sum(Fab, 1);
        F(b:end, :) = F(b:end, :) - Fab;
    end

    F = attractivity*F';

end
