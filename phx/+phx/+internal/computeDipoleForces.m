function F = computeDipoleForces(pos, charge, attractivity)
%phx.internal.computeDipoleForces Pairwise forces between the poles of a group of dipoles
% pos: 3x2N array of pole positions, columns 1..N the +Axis poles and
%      N+1..2N the -Axis poles of the same bodies
% charge: vector of N dipole charges (+charge at the +Axis pole)
% attractivity: scalar
% F: 3x2N array of forces at the poles
%
% The two poles of one body do not interact. The MEX build of this
% function is used where it exists; this file is the fallback with the
% same results.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    pos = pos';
    n = numel(charge);
    charge = [charge(:); -charge(:)];
    body = [1:n 1:n]; % pole -> body index
    F = zeros(2*n, 3);

    for a = 1:(2*n - 1)
        b = a + 1;
        dp = pos(b:end, :) - pos(a, :);
        l = sqrt(sum(dp.^2, 2));
        l = l.*l.*l;
        Fab = dp*charge(a).*charge(b:end)./l;
        Fab(body(b:end) == body(a), :) = 0; % the two poles of the same body do not interact
        F(a, :) = F(a, :) + sum(Fab, 1);
        F(b:end, :) = F(b:end, :) - Fab;
    end

    F = attractivity*F';

end
