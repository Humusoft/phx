function P = transformPoint(m, p)
%TRANSFORMPOINT  Apply a 4-by-4 transform (column-major M) to 3-D point P.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    P = zeros(1, 3);
    P(1) = m(1)*p(1) + m(5)*p(2) + m(9)*p(3) + m(13);
    P(2) = m(2)*p(1) + m(6)*p(2) + m(10)*p(3) + m(14);
    P(3) = m(3)*p(1) + m(7)*p(2) + m(11)*p(3) + m(15);

end