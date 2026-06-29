function XYZ  = computeFieldArrows(grid, pos, charge, count, seg, segLen)
%COMPUTEFIELDARROWS  Trace field-line arrow polylines from a charge configuration.
% grid: 3xN array
% pos: 3xM array
% charge: row vector
% count: scalar
% seg: scalar
% segLen: scalar
% XYZ: array of 3 x count*seg

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    XYZ = zeros(3, count*seg);
    si = 1;
    F = zeros(3, 1);
    
    for h = 1:count
        point = grid(:, h);
        XYZ(:, si) = point;
        for m = 1:(seg - 2)
            dp = pos - point;
            l = sqrt(sum(dp.^2));
            l = l.*l.*l;
            F = sum(dp.*charge./l, 2);
            Flen = sqrt(F(1)*F(1) + F(2)*F(2) + F(3)*F(3));
            F = segLen*F./Flen;
            point = point + F;
            si = si + 1;
            XYZ(:, si) = point;
        end
        arr = [-F(2); F(1); F(3)];
        XYZ(:, si + 1) = XYZ(:, si - 1) + 0.25*arr;
        si = si + 2;
    end

end