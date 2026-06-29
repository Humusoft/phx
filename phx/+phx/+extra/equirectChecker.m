function rgb = equirectChecker(res, count, trsh, filt)
%EQUIRECTCHECKER  Infinite top and bottom planes with checker pattern
%
% default phx.extra.equirectChecker(2048, 10, 0, [2 4]);

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    supersample = 2;
    res = res*supersample;
    W = res;
    H = res/4;
    
    r = tan(linspace(0, pi/2, H))';
    a = linspace(0, 2*pi, W);
    x = cos(a).*r;
    y = sin(a).*r;
    
    I = sin(x*count).*sin(y*count) > trsh;
    
    I = I.*(linspace(1, 0, H)'.^1.5);
    I = 1 - I;
    I = [I; flipud(I)];
    I = uint8((0.5 + I*0.5)*255);
    
    I = repmat(I, [1 1 3]);
    if filt(1) > 0
        I = imgaussfilt(I, filt, "Padding", "circular");
    end
    
    rgb = imresize(I, [H*2 W]/supersample);
    
end
