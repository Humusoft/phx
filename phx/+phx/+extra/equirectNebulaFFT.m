function [rgb, density] = equirectNebulaFFT(varargin)
%EQUIRECTNEBULAFFT  Fast deep-space nebula via FFT spectral synthesis.
%   The smooth gas/warp/dust/hue fields are generated with FFT power-law
%   spectra (1/f^beta) - all frequencies in a single O(N log N) transform
%   instead of a per-pixel octave loop - and domain warping is done by
%   periodic bilinear resampling. Stars are rendered at full resolution.
%
%   The fields are periodic, so the result is seamless across the 0/360 deg
%   meridian. Because the spectral field lives in the image plane (not on
%   the sphere) there is mild horizontal stretching toward the poles; the
%   pole rows are blended to their mean so the wrapped sphere has clean
%   poles instead of a pinwheel.
%
%   Name-Value options:
%     'Width'         image width in pixels          (default 1024)
%     'Height'        image height in pixels          (default Width/2, even)
%     'Palette'       'mixed'|'ember'|'azure'|'emerald' (default 'mixed')
%     'NebulaStrength'overall gas brightness         (default 1.0)
%     'NebulaCoverage'fraction of sky with gas, 0..1 (default 0.25)
%     'Warp'          domain-warp amount (turbulence) (default 0.6)
%     'DustStrength'  dark dust-lane strength, 0..1   (default 0.5)
%     'NumStars'      stars over the whole sphere      (default 9000)
%     'StarContrast'  brightness power law (>1)        (default 3)
%     'StarSize'      core glow radius in px (sigma)   (default 0.7)
%     'BloomStrength' halo strength on bright stars    (default 0.6)
%     'ColorJitter'   star colour spread, 0..1         (default 0.6)
%     'Seed'          RNG / noise seed                  (default 0)
%     'Roughness'  gas spectral slope, larger = smoother  (default 2.5)
%     'FeatureScale' number of largest gas clouds across   (default 3)
%
%   Outputs: RGB (H x W x 3, [0,1]), DENSITY (H x W gas density [0,1]).
%
%   Example:
%     rgb = equirectNebulaFFT('Width',4096,'Height',2048,'Seed',11);
%     imshow(rgb);

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    p = inputParser;
    addParameter(p,'Width',1024,    @(x)isnumeric(x)&&isscalar(x)&&x>=8);
    addParameter(p,'Height',[],      @(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=4));
    addParameter(p,'Palette','mixed',@(s)any(strcmpi(s,{'mixed','ember','azure','emerald'})));
    addParameter(p,'NebulaStrength',1.0, @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'NebulaCoverage',0.25,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'BandWidth',70,   @(x)isnumeric(x)&&isscalar(x)&&x>0&&x<=90);
    addParameter(p,'BandSoftness',0.45,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'BandCenter',0,   @(x)isnumeric(x)&&isscalar(x));
    addParameter(p,'BandTilt',0,     @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'Warp',0.6,       @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'DustStrength',0.5,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Roughness',2.5,  @(x)isnumeric(x)&&isscalar(x)&&x>=1);
    addParameter(p,'FeatureScale',3, @(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'NumStars',9000,  @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'StarContrast',3, @(x)isnumeric(x)&&isscalar(x)&&x>=1);
    addParameter(p,'StarSize',0.7,   @(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'BloomStrength',0.6,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'ColorJitter',0.6,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Seed',0,         @(x)isnumeric(x)&&isscalar(x));
    parse(p,varargin{:});
    o = p.Results;

    W = round(o.Width);
    if isempty(o.Height), H = round(W/2); else, H = round(o.Height); end
    H = 2 * round(H/2);
    s = o.Seed;

    % ---- FFT spectral fields (the fast part) ---------------------------
    % radial frequency grid (isotropic, width-equivalent) - computed once
    kx = mod((0:W-1) + W/2, W) - W/2;
    ky = mod((0:H-1) + H/2, H) - H/2;
    [KX, KY] = meshgrid(kx, ky*(W/H));
    kr = sqrt(KX.^2 + KY.^2);

    gasBase = fftField(kr, o.Roughness, o.FeatureScale,        s+5);
    wfx     = fftField(kr, 3.0,         max(o.FeatureScale-1,1), s+11);
    wfy     = fftField(kr, 3.0,         max(o.FeatureScale-1,1), s+23);
    hsel    = fftField(kr, 3.6,         1.5,                     s+71);
    dust    = fftField(kr, 2.6,         o.FeatureScale+1,        s+97);

    % ---- domain warp via periodic bilinear resampling ------------------
    [CX, CY] = meshgrid(0:W-1, 0:H-1);
    amp = o.Warp * 0.06 * W;                          % warp magnitude in px
    g = sampleWrap(gasBase, CX + amp.*(2*wfx-1), CY + amp.*(2*wfy-1));

    % ---- shape the gas into clouds vs empty space ----------------------
    v   = sort(g(:));
    thr = v(min(numel(v), max(1, round((1-o.NebulaCoverage)*numel(v)))));
    d   = smoothstep(thr-0.08, thr+0.22, g);

    % confine the gas to a latitude band so the (stretched) polar caps stay
    % clear - leaves clean starfield at the poles, like a galactic plane.
    lat   = (pi/2) - ((0:H-1).' + 0.5) / H * pi;          % H x 1
    lon   = (0:W-1) / W * 2*pi;                            % 1 x W
    cen   = o.BandCenter*pi/180 + (o.BandTilt*pi/180).*sin(lon);  % 1 x W
    maxA  = o.BandWidth*pi/180;
    coreA = maxA * (1 - o.BandSoftness);
    bandMask = 1 - smoothstep(coreA, maxA, abs(lat - cen));% H x W (or H x 1)
    d = d .* bandMask;
    density = d;

    % ---- colourise -----------------------------------------------------
    hsel = smoothstep(0.36, 0.64, hsel);
    [A, B, C] = palette(o.Palette);
    base = (1-hsel).*reshape(A,1,1,3) + hsel.*reshape(B,1,1,3);
    core = smoothstep(0.72, 0.98, d);
    col  = base + 0.55*core .* (reshape(C,1,1,3) - base);
    neb  = (d.^1.6) .* col * o.NebulaStrength;

    dustMask = smoothstep(0.55, 0.78, dust) .* o.DustStrength;
    neb = neb .* (1 - 0.85*dustMask);

    img = cat(3, 0.010, 0.012, 0.025) + neb;

    % ---- star field (full resolution, uniform on the sphere) -----------
    N = round(o.NumStars);
    if N > 0
        rng(s);
        lonS = 2*pi * rand(N,1);
        latS = asin(2*rand(N,1) - 1);
        bright = rand(N,1).^o.StarContrast;
        t  = (rand(N,1)-0.5) * o.ColorJitter;
        cR = 1 + 0.35*t;  cG = 1 - 0.05*abs(t);  cB = 1 - 0.35*t;
        col_ = lonS/(2*pi)*W;  row = (pi/2-latS)/pi*H + 0.5;
        ci = min(W, max(1, floor(col_)+1));  ri = min(H, max(1, round(row)));
        sub = [ri, ci];
        accR = accumarray(sub, bright.*cR, [H W]);
        accG = accumarray(sub, bright.*cG, [H W]);
        accB = accumarray(sub, bright.*cB, [H W]);
        m = bright > 0.5;
        if any(m)
            bR=accumarray(sub(m,:),bright(m).*cR(m),[H W]);
            bG=accumarray(sub(m,:),bright(m).*cG(m),[H W]);
            bB=accumarray(sub(m,:),bright(m).*cB(m),[H W]);
        else, bR=zeros(H,W); bG=bR; bB=bR; end
        kC = gauss1d(o.StarSize);  kB = gauss1d(max(o.StarSize*4,2.2));  gn = 3.0;
        img = img + cat(3, ...
            gn*(convWrap(accR,kC)+o.BloomStrength*convWrap(bR,kB)), ...
            gn*(convWrap(accG,kC)+o.BloomStrength*convWrap(bG,kB)), ...
            gn*(convWrap(accB,kC)+o.BloomStrength*convWrap(bB,kB)));
    end

    rgb = 1 - exp(-max(img,0) * 1.3);
    rgb = min(max(rgb,0),1);

    if nargout == 0
        figure('Color',[0 0 0]);
        subplot(1,2,1); imshow(rgb); title('Nebula (FFT) + stars','Color','w');
        subplot(1,2,2);
        [xs,ys,zs] = sphere(180);
        surf(xs,ys,zs,'FaceColor','texturemap','CData',flipud(rgb),'EdgeColor','none');
        axis equal off; view(35,15); set(gca,'Color','k');
        title('Wrapped onto a sphere','Color','w');
        clear rgb density
    end
end

% ======================================================================
function F = fftField(kr, beta, scale, seed)
%FFTFIELD  Periodic fractal field via 1/f^beta spectral synthesis -> [0,1].
%   The random spectrum is built directly from random phases, so only one
%   (inverse) transform is needed and RANDN is avoided entirely.
    rng(seed, 'twister');
    amp = zeros(size(kr));
    keep = kr >= scale;                      % drop DC + largest blobs
    amp(keep) = kr(keep) .^ (-beta/2);
    spec = amp .* exp(2i*pi * rand(size(kr)));
    F = real(ifft2(spec));
    F = (F - min(F(:))) / (max(F(:)) - min(F(:)) + eps);
end

function out = sampleWrap(Fld, sx, sy)
%SAMPLEWRAP  Vectorised periodic bilinear sampling of Fld at (sx,sy) [px].
    [H, W] = size(Fld);
    x0 = floor(sx); y0 = floor(sy);
    fx = sx - x0;   fy = sy - y0;
    x0w = mod(x0,   W) + 1;  x1w = mod(x0+1, W) + 1;
    y0w = mod(y0,   H) + 1;  y1w = mod(y0+1, H) + 1;
    F00 = Fld(sub2ind([H W], y0w, x0w));  F10 = Fld(sub2ind([H W], y0w, x1w));
    F01 = Fld(sub2ind([H W], y1w, x0w));  F11 = Fld(sub2ind([H W], y1w, x1w));
    out = (1-fy).*((1-fx).*F00 + fx.*F10) + fy.*((1-fx).*F01 + fx.*F11);
end

function [A,B,C] = palette(name)
    switch lower(name)
        case 'ember',   A=[0.95 0.25 0.05]; B=[0.85 0.55 0.10]; C=[1.00 0.90 0.70];
        case 'azure',   A=[0.15 0.35 0.95]; B=[0.55 0.20 0.90]; C=[0.80 0.90 1.00];
        case 'emerald', A=[0.10 0.80 0.45]; B=[0.10 0.45 0.80]; C=[0.85 1.00 0.90];
        otherwise,      A=[0.90 0.15 0.45]; B=[0.15 0.35 0.95]; C=[1.00 0.85 0.95];
    end
end

function out = convWrap(buf, k1d)
    p = (numel(k1d)-1)/2;
    bp = [buf(:,end-p+1:end), buf, buf(:,1:p)];
    tmp = conv2(1, k1d(:).', bp, 'same');  tmp = tmp(:, p+1:end-p);
    out = conv2(k1d(:), 1, tmp, 'same');
end

function k = gauss1d(sigma)
    r = max(1, ceil(3*sigma)); x = -r:r;
    k = exp(-(x.^2)/(2*sigma^2)); k = k/sum(k);
end

function s = smoothstep(e0, e1, x)
    t = min(max((x-e0)./(e1-e0), 0), 1); s = t.^2 .* (3 - 2.*t);
end