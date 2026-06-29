function [rgb, density] = equirectNebula(varargin)
%EQUIRECTNEBULA  Procedural deep-space nebula with a dense star field.
%   [RGB, DENSITY] = EQUIRECTNEBULA(Name,Value,...) renders a full-sphere
%   (equirectangular) cosmic background: colourful nebula gas built from
%   domain-warped 3-D fractal noise, dark dust lanes, and a star field
%   scattered uniformly on the sphere. Unlike the sky/horizon maps there is
%   no water reflection - this is deep space, so the whole sphere is used.
%
%   Everything is sampled on the unit sphere, so the texture is seamless
%   across the 0/360 deg meridian and free of pole distortion; the star
%   glow is convolved with circular horizontal wrap to keep the seam clean.
%
%   Name-Value options:
%     'Width'         image width in pixels          (default 1024)
%     'Height'        image height in pixels          (default Width/2, even)
%     'Palette'       'mixed'|'ember'|'azure'|'emerald' (default 'mixed')
%     'NebulaStrength'overall gas brightness         (default 0.9)
%     'NebulaCoverage'fraction of sky with gas, 0..1 (default 0.55)
%     'Warp'          domain-warp amount (turbulence) (default 0.6)
%     'DustStrength'  dark dust-lane strength, 0..1   (default 0.5)
%     'NumStars'      stars over the whole sphere      (default 10000)
%     'StarContrast'  brightness power law (>1)        (default 3)
%     'StarSize'      core glow radius in px (sigma)   (default 0.7)
%     'BloomStrength' halo strength on bright stars    (default 0.6)
%     'ColorJitter'   star colour spread, 0..1         (default 0.6)
%     'Seed'          RNG / noise seed                  (default 0)
%
%   Outputs:
%     RGB      H-by-W-by-3 double image in [0,1]
%     DENSITY  H-by-W nebula gas density in [0,1] (use as mask/emissive)
%
%   With no output arguments a figure compares the flat map with the same
%   map wrapped onto a sphere.
%
%   Example:
%     equirectNebula('Palette','mixed','Seed',11);
%     rgb = equirectNebula('Palette','ember','NumStars',15000,'Width',2048);

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    % ---- parse options -------------------------------------------------
    p = inputParser;
    addParameter(p,'Width',1024,    @(x)isnumeric(x)&&isscalar(x)&&x>=4);
    addParameter(p,'Height',[],      @(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=2));
    addParameter(p,'Palette','mixed',@(s)any(strcmpi(s,{'mixed','ember','azure','emerald'})));
    addParameter(p,'NebulaStrength',0.9, @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'NebulaCoverage',0.55,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Warp',0.6,       @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'DustStrength',0.5,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'NumStars',10000,  @(x)isnumeric(x)&&isscalar(x)&&x>=0);
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
    rng(o.Seed);

    % ---- full-sphere grid ----------------------------------------------
    lat = (pi/2) - ((0:H-1).' + 0.5) / H * pi;       % +pi/2 .. -pi/2
    lon = (0:W-1) / W * 2*pi;
    [LON, LAT] = meshgrid(lon, lat);                 % H x W
    cosLat = cos(LAT);
    X = cosLat .* cos(LON);
    Y = cosLat .* sin(LON);
    Z = sin(LAT);

    % ---- domain-warped gas density -------------------------------------
    s = o.Seed;
    wx = 2*fbm3(X,Y,Z, s+11, 4, 1.5, 0.5, 2.0) - 1;  % warp vector field
    wy = 2*fbm3(X,Y,Z, s+23, 4, 1.5, 0.5, 2.0) - 1;
    wz = 2*fbm3(X,Y,Z, s+37, 4, 1.5, 0.5, 2.0) - 1;
    Xw = X + o.Warp.*wx;  Yw = Y + o.Warp.*wy;  Zw = Z + o.Warp.*wz;
    g  = fbm3(Xw, Yw, Zw, s+5, 6, 2.0, 0.5, 2.1);    % main gas field [0,1]

    % shape it into defined clouds vs empty space (NebulaCoverage = extent)
    v   = sort(g(:));
    thr = v(min(numel(v), max(1, round((1-o.NebulaCoverage)*numel(v)))));
    d   = smoothstep(thr-0.08, thr+0.22, g);         % H x W in [0,1]
    density = d;

    % ---- colour the gas via a hue-selector field + palette -------------
    hsel = fbm3(X,Y,Z, s+71, 3, 1.2, 0.55, 2.0);     % large soft colour patches
    hsel = smoothstep(0.36, 0.64, hsel);             % push toward distinct A / B zones
    [A, B, C] = palette(o.Palette);
    % blend A->B by hsel, then lift only the hottest cores toward C
    base = (1-hsel).*reshape(A,1,1,3) + hsel.*reshape(B,1,1,3);   % H x W x 3
    core = smoothstep(0.72, 0.98, d);                % only the densest cores
    col  = base + 0.55*core .* (reshape(C,1,1,3) - base);
    neb  = (d.^1.6) .* col * o.NebulaStrength;       % emission (additive)

    % ---- carve dark dust lanes -----------------------------------------
    dust = fbm3(X*1.3, Y*1.3, Z*1.3, s+97, 5, 2.5, 0.5, 2.0);
    dustMask = smoothstep(0.55, 0.78, dust) .* o.DustStrength;
    neb = neb .* (1 - 0.85*dustMask);

    % ---- compose: faint space background + nebula ----------------------
    img = cat(3, 0.010, 0.012, 0.025) + neb;

    % ---- star field (uniform on the whole sphere) ----------------------
    N = round(o.NumStars);
    if N > 0
        lonS = 2*pi * rand(N,1);
        latS = asin(2*rand(N,1) - 1);                % uniform on full sphere
        bright = rand(N,1).^o.StarContrast;
        t  = (rand(N,1) - 0.5) * o.ColorJitter;
        cR = 1 + 0.35*t;  cG = 1 - 0.05*abs(t);  cB = 1 - 0.35*t;

        col_ = lonS / (2*pi) * W;
        row  = (pi/2 - latS) / pi * H + 0.5;
        ci   = min(W, max(1, floor(col_) + 1));
        ri   = min(H, max(1, round(row)));
        sub  = [ri, ci];

        accR = accumarray(sub, bright.*cR, [H W]);
        accG = accumarray(sub, bright.*cG, [H W]);
        accB = accumarray(sub, bright.*cB, [H W]);
        m = bright > 0.5;
        if any(m)
            bR = accumarray(sub(m,:), bright(m).*cR(m), [H W]);
            bG = accumarray(sub(m,:), bright(m).*cG(m), [H W]);
            bB = accumarray(sub(m,:), bright(m).*cB(m), [H W]);
        else
            bR = zeros(H,W); bG = bR; bB = bR;
        end
        kC = gauss1d(o.StarSize);  kB = gauss1d(max(o.StarSize*4, 2.2));
        gn = 3.0;
        sR = gn*(convWrap(accR,kC) + o.BloomStrength*convWrap(bR,kB));
        sG = gn*(convWrap(accG,kC) + o.BloomStrength*convWrap(bG,kB));
        sB = gn*(convWrap(accB,kC) + o.BloomStrength*convWrap(bB,kB));
        img = img + cat(3, sR, sG, sB);
    end

    % ---- highlight roll-off (filmic-ish) -------------------------------
    rgb = 1 - exp(-max(img, 0) * 1.3);
    rgb = min(max(rgb, 0), 1);

end

% ======================================================================
%  Local helper functions
% ======================================================================
function [A,B,C] = palette(name)
    switch lower(name)
        case 'ember'
            A=[0.95 0.25 0.05]; B=[0.85 0.55 0.10]; C=[1.00 0.90 0.70];
        case 'azure'
            A=[0.15 0.35 0.95]; B=[0.55 0.20 0.90]; C=[0.80 0.90 1.00];
        case 'emerald'
            A=[0.10 0.80 0.45]; B=[0.10 0.45 0.80]; C=[0.85 1.00 0.90];
        otherwise % 'mixed'
            A=[0.90 0.15 0.45]; B=[0.15 0.35 0.95]; C=[1.00 0.85 0.95];
    end
end

function f = fbm3(X, Y, Z, seed, octaves, baseFreq, gain, lac)
    f = zeros(size(X)); amp = 1; freq = baseFreq; as = 0;
    for k = 1:octaves
        f = f + amp .* valnoise3(X*freq, Y*freq, Z*freq, seed + 1000*k);
        as = as + amp; amp = amp*gain; freq = freq*lac;
    end
    f = f / as;
end

function out = convWrap(buf, k1d)
    p = (numel(k1d) - 1) / 2;
    bp  = [buf(:, end-p+1:end), buf, buf(:, 1:p)];
    tmp = conv2(1, k1d(:).', bp, 'same');
    tmp = tmp(:, p+1:end-p);
    out = conv2(k1d(:), 1, tmp, 'same');
end

function k = gauss1d(sigma)
    r = max(1, ceil(3*sigma));  x = -r:r;
    k = exp(-(x.^2) / (2*sigma^2));  k = k / sum(k);
end

function out = valnoise3(x, y, z, seed)
    xi = floor(x); yi = floor(y); zi = floor(z);
    xf = x - xi;   yf = y - yi;   zf = z - zi;
    u = fade(xf);  v = fade(yf);  w = fade(zf);
    c000=h3(xi,yi,zi,seed);     c100=h3(xi+1,yi,zi,seed);
    c010=h3(xi,yi+1,zi,seed);   c110=h3(xi+1,yi+1,zi,seed);
    c001=h3(xi,yi,zi+1,seed);   c101=h3(xi+1,yi,zi+1,seed);
    c011=h3(xi,yi+1,zi+1,seed); c111=h3(xi+1,yi+1,zi+1,seed);
    x00=lerp(c000,c100,u); x10=lerp(c010,c110,u);
    x01=lerp(c001,c101,u); x11=lerp(c011,c111,u);
    y0 =lerp(x00,x10,v);   y1 =lerp(x01,x11,v);
    out=lerp(y0,y1,w);
end

function h = h3(i, j, k, seed)
    nn = i.*127.1 + j.*311.7 + k.*74.7 + seed.*57.0;
    h  = sin(nn) .* 43758.5453;
    h  = h - floor(h);
end

function f = fade(t),     f = t.^3 .* (t .* (t.*6 - 15) + 10); end
function l = lerp(a,b,t), l = a + t .* (b - a);                end
function s = smoothstep(e0, e1, x)
    t = min(max((x - e0) ./ (e1 - e0), 0), 1);
    s = t.^2 .* (3 - 2.*t);
end