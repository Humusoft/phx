function [rgb, density] = equirectSky(varargin)
%EQUIRECTSKY  Random cloud cover as a seamless equirectangular image.
%   [RGB, DENSITY] = EQUIRECTSKY(Name,Value,...) synthesises a random
%   cloud layer laid out in an equirectangular (lat/long, "rectilinear")
%   grid that is ready to be texture-mapped onto a sphere.
%
%   The clouds come from 3-D fractal value noise (fractional Brownian
%   motion) sampled DIRECTLY on the unit sphere. Because the noise is a
%   continuous function of the 3-D surface point, the result is:
%       * perfectly seamless across the 0/360 deg meridian, and
%       * free of the "swirl" artefact at the poles
%   that you get when 2-D noise is wrapped onto a globe.
%
%   Name-Value options (all optional):
%     'Width'       image width  in pixels         (default 1024)
%     'Height'      image height in pixels          (default Width/2)
%     'Octaves'     number of fBm octaves           (default 6)
%     'BaseScale'   feature count around the globe  (default 10)
%     'Lacunarity'  frequency growth per octave     (default 2.0)
%     'Gain'        amplitude decay per octave      (default 0.5)
%     'Coverage'    fraction of sky covered, 0..1   (default 0.25)
%     'Softness'    cloud-edge softness, 0..1       (default 1.0)
%     'Seed'        RNG seed for repeatability       (default 0)
%
%   Outputs:
%     RGB      H-by-W-by-3 double image in [0,1] (white clouds / blue sky)
%     DENSITY  H-by-W cloud-density map in [0,1] (use as alpha / heightmap)
%
%   With no output arguments a figure is shown comparing the flat map with
%   the same map wrapped onto a sphere.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    % ---- parse options -------------------------------------------------
    p = inputParser;
    addParameter(p,'Width',2048,    @(x)isnumeric(x)&&isscalar(x)&&x>=4);
    addParameter(p,'Octaves',4,      @(x)isnumeric(x)&&isscalar(x)&&x>=1);
    addParameter(p,'BaseScale',10,    @(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'Lacunarity',2.0, @(x)isnumeric(x)&&isscalar(x)&&x>1);
    addParameter(p,'Gain',0.5,       @(x)isnumeric(x)&&isscalar(x)&&x>0&&x<1);
    addParameter(p,'Coverage',0.25,   @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Softness',1.0,  @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Seed',0,         @(x)isnumeric(x)&&isscalar(x));
    parse(p,varargin{:});
    o = p.Results;

    W = round(o.Width);
    H = round(W/4);

    % ---- equirectangular grid -> points on the unit sphere -------------
    % longitude 0..2*pi across columns (periodic: last col is NOT a copy of
    % the first), latitude +pi/2..-pi/2 down the rows (poles on edge rows).
    lon = (0:W-1) / W * 2*pi;                 % 1 x W
    lat = linspace(pi/2, 0, H).';         % H x 1
    [LON, LAT] = meshgrid(lon, lat);          % H x W each
    cosLat = cos(LAT);
    X = cosLat .* cos(LON);
    Y = cosLat .* sin(LON);
    Z = sin(LAT);
    Z = abs(Z).^0.5 .* sign(Z);

    % ---- fractal Brownian motion of 3-D value noise --------------------
    total = zeros(H,W);
    amp   = 1;
    freq  = o.BaseScale;
    ampSum = 0;
    for k = 1:o.Octaves
        total  = total + amp .* valnoise3(X*freq, Y*freq, Z*freq, o.Seed + 1000*k);
        ampSum = ampSum + amp;
        amp    = amp  * o.Gain;
        freq   = freq * o.Lacunarity;
    end
    n = total / ampSum;                       % roughly in [0,1]
    n = (n - min(n(:))) / max(max(n(:)) - min(n(:)), eps);  % stretch to [0,1]

    % ---- coverage threshold + soft edges -------------------------------
    v   = sort(n(:));
    idx = min(numel(v), max(1, round((1 - o.Coverage) * numel(v))));
    thr = v(idx);
    half = max(o.Softness, 1e-6) / 2;
    density = smoothstep(thr - half, thr + half, n);

    % ---- compose an RGB image (white clouds over blue sky) -------------
    sky   = cat(3, 0.45, 0.55, 0.75);         % 1 x 1 x 3
    shade = 0.82 + 0.18 .* density;           % thicker cloud -> brighter
    cloud = cat(3, shade, shade, shade);
    rgb1   = sky .* (1 - density) + cloud .* density;
    rgb1   = min(max(rgb1, 0), 1)*0.9;

    % Blurred mirror reflection
    rgb2 = flipud(imgaussfilt(rgb1*0.8, 8, "Padding", "circular"));
    rgb = vertcat(rgb1, rgb2);

    % Horizon gradient
    hw = size(rgb, 1)/2;
    n = hw/4;
    rm = linspace(1, 0, n)';
    skyl = (0.25 + sky)/1.25;
    rgb(hw-n+1:hw, :, :) = rgb(hw-n+1:hw, :, :).*rm + skyl.*(1 - rm);
    skyd = sky/1.25;
    rgb(hw+1:hw+n, :, :) = rgb(hw+1:hw+n, :, :).*(1 - rm) + skyd.*rm;

end

% ======================================================================
%  Local helper functions
% ======================================================================
function out = valnoise3(x, y, z, seed)
%VALNOISE3  3-D value noise with quintic interpolation, vectorised.
    xi = floor(x); yi = floor(y); zi = floor(z);
    xf = x - xi;   yf = y - yi;   zf = z - zi;
    u = fade(xf);  v = fade(yf);  w = fade(zf);

    c000 = h3(xi  , yi  , zi  , seed);
    c100 = h3(xi+1, yi  , zi  , seed);
    c010 = h3(xi  , yi+1, zi  , seed);
    c110 = h3(xi+1, yi+1, zi  , seed);
    c001 = h3(xi  , yi  , zi+1, seed);
    c101 = h3(xi+1, yi  , zi+1, seed);
    c011 = h3(xi  , yi+1, zi+1, seed);
    c111 = h3(xi+1, yi+1, zi+1, seed);

    x00 = lerp(c000, c100, u);  x10 = lerp(c010, c110, u);
    x01 = lerp(c001, c101, u);  x11 = lerp(c011, c111, u);
    y0  = lerp(x00, x10, v);    y1  = lerp(x01, x11, v);
    out = lerp(y0, y1, w);
end

function h = h3(i, j, k, seed)
%H3  Deterministic pseudo-random hash of integer lattice coords -> [0,1).
    nn = i.*127.1 + j.*311.7 + k.*74.7 + seed.*57.0;
    h  = sin(nn) .* 43758.5453;
    h  = h - floor(h);
end

function f = fade(t)
    f = (t.*t.*t) .* (t .* (t.*6 - 15) + 10);      % quintic smootherstep
end

function l = lerp(a, b, t)
    l = a + t .* (b - a);
end

function s = smoothstep(e0, e1, x)
    t = min(max((x - e0) ./ (e1 - e0), 0), 1);
    s = t.^2 .* (3 - 2.*t);
end