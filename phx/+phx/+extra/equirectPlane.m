function [rgb, ground] = equirectPlane(varargin)
%EQUIRECTPLANE  Infinite patterned ground plane fading into fog.
%   [RGB, GROUND] = EQUIRECTPLANE(Name,Value,...) renders, into an
%   equirectangular (sphere-mappable) image, the view from an observer
%   standing above an infinite horizontal plane. Rays below the horizon hit
%   the plane and are textured with a grid/checker pattern at a fixed world
%   scale, so the pattern foreshortens with distance and converges at the
%   horizon (clear depth + perspective). Distance drives exponential fog,
%   so the plane dissolves into mist toward the horizon, where it meets the
%   sky seamlessly.
%
%   The pattern is anti-aliased by the pixel's world-space footprint: once
%   cells become smaller than a pixel the grid fades to flat floor instead
%   of aliasing, and fog then takes over - so it stays clean at any range.
%
%   Name-Value options:
%     'Width','Height'                          (default 1024, Width/2)
%     'Pattern'     'grid' | 'checker' | 'both'  (default 'both')
%     'TileSize'    cell size in world units      (default 1.0)
%     'EyeHeight'   observer height above plane   (default 1.6)
%     'FogDistance' distance of ~63% fog          (default 6)
%     'LineWidth'   grid line width (world units) (default 0.01)
%     'MajorEvery'  major grid line every N cells (default 8)
%     'FloorColor'  1x3 base floor colour         (default [0.30 0.32 0.36])
%     'LineColor'   1x3 grid / light-square col   (default [0.58 0.61 0.66])
%     'FogColor'    1x3 mist colour               (default [0.78 0.81 0.85])
%     'SkyZenith'   1x3 zenith colour             (default [0.95 0.95 0.95])
%
%   Outputs: RGB (H x W x 3, [0,1]), GROUND (H x W mask, 1 below horizon).
%
%   With no output arguments a figure compares the flat map with a sphere.
%
%   Example:
%     rgb = equirectPlane('Pattern','checker','EyeHeight',2); imshow(rgb);
%     equirectPlane('TileSize',0.5,'FogDistance',40);

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    p = inputParser;
    addParameter(p,'Width',1024, @(x)isnumeric(x)&&isscalar(x)&&x>=8);
    addParameter(p,'Height',[],   @(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=4));
    addParameter(p,'Pattern','both', @(s)any(strcmpi(s,{'grid','checker','both'})));
    addParameter(p,'TileSize',1.0,  @(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'EyeHeight',1.6, @(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'FogDistance',6,@(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'LineWidth',0.01,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
    addParameter(p,'MajorEvery',8,  @(x)isnumeric(x)&&isscalar(x)&&x>=1);
    addParameter(p,'FloorColor',[0.30 0.32 0.36], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'LineColor', [0.58 0.61 0.66], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'FogColor',  [0.78 0.81 0.85], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'SkyZenith', [0.95 0.95 0.95], @(x)isnumeric(x)&&numel(x)==3);
    parse(p,varargin{:});
    o = p.Results;

    W = round(o.Width);
    if isempty(o.Height), H = round(W/2); else, H = round(o.Height); end
    H = 2 * round(H/2);
    c3 = @(c) reshape(c(:).',1,1,3);

    % ---- view directions (z up) ----------------------------------------
    lat = (pi/2) - ((0:H-1).' + 0.5) / H * pi;
    lon = (0:W-1) / W * 2*pi;
    [LON, LAT] = meshgrid(lon, lat);
    cl = cos(LAT);
    dx = cl.*cos(LON);  dy = cl.*sin(LON);  dz = sin(LAT);   % dz<0 => looking down

    isGround = dz < 0;

    % ---- ray/plane intersection (world coords on the plane) ------------
    dzc  = min(dz, -1e-4);                 % guard; sky pixels masked later
    t    = -o.EyeHeight ./ dzc;            % distance eye->hit (=true dist, |d|=1)
    hx   = t .* dx;  hy = t .* dy;         % world position on the plane
    % world-space footprint of one pixel at the hit (grows with dist/grazing)
    fw   = t .* (pi/H) ./ max(-dz, 1e-3);

    % ---- floor pattern -------------------------------------------------
    floorCol = repmat(c3(o.FloorColor), H, W);

    if any(strcmpi(o.Pattern, {'grid','both'}))
        gMinor = gridLines(hx, hy, o.TileSize,               o.LineWidth,     fw);
        gMajor = gridLines(hx, hy, o.TileSize*o.MajorEvery,  o.LineWidth*2.2, fw);
        lineAmt = min(0.85*gMinor + 1.0*gMajor, 1);
        floorCol = floorCol + lineAmt .* (c3(o.LineColor) - floorCol);
    end
    if any(strcmpi(o.Pattern, {'checker','both'}))
        par = mod(floor(hx./o.TileSize) + floor(hy./o.TileSize), 2);   % 0/1
        resFade = 1 - smoothstep(0.4, 0.9, fw./o.TileSize);            % fade when unresolved
        chk = c3(o.FloorColor) + par .* (c3(o.LineColor) - c3(o.FloorColor));
        if strcmpi(o.Pattern,'checker')
            floorCol = c3(o.FloorColor) + resFade .* (chk - c3(o.FloorColor));
        else  % 'both' - lay checker under the grid
            base2 = c3(o.FloorColor) + 0.6*resFade .* (chk - c3(o.FloorColor));
            floorCol = base2 + lineAmt .* (c3(o.LineColor) - base2);
        end
    end

    % ---- exponential fog: floor dissolves into mist with distance ------
    fogA = 1 - exp(-t ./ o.FogDistance);
    groundRGB = floorCol + fogA .* (c3(o.FogColor) - floorCol);

    % ---- sky above the horizon: fog -> slightly lighter zenith ---------
    skyT   = smoothstep(0, 1, max(dz,0));
    skyRGB = c3(o.FogColor) + skyT .* (c3(o.SkyZenith) - c3(o.FogColor));

    % ---- combine -------------------------------------------------------
    rgb = isGround .* groundRGB + (~isGround) .* skyRGB;
    ground = double(isGround);

    rgb = min(max(rgb, 0), 1);

    % Blur
    rgb = imgaussfilt(rgb, [1 2]/2);

    if nargout == 0
        figure('Color',[0 0 0]);
        subplot(1,2,1); imshow(rgb); title('Infinite plane in fog','Color','w');
        subplot(1,2,2);
        [xs,ys,zs] = sphere(180);
        surf(xs,ys,zs,'FaceColor','texturemap','CData',flipud(rgb),'EdgeColor','none');
        axis equal off; view(20,-10); set(gca,'Color','k');
        title('Wrapped onto a sphere','Color','w');
        clear rgb ground
    end
end

% ======================================================================
function g = gridLines(hx, hy, cell, lineW, fw)
%GRIDLINES  Footprint-anti-aliased grid intensity [0,1] for cell-size 'cell'.
    cx = hx ./ cell;   cy = hy ./ cell;
    lw  = lineW ./ cell;             % half line width in cells
    fwc = fw   ./ cell;              % footprint in cells
    dX = abs(mod(cx + 0.5, 1) - 0.5);   % distance to nearest line (cells)
    dY = abs(mod(cy + 0.5, 1) - 0.5);
    lineX = 1 - smoothstep(lw, lw + fwc + 1e-4, dX);
    lineY = 1 - smoothstep(lw, lw + fwc + 1e-4, dY);
    resFade = 1 - smoothstep(0.45, 0.95, fwc);   % stop drawing once unresolved
    g = max(lineX, lineY) .* resFade;
end

function s = smoothstep(e0, e1, x)
    t = min(max((x - e0) ./ (e1 - e0), 0), 1);
    s = t.^2 .* (3 - 2.*t);
end