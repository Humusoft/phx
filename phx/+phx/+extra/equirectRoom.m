function [rgb, shade] = equirectRoom(varargin)
%EQUIRECTROOM  Equirectangular texture of the inside of a tiled cubic room.
%   [RGB, SHADE] = EQUIRECTROOM(Name,Value,...) renders the view from the
%   centre of a cube (4 walls + floor + ceiling) into an equirectangular
%   image suitable for mapping onto a sphere. Each face is split into a
%   configurable grid of tiles, each tile a random grey-to-white shade,
%   with grout lines between tiles and soft darkening toward the corners.
%
%   Because the texture is a true direction->cube-face mapping it is
%   seamless across the 0/360 deg meridian AND free of pole distortion -
%   straight up/down land on the centre of the ceiling/floor face, which is
%   an ordinary face, not a singular point.
%
%   Name-Value options:
%     'Width'        image width in pixels            (default 1024)
%     'Height'       image height in pixels            (default Width/2, even)
%     'TilesPerFace' tiles along a face: scalar N (NxN) (default 5)
%                    or [Nu Nv]
%     'ShadeMin'     darkest tile shade, 0..1          (default 0.75)
%     'ShadeMax'     lightest tile shade, 0..1         (default 1.00)
%     'Grout'        grout width as fraction of a tile (default 0)
%     'GroutColor'   grout grey level, 0..1            (default 0.16)
%     'CornerShade'  corner darkening strength, 0..1   (default 0.5)
%     'Seed'         RNG / hash seed                    (default 0)
%
%   Outputs:
%     RGB    H-by-W-by-3 double image in [0,1] (neutral greys)
%     SHADE  H-by-W luminance map in [0,1]
%
%   With no output arguments a figure compares the flat map with the same
%   map wrapped onto a sphere.
%
%   Example:
%     rgb = cubicRoomMap('TilesPerFace',6,'Seed',3); imshow(rgb);
%     equirectRoom('TilesPerFace',[8 4]);   % wider tiles

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    p = inputParser;
    addParameter(p,'Width',1024,      @(x)isnumeric(x)&&isscalar(x)&&x>=8);
    addParameter(p,'Height',[],       @(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=4));
    addParameter(p,'TilesPerFace',5,  @(x)isnumeric(x)&&(isscalar(x)||numel(x)==2)&&all(x>=1));
    addParameter(p,'ShadeMin',0.75,   @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'ShadeMax',1.00,   @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Grout',0,         @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<0.5);
    addParameter(p,'GroutColor',0.16, @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'CornerShade',0.5, @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'Seed',0,          @(x)isnumeric(x)&&isscalar(x));
    parse(p,varargin{:});
    o = p.Results;

    if isscalar(o.TilesPerFace), Nu = round(o.TilesPerFace); Nv = Nu;
    else, Nu = round(o.TilesPerFace(1)); Nv = round(o.TilesPerFace(2)); end

    W = round(o.Width);
    if isempty(o.Height), H = round(W/2); else, H = round(o.Height); end
    H = 2 * round(H/2);

    % ---- pixel direction (full sphere) ---------------------------------
    lat = (pi/2) - ((0:H-1).' + 0.5) / H * pi;
    lon = (0:W-1) / W * 2*pi;
    [LON, LAT] = meshgrid(lon, lat);
    cl = cos(LAT);
    X = cl.*cos(LON);  Y = cl.*sin(LON);  Z = sin(LAT);

    % ---- which cube face does each ray hit? ----------------------------
    ax = abs(X); ay = abs(Y); az = abs(Z);
    mX = (ax >= ay) & (ax >= az);
    mY = (ay >  ax) & (ay >= az);
    mZ = ~mX & ~mY;
    axs = max(ax,1e-9); ays = max(ay,1e-9); azs = max(az,1e-9);

    % face-local coords U,V in [-1,1]; face id 1..6 (+X,-X,+Y,-Y,+Z,-Z)
    U = mX.*(Y./axs) + mY.*(X./ays) + mZ.*(X./azs);
    V = mX.*(Z./axs) + mY.*(Z./ays) + mZ.*(Y./azs);
    face = mX.*(1+(X<0)) + mY.*(3+(Y<0)) + mZ.*(5+(Z<0));

    % ---- tile index + within-tile position -----------------------------
    fu = (U+1)/2;  fv = (V+1)/2;                 % [0,1] across the face
    tu = min(Nu-1, floor(fu*Nu));                % integer tile column
    tv = min(Nv-1, floor(fv*Nv));                % integer tile row
    gu = fu*Nu - tu;  gv = fv*Nv - tv;           % [0,1] within tile

    % ---- random grey..white shade per (face,tile) ----------------------
    rnd  = hash01(tu, tv, face*7919 + o.Seed);   % [0,1) per tile
    tile = o.ShadeMin + (o.ShadeMax - o.ShadeMin) .* rnd;

    % ---- grout lines (anti-aliased) ------------------------------------
    aa  = min(max(8*max(Nu,Nv)/W, 0.004), 0.05);
    dEdge = min(min(gu, 1-gu), min(gv, 1-gv));    % 0 at tile border
    gmix  = smoothstep(o.Grout, o.Grout + aa, dEdge); % 0 in grout, 1 in tile
    col   = o.GroutColor + gmix .* (tile - o.GroutColor);

    % ---- soft corner darkening (fake ambient occlusion) ----------------
    edgeDist = 1 - max(abs(U), abs(V));           % 0 at face edge .. 1 centre
    ao = 1 - o.CornerShade .* (1 - smoothstep(0, 0.35, edgeDist));
    col = col .* ao;

    col = min(max(col, 0), 1);
    shade = col;
    rgb = repmat(col, 1, 1, 3);

    % ---- optional demo display -----------------------------------------
    if nargout == 0
        figure('Color',[0 0 0]);
        subplot(1,2,1); imshow(rgb); title('Cubic room (equirectangular)','Color','w');
        subplot(1,2,2);
        [xs,ys,zs] = sphere(180);
        surf(xs,ys,zs,'FaceColor','texturemap','CData',flipud(rgb),'EdgeColor','none');
        axis equal off; view(35,15); set(gca,'Color','k');
        title('Wrapped onto a sphere','Color','w');
        clear rgb shade
    end
end

% ======================================================================
function h = hash01(i, j, seed)
%HASH01  Deterministic pseudo-random hash of integer tile coords -> [0,1).
    nn = i.*127.1 + j.*311.7 + seed.*0.731 + 13.37;
    h  = sin(nn) .* 43758.5453;
    h  = h - floor(h);
end

function s = smoothstep(e0, e1, x)
    t = min(max((x - e0) ./ (e1 - e0), 0), 1);
    s = t.^2 .* (3 - 2.*t);
end