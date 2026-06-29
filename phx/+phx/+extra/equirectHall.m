function [rgb, surface] = equirectHall(varargin)
%EQUIRECTHALL  Soft white cylindrical room with corner AO and wall text.
%   [RGB, SURFACE] = EQUIRECTHALL(Name,Value,...) renders the inside
%   of a white cylindrical room (wall + floor + ceiling) into an
%   equirectangular (sphere-mappable) image. The wall height is adjustable,
%   soft ambient-occlusion shading darkens the floor/ceiling corners, and an
%   optional text string is drawn on the wall using a built-in 5x7 font.
%
%   Built from the 3-D view direction, so it is seamless across 0/360 deg
%   and has no pole distortion (straight up/down = ceiling/floor centre).
%
%   Name-Value options:
%     'Width','Height'                          (default 1024, Width/2)
%     'Radius'      cylinder radius (world)       (default 4)
%     'WallHeight'  wall height (world)           (default 3)
%     'EyeHeight'   observer height (0..WallHeight)(default 1.5)
%     'WallColor'   wall grey 0..1                (default 0.92)
%     'FloorColor'  floor grey 0..1              (default 0.86)
%     'CeilingColor'ceiling grey 0..1            (default 0.95)
%     'AOStrength'  corner shadow strength 0..1   (default 0.40)
%     'AOSize'      corner shadow size (fraction) (default 0.20)
%     'Text'        string, or cell of strings    (default '')
%                   placed evenly around the wall
%     'TextAngle'   azimuth of first label, deg    (default 180, image centre)
%     'TextCenterV' vertical centre on wall 0..1  (default 0.55)
%     'TextSize'    text height as wall fraction  (default 0.25)
%     'TextColor'   text grey 0..1                (default 0.30)
%     'PixelGap'    gap between square font pixels (default 0.20; 0 = solid)
%
%   Outputs: RGB (H x W x 3, [0,1]), SURFACE (H x W: 1 wall, 2 floor, 3 ceiling)
%
%   With no output arguments a figure compares the flat map with a sphere.
%
%   Example:
%     rgb = cylindricalRoomMap('Text','GALLERY 01','WallHeight',4); imshow(rgb);

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    p = inputParser;
    addParameter(p,'Width',1024, @(x)isnumeric(x)&&isscalar(x)&&x>=8);
    addParameter(p,'Height',[],   @(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=4));
    addParameter(p,'Radius',4,    @(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'WallHeight',3,@(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'EyeHeight',1.5,@(x)isnumeric(x)&&isscalar(x)&&x>0);
    addParameter(p,'WallColor',0.86,   @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'FloorColor',0.80,  @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'CeilingColor',0.95,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'AOStrength',0.35,  @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'AOSize',0.20,      @(x)isnumeric(x)&&isscalar(x)&&x>0&&x<=1);
    addParameter(p,'Text','',          @(s)ischar(s)||isstring(s)||iscell(s));
    addParameter(p,'TextAngle',180,   @(x)isnumeric(x)&&isscalar(x));
    addParameter(p,'TextCenterV',0.55, @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'TextSize',0.25,    @(x)isnumeric(x)&&isscalar(x)&&x>0&&x<1);
    addParameter(p,'TextColor',0.30,   @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
    addParameter(p,'PixelGap',0.20,    @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<0.95);
    parse(p,varargin{:});
    o = p.Results;

    R = o.Radius;  Hw = o.WallHeight;  ze = min(max(o.EyeHeight,1e-3), Hw-1e-3);

    W = round(o.Width);
    if isempty(o.Height), H = round(W/2); else, H = round(o.Height); end
    H = 2 * round(H/2);

    lat = (pi/2) - ((0:H-1).' + 0.5) / H * pi;
    lon = (0:W-1) / W * 2*pi;
    [LON, LAT] = meshgrid(lon, lat);
    cl = cos(LAT);
    dx = cl.*cos(LON);  dy = cl.*sin(LON);  dz = sin(LAT);

    horiz  = sqrt(dx.^2 + dy.^2);
    horizs = max(horiz, 1e-9);

    % ---- which surface does each ray hit? ------------------------------
    tWall = R ./ horizs;
    zWall = ze + dz.*tWall;
    isWall  = (zWall >= 0) & (zWall <= Hw);
    isCeil  = ~isWall & (dz > 0);
    isFloor = ~isWall & (dz <= 0);

    % ---- per-surface coordinates ---------------------------------------
    th = atan2(dy, dx);                       % azimuth on wall
    vWall = zWall ./ Hw;                       % 0 floor .. 1 ceiling
    rFloor = (-ze ./ min(dz,-1e-9)) .* horiz;  % radius where floor is hit
    rCeil  = ((Hw-ze) ./ max(dz, 1e-9)) .* horiz;

    % ---- base colour + ambient occlusion -------------------------------
    col = o.WallColor .* ones(H, W);

    aoW = max(1 - smoothstep(0, o.AOSize, vWall), 1 - smoothstep(0, o.AOSize, 1 - vWall));
    wallCol = o.WallColor .* (1 - o.AOStrength.*aoW);

    aoF = 1 - smoothstep(0, o.AOSize*R, R - rFloor);   % 1 next to the wall
    floorCol = o.FloorColor .* (1 - o.AOStrength.*aoF);

    aoC = 1 - smoothstep(0, o.AOSize*R, R - rCeil);
    ceilCol = o.CeilingColor .* (1 - o.AOStrength.*aoC);

    col = isWall.*wallCol + isFloor.*floorCol + isCeil.*ceilCol;

    % ---- text on the wall (one or several labels around the circle) ----
    if ischar(o.Text) || (isstring(o.Text) && isscalar(o.Text))
        labels = {char(o.Text)};
        if isempty(labels{1}), labels = {}; end
    else
        labels = cellstr(o.Text);              % cell / string array -> cellstr
    end
    N = numel(labels);
    if N > 0
        period = 2*pi / N;                     % angular spacing between labels
        thV  = o.TextSize;                     % height as wall fraction
        topV = o.TextCenterV + thV/2;
        inRow = (topV - vWall) ./ thV;         % 0..1 top->bottom (shared)
        rowOK = isWall & inRow>=0 & inRow<=1;
        for i = 1:N
            if isempty(labels{i}), continue; end
            bmp = textBitmap(labels{i});
            [nr, nc] = size(bmp);
            azW = (thV*(nc/nr)*Hw) / R;         % azimuth extent of THIS label
            cen = o.TextAngle*pi/180 + (i-1)*period;
            dth = atan2(sin(th - cen), cos(th - cen));
            inCol = (dth + azW/2) ./ azW;       % 0..1 across width
            inText = rowOK & inCol>=0 & inCol<=1;
            if ~any(inText(:)), continue; end
            fCol = inCol*nc;  fRow = inRow*nr;
            ci = min(nc, max(1, floor(fCol) + 1));
            ri = min(nr, max(1, floor(fRow) + 1));
            on = false(H, W);
            on(inText) = bmp(sub2ind([nr nc], ri(inText), ci(inText)));
            if o.PixelGap <= 0
                alpha = double(on);
            else
                % keep only a centred square in each font cell (gap around it),
                % hard-thresholded so the pixels stay sharp like solid text
                fracC = fCol - floor(fCol);
                fracR = fRow - floor(fRow);
                dist  = max(abs(fracC - 0.5), abs(fracR - 0.5));   % square (Chebyshev)
                half  = (1 - o.PixelGap) / 2;
                alpha = double(on & (dist < half));
            end
            col = col + alpha .* (o.TextColor - col);
        end
    end

    % ---- output -----------------------------------------------
    col = min(max(col, 0), 1);
    rgb = repmat(col, 1, 1, 3);
    surface = isWall*1 + isFloor*2 + isCeil*3;

    if nargout == 0
        figure('Color',[0 0 0]);
        subplot(1,2,1); imshow(rgb); title('Cylindrical room','Color','w');
        subplot(1,2,2);
        [xs,ys,zs] = sphere(180);
        surf(xs,ys,zs,'FaceColor','texturemap','CData',flipud(rgb),'EdgeColor','none');
        axis equal off; view(35,12); set(gca,'Color','k');
        title('Wrapped onto a sphere','Color','w');
        clear rgb surface
    end
end

% ======================================================================
function bmp = textBitmap(str)
%TEXTBITMAP  Assemble a string into a logical bitmap using a 5x7 font.
    F = fontTable();
    str = upper(char(str));
    cols = {};
    for i = 1:numel(str)
        ch = str(i);
        if isKey(F, ch), g = F(ch); else, g = F(' '); end
        cols{end+1} = g;                 %#ok<AGROW>
        cols{end+1} = false(7,1);        % 1-col spacing
    end
    if isempty(cols), bmp = false(7,1); else, bmp = [cols{:}]; end
end

function F = fontTable()
%FONTTABLE  Build (once) a Map of char -> 7x5 logical glyph.
    persistent M
    if ~isempty(M), F = M; return; end
    D = {
    'A',{'.###.','#...#','#...#','#####','#...#','#...#','#...#'}
    'B',{'####.','#...#','#...#','####.','#...#','#...#','####.'}
    'C',{'.####','#....','#....','#....','#....','#....','.####'}
    'D',{'####.','#...#','#...#','#...#','#...#','#...#','####.'}
    'E',{'#####','#....','#....','####.','#....','#....','#####'}
    'F',{'#####','#....','#....','####.','#....','#....','#....'}
    'G',{'.####','#....','#....','#.###','#...#','#...#','.####'}
    'H',{'#...#','#...#','#...#','#####','#...#','#...#','#...#'}
    'I',{'#####','..#..','..#..','..#..','..#..','..#..','#####'}
    'J',{'..###','...#.','...#.','...#.','#..#.','#..#.','.##..'}
    'K',{'#...#','#..#.','#.#..','##...','#.#..','#..#.','#...#'}
    'L',{'#....','#....','#....','#....','#....','#....','#####'}
    'M',{'#...#','##.##','#.#.#','#.#.#','#...#','#...#','#...#'}
    'N',{'#...#','##..#','#.#.#','#..##','#...#','#...#','#...#'}
    'O',{'.###.','#...#','#...#','#...#','#...#','#...#','.###.'}
    'P',{'####.','#...#','#...#','####.','#....','#....','#....'}
    'Q',{'.###.','#...#','#...#','#...#','#.#.#','#..#.','.##.#'}
    'R',{'####.','#...#','#...#','####.','#.#..','#..#.','#...#'}
    'S',{'.####','#....','#....','.###.','....#','....#','####.'}
    'T',{'#####','..#..','..#..','..#..','..#..','..#..','..#..'}
    'U',{'#...#','#...#','#...#','#...#','#...#','#...#','.###.'}
    'V',{'#...#','#...#','#...#','#...#','#...#','.#.#.','..#..'}
    'W',{'#...#','#...#','#...#','#.#.#','#.#.#','##.##','#...#'}
    'X',{'#...#','#...#','.#.#.','..#..','.#.#.','#...#','#...#'}
    'Y',{'#...#','#...#','.#.#.','..#..','..#..','..#..','..#..'}
    'Z',{'#####','....#','...#.','..#..','.#...','#....','#####'}
    '0',{'.###.','#...#','#..##','#.#.#','##..#','#...#','.###.'}
    '1',{'..#..','.##..','..#..','..#..','..#..','..#..','.###.'}
    '2',{'.###.','#...#','....#','..##.','.#...','#....','#####'}
    '3',{'####.','....#','....#','.###.','....#','....#','####.'}
    '4',{'#...#','#...#','#...#','#####','....#','....#','....#'}
    '5',{'#####','#....','#....','####.','....#','....#','####.'}
    '6',{'.###.','#....','#....','####.','#...#','#...#','.###.'}
    '7',{'#####','....#','...#.','..#..','.#...','.#...','.#...'}
    '8',{'.###.','#...#','#...#','.###.','#...#','#...#','.###.'}
    '9',{'.###.','#...#','#...#','.####','....#','....#','.###.'}
    ' ',{'.....','.....','.....','.....','.....','.....','.....'}
    '.',{'.....','.....','.....','.....','.....','.##..','.##..'}
    ',',{'.....','.....','.....','.....','.##..','.##..','.#...'}
    '!',{'..#..','..#..','..#..','..#..','..#..','.....','..#..'}
    '?',{'.###.','#...#','....#','..##.','..#..','.....','..#..'}
    '-',{'.....','.....','.....','#####','.....','.....','.....'}
    ':',{'.....','.##..','.##..','.....','.##..','.##..','.....'}
    '/',{'....#','....#','...#.','..#..','.#...','#....','#....'}
    '#',{'.#.#.','.#.#.','#####','.#.#.','#####','.#.#.','.#.#.'}
    '^',{'..#..','.#.#.','#...#','.....','.....','.....','.....'}
    };
    M = containers.Map('KeyType','char','ValueType','any');
    for k = 1:size(D,1)
        rows = D{k,2};
        g = false(7,5);
        for r = 1:7, g(r,:) = (rows{r} == '#'); end
        M(D{k,1}) = g;
    end
    F = M;
end

function s = smoothstep(e0, e1, x)
    t = min(max((x - e0) ./ (e1 - e0), 0), 1);
    s = t.^2 .* (3 - 2.*t);
end