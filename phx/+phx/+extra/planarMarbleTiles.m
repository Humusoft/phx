function img = planarMarbleTiles(varargin)
%PLANARMARBLETILES  Generate a marble tile texture.
%
%   IMG = PLANARMARBLETILES() returns a grayscale image (a double matrix in the
%   range 0..1) with an 8x8 grid of marble tiles separated by light grout.
%
%   IMG = PLANARMARBLETILES(Name, Value, ...) lets you set the parameters:
%
%     'ImageSize'     Size of the resulting image in pixels. Scalar (square)
%                     or [height width]. The tile size is computed from this,
%                     the number of tiles and the grout width; the remaining
%                     pixels are distributed as evenly as possible, so some
%                     tiles may be 1 px larger. The output always has exactly
%                     the requested size. Default: 512
%     'NumTiles'      Number of tiles. Scalar (square) or [rows cols].
%                     Default: 8
%     'LightnessVar'  Spread of tile lightness (0..1). Each tile gets a
%                     random shift of its mean lightness in the interval
%                     [-LightnessVar, +LightnessVar]. Default: 0.16
%     'GroutWidth'    Grout line width in pixels. Default: 2
%     'GroutColor'    Grout lightness (0..1). Default: 0.85
%     'BaseLightness' Mean (base) lightness of the tiles (0..1). Default: 0.68
%     'TileContrast'  Contrast of the marbling within a tile. Default: 0.05
%     'VeinFreq'      Density of the marble veins. Default: 5
%     'TurbPower'     Amount of vein distortion by turbulence. Default: 8.5
%     'VeinWeight'    Weight of veins vs. cloudy turbulence (0..1). Higher =
%                     more pronounced veins, lower = cloudier marble.
%                     Default: 0.15
%     'Octaves'       Number of fractal-noise octaves. Default: 5
%     'Grain'         Amplitude of fine grain (0 disables it). Default: 0.02
%     'Seed'          Random number generator seed for reproducibility.
%                     Default: 0 ([] to not set)
%
%   Examples:
%       img = planarMarbleTiles();                       % default 512x512
%       imshow(img)
%
%       img = planarMarbleTiles('ImageSize',1024, ...    % larger image,
%                         'LightnessVar',0.30);     % stronger variation
%       imshow(img)
%
%       img = planarMarbleTiles('ImageSize',[600 900], 'NumTiles',[6 9], 'Seed',42);
%       imwrite(img, 'tiles.png');

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    % --- parse inputs -----------------------------------------------------
    p = inputParser;
    addParameter(p, 'ImageSize',     512,  @(x) isnumeric(x) && all(x(:) > 0));
    addParameter(p, 'NumTiles',      8,    @(x) isnumeric(x) && all(x(:) > 0));
    addParameter(p, 'LightnessVar',  0.16, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'GroutWidth',    2,    @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'GroutColor',    0.85, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
    addParameter(p, 'BaseLightness', 0.68, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
    addParameter(p, 'TileContrast',  0.05, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'VeinFreq',      5,    @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'TurbPower',     8.5,  @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'VeinWeight',    0.15, @(x) isnumeric(x) && isscalar(x) && x >= 0 && x <= 1);
    addParameter(p, 'Octaves',       5,    @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'Grain',         0.02, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'Seed',          0,    @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
    parse(p, varargin{:});
    opt = p.Results;

    if ~isempty(opt.Seed)
        rng(opt.Seed);
    end

    % requested image size in pixels
    if isscalar(opt.ImageSize)
        H = round(opt.ImageSize);
        W = round(opt.ImageSize);
    else
        H = round(opt.ImageSize(1));
        W = round(opt.ImageSize(2));
    end

    % number of tile rows/columns
    if isscalar(opt.NumTiles)
        nRows = round(opt.NumTiles);
        nCols = round(opt.NumTiles);
    else
        nRows = round(opt.NumTiles(1));
        nCols = round(opt.NumTiles(2));
    end

    gw = round(opt.GroutWidth);

    % pixels left for the tiles after subtracting the grout lines
    tilePixV = H - (nRows+1)*gw;
    tilePixH = W - (nCols+1)*gw;
    if tilePixV < nRows || tilePixH < nCols
        error('marbleTiles:ImageTooSmall', ...
            ['ImageSize is too small for the given NumTiles and ' ...
             'GroutWidth (tiles would be smaller than 1 px).']);
    end

    % per-tile heights/widths (distribute remaining pixels as evenly as possible)
    rowH = splitPixels(tilePixV, nRows);
    colW = splitPixels(tilePixH, nCols);

    % --- prepare the canvas (filled with the grout color) -----------------
    img = opt.GroutColor * ones(H, W);

    % --- generate the individual tiles ------------------------------------
    y0 = gw + 1;
    for r = 1:nRows
        th = rowH(r);
        x0 = gw + 1;
        for c = 1:nCols
            tw = colW(c);

            % marble pattern in the range 0..1
            m = makeTile(th, tw, opt.Octaves, opt.VeinFreq, opt.TurbPower, opt.VeinWeight);

            % standardize (zero mean, unit standard deviation) so that the
            % contrast does not depend on the particular noise realization
            m = m - mean(m(:));
            m = m / (std(m(:)) + eps);

            % target mean lightness of the tile + random variation
            offset     = (rand - 0.5) * 2 * opt.LightnessVar;
            targetMean = opt.BaseLightness + offset;

            tile = targetMean + m * opt.TileContrast;

            % fine grain for a more realistic surface
            if opt.Grain > 0
                tile = tile + (rand(th, tw) - 0.5) * opt.Grain;
            end

            tile = min(max(tile, 0), 1);

            % place into the canvas
            img(y0:y0+th-1, x0:x0+tw-1) = tile;

            x0 = x0 + tw + gw;
        end
        y0 = y0 + th + gw;
    end

    % Compose rgb output
    img = img.*ones(1, 1, 3);
end

% ========================================================================
function s = splitPixels(totalPix, nSeg)
%SPLITPIXELS  Split TOTALPIX pixels into NSEG segments as evenly as possible.
%   The first MOD(TOTALPIX,NSEG) segments are 1 px larger so that the sum is
%   exactly TOTALPIX.

    base = floor(totalPix / nSeg);
    rem  = totalPix - base*nSeg;
    s = base * ones(1, nSeg);
    s(1:rem) = s(1:rem) + 1;
end

% ========================================================================
function m = makeTile(rows, cols, octaves, veinFreq, turbPower, veinWeight)
%MAKETILE  Create a single marble tile of size ROWS x COLS (values in 0..1).

    persistence = 0.6;    % amplitude falloff between octaves
    baseCells   = 2;      % coarseness of the lowest octave

    % fractal turbulence (clouds)
    turb = fractalNoise(rows, cols, octaves, persistence, baseCells);

    % coordinate grid
    [X, Y] = meshgrid(linspace(0, 1, cols), linspace(0, 1, rows));

    % random vein direction for each tile
    theta = rand * pi;
    coord = X*cos(theta) + Y*sin(theta);

    % marble veins: sine bands distorted by the turbulence
    veins = abs(sin(pi * (coord*veinFreq + turb*turbPower)));

    % blend of clouds and veins (vein weight is configurable)
    m = (1 - veinWeight)*turb + veinWeight*veins;

    % normalize to 0..1
    m = m - min(m(:));
    m = m / (max(m(:)) + eps);
end

% ========================================================================
function n = fractalNoise(rows, cols, octaves, persistence, baseCells)
%FRACTALNOISE  Sum of several octaves of smooth noise (1/f / "value noise").

    n     = zeros(rows, cols);
    amp   = 1;
    total = 0;
    for o = 1:octaves
        cells = baseCells * 2^(o-1);
        n     = n + amp * smoothNoise(rows, cols, cells);
        total = total + amp;
        amp   = amp * persistence;
    end
    n = n / total;
end

% ========================================================================
function layer = smoothNoise(rows, cols, cells)
%SMOOTHNOISE  Smooth noise: a random (cells+1)^2 grid interpolated with a spline.
%   Uses base MATLAB only (interp2), no toolbox required.

    cells = max(1, round(cells));
    g = rand(cells+1, cells+1);

    [Xg, Yg] = meshgrid(1:cells+1, 1:cells+1);
    [Xq, Yq] = meshgrid(linspace(1, cells+1, cols), ...
                        linspace(1, cells+1, rows));

    layer = interp2(Xg, Yg, g, Xq, Yq, 'spline');
end