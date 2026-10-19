function rgb = planarStone(res, opts)
%phx.extra.planarStone Generate a stone (granite-like) noise texture image
%
%   rgb = phx.extra.planarStone() returns a 256-by-256-by-3 uint8 RGB image
%   of grey stone: cloudy mineral variation, finer mottling and grain with
%   sparse light and dark crystals. The image tiles seamlessly in both
%   directions. It is the built-in "stone" texture.
%
%   rgb = phx.extra.planarStone(res) sets the image resolution in pixels
%   (a square image of res-by-res pixels).
%
%   rgb = phx.extra.planarStone(___, Name, Value) specifies options as
%   name-value pairs:
%   - Seed: seed of the random number generator, so the same seed gives
%     the same image. Default 11 (the built-in texture). The global random
%     number generator is left untouched.
%
%   The colors are an almost neutral, slightly warm grey, so that the Color
%   of a shape tints the texture through TextureBlend. The image is meant
%   to be written to a file and used as the Texture of a shape, for example
%   of phx.shape.Rock, whose flat faces hide the jumps of the projection:
%
%       imwrite(phx.extra.planarStone(512, Seed = 3), "granite.jpg")
%       b = phx.Body(Shape = phx.shape.Rock(Texture = "granite.jpg", ...
%                                           Color = [0.6 0.5 0.4], ...
%                                           TextureBlend = 0.7));
%
%   Note that textures are only rendered in phx.extra.Viewer axes.
%
%   Examples:
%       rgb = phx.extra.planarStone();                  % the built-in one
%       imshow(repmat(rgb, 2, 2))                       % tiles seamlessly
%
%   See also phx.extra.planarChecker, phx.extra.planarMarbleTiles,
%   phx.base.Shape.Texture, phx.shape.Rock, imwrite.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        res (1, 1) double {mustBeInteger, mustBeGreaterThanOrEqual(res, 16)} = 256
        opts.Seed (1, 1) double {mustBeInteger, mustBeNonnegative} = 11
    end

    rs = RandStream("mt19937ar", "Seed", opts.Seed);

    % Domain-warped fBm gives the cloudy mineral variation
    wx = fbm(rs, res, 3, 4)*0.08;
    wy = fbm(rs, res, 3, 4)*0.08;
    base = rescale(fbm(rs, res, 4, 6, wx, wy));
    base = 0.5 + (base - 0.5)*0.6;

    % Mid-scale mottling, fine grain and sparse light/dark crystals
    mottle = fbm(rs, res, 16, 3);
    grain = fbm(rs, res, 64, 2);
    L = base + 0.12*(rescale(mottle) - 0.5) + 0.16*(rescale(grain) - 0.5);
    r = rand(rs, res);
    L(r > 0.975) = L(r > 0.975) + 0.15;
    L(r < 0.02) = L(r < 0.02) - 0.18;
    L = min(max(0.08 + 0.84*L, 0), 1);

    % Slightly warm, low-saturation grey
    tint = 0.04*(rescale(fbm(rs, res, 4, 3)) - 0.5);
    rgb = cat(3, L + tint, L, L - 0.6*tint - 0.02);
    rgb = uint8(255*min(max(rgb, 0), 1));
end

% ========================================================================
function n = fbm(rs, res, cells, octaves, wx, wy)
%fbm Periodic value noise summed over octaves.
%   Each octave doubles the number of lattice cells, starting from CELLS,
%   and halves the amplitude. The lattice wraps around, so the result tiles
%   seamlessly; WX and WY optionally warp the sampling coordinates.

    if nargin < 5
        wx = 0;
        wy = 0;
    end
    [X, Y] = meshgrid((0:res - 1)/res);
    X = mod(X + wx, 1);
    Y = mod(Y + wy, 1);

    n = zeros(res);
    amp = 1;
    for o = 1:octaves
        m = cells*2^(o - 1);
        g = rand(rs, m);
        k = -1:m + 2; % lattice with a periodic margin for the cubic kernel
        idx = mod(k, m) + 1;
        n = n + amp*interp2(k/m, k/m, g(idx, idx), X, Y, "cubic");
        amp = amp/2;
    end
end
