function rgb = planarChecker(res, count, opts)
%phx.extra.planarChecker Generate a checkerboard texture image
%
%   rgb = phx.extra.planarChecker() returns a 512-by-512-by-3 uint8 RGB
%   image with an 8-by-8 black and white checkerboard. The top left square
%   is the first color.
%
%   rgb = phx.extra.planarChecker(res) sets the image resolution in pixels,
%   either as a scalar for a square image or as [height width].
%
%   rgb = phx.extra.planarChecker(res, count) sets the number of squares,
%   either as a scalar for the same count in both directions or as
%   [rows columns], e.g. [2 2] or [3 6].
%
%   The square size follows from the resolution and the square count. When
%   the count does not divide the resolution evenly, the remaining pixels
%   are spread over the first rows and columns, so some squares come out
%   one pixel larger and the image always has exactly the requested size.
%
%   rgb = phx.extra.planarChecker(___, Name, Value) specifies options as
%   name-value pairs:
%   - Color1: color of the first square (the top left one), given as an RGB
%     triplet in the range 0..1 or as a scalar shade of gray. Default 1
%     (white).
%   - Color2: color of the other square, same format. Default 0 (black).
%
%   The image is meant to be written to a file and used as the Texture of a
%   shape, which takes a file name rather than image data:
%
%       imwrite(phx.extra.planarChecker(512, [3 6]), "floor.png")
%       b = phx.Body(Shape = phx.shape.Box(Size = [3 6 0.1], ...
%                                          Texture = "floor.png"));
%
%   Note that textures are only rendered in phx.extra.Viewer axes.
%
%   Examples:
%       rgb = phx.extra.planarChecker();                % 512x512, 8x8
%       imshow(rgb)
%
%       rgb = phx.extra.planarChecker(1024, [2 2]);     % four large squares
%       imshow(rgb)
%
%       rgb = phx.extra.planarChecker([300 600], [3 6], ...
%                                     Color1 = [0.9 0.9 0.2], Color2 = 0.2);
%       imwrite(rgb, "checker.png")
%
%   See also phx.extra.planarMarbleTiles, phx.base.ShapeMesh.Texture,
%   phx.shape.Box, imwrite.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        res (1, :) double {mustBeInteger, mustBePositive, mustBeNumElements(res, 2)} = 512
        count (1, :) double {mustBeInteger, mustBePositive, mustBeNumElements(count, 2)} = 8
        opts.Color1 (1, :) double {mustBeInRange(opts.Color1, 0, 1), mustBeNumElements(opts.Color1, 3)} = 1
        opts.Color2 (1, :) double {mustBeInRange(opts.Color2, 0, 1), mustBeNumElements(opts.Color2, 3)} = 0
    end

    % Expand the scalar shorthands
    if isscalar(res)
        res = [res res];
    end
    if isscalar(count)
        count = [count count];
    end
    color1 = expandColor(opts.Color1);
    color2 = expandColor(opts.Color2);

    if any(res < count)
        error("phx:extra:imageTooSmall", ...
            "Resolution [%d %d] is too small for [%d %d] squares.", ...
            res(1), res(2), count(1), count(2));
    end

    % Pixel height of every square row and pixel width of every column
    rowHeight = splitPixels(res(1), count(1));
    colWidth = splitPixels(res(2), count(2));

    % Square indices of every pixel row/column, combined into the pattern
    rowIndex = repelem(1:count(1), rowHeight);
    colIndex = repelem(1:count(2), colWidth);
    first = mod(rowIndex' + colIndex, 2) == 0;

    % Paint the two colors channel by channel
    rgb = zeros([res 3]);
    for k = 1:3
        channel = repmat(color2(k), res);
        channel(first) = color1(k);
        rgb(:, :, k) = channel;
    end

    rgb = uint8(round(rgb*255));
end

% ========================================================================
function clr = expandColor(clr)
%expandColor Turn a scalar shade of gray into an RGB triplet.

    if isscalar(clr)
        clr = [clr clr clr];
    end
end

% ========================================================================
function pix = splitPixels(totalPix, nSeg)
%splitPixels Split TOTALPIX pixels into NSEG segments as evenly as possible.
%   The first MOD(TOTALPIX,NSEG) segments are 1 px larger, so that the sum
%   is exactly TOTALPIX.

    base = floor(totalPix/nSeg);
    pix = base*ones(1, nSeg);
    pix(1:totalPix - base*nSeg) = base + 1;
end

% ========================================================================
function mustBeNumElements(value, allowed)
%mustBeNumElements Validate that a value is a scalar or has ALLOWED elements.

    if ~isscalar(value) && numel(value) ~= allowed
        throwAsCaller(MException("phx:extra:invalidSize", ...
            "Value must be a scalar or have %d elements.", allowed));
    end
end
