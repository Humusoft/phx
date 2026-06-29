function rgb = equirectGradient(varargin)
%EQUIRECTGRADIENT  Gridless orientation sky from four colours.
%
%   RGB = EQUIRECTGRADIENT(Name,Value,...) builds an seamless
%   equirectangular background coloured purely by a gradient between four
%   colours: NORTH and SOUTH are the sphere's poles (a vertical, pole-to-
%   pole gradient), while EAST and WEST are two opposite directions around
%   the equator (a horizontal gradient as you turn around).
%
%   At each point the colour is a weighted blend of the vertical gradient
%   (weight |sin lat|) and the horizontal gradient (weight cos lat): the
%   poles are pure North/South, the equator is a pure East/West blend, and
%   everything in between mixes smoothly.
%
%   Name-Value options:
%     'Width','Height'                          (default 1024, Width/2)
%     'NorthColor' 1x3  top pole   (default [0.30 0.45 0.70])  cool blue
%     'SouthColor' 1x3  bottom pole(default [0.72 0.42 0.40])  warm red
%     'EastColor'  1x3             (default [0.35 0.65 0.50])  green
%     'WestColor'  1x3             (default [0.78 0.68 0.40])  amber
%     'EastAngle'  azimuth of East, deg             (default 0)
%
%   Output: RGB (H x W x 3, [0,1]).
%
%   With no output arguments a figure compares the flat map with a sphere.
%
%   Example:
%     rgb = equirectGradient; imshow(rgb);
%     equirectGradient('NorthColor',[0.9 0.9 0.95], ...
%                                  'SouthColor',[0.12 0.13 0.16]);

    p = inputParser;
    addParameter(p,'Width',1024, @(x)isnumeric(x)&&isscalar(x)&&x>=8);
    addParameter(p,'Height',[],   @(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=4));
    addParameter(p,'NorthColor',[0.30 0.45 0.70], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'SouthColor',[0.72 0.42 0.40], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'EastColor', [0.35 0.65 0.50], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'WestColor', [0.78 0.68 0.40], @(x)isnumeric(x)&&numel(x)==3);
    addParameter(p,'EastAngle',0, @(x)isnumeric(x)&&isscalar(x));
    parse(p,varargin{:});
    o = p.Results;

    W = round(o.Width);
    if isempty(o.Height), H = round(W/2); else, H = round(o.Height); end
    H = 2 * round(H/2);
    c3 = @(c) reshape(c(:).',1,1,3);

    lat = (pi/2) - ((0:H-1).' + 0.5) / H * pi;     % +pi/2 top .. -pi/2 bottom
    lon = (0:W-1) / W * 2*pi;
    [LON, LAT] = meshgrid(lon, lat);

    % ---- vertical pole-to-pole gradient (North <-> South) --------------
    vfac = (sin(LAT) + 1) / 2;                      % 0 south .. 1 north
    Cv = c3(o.SouthColor) + vfac .* (c3(o.NorthColor) - c3(o.SouthColor));

    % ---- horizontal East <-> West gradient -----------------------------
    hfac = (cos(LON - o.EastAngle*pi/180) + 1) / 2; % 1 at East, 0 at West
    Ch = c3(o.WestColor) + hfac .* (c3(o.EastColor) - c3(o.WestColor));

    % ---- blend: poles weighted by |sin lat|, equator by cos lat --------
    wV = abs(sin(LAT));
    wH = cos(LAT);
    rgb = (wV.*Cv + wH.*Ch) ./ (wV + wH);

    rgb = min(max(rgb, 0), 1);

    if nargout == 0
        figure('Color',[0 0 0]);
        subplot(1,2,1); imshow(rgb); title('Four-colour orientation sky','Color','w');
        subplot(1,2,2);
        [xs,ys,zs] = sphere(180);
        surf(xs,ys,zs,'FaceColor','texturemap','CData',flipud(rgb),'EdgeColor','none');
        axis equal off; view(35,15); set(gca,'Color','k');
        title('Wrapped onto a sphere','Color','w');
        clear rgb
    end
end