function captureExampleThumb(name, outFile, delay, width, height, press)
%CAPTUREEXAMPLETHUMB Grab a frame of a running phxex_* example.
%
%   CAPTUREEXAMPLETHUMB(NAME, OUTFILE, DELAY, WIDTH, HEIGHT) runs the example
%   NAME, waits DELAY seconds, exports whatever the example is drawing at that
%   moment and writes it to OUTFILE scaled to WIDTH-by-HEIGHT pixels. A NAME
%   that is a Simulink model is simulated instead of called.
%
%   CAPTUREEXAMPLETHUMB(___, PRESS) first presses the buttons named in PRESS
%   (comma separated), which is what an interactive example needs before there
%   is anything worth a picture - it opens its window and returns, so the run
%   only starts when a button is pushed.
%
%   The capture happens in a timer callback, i.e. while the example is still
%   simulating - PHX pumps the event queue on every redraw, so the timer fires
%   between two simulation steps and the thumbnail shows the scene in motion
%   rather than its initial or final pose. Examples that finish sooner than
%   DELAY are captured on return instead.
%
%   This function terminates the MATLAB process when it is done, so it must be
%   run in its own headless session (that is also what keeps one misbehaving
%   example from poisoning the next one):
%
%       matlab -batch "addpath('phx');addpath('tools');cd examples;
%                      captureExampleThumb('phxex_gears','out.png',6,320,240)"
%
%   buildExampleDocs spawns exactly that command for every example; call this
%   function directly only when re-shooting a single thumbnail by hand.
%
%   See also buildExampleDocs, exampleCatalog

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        name (1, 1) string
        outFile (1, 1) string
        delay (1, 1) double = 6
        width (1, 1) double = 320
        height (1, 1) double = 240
        press (1, 1) string = ""
    end

    captured = false;

    % Fires while the example runs; the hard-kill timer only matters for
    % examples that block on user input and would otherwise hang the process.
    shot = timer("StartDelay", delay, "TimerFcn", @onShot, "ErrorFcn", @onShotError);
    kill = timer("StartDelay", delay + 60, "TimerFcn", @(~, ~) finish(2));
    start(shot);
    start(kill);

    try
        if isfile(name + ".slx")
            sim(name);                      % a Simulink example
        else
            feval(name);
            pushButtons;                    % an interactive one needs a nudge
        end
    catch err
        fprintf("THUMB error running %s: %s\n", name, err.message);
    end

    % The example returned before the timer fired - capture its final frame.
    % (If the timer got there first it has already asked for the shutdown, and
    % the workspace this code runs in may be gone by now.)
    try
        grab;
        finish(double(~captured));
    catch
        exit(0);
    end

    function pushButtons
        % Press the buttons of an interactive example, in order. Each callback
        % runs its whole stage, so the shot timer fires inside one of them.
        for label = strip(split(press, ","))'
            if label == "" || captured
                continue
            end
            b = findall(groot, "Type", "uibutton", "Text", label);
            if isempty(b)
                fprintf("THUMB no button ""%s"" in %s\n", label, name);
            else
                feval(b(1).ButtonPushedFcn, b(1), []);
            end
        end
    end

    function onShot(~, ~)
        grab;
        finish(double(~captured));
    end

    function onShotError(~, evt)
        fprintf("THUMB capture failed: %s\n", evt.Data.message);
        finish(1);
    end

    function grab
        if captured
            return;
        end
        fig = currentFigure;
        if isempty(fig)
            fprintf("THUMB no figure for %s\n", name);
            return;
        end

        % A 4:3 window makes the exported axes match the thumbnail aspect
        % ratio, so the fitting below has almost nothing left to do.
        try
            fig.Position(3:4) = [4 3]*height;
            drawnow;
        catch
            % Some figures refuse to be resized - export them as they are.
        end

        raw = string(tempname) + ".png";
        try
            exportgraphics(exportTarget(fig), raw, "Resolution", 150);
            imwrite(fitImage(imread(raw), width, height), outFile);
            captured = true;
            fprintf("THUMB ok %s\n", name);
        catch err
            fprintf("THUMB export failed for %s: %s\n", name, err.message);
        end
        delete(raw);
    end

    function finish(code)
        stop([shot kill]);
        exit(code);
    end

end

function fig = currentFigure
% The figure showing the scene. Many examples pop up result plots when they
% finish, and those must not win over the simulation itself - a figure holding
% hgtransforms is a PHX scene, anything else is a plot.
    figs = findall(groot, "Type", "figure");
    if isempty(figs)
        fig = [];
        return;
    end

    scenes = figs(arrayfun(@(f) ~isempty(findall(f, "Type", "hgtransform")), figs));
    if ~isempty(scenes)
        fig = scenes(1);
        return;
    end

    fig = get(groot, "CurrentFigure");
    if isempty(fig) || ~isvalid(fig)
        fig = figs(1);
    end
end

function target = exportTarget(fig)
% Export a single axes on its own (that drops dialogs, buttons and the HUD
% overlay), but keep the whole figure when the example builds a subplot
% layout - there the arrangement is the point.
    ax = findall(fig, "-isa", "matlab.graphics.axis.AbstractAxes");
    ax = ax(arrayfun(@(a) strcmp(a.Visible, "on"), ax));
    if isscalar(ax)
        target = ax;
    else
        target = fig;
    end
end

function out = fitImage(img, width, height)
% Bring an arbitrary screenshot to exactly width-by-height: crop to the target
% aspect ratio when the mismatch is small, pad with white when it is not (a
% plot with axis labels must not lose its edges).
    [h, w, ~] = size(img);
    if size(img, 3) == 1
        img = repmat(img, 1, 1, 3);
    end
    target = width/height;
    actual = w/h;

    if abs(actual - target)/target < 0.3
        if actual > target                          % too wide - trim sides
            keep = round(h*target);
            first = floor((w - keep)/2) + 1;
            img = img(:, first:first + keep - 1, :);
        else                                        % too tall - trim top/bottom
            keep = round(w/target);
            first = floor((h - keep)/2) + 1;
            img = img(first:first + keep - 1, :, :);
        end
        out = resizeImage(img, [height width]);
    else
        scale = min(width/w, height/h);
        inner = resizeImage(img, max(round([h w]*scale), 1));
        out = repmat(uint8(255), height, width, 3);
        r = floor((height - size(inner, 1))/2) + 1;
        c = floor((width - size(inner, 2))/2) + 1;
        out(r:r + size(inner, 1) - 1, c:c + size(inner, 2) - 1, :) = inner;
    end
end

function out = resizeImage(img, outSize)
% imresize when Image Processing Toolbox is around, bilinear interp2 otherwise.
    if exist("imresize", "file") == 2
        out = imresize(img, outSize);
        return;
    end
    [h, w, ~] = size(img);
    xi = linspace(1, w, outSize(2));
    yi = linspace(1, h, outSize(1));
    [X, Y] = meshgrid(xi, yi);
    out = zeros([outSize 3], "uint8");
    for k = 1:3
        out(:, :, k) = uint8(interp2(double(img(:, :, k)), X, Y, "linear"));
    end
end
