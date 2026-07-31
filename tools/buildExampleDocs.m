function buildExampleDocs(options)
%BUILDEXAMPLEDOCS Regenerate the Examples gallery of the HTML help.
%
%   BUILDEXAMPLEDOCS scans examples/phxex_*.m, captures a thumbnail for every
%   example that does not have one yet, writes phx/doc/phx_ex_gallery.html and
%   refreshes the generated part of phx/doc/helptoc.xml. Nothing in the gallery
%   is maintained by hand: the tile title is the command name and the caption is
%   the H1 line of the example itself.
%
%   BUILDEXAMPLEDOCS(Name = Value) accepts:
%
%     Thumbnails - "missing" (default) captures only the ones that are absent,
%                  "all" re-shoots everything, "none" only rewrites the HTML,
%                  or a list of example names to re-shoot
%     SearchDb   - rebuild the help search database afterwards (default true)
%     Width      - thumbnail width in pixels (default 320)
%     Height     - thumbnail height in pixels (default 240)
%
%   Example:
%       buildExampleDocs                                    % incremental
%       buildExampleDocs(Thumbnails = "all")                % re-shoot all
%       buildExampleDocs(Thumbnails = "phxex_galton")       % fix one tile
%
%   The section an example belongs to comes from a "% Category: <name>" line in
%   its header if present, from exampleCatalog otherwise. tests/tExampleGallery
%   fails when an example is missing from the gallery, so a new demo cannot be
%   forgotten here.
%
%   This is a development tool - it is not packaged into the toolbox.
%
%   See also exampleCatalog, captureExampleThumb

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        options.Thumbnails (1, :) string = "missing"
        options.SearchDb (1, 1) logical = true
        options.Width (1, 1) double = 320
        options.Height (1, 1) double = 240
    end

    root = string(fileparts(fileparts(mfilename("fullpath"))));
    exampleDir = fullfile(root, "examples");
    docDir = fullfile(root, "phx", "doc");
    thumbDir = fullfile(docDir, "img", "ex");
    if ~isfolder(thumbDir)
        mkdir(thumbDir);
    end

    examples = scanExamples(exampleDir);
    fprintf("Found %d examples in %s\n", numel(examples), exampleDir);

    shoot = selectThumbnails([examples.Name], options.Thumbnails, thumbDir);
    for k = 1:numel(examples)
        if ismember(examples(k).Name, shoot)
            captureThumb(root, exampleDir, examples(k), thumbDir, options);
        end
        examples(k).Thumbnail = fullfile(thumbDir, examples(k).Name + ".png");
        examples(k).HasThumbnail = isfile(examples(k).Thumbnail);
    end

    missing = [examples(~[examples.HasThumbnail]).Name];
    if ~isempty(missing)
        fprintf("No thumbnail for: %s\n", join(missing, ", "));
    end

    writeGallery(docDir, examples, options);
    patchToc(docDir, examples);

    if options.SearchDb
        fprintf("Rebuilding the help search database...\n");
        addpath(fullfile(root, "phx"));
        builddocsearchdb(char(docDir));
    end
    fprintf("Examples gallery written to %s\n", fullfile(docDir, "phx_ex_gallery.html"));

end

% ---------------------------------------------------------------- scanning ---

function examples = scanExamples(exampleDir)
% One struct per phxex_*.m file, with the metadata scraped from its header.
    files = dir(fullfile(exampleDir, "phxex_*.m"));
    catalog = exampleCatalog;
    examples = struct("Name", {}, "Summary", {}, "Category", {}, "Delay", {}, ...
        "Rank", {}, "Thumbnail", {}, "HasThumbnail", {});

    for k = 1:numel(files)
        name = string(erase(files(k).name, ".m"));
        header = readHeader(fullfile(exampleDir, files(k).name));

        category = tagValue(header, "Category");
        delay = NaN;
        rank = Inf;
        listed = find(catalog.Name == name, 1);
        if ~isempty(listed)
            if category == ""
                category = catalog.Category(listed);
            end
            delay = catalog.Delay(listed);
            rank = listed;
        end
        if category == ""
            category = "Other";
        end

        examples(end + 1) = struct("Name", name, "Summary", summaryLine(header, name), ...
            "Category", category, "Delay", delay, "Rank", rank, ...
            "Thumbnail", "", "HasThumbnail", false); %#ok<AGROW>
    end

    % Inside a section the examples follow the order of the catalog (easiest
    % first), and unlisted ones drop to the end in alphabetical order.
    order = categoryOrder;
    section = arrayfun(@(e) sectionRank(e.Category, order), examples);
    [~, perm] = sortrows([section(:) [examples.Rank]']);
    examples = examples(perm);
end

function r = sectionRank(category, order)
    r = find(order == category, 1);
    if isempty(r)
        r = numel(order) + 1;
    end
end

function lines = readHeader(file)
% The contiguous comment block that follows the function line.
    text = splitlines(string(fileread(file)));
    lines = strings(0);
    started = false;
    for k = 1:numel(text)
        line = strip(text(k));
        if startsWith(line, "%")
            started = true;
            lines(end + 1) = strip(extractAfter(line, 1)); %#ok<AGROW>
        elseif started || line == "" || startsWith(line, "function")
            if started
                break;
            end
        end
    end
end

function s = summaryLine(header, name)
% The H1 text - "PHXEX_GEARS Meshing STL gears driven by an applied torque"
% becomes "Meshing STL gears driven by an applied torque".
    s = "";
    if isempty(header)
        return;
    end
    s = strip(erase(header(1), upper(name)));
    if s == ""
        s = header(1);
    end
end

function value = tagValue(header, tag)
% Value of an optional "% Tag: value" line in the header.
    value = "";
    hit = startsWith(header, tag + ":", "IgnoreCase", true);
    if any(hit)
        first = find(hit, 1);
        value = strip(extractAfter(header(first), strlength(tag) + 1));
    end
end

% ------------------------------------------------------------- thumbnails ---

function shoot = selectThumbnails(names, request, thumbDir)
    if isscalar(request) && request == "none"
        shoot = strings(0);
    elseif isscalar(request) && request == "all"
        shoot = names;
    elseif isscalar(request) && request == "missing"
        shoot = names(arrayfun(@(n) ~isfile(fullfile(thumbDir, n + ".png")), names));
    else
        unknown = setdiff(request, names);
        if ~isempty(unknown)
            error("phx:buildExampleDocs:unknownExample", ...
                "Not an example: %s.", join(unknown, ", "));
        end
        shoot = request;
    end
end

function captureThumb(root, exampleDir, example, thumbDir, options)
% Shoot one example in its own headless MATLAB (see captureExampleThumb).
    delay = example.Delay;
    if isnan(delay)
        delay = 6;
    end
    outFile = fullfile(thumbDir, example.Name + ".png");
    exe = fullfile(matlabroot, "bin", "matlab");
    batch = sprintf("addpath('%s');addpath('%s');cd('%s');captureExampleThumb('%s','%s',%g,%d,%d)", ...
        fullfile(root, "phx"), fullfile(root, "tools"), exampleDir, ...
        example.Name, replace(outFile, "\", "/"), delay, options.Width, options.Height);

    fprintf("Capturing %-18s (%4.1f s) ... ", example.Name, delay);
    started = tic;
    [status, out] = system(sprintf('"%s" -batch "%s"', exe, batch));
    if isfile(outFile)
        % Tearing an engine world down under exit() sometimes kills the child
        % after the file is safely on disk - that is not a failed capture.
        note = "";
        if status ~= 0
            note = sprintf(" (child exited with %d)", status);
        end
        fprintf("%.0f s%s\n", toc(started), note);
    else
        fprintf("FAILED (status %d)\n", status);
        disp(strip(string(out)));
    end
end

% ------------------------------------------------------------------- HTML ---

function writeGallery(docDir, examples, options)
    categories = categoryOrder;
    used = unique([examples.Category], "stable");
    categories = [categories(ismember(categories, used)), setdiff(used, categories, "stable")];

    html = strings(0);
    html(end + 1) = "<!DOCTYPE html>";
    html(end + 1) = "<html lang=""en"">";
    html(end + 1) = "<head>";
    html(end + 1) = "<meta charset=""utf-8"">";
    html(end + 1) = "<meta name=""viewport"" content=""width=device-width, initial-scale=1"">";
    html(end + 1) = "<title>PHX Examples</title>";
    html(end + 1) = "<link rel=""stylesheet"" href=""phx.css"">";
    html(end + 1) = "</head>";
    html(end + 1) = "<body>";
    html(end + 1) = "<div class=""phx-topbar""></div>";
    html(end + 1) = "<div class=""phx-wrap"">";
    html(end + 1) = "";
    html(end + 1) = "<!-- Generated by tools/buildExampleDocs.m - do not edit by hand. -->";
    html(end + 1) = "";
    html(end + 1) = "<div class=""phx-crumb""><a href=""phx_product_page.html"">PHX</a> &rsaquo; Examples</div>";
    html(end + 1) = "";
    html(end + 1) = "<h1 class=""phx-title"">Examples</h1>";
    html(end + 1) = sprintf("<p class=""phx-purpose"">%d ready-made scenes shipped with the toolbox</p>", numel(examples));
    html(end + 1) = "";
    html(end + 1) = "<p>Every example is a single command &mdash; type its name in the Command Window to run it," + newline + ...
        "or open it in the Editor and change it. The files live in the <code class=""phx-inline"">examples</code>" + newline + ...
        "folder of the toolbox; each thumbnail below is a frame of the running simulation.</p>";
    html(end + 1) = "";
    html(end + 1) = "<div class=""phx-note"">";
    html(end + 1) = "<span class=""lbl"">Tip.</span> The <b>Run</b> and <b>Open</b> links work inside the MATLAB&reg;" + newline + ...
        "Help Browser. Outside it, use <code class=""phx-inline"">phxex_minimal</code> and" + newline + ...
        "<code class=""phx-inline"">edit phxex_minimal</code> in the Command Window.";
    html(end + 1) = "</div>";

    for c = categories
        inSection = examples(strcmp([examples.Category], c));
        html(end + 1) = ""; %#ok<AGROW>
        html(end + 1) = sprintf("<h2 class=""phx-h2"" id=""%s"">%s</h2>", slug(c), escape(c)); %#ok<AGROW>
        html(end + 1) = "<div class=""phx-gal"">"; %#ok<AGROW>
        for e = inSection
            html(end + 1) = tile(e, options); %#ok<AGROW>
        end
        html(end + 1) = "</div>"; %#ok<AGROW>
    end

    html(end + 1) = "";
    html(end + 1) = "<div class=""phx-seealso"">";
    html(end + 1) = "<span class=""lbl"">Topics</span>";
    html(end + 1) = "<a href=""phx_ug_gettingstarted.html"">Getting started</a>,";
    html(end + 1) = "<a href=""phx_ug_concepts.html"">Core concepts</a>,";
    html(end + 1) = "<a href=""phx_ug_control.html"">Automation and control</a>";
    html(end + 1) = "</div>";
    html(end + 1) = "";
    html(end + 1) = "<div class=""phx-foot"">Copyright 2026 HUMUSOFT s.r.o. &middot; PHX Toolbox Examples</div>";
    html(end + 1) = "";
    html(end + 1) = "</div>";
    html(end + 1) = "</body>";
    html(end + 1) = "</html>";

    writeText(fullfile(docDir, "phx_ex_gallery.html"), join(html, newline) + newline);
end

function s = tile(example, options)
    name = example.Name;
    if example.HasThumbnail
        thumb = sprintf("<img src=""img/ex/%s.png"" alt=""%s"" width=""%d"" height=""%d"">", ...
            name, escape(name), options.Width, options.Height);
    else
        thumb = sprintf("<span class=""phx-tile-noimg"">%s</span>", escape(name));
    end

    s = "<div class=""phx-tile"">" + newline + ...
        sprintf("<a class=""phx-tile-shot"" href=""matlab:%s"" title=""Run %s"">%s</a>", name, name, thumb) + newline + ...
        "<div class=""phx-tile-text"">" + newline + ...
        sprintf("<div class=""phx-tile-cmd"">%s</div>", escape(name)) + newline + ...
        sprintf("<p>%s</p>", dashes(escape(example.Summary))) + newline + ...
        "</div>" + newline + ...
        sprintf("<div class=""phx-tile-acts""><a href=""matlab:%s"">Run</a><a href=""matlab:edit('%s')"">Open</a></div>", name, name) + newline + ...
        "</div>";
end

function order = categoryOrder
% Section order of the gallery; unknown sections are appended at the end.
    order = ["Basics and geometry"
        "Contacts, stacking and granular"
        "Mechanisms and machines"
        "Vehicles, robots and control"
        "Fields, magnetism and fluids"
        "Analysis and workflows"
        "Other"]';
end

% -------------------------------------------------------------- helptoc ---

function patchToc(docDir, examples)
% Replace the generated block of helptoc.xml with one tocitem per section.
    file = fullfile(docDir, "helptoc.xml");
    text = string(fileread(file));
    startTag = "<!-- BEGIN generated by tools/buildExampleDocs.m -->";
    endTag = "<!-- END generated -->";

    if ~contains(text, startTag) || ~contains(text, endTag)
        error("phx:buildExampleDocs:missingMarkers", ...
            "%s has no generated block - add the %s / %s markers first.", file, startTag, endTag);
    end

    categories = unique([examples.Category], "stable");
    order = categoryOrder;
    categories = [order(ismember(order, categories)), setdiff(categories, order, "stable")];

    items = arrayfun(@(c) sprintf("        <tocitem target=""phx_ex_gallery.html#%s"">%s</tocitem>", ...
        slug(c), escape(c)), categories);

    before = extractBefore(text, startTag);
    after = extractAfter(text, endTag);
    writeText(file, before + startTag + newline + join(items, newline) + newline + "        " + endTag + after);
end

% --------------------------------------------------------------- helpers ---

function s = escape(s)
    s = replace(s, "&", "&amp;");
    s = replace(s, "<", "&lt;");
    s = replace(s, ">", "&gt;");
end

function s = dashes(s)
% " - " reads better as an en dash in the caption.
    s = replace(s, " - ", " &ndash; ");
end

function s = slug(s)
    s = lower(regexprep(s, "[^A-Za-z0-9]+", "-"));
    s = regexprep(s, "(^-)|(-$)", "");
end

function writeText(file, text)
    fid = fopen(file, "w", "n", "UTF-8");
    if fid < 0
        error("phx:buildExampleDocs:cannotWrite", "Cannot write %s.", file);
    end
    closer = onCleanup(@() fclose(fid));
    fwrite(fid, text, "char");
end
