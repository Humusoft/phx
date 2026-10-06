function files = compileMEX(names)
%phx.internal.compileMEX Build the MEX files of phx.internal for the current platform
%
%   phx.internal.compileMEX builds every C++ source in the mex_sources
%   folder of phx.internal into a MEX file next to its .m fallback, for
%   the platform MATLAB runs on (see mexext). A C++ compiler must be
%   configured, see mex -setup C++.
%
%   phx.internal.compileMEX(names) builds only the given sources, named
%   without the extension, e.g. "computeMonopoleForces".
%
%   files = phx.internal.compileMEX(___) returns the paths of the MEX
%   files built.
%
%   All sources are built with the interleaved complex API (-R2018a) and
%   optimization (-O). A source that fails to build does not stop the
%   others; the function errors at the end with the list of failures.
%
%   See also mex, mexext

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        names (1, :) string = string.empty
    end

    outDir = fileparts(mfilename("fullpath"));
    srcDir = fullfile(outDir, "mex_sources");
    src = dir(fullfile(srcDir, "*.cpp"));
    [~, available] = fileparts(string({src.name}));

    if isempty(names)
        names = available;
    else
        unknown = setdiff(names, available, "stable");
        if ~isempty(unknown)
            error("phx:compileMEX:unknownSource", ...
                "No C++ source in %s for: %s.", srcDir, join(unknown, ", "));
        end
    end

    % The sources use the C API, but on macOS mex links .cpp files with the
    % C++ API exports as well and the Xcode linker fails on the missing
    % _mexCreateMexFunction & co., so drop those exports
    extra = {};
    if ismac
        extra = {'LINKEXPORTCPP='};
    end

    files = strings(1, 0);
    failed = strings(1, 0);
    for name = names
        % A loaded MEX file is locked on Windows and cannot be overwritten.
        % clear unloads a package MEX only by its bare name, and only in
        % this MATLAB process; another one that has it loaded keeps it locked
        clear(name);
        try
            mex("-R2018a", "-O", "-silent", extra{:}, "-outdir", outDir, fullfile(srcDir, name + ".cpp"));
            files(end + 1) = fullfile(outDir, name + "." + mexext); %#ok<AGROW> few sources
            fprintf("Built %s\n", files(end));
        catch err
            failed(end + 1) = name; %#ok<AGROW>
            warning("phx:compileMEX:buildFailed", "%s did not build: %s", name, err.message);
        end
    end

    if ~isempty(failed)
        error("phx:compileMEX:buildFailed", ...
            "%d of %d sources did not build: %s.", numel(failed), numel(names), join(failed, ", "));
    end

end

