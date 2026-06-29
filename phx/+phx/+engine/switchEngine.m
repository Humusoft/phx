function switchEngine(version, name)
%phx.engine.switchEngine Auxiliary function to switch engine version

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        version (1, 1) string = "release"
        name (1, 1) string = "bullet"
    end

    clear("mex"); %#ok<CLMEX> the mex file will be overwritten

    mexFile = which("phx.engine.io");
    [filepath, ~, ext] = fileparts(mexFile);
    copyfile(fullfile(filepath, name+"_"+version+ext), mexFile, "f");
end