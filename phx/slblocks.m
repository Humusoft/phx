function blkStruct = slblocks
%SLBLOCKS Register the PHX block library with the Simulink Library Browser.
%
%   Simulink calls this function to discover block libraries on the MATLAB
%   path. It lists PhxLibrary as "PHX Toolbox" in the Library Browser.
%
%   See also PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    Browser.Library = 'PhxLibrary';
    Browser.Name = 'PHX Toolbox';

    blkStruct.Browser = Browser;
end
