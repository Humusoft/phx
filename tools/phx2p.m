function phx2p(action)
%PHX2P Convert the toolbox sources to P-code, or remove the P-code again.
%
%   PHX2P or PHX2P("convert") writes a .p file next to every .m file of the
%   toolbox folder phx (the sibling of this tools folder), for a P-coded
%   release.
%
%   PHX2P("clean") deletes all .p files of the toolbox folder again.
%
%   See also pcode

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        action {mustBeMember(action, ["convert", "clean"])} = "convert";
    end

    rootFolder = fullfile(fileparts(fileparts(mfilename("fullpath"))), "phx");

    switch action
        case "convert"
            mFiles = dir(fullfile(rootFolder, "**", "*.m"));
            for k = 1:numel(mFiles)
                fileName = fullfile(mFiles(k).folder, mFiles(k).name);
                pcode(fileName, "-inplace", "-R2022a");
                disp("Converted: " + fileName);
            end
            disp("All .m files have been converted to .p files.");

        case "clean"
            pFiles = dir(fullfile(rootFolder, "**", "*.p"));
            for k = 1:numel(pFiles)
                fileName = fullfile(pFiles(k).folder, pFiles(k).name);
                delete(fileName);
                disp("Deleted: " + fileName);
            end
            disp("All .p files have been deleted.");
    end

end
