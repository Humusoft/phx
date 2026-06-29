function phx2p(action)
%PHX2P Internal function

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^
    
    arguments
        action {mustBeMember(action, ["convert", "clean"])} = "convert";
    end

    % Specify the root folder containing the subfolders with .m files
    rootFolder = fileparts(mfilename("fullpath"));

    switch action
        case "convert"
            % Get a list of all .m files in the root folder and its subfolders
            mFiles = dir(fullfile(rootFolder, '**', '*.m'));
            
            % Loop through each .m file and convert it to .p
            for k = 1:length(mFiles)
                if strcmp(mFiles(k).name, 'phx2p.m')
                    continue
                end
            
                % Get the full path of the .m file
                fileName = fullfile(mFiles(k).folder, mFiles(k).name);
            
                % Convert to P-code
                pcode(fileName, "-inplace", "-R2022a");
                disp("Converted: "+fileName);
            end
            
            disp('All .m files have been converted to .p files.');

        case "clean"
            % Get a list of all .m files in the root folder and its subfolders
            mFiles = dir(fullfile(rootFolder, '**', '*.p'));
            
            % Loop through each .m file and convert it to .p
            for k = 1:length(mFiles)
                if strcmp(mFiles(k).name, 'phx2p.m')
                    continue
                end
            
                % Get the full path of the .m file
                fileName = fullfile(mFiles(k).folder, mFiles(k).name);
            
                % Convert to P-code
                delete(fileName);
                disp("Deleted: "+fileName);
            end

            disp('All .p files have been deleted.');
            
    end

end