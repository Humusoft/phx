function listNotes(keyword)
%LISTNOTES Internal function

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        keyword = "TODO"
    end

    clc;
    mfiles = dir(fullfile(pwd, "**\*.m"));
    myself = mfilename+".m";

    keyword = "% "+upper(keyword)+" ";
    totalLines = 0;
    commentLines = 0;
    emptyLines = 0;

    for i = 1:numel(mfiles)
        if mfiles(i).name == myself
            continue
        end
        fileName = fullfile(mfiles(i).folder, mfiles(i).name);
        code = strtrim(readlines(fileName));
        totalLines = totalLines + numel(code);
        commentLines = commentLines + sum(startsWith(code, "%"));
        emptyLines = emptyLines + sum(code == "");
        todos = extractAfter(code, keyword);
        id = ~ismissing(todos);
        ln = find(id);
        todos = rmmissing(todos);
        if ~isempty(todos)
            disp(mfiles(i).name);
            for j = 1:numel(todos)
                disp("- "+todos(j)+" ["+ln(j)+"]");
            end
            disp(" ");
        end
    end
    
    disp("Total lines: "+totalLines);
    disp("Code lines: "+(totalLines - commentLines - emptyLines));
    disp("Comment lines: "+commentLines);
    disp(" ");

end