function listNotes(keyword, folder)
%LISTNOTES List TODO-style notes and line counts of the MATLAB sources.
%
%   LISTNOTES prints every "% TODO " note found in the .m files of the
%   toolbox folder phx (the sibling of this tools folder), grouped by file
%   with the line number of each note, followed by the total, code and
%   comment line counts.
%
%   LISTNOTES(keyword) looks for "% KEYWORD " notes instead, e.g. "FIXME".
%
%   LISTNOTES(keyword, folder) scans the given folder and its subfolders.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        keyword (1, 1) string = "TODO"
        folder (1, 1) string = fullfile(fileparts(fileparts(mfilename("fullpath"))), "phx")
    end

    mfiles = dir(fullfile(folder, "**", "*.m"));

    keyword = "% " + upper(keyword) + " ";
    totalLines = 0;
    commentLines = 0;
    emptyLines = 0;

    for i = 1:numel(mfiles)
        fileName = fullfile(mfiles(i).folder, mfiles(i).name);
        code = strtrim(readlines(fileName));
        totalLines = totalLines + numel(code);
        commentLines = commentLines + sum(startsWith(code, "%"));
        emptyLines = emptyLines + sum(code == "");
        notes = extractAfter(code, keyword);
        ln = find(~ismissing(notes));
        if ~isempty(ln)
            disp(mfiles(i).name);
            for j = ln'
                disp("- " + notes(j) + " [" + j + "]");
            end
            disp(" ");
        end
    end

    disp("Total lines: " + totalLines);
    disp("Code lines: " + (totalLines - commentLines - emptyLines));
    disp("Comment lines: " + commentLines);
    disp(" ");

end
