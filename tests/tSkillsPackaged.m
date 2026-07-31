classdef tSkillsPackaged < matlab.unittest.TestCase
%tSkillsPackaged Keeps the shipped AI agent skills in sync with the working ones.
%
%   The skills exist twice on purpose: .claude/skills is what this repository's
%   own agents load, phx/skills is what goes into the packaged toolbox and what
%   users copy into their projects (see phx/doc/phx_ug_skills.html). These tests
%   fail when the two copies drift apart, when a skill is added to one side only,
%   or when the packaging configuration would silently drop the Markdown files
%   from the .mltbx.
%
%   Pure tests - no engine, no graphics.
%
%   See also tPhxSimulinkRefs

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant)
        Working = ".claude/skills"      % source of truth, used by this repo
        Shipped = "phx/skills"          % copy packaged into the toolbox
    end

    properties
        Root
    end

    methods (TestClassSetup)
        function locateRoot(tc)
            tc.Root = string(fileparts(fileparts(mfilename("fullpath"))));
            tc.assumeTrue(isfolder(fullfile(tc.Root, tc.Working)), ...
                "Working skills folder not found - not a source checkout.");
        end
    end

    methods (Test)
        function bothCopiesHoldTheSameSkills(tc)
            tc.verifyEqual(tc.skillNames(tc.Shipped), tc.skillNames(tc.Working), ...
                "The shipped skills (" + tc.Shipped + ") and the working skills (" + ...
                tc.Working + ") are not the same set.");
        end

        function shippedCopiesAreIdentical(tc)
            for name = tc.skillNames(tc.Working)
                working = tc.readSkill(tc.Working, name);
                shipped = tc.readSkill(tc.Shipped, name);
                tc.assumeNotEmpty(shipped, "Skill " + name + " is not shipped.");
                tc.verifyEqual(shipped, working, ...
                    "Skill " + name + " differs between " + tc.Working + " and " + ...
                    tc.Shipped + ". Re-copy it into " + tc.Shipped + ".");
            end
        end

        function frontMatterNameMatchesFolder(tc)
            % An agent addresses a skill by the name in its front matter; a
            % mismatch with the folder name makes it unaddressable.
            for name = tc.skillNames(tc.Working)
                lines = splitlines(tc.readSkill(tc.Working, name));
                tc.verifyEqual(strtrim(lines(1)), "---", ...
                    "Skill " + name + " does not start with YAML front matter.");
                declared = lines(startsWith(lines, "name:"));
                tc.assertNotEmpty(declared, "Skill " + name + " declares no name.");
                tc.verifyEqual(strtrim(erase(declared(1), "name:")), name);
            end
        end

        function packagingKeepsMarkdownFiles(tc)
            % A blanket "**/*.md" exclusion in toolbox.ignore would drop every
            % SKILL.md from the .mltbx without any other visible symptom.
            ignoreFile = fullfile(tc.Root, "toolbox.ignore");
            tc.assumeTrue(isfile(ignoreFile), "toolbox.ignore not found.");

            lines = strtrim(splitlines(string(fileread(ignoreFile))));
            lines = lines(~startsWith(lines, "%") & ~startsWith(lines, "#") & lines ~= "");

            tc.verifyFalse(any(lines == "**/*.md"), ...
                "toolbox.ignore excludes **/*.md, which would drop the shipped skills.");
            tc.verifyFalse(any(lines == "phx/skills/" | lines == "phx/skills/**"), ...
                "toolbox.ignore excludes the shipped skills folder.");
        end
    end

    methods (Access = private)
        function names = skillNames(tc, where)
            d = dir(fullfile(tc.Root, where));
            d = d([d.isdir] & ~startsWith({d.name}, "."));
            names = sort(string({d.name}));
            names = names(arrayfun(@(n) ...
                isfile(fullfile(tc.Root, where, n, "SKILL.md")), names));
        end

        function txt = readSkill(tc, where, name)
            f = fullfile(tc.Root, where, name, "SKILL.md");
            if isfile(f)
                txt = string(fileread(f));
            else
                txt = string.empty;
            end
        end
    end

end
