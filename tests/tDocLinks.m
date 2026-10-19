classdef tDocLinks < PhxTestCase
%tDocLinks Keeps the HTML help free of dead links and the class lists in step.
%
%   Every page under phx/doc is written by hand except the generated Examples
%   gallery, so a renamed or deleted topic leaves behind links that are a dead
%   end in the Help Browser - which is how the old "Examples and larger
%   scenes" topic used to linger after the gallery replaced it.
%
%   The classes are listed in three places - Contents.m, the Reference part
%   of helptoc.xml and the class cards of the product page - and all three
%   keep the same groups in the same order, so that a new class is not
%   forgotten in one of them.
%
%   Pure tests - no engine, no graphics.
%
%   See also tExampleGallery

%   Copyright 2026 HUMUSOFT s.r.o.

    properties
        Root
        DocFolder
    end

    methods (TestClassSetup)
        function locateDoc(tc)
            tc.Root = string(fileparts(fileparts(mfilename("fullpath"))));
            tc.DocFolder = fullfile(tc.Root, "phx", "doc");
            tc.assumeTrue(isfolder(tc.DocFolder), ...
                "Documentation folder not found - not a source checkout.");
        end
    end

    methods (Test)
        function everyPageLinkResolves(tc)
            tc.verifyReferencesExist('(?<=href=")[^"#:]+\.html', "links to");
        end

        function everyPageImageExists(tc)
            % A missing figure shows as a broken-image icon in the Help
            % Browser and is easy to miss when a page is edited.
            tc.verifyReferencesExist('(?<=src=")[^"#:]+', "shows");
        end

        function classListsAgree(tc)
            % The reference toc also lists the phx.base.* superclasses, which
            % are for writing new classes, and Contents.m the phxdoc helper;
            % apart from those the three lists are the same.
            [cGroups, cClasses] = tc.contentsGroups;
            [tGroups, tClasses] = tc.tocGroups;
            [pGroups, pClasses] = tc.productPageGroups;

            tc.verifyEqual(tGroups, cGroups, "helptoc.xml and Contents.m group the classes differently.");
            tc.verifyEqual(pGroups, cGroups, "The product page and Contents.m group the classes differently.");
            for i = 1:min([numel(cGroups) numel(tGroups) numel(pGroups)])
                tc.verifyEqual(tClasses{i}, cClasses{i}, ...
                    "helptoc.xml lists other classes under """ + cGroups(i) + """ than Contents.m.");
                tc.verifyEqual(pClasses{i}, cClasses{i}, ...
                    "The product page lists other classes under """ + cGroups(i) + """ than Contents.m.");
            end
        end
    end

    methods (Access = private)
        function [groups, classes] = contentsGroups(tc)
            % "% Group" lines followed by "%   name - description" lines
            lines = strtrim(readlines(fullfile(tc.Root, "phx", "Contents.m")));
            groups = strings(1, 0);
            classes = {};
            for k = 1:numel(lines)
                header = regexp(lines(k), '^% ([A-Z][^-]*)$', "tokens", "once");
                item = regexp(lines(k), '^%\s+(\S+)\s+- ', "tokens", "once");
                if ~isempty(header) && k < numel(lines) && ~isempty(regexp(lines(k + 1), '^%\s+\S+\s+- ', "once"))
                    groups(end + 1) = header; %#ok<AGROW>
                    classes{end + 1} = strings(1, 0); %#ok<AGROW>
                elseif ~isempty(item) && ~isempty(groups)
                    classes{end}(end + 1) = item;
                end
            end
            [groups, classes] = tc.dropGroup(groups, classes, "Documentation");
        end

        function [groups, classes] = tocGroups(tc)
            % Group items of the Reference entry, each with its class items
            text = string(fileread(fullfile(tc.DocFolder, "helptoc.xml")));
            reference = extractAfter(text, ">Reference");
            groups = strings(1, 0);
            classes = {};
            for line = splitlines(reference)'
                header = regexp(line, '^ {8}<tocitem [^>]*>([^<]+)$', "tokens", "once");
                item = regexp(line, '^ {12}<tocitem [^>]*>([^<]+)</tocitem>', "tokens", "once");
                if ~isempty(header)
                    groups(end + 1) = tc.decode(header); %#ok<AGROW>
                    classes{end + 1} = strings(1, 0); %#ok<AGROW>
                elseif ~isempty(item) && ~startsWith(item, "phx.base.")
                    classes{end}(end + 1) = item;
                end
            end
        end

        function [groups, classes] = productPageGroups(tc)
            % Class cards; "Shapes - primitives" and "Shapes - custom" are two
            % cards of one group
            text = string(fileread(fullfile(tc.DocFolder, "phx_product_page.html")));
            cards = regexp(extractAfter(text, ">Classes<"), '<h3>(.*?)</h3>(.*?)</ul>', "tokens");
            groups = strings(1, 0);
            classes = {};
            for k = 1:numel(cards)
                name = strtrim(extractBefore(tc.decode(cards{k}(1)) + " - ", " - "));
                items = string(regexp(cards{k}(2), '<li><a href="[^"]*">([^<]+)</a>', "tokens"));
                if isempty(groups) || groups(end) ~= name
                    groups(end + 1) = name; %#ok<AGROW>
                    classes{end + 1} = strings(1, 0); %#ok<AGROW>
                end
                classes{end} = [classes{end} reshape(items, 1, [])];
            end
        end

        function [groups, classes] = dropGroup(~, groups, classes, name)
            keep = groups ~= name;
            groups = groups(keep);
            classes = classes(keep);
        end

        function text = decode(~, text)
            text = replace(text, ["&amp;" "&mdash;"], ["&" "-"]);
        end

        function verifyReferencesExist(tc, pattern, verb)
            % Every file the pages point at through the given attribute is
            % really there, relative to the doc folder.
            pages = dir(fullfile(tc.DocFolder, "*.html"));
            for p = string({pages.name})
                text = string(fileread(fullfile(tc.DocFolder, p)));
                % reshape: unique returns a 0x1 for a page with no match, and
                % a for loop would walk that empty column once
                targets = reshape(unique(string(regexp(text, pattern, "match"))), 1, []);
                for t = targets
                    tc.verifyTrue(isfile(fullfile(tc.DocFolder, t)), ...
                        p + " " + verb + " " + t + ", which does not exist.");
                end
            end
        end
    end

end
