classdef tDocLinks < PhxTestCase
%tDocLinks Keeps the HTML help free of dead links.
%
%   Every page under phx/doc is written by hand except the generated Examples
%   gallery, so a renamed or deleted topic leaves behind links that are a dead
%   end in the Help Browser - which is how the old "Examples and larger
%   scenes" topic used to linger after the gallery replaced it.
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
    end

    methods (Access = private)
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
