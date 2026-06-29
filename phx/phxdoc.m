function phxdoc(name)
%PHXDOC Open a PHX HTML reference page in the Help browser.
%
%   PHX integrates its HTML reference pages into the Help browser under
%   Supplemental Software, but the built-in DOC command shows the code
%   comments of an on-path class rather than its custom HTML page. Use
%   PHXDOC to open the HTML reference page by class name instead.
%
%   phxdoc            opens the PHX toolbox landing page.
%
%   phxdoc name       opens the reference page of the given class, e.g.
%                     phxdoc phx.Body
%                     phxdoc phx.shape.Box
%                     phxdoc('phx.RevoluteJoint')
%
%   See also doc, web

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        name (1, 1) string = "phx_product_page"
    end

    docDir = fullfile(fileparts(mfilename('fullpath')), 'doc');

    if name == "phx_product_page"
        page = fullfile(docDir, 'phx_product_page.html');
    else
        page = fullfile(docDir, name + ".html");
    end

    if ~isfile(page)
        error("phx:phxdoc:noPage", ...
            "No PHX reference page for '%s' (expected file: %s).", name, page);
    end

    web(page, '-helpbrowser');
end
