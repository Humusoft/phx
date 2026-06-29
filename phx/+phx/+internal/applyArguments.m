function target = applyArguments(pairs, target)
%APPLYARGUMENTS  Copy name-value fields from struct PAIRS onto TARGET.

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    names = fieldnames(pairs);
    for i = 1:numel(names)
        target.(names{i}) = pairs.(names{i});
    end
end