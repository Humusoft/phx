function [ax, args] = axesTarget(args)
%axesTarget Pop an optional leading axes target from an argument list
%
%   [ax, args] = axesTarget(args) removes a leading drawing target from
%   the cell array of input arguments and returns it separately. The
%   target is either an axes object, or an empty [] requesting objects
%   without graphics (the phx.Body([], ...) headless convention, see
%   phxex_noview). When the arguments carry no target, ax comes back as
%   missing and the builders fall back to gca at drawing time.
%
%   See also phx.assembly.arena, phx.assembly.chain, phx.assembly.scatter,
%   phx.assembly.import, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    if ~isempty(args) && (isequal(args{1}, []) || (isscalar(args{1}) && ...
            (isa(args{1}, "matlab.graphics.axis.Axes") || isa(args{1}, "matlab.ui.control.UIAxes"))))
        ax = args{1};
        args(1) = [];
    else
        ax = missing;
    end
end
