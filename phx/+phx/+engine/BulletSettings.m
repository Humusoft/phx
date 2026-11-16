function s = BulletSettings(Options)
%phx.engine.BulletSettings Engine settings of PHX 1.x (not recommended)
%
%   BulletSettings is not recommended and will be removed in a future
%   release. Use phx.engine.Settings instead.
%
%   s = phx.engine.BulletSettings(Name, Value, ...) takes the name-value
%   arguments of PHX 1.x, warns, and returns phx.engine.Settings that carry
%   LengthScale, Margin and MarginMode over unchanged. The other settings
%   of PHX 1.x have no counterpart and are left out:
%
%     Solver        - there is one solver; it solves the joints of a group
%                     of connected bodies together and directly
%     Iterations    - the iterations now end at JointTolerance and
%                     ContactTolerance, capped by JointIterations and
%                     ContactIterations; the defaults usually do better
%     AutoActivated - bodies never fall asleep
%
%   The result is a phx.engine.Settings object: the properties of PHX 1.x
%   cannot be assigned to it afterwards.
%
%   See also phx.engine.Settings, phx.Simulation

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        Options.Solver {mustBeMember(Options.Solver, ["sequentialimpulse", "dantzig", "lemke", "gaussseidelprojected", "nncg"])}
        Options.Iterations (1, 1) double {mustBeInteger, mustBePositive}
        Options.AutoActivated (1, 1) logical
        Options.LengthScale
        Options.Margin
        Options.MarginMode
    end

    dropped = intersect(["Solver", "Iterations", "AutoActivated"], string(fieldnames(Options))', "stable");
    msg = "phx.engine.BulletSettings will be removed in a future release; use phx.engine.Settings instead.";
    if ~isempty(dropped)
        msg = msg + " Ignored, with no counterpart in phx.engine.Settings: " + join(dropped, ", ") + ".";
    end
    warning("phx:BulletSettings:replaced", "%s", msg);

    % The remaining options are applied as the former constructor applied
    % them
    Options = rmfield(Options, cellstr(dropped));
    args = namedargs2cell(Options);
    s = phx.engine.Settings(args{:});
end
