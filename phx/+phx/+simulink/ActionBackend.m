classdef ActionBackend < handle
%ActionBackend Support object for the PhxAction block.
%   Holds the binding to the scene-defining PhxModel block and the persistent
%   user state between triggered action calls.

%   Copyright 2026 HUMUSOFT s.r.o.
%   ^..^

    properties
        Code (1, :) char = ''      % user action code
        MainSID (1, :) char = ''   % SID of the bound PhxModel block ('' = auto-bind)
        NumOut (1, 1) double = 0
        Out cell = {}              % last output values (held between triggers)
        State struct = struct      % persistent user state across calls
        PrevTrig (1, 1) double = 0
        Resolved (1, 1) logical = false
        Sim = []                   % bound phx.Simulation (from the PhxModel block)
        Ax = []                    % bound viewer axes (BB.hA)
    end

end
