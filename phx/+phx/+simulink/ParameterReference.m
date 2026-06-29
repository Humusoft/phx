classdef ParameterReference
%ParameterReference Port definition structure
%
%   A reference describes one input or output port of the PhxModel block as
%   a piece of text:
%
%       ObjectName.Property            - the whole property
%       ObjectName.Property(indices)   - selected linear indices
%
%   The bracket-less form leaves Indices empty; the signal width is then
%   resolved against the model interface (see
%   phx.simulink.BlockBackend.getModelInterface and resolveRefs).
%
%   Kind distinguishes plain property access ("property", the default) from
%   method access ("method"). Method ports are reserved for future use and
%   are currently rejected during reference resolution.
%
%   See also phx.simulink.BlockBackend, PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    properties
        ObjectName (1, 1) string
        Object = []
        Property (1, 1) string
        Indices (1, :) double
        Size (1, 1) double
        Kind (1, 1) string = "property"
    end

    methods
        function obj = ParameterReference(def)
            arguments
                def (1, 1) string
            end

            obj.ObjectName = strtrim(extractBefore(def, "."));
            if contains(def, "(")
                obj.Property = strtrim(extractBetween(def, ".", "("));
                obj.Indices = str2num(extractBetween(def, "(", ")")); %#ok<ST2NM>
            else
                % Whole property; width resolved later against the interface.
                obj.Property = strtrim(extractAfter(def, "."));
                obj.Indices = [];
            end
            obj.Size = numel(obj.Indices);
        end
    end

end
