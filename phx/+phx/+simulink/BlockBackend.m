classdef BlockBackend < handle
%BlockBackend Support object for the PhxModel block

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    % properties (WeakHandle)
    %     Sim (1, :) phx.Simulation = phx.Simulation.empty
    % end

    properties
        Sim = phx.Simulation.empty
        hF = []
        hA = []
        Source (1, 1) string
        InputRefs (1, :) phx.simulink.ParameterReference
        OutputRefs (1, :) phx.simulink.ParameterReference
        Viewer (1, 1) logical = false
        RenderEachStep (1, 1) logical = true
        Substeps (1, 1) double {mustBePositive, mustBeInteger} = 1
        CameraResolution (1, :) double = []   % [H W]; non-empty/positive enables the rendered-image output port
        CameraOn (1, 1) logical = false       % resolved in setup (needs the viewer)
        CameraPort (1, 1) double = 0           % index of the image output port
        DefaultCameraPosition (1, 3) double = [-10 -10 10]
        DefaultCameraTarget (1, 3) double = [0 0 0]
    end

    methods
    end

    methods (Static)
        % Introspect a scene source without creating the physics engine.
        % Returns a struct array (one entry per object) with fields
        % Name, Class, Properties and Methods. Used for edit-time validation
        % and signal sizing of the PhxModel block.
        function iface = getModelInterface(source)
            arguments
                source (1, 1) string
            end

            [folder, name, ext] = fileparts(source);
            switch lower(ext)
                case ".mat"
                    objs = phx.simulink.BlockBackend.collectObjects(load(source));
                case {".m", ""}
                    if folder ~= ""
                        addpath(folder);
                    end
                    objs = phx.simulink.BlockBackend.collectObjects(feval(name));
                otherwise
                    error("phx:PhxModel:unsupportedSource", "Unsupported model source '%s'.", source);
            end

            iface = struct('Name', {}, 'Class', {}, 'Properties', {}, 'Methods', {});
            for i = 1:numel(objs)
                o = objs{i};
                iface(i) = struct( ...
                    'Name',       string(o.Name), ...
                    'Class',      string(class(o)), ...
                    'Properties', reshape(string(properties(o)), 1, []), ...
                    'Methods',    reshape(string(methods(o)), 1, []));
            end
        end

        % Width (number of elements) of a property, from class metadata only.
        % Returns NaN when the width cannot be determined statically (e.g.
        % unrestricted dimensions or non-numeric/defaultless properties).
        function w = propertyWidth(className, prop)
            arguments
                className (1, 1) string
                prop (1, 1) string
            end

            w = NaN;
            p = findobj(meta.class.fromName(className).PropertyList, 'Name', char(prop));
            if isempty(p)
                return
            end

            % 1) Fixed size validation, e.g. Position (1, 3) -> 3.
            if ~isempty(p.Validation) && ~isempty(p.Validation.Size)
                dims = p.Validation.Size;
                n = ones(1, numel(dims));
                fixed = true;
                for i = 1:numel(dims)
                    if isa(dims(i), 'meta.FixedDimension')
                        n(i) = dims(i).Length;
                    else
                        fixed = false; % unrestricted dimension (:)
                    end
                end
                if fixed
                    w = prod(n);
                    return
                end
            end

            % 2) Fall back to the numeric default value, e.g. [0 0 0] -> 3.
            if p.HasDefault && ~isempty(p.DefaultValue) && isnumeric(p.DefaultValue)
                w = numel(p.DefaultValue);
            end
        end

        % Validate references against a model interface, size bracket-less
        % references and classify their Kind. portKind labels the messages.
        function refs = resolveRefs(refs, iface, portKind)
            arguments
                refs phx.simulink.ParameterReference
                iface struct
                portKind (1, 1) string = "port"
            end

            names = [iface.Name];
            for i = 1:numel(refs)
                r = refs(i);
                idx = find(names == r.ObjectName, 1);
                if isempty(idx)
                    error("phx:PhxModel:objectNotFound", ...
                        "%s reference: object ""%s"" not found in the model.", portKind, r.ObjectName);
                end

                isProp = ismember(r.Property, iface(idx).Properties);
                if ~isProp
                    % Reserved seam: method ports are recognised but not yet supported.
                    if ismember(r.Property, iface(idx).Methods)
                        error("phx:PhxModel:methodPortsUnsupported", ...
                            "%s reference ""%s.%s"": method ports are not yet supported.", ...
                            portKind, r.ObjectName, r.Property);
                    end
                    error("phx:PhxModel:propertyNotFound", ...
                        "%s reference ""%s.%s"": no such public property.", ...
                        portKind, r.ObjectName, r.Property);
                end

                if isempty(r.Indices)
                    w = phx.simulink.BlockBackend.propertyWidth(iface(idx).Class, r.Property);
                    if isnan(w)
                        error("phx:PhxModel:cannotSizeProperty", ...
                            "%s reference ""%s.%s"": cannot determine signal width; specify indices, e.g. %s(1:N).", ...
                            portKind, r.ObjectName, r.Property, r.Property);
                    end
                    refs(i).Indices = 1:w;
                    refs(i).Size = w;
                end
            end
        end

        % List the properties of a class usable as signal ports: public,
        % non-hidden and with a statically determinable numeric width.
        % Returns a struct array with fields Name, Width and ReadOnly
        % (ReadOnly = not settable, so usable only as an output). Engine-free
        % (metadata only).
        function props = signalProperties(className)
            arguments
                className (1, 1) string
            end

            props = struct('Name', {}, 'Width', {}, 'ReadOnly', {});
            for p = meta.class.fromName(className).PropertyList'
                if p.Hidden || ~phx.simulink.BlockBackend.accessIsPublic(p.GetAccess)
                    continue
                end
                w = phx.simulink.BlockBackend.propertyWidth(className, p.Name);
                if isnan(w)
                    continue
                end
                settable = phx.simulink.BlockBackend.accessIsPublic(p.SetAccess) ...
                    && (~p.Dependent || ~isempty(p.SetMethod));
                props(end + 1) = struct('Name', string(p.Name), 'Width', w, 'ReadOnly', ~settable); %#ok<AGROW>
            end
        end

        % True when a meta property access specifier is plain public (not a
        % restricted access list or private/protected).
        function tf = accessIsPublic(access)
            tf = (ischar(access) || isstring(access)) && string(access) == "public";
        end

        % Edit-time validation of a reference list against the model source.
        % Best-effort: returns quietly when the source is empty, not a MAT
        % file, or not loadable yet (validation is then deferred to block
        % setup). Otherwise throws the same phx:PhxModel:* errors as
        % resolveRefs, which the mask dialog surfaces to the user.
        function validateRefs(source, listText, portKind)
            arguments
                source (1, 1) string
                listText (1, 1) string
                portKind (1, 1) string = "port"
            end

            % Skip .m sources: validating them would run the scene builder on
            % every edit. MAT files only load detached objects (no side effects).
            [~, ~, ext] = fileparts(strtrim(source));
            if lower(ext) ~= ".mat"
                return
            end

            try
                iface = phx.simulink.BlockBackend.getModelInterface(source);
            catch
                return % source not resolvable during editing; defer to setup
            end

            refs = phx.simulink.BlockBackend.processRefList(listText);
            phx.simulink.BlockBackend.resolveRefs(refs, iface, portKind);
        end

        % Collect all phx.base.Object instances reachable from a scene source
        % (struct from load / model script return / body array), walking the
        % DAG via Children. De-duplicated by handle, no engine access.
        function objs = collectObjects(data)
            roots = {};
            switch class(data)
                case 'struct'
                    f = fieldnames(data);
                    for i = 1:numel(f)
                        v = data.(f{i});
                        if isa(v, 'phx.base.Object')
                            roots = [roots num2cell(reshape(v, 1, []))]; %#ok<AGROW>
                        end
                    end
                case 'cell'
                    roots = reshape(data, 1, []);
                otherwise
                    if isa(data, 'phx.base.Object')
                        roots = num2cell(reshape(data, 1, []));
                    end
            end

            objs = {};
            stack = roots;
            while ~isempty(stack)
                o = stack{end};
                stack(end) = [];
                if ~isa(o, 'phx.base.Object')
                    continue
                end
                isDup = false;
                for k = 1:numel(objs)
                    if objs{k} == o
                        isDup = true;
                        break
                    end
                end
                if isDup
                    continue
                end
                objs{end + 1} = o; %#ok<AGROW>
                if isprop(o, 'Children') && ~isempty(o.Children)
                    stack = [stack reshape(o.Children, 1, [])]; %#ok<AGROW>
                end
            end
        end

        % Process list of references
        function refs = processRefList(list)
            defs = strtrim(strsplit(string(list), {newline, ','}));
            refs = phx.simulink.ParameterReference.empty;
            for i = 1:numel(defs)
                if defs(i) ~= ""
                    refs(end + 1) = phx.simulink.ParameterReference(defs(i));
                end
            end
        end
        
        % Set input params of S-Function block
        function sfInput(Block, IDs, DataType, Complexity, Sampling, DimMode, Dimensions)
            dataID = find(strcmpi({'inherited', 'double', 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'boolean'}, DataType)) - 2;
            if isempty(dataID)
                dataID = -1;
            end
            for i = IDs
                Block.InputPort(i).DatatypeID = dataID;
                Block.InputPort(i).Complexity = Complexity;
                Block.InputPort(i).SamplingMode = Sampling;
                Block.InputPort(i).DimensionsMode = DimMode;
                Block.InputPort(i).Dimensions = Dimensions;
            end
        end

        % Set output params of S-Function block
        function sfOutput(Block, IDs, DataType, Complexity, Sampling, DimMode, Dimensions)
            dataID = find(strcmpi({'inherited', 'double', 'single', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'boolean'}, DataType)) - 2;
            if isempty(dataID)
                dataID = -1;
            end
            for i = IDs
                Block.OutputPort(i).DatatypeID = dataID;
                Block.OutputPort(i).Complexity = Complexity;
                Block.OutputPort(i).SamplingMode = Sampling;
                Block.OutputPort(i).DimensionsMode = DimMode;
                Block.OutputPort(i).Dimensions = Dimensions;
            end
        end
    end

end