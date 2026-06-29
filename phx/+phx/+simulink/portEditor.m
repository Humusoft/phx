function portEditor(block)
%portEditor Interactive editor for PhxModel input/output port references.
%
%   portEditor(block) opens a dialog that lists the objects and properties of
%   the block's model source and lets the user assemble the InputRefs and
%   OutputRefs lists. Pick an object and a property, set the element indices,
%   then add the reference to the Inputs or Outputs list with the Add button
%   below that list. On OK the assembled lists are written back to the block,
%   which triggers the mask validation.
%
%   The property list shows every addressable, statically sizeable numeric
%   property with its width; read-only properties are marked and may only be
%   used as outputs. Only objects with a non-empty Name are offered.
%
%   See also phx.simulink.BlockBackend, PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    source = string(get_param(block, 'ModelSource'));

    try
        iface = phx.simulink.BlockBackend.getModelInterface(source);
    catch ME
        uialert(uifigure('Name', 'PhxModel', 'Position', [300 300 420 120]), ...
            "Cannot read model """ + source + """:" + newline + string(ME.message), ...
            "Model interface unavailable");
        return
    end

    iface = iface([iface.Name] ~= "");             % addressable objects only
    if isempty(iface)
        uialert(uifigure('Name', 'PhxModel', 'Position', [300 300 420 120]), ...
            "No named objects found in """ + source + """.", "Nothing to edit");
        return
    end
    [~, ia] = unique([iface.Name], 'stable');
    iface = iface(ia);
    names = [iface.Name];

    inputs  = splitRefs(string(get_param(block, 'InputRefs')));
    outputs = splitRefs(string(get_param(block, 'OutputRefs')));
    props = struct('Name', {}, 'Width', {}, 'ReadOnly', {});   % property list for current object

    % --- build UI ---
    fig = uifigure('Name', 'PhxModel - Edit ports', 'Position', [100 100 760 480]);

    uilabel(fig, 'Text', "Source: " + source, 'FontWeight', 'bold', 'Position', [20 446 720 22]);

    uilabel(fig, 'Text', 'Object', 'Position', [20 414 200 18]);
    lstObj = uilistbox(fig, 'Position', [20 120 200 294], 'Items', cellstr(names), ...
        'ValueChangedFcn', @(~, ~) refreshProps());

    uilabel(fig, 'Text', 'Property', 'Position', [232 414 248 18]);
    lstProp = uilistbox(fig, 'Position', [232 160 248 254], 'Items', {}, ...
        'ValueChangedFcn', @(~, ~) onPropSelect());
    uilabel(fig, 'Text', 'Indices', 'Position', [232 120 60 22]);
    edIdx = uieditfield(fig, 'text', 'Position', [292 120 188 22], ...
        'Placeholder', 'all', 'Tooltip', 'Element indices, e.g. 1:3. Leave empty for the whole property.');

    uilabel(fig, 'Text', 'Inputs', 'Position', [500 414 240 18]);
    lstIn = uilistbox(fig, 'Position', [500 300 240 114], 'Items', cellstr(inputs));
    uibutton(fig, 'Text', 'Add', 'Position', [500 270 110 24], ...
        'ButtonPushedFcn', @(~, ~) onAdd("input"));
    uibutton(fig, 'Text', 'Remove', 'Position', [630 270 110 24], ...
        'ButtonPushedFcn', @(~, ~) onRemove("input"));

    uilabel(fig, 'Text', 'Outputs', 'Position', [500 238 240 18]);
    lstOut = uilistbox(fig, 'Position', [500 124 240 114], 'Items', cellstr(outputs));
    uibutton(fig, 'Text', 'Add', 'Position', [500 94 110 24], ...
        'ButtonPushedFcn', @(~, ~) onAdd("output"));
    uibutton(fig, 'Text', 'Remove', 'Position', [630 94 110 24], ...
        'ButtonPushedFcn', @(~, ~) onRemove("output"));

    uibutton(fig, 'Text', 'OK', 'Position', [560 20 90 30], ...
        'ButtonPushedFcn', @(~, ~) onOK());
    uibutton(fig, 'Text', 'Cancel', 'Position', [660 20 90 30], ...
        'ButtonPushedFcn', @(~, ~) delete(fig));

    % Initial selection
    lstObj.Value = names(1);
    refreshProps();

    % --- nested callbacks ---
    function refreshProps()
        cls = iface(names == string(lstObj.Value)).Class;
        props = phx.simulink.BlockBackend.signalProperties(cls);
        lstProp.Items = arrayfun(@propLabel, props, 'UniformOutput', false);
        lstProp.ItemsData = 1:numel(props);
        edIdx.Value = '';
    end

    function onPropSelect()
        % Default to the whole property (bracket-less); the placeholder shows
        % "all". Indices are added only if the user types them.
        edIdx.Value = '';
    end

    function onAdd(target)
        if isempty(lstObj.Value) || isempty(lstProp.Value)
            return
        end
        p = props(lstProp.Value);
        if target == "input" && p.ReadOnly
            uialert(fig, """" + p.Name + """ is read-only and cannot be used as an input.", ...
                "Read-only property");
            return
        end
        idx = strtrim(edIdx.Value);
        if isempty(idx)
            line = string(lstObj.Value) + "." + p.Name;                 % whole property
        elseif isempty(str2num(idx)) %#ok<ST2NM>
            uialert(fig, "Indices must be numeric, e.g. 1:3 or 1. Leave empty for the whole property.", ...
                "Invalid indices");
            return
        else
            line = string(lstObj.Value) + "." + p.Name + "(" + idx + ")";
        end
        if target == "input"
            inputs = appendUnique(inputs, line);
            lstIn.Items = cellstr(inputs);
        else
            outputs = appendUnique(outputs, line);
            lstOut.Items = cellstr(outputs);
        end
    end

    function onRemove(target)
        if target == "input"
            if ~isempty(lstIn.Value)
                inputs(inputs == string(lstIn.Value)) = [];
                lstIn.Items = cellstr(inputs);
            end
        else
            if ~isempty(lstOut.Value)
                outputs(outputs == string(lstOut.Value)) = [];
                lstOut.Items = cellstr(outputs);
            end
        end
    end

    function onOK()
        try
            set_param(block, 'InputRefs',  char(join(inputs, newline)));
            set_param(block, 'OutputRefs', char(join(outputs, newline)));
        catch ME
            uialert(fig, string(ME.message), "Invalid references");
            return
        end
        delete(fig);
    end
end

% --- local helpers ---
function s = propLabel(p)
    if p.ReadOnly
        s = sprintf('%s  (%d, read-only)', p.Name, p.Width);
    else
        s = sprintf('%s  (%d)', p.Name, p.Width);
    end
end

function refs = splitRefs(listText)
    parts = strtrim(split(listText, [string(newline) ","]));
    refs = parts(parts ~= "");
    refs = reshape(refs, 1, []);
end

function arr = appendUnique(arr, item)
    if ~any(arr == item)
        arr = [arr item];
    end
end
