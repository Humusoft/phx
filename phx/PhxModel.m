function PhxModel(block, varargin)
%PhxModel Simulink block

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    if ischar(block)
        feval(block, varargin{:});
    else
        setup(block);
    end
end


% --- S-FUNCTION CALLBACKS ---

function setup(block)
    % Create internal support class
    BB = get_param(block.BlockHandle, 'UserData');
    if isempty(BB)
        BB = phx.simulink.BlockBackend;
        set_param(block.BlockHandle, 'UserData', BB);
    end

    % Initialize default values of internal parameters
    BB.Source = block.DialogPrm(1).Data;
    BB.InputRefs = phx.simulink.BlockBackend.processRefList(block.DialogPrm(2).Data);
    BB.OutputRefs = phx.simulink.BlockBackend.processRefList(block.DialogPrm(3).Data);
    BB.Viewer = block.DialogPrm(4).Data;
    BB.RenderEachStep = block.DialogPrm(5).Data;
    BB.DefaultCameraPosition = block.DialogPrm(6).Data;
    BB.DefaultCameraTarget = block.DialogPrm(7).Data;

    % Sample time
    block.SampleTimes = [block.DialogPrm(8).Data zeros(1, 2-numel(block.DialogPrm(8).Data))];

    % Number of physics substeps per sample time
    BB.Substeps = block.DialogPrm(9).Data;

    % Optional rendered-image output (synthetic camera): [H W], needs the viewer
    BB.CameraResolution = block.DialogPrm(10).Data;
    BB.CameraOn = ~isempty(BB.CameraResolution) && all(BB.CameraResolution > 0) && BB.Viewer;
    if BB.CameraOn
        block.AllowSignalsWithMoreThan2D = true;   % the camera image is an [H W 3] signal
    end

    % Resolve references against the model interface: validates object and
    % property names and sizes bracket-less references. Engine-free; based on
    % class metadata. Falls back to explicit indices if introspection fails.
    try
        iface = phx.simulink.BlockBackend.getModelInterface(BB.Source);
    catch ME
        iface = [];
        warning("phx:PhxModel:interfaceUnavailable", ...
            "Could not introspect model ""%s"" (%s). References must use explicit indices.", ...
            BB.Source, ME.message);
    end
    if ~isempty(iface)
        BB.InputRefs = phx.simulink.BlockBackend.resolveRefs(BB.InputRefs, iface, "input");
        BB.OutputRefs = phx.simulink.BlockBackend.resolveRefs(BB.OutputRefs, iface, "output");
    end

    % Setup input ports
    block.NumInputPorts = numel(BB.InputRefs);
    for i = 1:block.NumInputPorts
        phx.simulink.BlockBackend.sfInput(block, i, 'double', 'Real', 'Sample', 'Fixed', BB.InputRefs(i).Size);
    end

    % Setup output ports (property references + optional camera image as the last port)
    nOut = numel(BB.OutputRefs);
    block.NumOutputPorts = nOut + BB.CameraOn;
    for i = 1:nOut
        phx.simulink.BlockBackend.sfOutput(block, i, 'double', 'Real', 'Sample', 'Fixed', BB.OutputRefs(i).Size);
    end
    if BB.CameraOn
        BB.CameraPort = nOut + 1;
        p = block.OutputPort(BB.CameraPort);
        p.DatatypeID = 3;                 % uint8
        p.Complexity = 'Real';
        p.SamplingMode = 'Sample';
        p.DimensionsMode = 'Fixed';
        p.Dimensions = [BB.CameraResolution(1) BB.CameraResolution(2) 3];
    end

    % Register the parameters
    block.NumDialogPrms = 10;
    block.DialogPrmsTunable = repmat({'Nontunable'}, 1, block.NumDialogPrms);

    % Options
    block.SetAccelRunOnTLC(false);
    block.SetSimViewingDevice(true);
    block.OperatingPointCompliance = 'UseEmpty';

    % Register methods called at run-time
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
end

function Start(block)
    BB = get_param(block.BlockHandle, 'UserData');

    % % Load model
    % [~, file, ext] = fileparts(BB.Source);
    % switch lower(ext)
    %     case ".m"
    %         bodies = feval(file);
    %     case ".mat"
    %         srcData = load(BB.Source);
    %         bodies = srcData.bodies;
    %     otherwise
    %         error("No model available.");
    % end

    % Setup viewer
    if BB.Viewer
        if isempty(BB.hF) || ~isvalid(BB.hF)
            BB.hF = uifigure;
        else
            clf(BB.hF);
        end
        BB.hA = axes(BB.hF);
        phx.extra.Viewer(BB.hA, "DefaultCameraPosition", BB.DefaultCameraPosition, "DefaultCameraTarget", BB.DefaultCameraTarget, "ViewMode", "plain", "ArrowsEnable", false, "WASDEnable", false);
    else
        delete(BB.hF);
        BB.hF = [];
        BB.hA = [];
    end

    % Create simulation object
    %BB.Sim = phx.Simulation([]);
    BB.Sim = phx.Simulation([], "EngineSettings", phx.engine.BulletSettings("AutoActivated", false, "SubstepLimit", 100));
    BB.Sim.addObjects(BB.Source);
    BB.Sim.propagate("ParentAxes", BB.hA);

    % Assign objects for input ports
    for i = 1:block.NumInputPorts
        object = BB.Sim.findBy("Name", BB.InputRefs(i).ObjectName);
        if ~isempty(object)
            BB.InputRefs(i).Object = object{1};
            if isequal(BB.InputRefs(i).Indices, 1:numel(object{1}.(BB.InputRefs(i).Property)))
                BB.InputRefs(i).Indices = []; % optimization
            end
        else
            error("phx:PhxModel:inputObjectNotFound", "Object ""%s"" not found.", BB.InputRefs(i).ObjectName)
        end
    end

    % Assign objects for output ports (property references; camera port has none)
    for i = 1:numel(BB.OutputRefs)
        object = BB.Sim.findBy("Name", BB.OutputRefs(i).ObjectName);
        if ~isempty(object)
            BB.OutputRefs(i).Object = object{1};
            if isequal(BB.OutputRefs(i).Indices, 1:numel(object{1}.(BB.OutputRefs(i).Property)))
                BB.OutputRefs(i).Indices = []; % optimization
            end
        else
            error("phx:PhxModel:outputObjectNotFound", "Object ""%s"" not found.", BB.OutputRefs(i).ObjectName)
        end
    end
end

function Outputs(block)
    % BB = get_param(block.BlockHandle, 'UserData');

    persistent BB

    if block.CurrentTime == 0
        BB = get_param(block.BlockHandle, 'UserData');
    end

    % Simulation step
    dt = block.SampleTimes(1);
    if BB.Viewer
        % redrawStep: RenderEachStep true -> draw every substep, false -> once per sample time
        BB.Sim.step(dt, BB.Substeps, BB.RenderEachStep);
    else
        BB.Sim.step(dt, BB.Substeps, -1);
    end

    % Set input data
    for i = 1:block.NumInputPorts
        ref = BB.InputRefs(i);
        if isempty(ref.Indices)
            ref.Object.(ref.Property) = block.InputPort(i).Data;
        else
            ref.Object.(ref.Property)(ref.Indices) = block.InputPort(i).Data;
        end
    end

    % Get output data (property references)
    for i = 1:numel(BB.OutputRefs)
        ref = BB.OutputRefs(i);
        if isempty(ref.Indices)
            block.OutputPort(i).Data = ref.Object.(ref.Property);
        else
            block.OutputPort(i).Data = ref.Object.(ref.Property)(ref.Indices);
        end
    end

    % Rendered-image output (synthetic camera): capture the viewer after the
    % step's redraw and resize to the declared resolution.
    if BB.CameraOn
        frame = getframe(BB.hA);
        block.OutputPort(BB.CameraPort).Data = imresize(frame.cdata, BB.CameraResolution);
    end
end

function Terminate(block)
    BB = get_param(block.BlockHandle, 'UserData');
    delete(BB.Sim);
end


% --- SUPPORT FUNCTIONS ---

function closeFcn(block)
    BB = get_param(block, "UserData");
    if ~isempty(BB)
        delete(BB.hF);
    end
end