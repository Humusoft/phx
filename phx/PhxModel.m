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

    % Setup input ports. The inputs are consumed in Update, together with the
    % step, and never in Outputs, so the block is not a direct feedthrough: an output at
    % time t never depends on an input at time t. Declaring that keeps a plain
    % feedback wiring (PHX -> controller -> PHX) from raising an algebraic loop.
    block.NumInputPorts = numel(BB.InputRefs);
    for i = 1:block.NumInputPorts
        phx.simulink.BlockBackend.sfInput(block, i, 'double', 'Real', 'Sample', 'Fixed', BB.InputRefs(i).Size);
        block.InputPort(i).DirectFeedthrough = false;
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

    % Register methods called at run-time. The backend is captured in the
    % closures instead of being looked up per call: setup runs once per block
    % instance, so each instance carries its own BB and no run-time method ever
    % touches get_param (5.3 us per call in a running model, most of it already
    % in the block.BlockHandle property access).
    block.RegBlockMethod('Start', @(b) Start(b, BB));
    block.RegBlockMethod('Outputs', @(b) Outputs(b, BB));
    block.RegBlockMethod('Update', @(b) Update(b, BB));
    block.RegBlockMethod('Terminate', @(b) Terminate(b, BB));
end

function Start(block, BB)
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
        phx.extra.Viewer(BB.hA, "DefaultCameraPosition", BB.DefaultCameraPosition, "DefaultCameraTarget", BB.DefaultCameraTarget, "ViewMode", "plain");
    else
        delete(BB.hF);
        BB.hF = [];
        BB.hA = [];
    end

    % Create simulation object
    %BB.Sim = phx.Simulation([]);
    BB.Sim = phx.Simulation([]);
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
            BB.InputRefs(i) = BB.InputRefs(i).bindGetter();
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
            BB.OutputRefs(i) = BB.OutputRefs(i).bindGetter();
        else
            error("phx:PhxModel:outputObjectNotFound", "Object ""%s"" not found.", BB.OutputRefs(i).ObjectName)
        end
    end
end

function Outputs(block, BB)
    % Read the scene; the step itself runs in Update. Outputs must stay a pure
    % function of the scene state, because Simulink is free to call it more than
    % once per major time step (an algebraic loop elsewhere, a rejected solver
    % step) and the physics must not be advanced twice for one sample time.
    % The port values are therefore the state at the current time.

    % Get output data (property references). Reading through ref.Getter and
    % indexing afterwards beats the dynamic ref.Object.(ref.Property) access.
    for i = 1:numel(BB.OutputRefs)
        ref = BB.OutputRefs(i);
        if isempty(ref.Indices)
            block.OutputPort(i).Data = ref.Getter(ref.Object);
        else
            value = ref.Getter(ref.Object);
            block.OutputPort(i).Data = value(ref.Indices);
        end
    end

    % Rendered-image output (synthetic camera): capture the viewer as redrawn by
    % the preceding step and resize to the declared resolution.
    if BB.CameraOn
        frame = getframe(BB.hA);
        block.OutputPort(BB.CameraPort).Data = imresize(frame.cdata, BB.CameraResolution);
    end
end

function Update(block, BB)
    % Advance the scene by one sample time. Update is called exactly once per
    % major time step, which is what makes it the right place for a state
    % change; Outputs only reads. The inputs are written first, so the values
    % arriving at time t drive the step from t to t+dt.

    % Set input data. A partial write has to read the property first; doing that
    % read through ref.Getter and writing the whole value back is cheaper than
    % letting ref.Object.(ref.Property)(ref.Indices) = ... do it dynamically.
    for i = 1:block.NumInputPorts
        ref = BB.InputRefs(i);
        if isempty(ref.Indices)
            ref.Object.(ref.Property) = block.InputPort(i).Data;
        else
            value = ref.Getter(ref.Object);
            value(ref.Indices) = block.InputPort(i).Data;
            ref.Object.(ref.Property) = value;
        end
    end

    % Simulation step
    dt = block.SampleTimes(1);
    if BB.Viewer
        % redrawStep: RenderEachStep true -> draw every substep, false -> once per sample time
        BB.Sim.step(dt, BB.Substeps, BB.RenderEachStep);
    else
        BB.Sim.step(dt, BB.Substeps, -1);
    end
end

function Terminate(~, BB)
    delete(BB.Sim);
end


% --- SUPPORT FUNCTIONS ---

function closeFcn(block)
    BB = get_param(block, "UserData");
    if ~isempty(BB)
        delete(BB.hF);
    end
end