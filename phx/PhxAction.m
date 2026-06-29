function PhxAction(block)
%PhxAction Simulink block: run user MATLAB code against a running PHX scene.
%
%   On a rising edge of the trigger (input port 1) the block executes the user
%   Code against the scene of a bound PhxModel block, enabling structural
%   changes (add/remove bodies) and occasional method calls during simulation.
%
%   Dialog parameters: Code (char), NumInputs, OutputSizes (cell of output
%   dimensions, e.g. {1, 3, [2 4]}), MainBlock (SID).
%   Ports: input 1 = trigger; inputs 2..1+NumInputs = input1..inputN (data,
%          inherited size); outputs 1..numel(OutputSizes) = output1..outputM
%          sized per OutputSizes.
%   The Code runs with these variables in scope: sim (phx.Simulation), ax
%   (viewer axes or []), t (time), input1..inputN, state (persistent struct,
%   read/write), and must assign output1..outputN.
%
%   The scene block is found automatically (first block with MaskType
%   "PHX Physics Simulation"); resolution is lazy (first step).
%
%   See also PhxModel, phx.simulink.ActionBackend

%   Copyright 2026 HUMUSOFT s.r.o.
%   ^..^

    setup(block);
end

function setup(block)
    block.NumDialogPrms = 4;                  % Code, NumInputs, OutputSizes, MainBlock(SID)
    nIn  = block.DialogPrm(2).Data;
    outSizes = block.DialogPrm(3).Data;       % cell of output dimensions, e.g. {1, 3, [2 4]}
    nOut = numel(outSizes);

    block.NumInputPorts = 1 + nIn;            % port 1 = trigger; data inputs inherit their size
    for i = 1:block.NumInputPorts
        block.InputPort(i).DimensionsMode = 'Inherited';
        block.InputPort(i).DatatypeID = 0;    % double
        block.InputPort(i).Complexity = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
    end

    if any(cellfun(@(d) numel(d) > 2, outSizes))
        block.AllowSignalsWithMoreThan2D = true;
    end
    block.NumOutputPorts = nOut;
    for i = 1:nOut
        block.OutputPort(i).Dimensions = outSizes{i};   % scalar N, vector, or [m n] matrix
        block.OutputPort(i).DatatypeID = 0;
        block.OutputPort(i).Complexity = 'Real';
    end

    block.SampleTimes = [-1 0];               % inherited
    block.DialogPrmsTunable = repmat({'Nontunable'}, 1, 4);
    block.SetAccelRunOnTLC(false);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
    block.RegBlockMethod('SetInputPortDimensions', @SetInputPortDimensions);
    block.RegBlockMethod('SetInputPortDimensionsMode', @SetInputPortDimensionsMode);
end

% Accept whatever dimensions/mode the connected signal propagates (inherited inputs)
function SetInputPortDimensions(block, idx, di)
    block.InputPort(idx).Dimensions = di;
end

function SetInputPortDimensionsMode(block, idx, mode)
    block.InputPort(idx).DimensionsMode = mode;
end

function Start(block)
    AB = phx.simulink.ActionBackend;
    AB.Code = block.DialogPrm(1).Data;
    AB.MainSID = block.DialogPrm(4).Data;
    outSizes = block.DialogPrm(3).Data;
    AB.NumOut = numel(outSizes);
    AB.Out = cell(1, AB.NumOut);
    for k = 1:AB.NumOut
        d = outSizes{k};
        if isscalar(d), d = [d 1]; end       % held value matches the port size
        AB.Out{k} = zeros(d);
    end
    set_param(block.BlockHandle, 'UserData', AB);
end

function Outputs(block)
    persistent AB
    if block.CurrentTime == 0
        AB = get_param(block.BlockHandle, 'UserData');
    end

    % Lazy bind to the scene-defining PhxModel block (all Starts have run by now)
    if ~AB.Resolved
        if ~isempty(AB.MainSID)
            sid = AB.MainSID;
            if ~contains(sid, ':')                       % bare SID -> prepend current model
                sid = [get_param(bdroot(block.BlockHandle), 'Name') ':' sid];
            end
            target = Simulink.ID.getHandle(sid);
        else                                              % auto-bind to the single PhxModel block
            blks = find_system(bdroot(block.BlockHandle), 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MaskType', 'PHX Physics Simulation');
            if isempty(blks)
                error('phx:PhxAction:noMainBlock', 'No PHX (PhxModel) block found to bind to.');
            end
            target = blks(1);   % find_system on a handle root returns handles
        end
        bb = get_param(target, 'UserData');
        AB.Sim = bb.Sim;
        AB.Ax = bb.hA;
        AB.Resolved = true;
    end

    % Run the user action on a rising edge of the trigger
    trig = block.InputPort(1).Data;
    if trig > 0 && AB.PrevTrig <= 0
        ins = cell(1, block.NumInputPorts - 1);
        for k = 1:numel(ins)
            ins{k} = block.InputPort(1 + k).Data;
        end
        [AB.Out, AB.State] = runAction(AB.Code, AB.Sim, AB.Ax, block.CurrentTime, ins, AB.NumOut, AB.State);
    end
    AB.PrevTrig = trig;

    for k = 1:block.NumOutputPorts
        block.OutputPort(k).Data = AB.Out{k};
    end
end

function Terminate(~)
end

% Execute the user code with the agreed variables in scope.
function [out, state] = runAction(code, sim, ax, t, ins, nOut, state) %#ok<INUSL>
    for k = 1:numel(ins)
        eval(sprintf('input%d = ins{%d};', k, k)); %#ok<EVLDOT>
    end
    eval(code);                                  % user code: uses sim/ax/t/input*/state, sets output*
    out = cell(1, nOut);
    for k = 1:nOut
        vn = sprintf('output%d', k);
        if exist(vn, 'var')
            out{k} = eval(vn);
        else
            out{k} = 0;
        end
    end
end
