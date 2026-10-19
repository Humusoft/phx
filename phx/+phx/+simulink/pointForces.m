function [AF, AM] = pointForces(block, Points, Origin, TorqueInput)
%phx.simulink.pointForces Backend of the PHX Point Forces block
%
%   [AF, AM] = phx.simulink.pointForces(block, Points, Origin, TorqueInput)
%   is called by the mask initialization of a PHX Point Forces block. It
%   gives the 3-by-3N matrices that turn the stacked point forces
%   u = [F1; F2; ...; FN] into the total force F = AF*u and the moment
%   M = AM*u about Origin, and it sets up the block's ports: one force input
%   per row of Points, plus a pure torque input when TorqueInput is on.
%
%   Points is an N-by-3 matrix of the points where the forces act, Origin
%   the 1-by-3 reference point of the moment; the forces, the points and
%   the results are all expressed in one frame, typically a body's own.
%
%   See also phx.simulink.BlockBackend

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

    arguments
        block
        Points (:, 3) double {mustBeFinite, mustBeNonempty}
        Origin (1, 3) double {mustBeFinite}
        TorqueInput (1, 1) logical
    end

    % M = sum((Pk - Origin) x Fk), with r x F = skew(r)*F
    N = size(Points, 1);
    AF = repmat(eye(3), 1, N);
    AM = zeros(3, 3*N);
    for k = 1:N
        r = Points(k, :) - Origin;
        AM(:, 3*k-2:3*k) = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
    end

    configurePorts(block, N, TorqueInput);
end

function configurePorts(block, N, TorqueInput)
% Changes the contents only when the port set changes, so that an
% unchanged block does not become dirty on every update. The input ports
% that stay are kept, so the lines connected to them outside stay too.

    sys = getfullname(block);
    want = "F" + (1:N);
    if TorqueInput
        want(end + 1) = "T";
    end
    look = {'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on'};
    inports = find_system(sys, look{:}, 'BlockType', 'Inport');
    have = string(get_param(inports, 'Name'))';
    if isequal(have, want)
        return
    end

    % Rebuild the inside: everything but the ports goes, and so do the
    % input ports that are no longer wanted
    delete_line(find_system(sys, look{:}, 'FindAll', 'on', 'Type', 'line'));
    blocks = find_system(sys, look{:}, 'Type', 'block');
    blocks = blocks(~strcmp(blocks, sys));
    names = string(get_param(blocks, 'Name'));
    types = string(get_param(blocks, 'BlockType'));
    drop = ~(types == "Outport" | (types == "Inport" & ismember(names, want)));
    delete_block(blocks(drop));
    if ~any(types == "Outport")
        add_block('built-in/Outport', [sys '/F'], 'Position', [520 33 550 47]);
        add_block('built-in/Outport', [sys '/M'], 'Position', [520 113 550 127]);
    end

    % Force inputs -> Mux -> two matrix gains; the ports are numbered in
    % order, F1..FN then T
    for k = 1:numel(want)
        y = 30 + 40*(k - 1);
        port = [sys '/' char(want(k))];
        if ~ismember(want(k), have)
            add_block('built-in/Inport', port, 'Position', [30 y 60 y + 14]);
        end
    end
    for k = 1:numel(want)
        set_param([sys '/' char(want(k))], 'Port', num2str(k));
    end
    add_block('built-in/Mux', [sys '/Stack'], 'Inputs', num2str(N), 'DisplayOption', 'bar', ...
        'Position', [140 20 145 max(60, 40*N)]);
    add_block('built-in/Gain', [sys '/Total Force'], 'Gain', 'AF', 'Multiplication', 'Matrix(K*u)', ...
        'Position', [240 25 300 55]);
    add_block('built-in/Gain', [sys '/Moment'], 'Gain', 'AM', 'Multiplication', 'Matrix(K*u)', ...
        'Position', [240 105 300 135]);
    for k = 1:N
        add_line(sys, sprintf('F%d/1', k), sprintf('Stack/%d', k), 'autorouting', 'smart');
    end
    add_line(sys, 'Stack/1', 'Total Force/1', 'autorouting', 'smart');
    add_line(sys, 'Stack/1', 'Moment/1', 'autorouting', 'smart');
    add_line(sys, 'Total Force/1', 'F/1', 'autorouting', 'smart');

    if TorqueInput
        add_block('built-in/Sum', [sys '/Add Torque'], 'Inputs', '++', 'Position', [400 110 420 140]);
        add_line(sys, 'Moment/1', 'Add Torque/1', 'autorouting', 'smart');
        add_line(sys, 'T/1', 'Add Torque/2', 'autorouting', 'smart');
        add_line(sys, 'Add Torque/1', 'M/1', 'autorouting', 'smart');
    else
        add_line(sys, 'Moment/1', 'M/1', 'autorouting', 'smart');
    end
end
