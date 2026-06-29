classdef Resistance < phx.base.Object
%phx.Resistance Resistance
%
%   Resistance object allows you to apply a resistance force to one or more
%   bodies, depending on the movement of each body relative to the environment.
%
%   phx.Resistance(bodies) creates a Resistance object and assigns it to all given
%   bodies. All assigned bodies share the same environmental parameters.
%
%   phx.Resistance(___, name, value, ...) creates a Resistance object and sets
%   properties values according to given name-value pairs.
%
%   See also phx.Spring

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private)
        ehs
        hL
        XYZData
    end

    properties
        % Linear movement resistance force factors (F = f(1)*v^0 + f(2)*v^1 + ...)
        VelocityFactors (1, :) double = [0]
        
        % Longitudinal velocity vector of the environment
        EnvironmentVelocity (1, 3) double = [0 0 0]

        % Origin of the environment
        EnvironmentOrigin (1, 3) double = [0 0 0]

        % Angular velocity vector of the environment
        EnvironmentTwist (1, 3) double = [0 0 0]

        % Force vector multiplication factor for drawing
        ForceVectorSize (1, 1) double = 0.001

        % Draw resistance vectors as overlay
        Overlay (1, 1) logical = false
    end

    methods
        function obj = Resistance(Parents, Options)
            arguments
                Parents (1, :) {mustBeA(Parents, "phx.Body")}
                Options.?phx.Resistance
            end

            % Set default values
            obj.SimulationOrder = "before";
            obj.RedrawOrder = "after";
            obj.ParentAxes = Parents(1).ParentAxes;

            % Process input arguments
            obj.Parents = addChild(Parents, obj);
            phx.internal.applyArguments(Options, obj);

            % Create graphics objects
            obj.hL = matlab.graphics.primitive.world.LineStrip('Parent', obj.Graphics, 'LineWidth', 0.5, 'ColorBinding', 'object', 'ColorData', uint8([obj.Color*255 255]'), 'Layer', phx.internal.choose({'middle', 'front'}, obj.Overlay + 1));
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            % Prepare constant data
            P = obj.Parents;
            n = numel(P);
            obj.ehs = zeros(1, n, 'uint64');
            for a = 1:n
                obj.ehs(a) = P{a}.ObjectHandle;
            end

            valid = true;
        end

        function destroyObject(obj)
        end
    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};

                P = obj.Parents;
                n = numel(P);
                pos = zeros(3, n);
                for a = 1:n
                    pos(:, a) = P{a}.Matrix(13:15);
                end

                if any(obj.VelocityFactors)
                    v = zeros(3, n);
                    ve = obj.EnvironmentVelocity;
                    we = obj.EnvironmentTwist;
                    oe = obj.EnvironmentOrigin;
                    for a = 1:n
                        pp = P{a}.Position;
                        pv = P{a}.LinearVelocity;
                        v(:, a) = (pv - ve - cross(pp - oe, we))';
                    end
                    pow = permute(0:numel(obj.VelocityFactors)-1, [3 1 2]);
                    fk = permute(obj.VelocityFactors, [3 1 2]);
                    F = -sum(abs(v).^pow.*fk, 3).*sign(v);
                    phx.engine.io('apply', world, obj.ehs, 'centralforces', F, false);
                else
                    F = zeros(3, n);
                end

                % Interleave start- and end-points matrices (for drawing)
                obj.XYZData = reshape([pos; pos + F*obj.ForceVectorSize], size(pos, 1), []);
            end
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                obj.hL.VertexData = single(obj.XYZData);
            end
        end
    end

end