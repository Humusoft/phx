classdef tPhxSimulinkRefs < matlab.unittest.TestCase
%tPhxSimulinkRefs Unit tests for the Simulink port-reference machinery.
%
%   Exercises phx.simulink.ParameterReference parsing and the static helpers
%   phx.simulink.BlockBackend.propertyWidth / resolveRefs. Engine-free and
%   graphics-free: property widths come from class metadata and reference
%   resolution runs against a fabricated model interface, so no scene is
%   built and no MEX is needed.
%
%   See also phx.simulink.ParameterReference, phx.simulink.BlockBackend

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        % --- ParameterReference parsing ---
        function parseWithIndices(tc)
            r = phx.simulink.ParameterReference("Chassis.Position(1:3)");
            tc.verifyEqual(r.ObjectName, "Chassis");
            tc.verifyEqual(r.Property, "Position");
            tc.verifyEqual(r.Indices, [1 2 3]);
            tc.verifyEqual(r.Size, 3);
            tc.verifyEqual(r.Kind, "property");
        end

        function parseScalarIndex(tc)
            r = phx.simulink.ParameterReference("StandFL.Position(3)");
            tc.verifyEqual(r.Indices, 3);
            tc.verifyEqual(r.Size, 1);
        end

        function parseBracketless(tc)
            % Whole-property form leaves Indices empty (sized later).
            r = phx.simulink.ParameterReference("Chassis.Position");
            tc.verifyEqual(r.ObjectName, "Chassis");
            tc.verifyEqual(r.Property, "Position");
            tc.verifyEmpty(r.Indices);
            tc.verifyEqual(r.Size, 0);
            tc.verifyEqual(r.Kind, "property");
        end

        % --- propertyWidth (class metadata only) ---
        function widthFromFixedSize(tc)
            tc.verifyEqual(phx.simulink.BlockBackend.propertyWidth("phx.Body", "Position"), 3);
            tc.verifyEqual(phx.simulink.BlockBackend.propertyWidth("phx.Body", "Quaternion"), 4);
            tc.verifyEqual(phx.simulink.BlockBackend.propertyWidth("phx.Body", "Transform"), 16);
            tc.verifyEqual(phx.simulink.BlockBackend.propertyWidth("phx.Body", "Energy"), 1);
        end

        function widthFromDefaultValue(tc)
            % LinearVelocity has no size validation but a [0 0 0] default.
            tc.verifyEqual(phx.simulink.BlockBackend.propertyWidth("phx.Body", "LinearVelocity"), 3);
        end

        function widthUnknownForNonSignal(tc)
            % Type is a string enum: not statically sizeable -> NaN.
            tc.verifyTrue(isnan(phx.simulink.BlockBackend.propertyWidth("phx.Body", "Type")));
        end

        % --- resolveRefs against a fabricated interface ---
        function resolveExpandsBracketless(tc)
            iface = tPhxSimulinkRefs.fakeIface();
            r = phx.simulink.ParameterReference("Chassis.Position");
            r = phx.simulink.BlockBackend.resolveRefs(r, iface, "output");
            tc.verifyEqual(r.Indices, [1 2 3]);
            tc.verifyEqual(r.Size, 3);
        end

        function resolveKeepsExplicitIndices(tc)
            iface = tPhxSimulinkRefs.fakeIface();
            r = phx.simulink.ParameterReference("Chassis.Position(2)");
            r = phx.simulink.BlockBackend.resolveRefs(r, iface, "input");
            tc.verifyEqual(r.Indices, 2);
            tc.verifyEqual(r.Size, 1);
        end

        function resolveUnknownObjectErrors(tc)
            iface = tPhxSimulinkRefs.fakeIface();
            r = phx.simulink.ParameterReference("Nope.Position");
            tc.verifyError(@() phx.simulink.BlockBackend.resolveRefs(r, iface), ...
                "phx:PhxModel:objectNotFound");
        end

        function resolveUnknownPropertyErrors(tc)
            iface = tPhxSimulinkRefs.fakeIface();
            r = phx.simulink.ParameterReference("Chassis.Nope");
            tc.verifyError(@() phx.simulink.BlockBackend.resolveRefs(r, iface), ...
                "phx:PhxModel:propertyNotFound");
        end

        function resolveMethodPortRejected(tc)
            % Reserved seam: methods are recognised but not yet supported.
            iface = tPhxSimulinkRefs.fakeIface();
            r = phx.simulink.ParameterReference("Chassis.applyTorque");
            tc.verifyError(@() phx.simulink.BlockBackend.resolveRefs(r, iface), ...
                "phx:PhxModel:methodPortsUnsupported");
        end

        function resolveReadOnlyInputRejected(tc)
            % An input port writes its property every step, so a get-only one
            % must be refused up front rather than failing mid-simulation.
            iface = tPhxSimulinkRefs.fakeIface();
            for prop = ["TotalForce", "Energy"]
                r = phx.simulink.ParameterReference("Chassis." + prop);
                tc.verifyError(@() phx.simulink.BlockBackend.resolveRefs(r, iface, "input"), ...
                    "phx:PhxModel:propertyReadOnly", prop);
            end
        end

        function resolveReadOnlyOutputAccepted(tc)
            % The same properties are perfectly valid as outputs.
            iface = tPhxSimulinkRefs.fakeIface();
            r = phx.simulink.ParameterReference("Chassis.TotalForce");
            r = phx.simulink.BlockBackend.resolveRefs(r, iface, "output");
            tc.verifyEqual(r.Size, 3);
        end

        function resolveSettableInputAccepted(tc)
            % Guard against over-eager rejection: these are the port kinds the
            % shipped co-simulation models drive.
            iface = tPhxSimulinkRefs.fakeIface();
            for prop = ["Position", "Quaternion", "LinearVelocity"]
                r = phx.simulink.ParameterReference("Chassis." + prop);
                tc.verifyWarningFree(@() phx.simulink.BlockBackend.resolveRefs(r, iface, "input"), prop);
            end
        end

        function propertyIsSettableClassifies(tc)
            isSettable = @(p) phx.simulink.BlockBackend.propertyIsSettable("phx.Body", p);
            tc.verifyTrue(isSettable("Position"));
            tc.verifyTrue(isSettable("Mass"));
            tc.verifyFalse(isSettable("TotalForce"));
            tc.verifyFalse(isSettable("Energy"));
            % Unknown property / class must fail open, never block a reference.
            tc.verifyTrue(isSettable("NoSuchProperty"));
            tc.verifyTrue(phx.simulink.BlockBackend.propertyIsSettable("no.such.Class", "Position"));
        end

        % --- signalProperties (port candidate listing) ---
        function signalListsAllSizeable(tc)
            p = phx.simulink.BlockBackend.signalProperties("phx.Body");
            names = [p.Name];
            % both settable and read-only sizeable properties are listed
            tc.verifyTrue(all(ismember(["Position", "LinearVelocity", "Mass", "TotalForce", "Energy"], names)));
            pos = p(names == "Position");
            tc.verifyEqual(pos.Width, 3);
        end

        function signalReadOnlyFlag(tc)
            p = phx.simulink.BlockBackend.signalProperties("phx.Body");
            names = [p.Name];
            % settable kinematic/general props are writable ...
            tc.verifyFalse(p(names == "Position").ReadOnly);
            tc.verifyFalse(p(names == "LinearVelocity").ReadOnly);
            tc.verifyFalse(p(names == "Mass").ReadOnly);
            % ... get-only dependent properties are read-only.
            tc.verifyTrue(p(names == "TotalForce").ReadOnly);
            tc.verifyTrue(p(names == "Energy").ReadOnly);
        end

        % --- validateRefs edit-time graceful degradation ---
        function validateSkipsEmptySource(tc)
            % No source yet: must not error.
            tc.verifyWarningFree(@() phx.simulink.BlockBackend.validateRefs("", "A.Position", "input"));
        end

        function validateSkipsNonMatSource(tc)
            % .m sources are not validated at edit time (would run builder).
            tc.verifyWarningFree(@() phx.simulink.BlockBackend.validateRefs("model_stand.m", "A.Position", "input"));
        end

        function validateSkipsUnloadableMat(tc)
            % Missing/unresolvable MAT: defer to setup, do not block editing.
            tc.verifyWarningFree(@() phx.simulink.BlockBackend.validateRefs("does_not_exist_xyz.mat", "A.Position", "input"));
        end
    end

    methods (Static)
        function iface = fakeIface()
            iface = struct( ...
                'Name', "Chassis", ...
                'Class', "phx.Body", ...
                'Properties', ["Position", "Quaternion", "LinearVelocity", "Name", "TotalForce", "Energy"], ...
                'Methods', ["applyTorque", "applyForce"]);
        end
    end

end
