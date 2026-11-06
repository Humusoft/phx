classdef Revolution < phx.base.Shape
%phx.shape.Revolution Profile revolved about an axis
%
%   Visual appearance is based on a geometry of revolved profile defined by
%   the user.
%
%   Collision shape is selected by the Envelope property: the convex hull of
%   the revolved mesh, the exact concave triangle mesh, a bounding cylinder
%   about the revolution axis, or the profile revolved exactly as a stack of
%   truncated cones, one per profile segment. The cones are round about the
%   axis, so a body rolls on them smoothly where a faceted hull or mesh
%   would not; they need a profile with one radius per height (no bore, no
%   overhang). Mass properties are always calculated from the actual
%   triangular mesh.
%   
%   phx.shape.Revolution() creates a shape with default parameters.
%
%   phx.shape.Revolution(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.Extrusion

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Revolution profile along modeling axis
        % Matrix Mx2 where each row is one point of the generatrix as
        % [axis radius]. The curve is reversed automatically when it runs the
        % way that would turn the solid inside out; it is not closed, because a
        % generatrix ending on the axis already closes the body by revolving.
        Profile (:, 2) double

        % Modeling axis of the revolution
        Axis {mustBeMember(Axis, ["x", "y", "z"])} = "z"

        % Number of revolution segments
        Segments (1, 1) double = 45

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Collision envelope: "convex" hull, "concave" triangle mesh,
        % bounding "cylinder" about the revolution axis, or "cones": the
        % profile revolved exactly, one truncated cone per segment, which
        % needs the axial coordinates of the profile not to turn back
        Envelope {mustBeMember(Envelope, ["convex", "concave", "cylinder", "cones"])} = "convex"
    end

    methods
        function obj = Revolution(Options)
            arguments
                Options.?phx.shape.Revolution
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
            if obj.Envelope == "cones"
                phx.shape.Revolution.conesProfile(obj.Profile);
            end
        end

        function obj = set.Profile(obj, value)
            % The vertex normals and the triangle winding both follow the
            % direction the generatrix runs in - and, unlike the extrusion,
            % they already agree - so a generatrix drawn the other way comes
            % out inside out in the picture and in the collision mesh alike.
            % Close it through the axis and require the enclosed area to be
            % negative; for a generatrix that is already a closed loop the two
            % points added on the axis coincide and contribute nothing, so the
            % one test covers both kinds.
            if size(value, 1) > 1
                g = [value; value(end, 1) 0; value(1, 1) 0];
                if sum(g(1:end - 1, 1).*g(2:end, 2) - g(2:end, 1).*g(1:end - 1, 2)) > 0
                    value = flipud(value);
                end
            end
            obj.Profile = value;
        end

        function drawTo(obj, target)
            [V, N, F, T] = phx.internal.Geometry.revolution((obj.Profile), obj.Segments, false, false);
            [V, N] = phx.internal.Geometry.switchZAxis(obj.Axis, V, N);
            
            primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            [V, ~, F] = phx.internal.Geometry.revolution((obj.Profile), obj.Segments, false, false);
            V = phx.internal.Geometry.switchZAxis(obj.Axis, V);

            [mass, ~, Icm, center] = phx.internal.Geometry.meshMass(V, F, obj.Density);
            inertia = Icm([1 5 9]);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            switch obj.Envelope
                case "cylinder"
                    % Bounding cylinder (centered on the shape origin)
                    ext = max(abs(obj.Profile), [], 1);
                    eh = phx.engine.io('add', body.WorldHandle, 'cylinder', body.TypeID, ext(2), 2*ext(1), char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
                case "cones"
                    % The profile revolved exactly (no graphics needed)
                    profile = phx.shape.Revolution.conesProfile(obj.Profile);
                    eh = phx.engine.io('add', body.WorldHandle, 'revolution', body.TypeID, profile, char(obj.Axis), body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
                case "convex"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = obj.createHullBody(body, vertices);
                case "concave"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = obj.createTrimeshBody(body, vertices, ph.LinearizedFaces);
            end
        end
    end

    methods (Static, Access = private)
        function profile = conesProfile(profile)
            % The profile as the engine's cones take it, [axial; radial] with
            % the axial coordinates not decreasing. Points at either end that
            % run along the axis enclose nothing and go (a closed profile
            % returns that way); a profile that turns back anywhere else
            % cannot be built from cones.
            off = find(profile(:, 2) ~= 0);
            if ~isempty(off)
                profile = profile(max(off(1) - 1, 1):min(off(end) + 1, end), :);
            end
            if size(profile, 1) < 2
                profile = profile';
                return
            end
            if profile(end, 1) < profile(1, 1)
                profile = flipud(profile);
            end
            tolerance = 1e-6*max([abs(profile(:, 1) - profile(1, 1)); profile(:, 2)]);
            if any(diff(profile(:, 1)) < -tolerance)
                error("phx:Revolution:profileNotMonotonic", ...
                    "Envelope ""cones"" needs a profile with one radius per height: its axial " + ...
                    "coordinates must not turn back. Use Envelope ""concave"" or ""convex"" for this profile.");
            end
            profile = profile';
        end
    end

end