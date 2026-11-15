classdef Box < phx.base.Shape
%phx.shape.Box Box shape
%
%   Visual appearance is based on a simple triangular mesh.
%
%   Collision shape and mass properties are modeled as an ideal block.
%
%   As the Shape of a phx.FlexibleBody, the box is filled with a mesh of
%   tetrahedra (generateVolumetricMesh); as the Shape of a phx.SoftBody, it
%   is covered with the surface of that mesh (generateSurfaceMesh). The
%   ElementResolution sets how fine they are.
%
%   phx.shape.Box() creates a shape with default parameters.
%
%   phx.shape.Box(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.FlexibleBody, phx.shape.Sphere

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties
        % Box size (X Y Z)
        Size (1, 3) double = [1 1 1]

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Resolution of the meshes of flexible and soft bodies: the
        % number of cells across the smallest side, the other sides cut
        % into cells as close to cubes as possible, or the numbers of cells
        % along X, Y and Z (1x3)
        ElementResolution (1, :) double {mustBeResolution} = 2
    end

    properties (Dependent, SetAccess = private)
        % Numbers of cells along X, Y and Z that ElementResolution gives
        ElementDivisions (1, 3) double
    end

    methods
        function obj = Box(Options)
            arguments
                Options.?phx.shape.Box
            end

            % Defaults
            obj.Style = "flat";

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.Shape.newColor;
            end
        end

        function drawTo(obj, target)
            if obj.ForcePatch
                [V, N, F] = phx.internal.Geometry.quadBox(obj.Size);
                primitive = obj.drawMesh(target, V, N, F, [], []);
            else
                [V, N, F, T] = phx.internal.Geometry.triBox(obj.Size);
                primitive = obj.drawMesh(target, V, N, F, T, obj.getTexture);
            end
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia, center] = computeMass(obj)
            s = obj.Size;
            mass = obj.Density*s(1)*s(2)*s(3);
            q = s.^2;
            inertia = [q(2) + q(3), q(1) + q(3), q(1) + q(2)]*mass/12;
            center = [0 0 0];
        end

        function value = get.ElementDivisions(obj)
            r = obj.ElementResolution;
            if isscalar(r)
                s = obj.Size;
                value = max(1, round(s*r/min(s)));
            else
                value = r;
            end
        end

        function [V, T] = generateVolumetricMesh(obj)
        %generateVolumetricMesh Returns a mesh of tetrahedra that fills the box.
        %
        %   [V, T] = generateVolumetricMesh(box) returns a mesh of
        %   tetrahedra that fills the box: its vertices V (Mx3, in the shape
        %   frame) and its tetrahedra T (Kx4 vertex indices, of positive
        %   volume). The box is cut into a grid of ElementDivisions cells,
        %   which ElementResolution sets; each cell is split into six
        %   tetrahedra along its main diagonal.
        %
        %   See also phx.FlexibleBody, phx.shape.Box.ElementResolution,
        %   phx.base.Shape.generateSurfaceMesh

            % Grid vertices, X running fastest
            s = obj.Size;
            n = obj.ElementDivisions;
            [x, y, z] = ndgrid(linspace(-s(1)/2, s(1)/2, n(1) + 1), ...
                linspace(-s(2)/2, s(2)/2, n(2) + 1), linspace(-s(3)/2, s(3)/2, n(3) + 1));
            V = [x(:) y(:) z(:)];
            T = phx.internal.Geometry.hexTetrahedra(phx.internal.Geometry.gridHexes(n));
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            bsize = obj.Size/2; % half-extents
            eh = phxEngine('add', body.WorldHandle, 'box', body.TypeID, bsize, body.Transform, body.Mass, body.Inertia, body.CenterOfMass);
        end
    end

end

function mustBeResolution(value)
%mustBeResolution Validates ElementResolution: one or three positive whole numbers
    if ~any(numel(value) == [1 3]) || any(value ~= round(value)) || any(value < 1) || any(~isfinite(value))
        error("phx:Box:badResolution", ...
            "ElementResolution must be a number of cells across the smallest side, or the numbers of cells along X, Y and Z (1x3), whole numbers of at least 1.");
    end
end