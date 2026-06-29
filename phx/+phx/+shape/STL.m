classdef STL < phx.base.Shape & phx.base.ShapeMesh
%phx.shape.STL STL imported shape
%
%   Visual appearance is loaded from an STL file specified by the Source
%   property. The mesh can be decimated (Details), scaled (Scale) and
%   recentred to its bounding box origin (Centered).
%
%   The collision shape is selected by the Envelope property and can be a
%   bounding box, cylinder, sphere, convex hull or concave triangle mesh
%   (concavef uses the flipped face winding). Mass properties are computed
%   from the actual mesh volume and Density.
%
%   phx.shape.STL() creates a shape with default parameters.
%
%   phx.shape.STL(Name, Value, ...) creates a shape and sets properties
%   values according to given name-value pairs.
%
%   See also phx.Body, phx.shape.OBJ, phx.shape.Mesh

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        Vertices
        Normals
        Faces
    end

    properties (Access = private)
        BBox
        Center
    end

    properties
        % Mesh details reduction factor
        Details (1, 1) double = 1

        % Scale
        Scale (1, 3) double = [1 1 1]

        % Move the origin to the centre of the shape
        Centered (1, 1) logical = true

        % Volumetric density (kg/m^3)
        Density (1, 1) double = 1000

        % Envelope
        Envelope {mustBeMember(Envelope, ["box", "cylinder", "sphere", "convex", "concave", "concavef"])} = "box"
    end

    properties (Transient)
        % Source file name
        Source (1, 1) string
    end

    methods
        function obj = STL(Options)
            arguments
                Options.?phx.shape.STL
            end

            % Process input arguments
            obj = phx.internal.applyArguments(Options, obj);
            if isnan(obj.Color(1))
                obj.Color = phx.base.ShapeMesh.newColor;
            end
        end

        function obj = set.Source(obj, fileName)
            % Load data
            if isempty(obj.Vertices)
                cFileName = matlab.io.internal.validators.validateFileName(fileName);
                stl = matlab.internal.meshio.stlread(cFileName{1});
                if obj.Details < 1
                    nfv = reducepatch(stl.Faces, stl.Vertices, obj.Details);
                    tri = triangulation(nfv.faces, nfv.vertices);
                    obj.Faces = tri.ConnectivityList;
                    obj.Vertices = tri.Points;
                    obj.Normals = tri.faceNormal;
                else
                    obj.Faces = stl.Faces;
                    obj.Vertices = stl.Vertices;
                    obj.Normals = stl.Normals;
                end
                [pmin, pmax] = bounds(obj.Vertices);
                obj.Center = (pmin + pmax)/2;
                obj.BBox = pmax - pmin;
            end
            obj.Source = fileName;
        end

        function drawTo(obj, target)
            obj.drawSkelet(target, obj.Color);
            
            if obj.Centered
                vertices = (obj.Vertices - obj.Center).*obj.Scale;
            else
                vertices = obj.Vertices.*obj.Scale;
            end

            tri = triangulation(obj.Faces, obj.Vertices);
            if obj.Style == "flat"
                normals = tri.faceNormal;
            else
                normals = tri.vertexNormal;
            end

            primitive = obj.drawMesh(target, vertices, normals, fliplr(obj.Faces), [], []);
            setappdata(primitive, "phxShape", obj);
        end

        function [mass, inertia] = computeMass(obj)
            [mass, I0] = phx.internal.Geometry.meshMass(obj.Vertices.*obj.Scale, obj.Faces, obj.Density);
            inertia = I0([1 5 9]);
        end
    end

    methods (Access = {?phx.base.Shape, ?phx.base.Object})
        function eh = createBody(obj, body, primitive)
            bsize = (obj.BBox.*obj.Scale)/2; % half-extents
            switch obj.Envelope
                case "box"
                    eh = phx.engine.io('add', body.WorldHandle, 'box', body.TypeID, bsize, body.Transform, body.Mass, body.Inertia);
                case "cylinder"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    ax = obj.detectAxis(ph.Vertices);
                    eh = phx.engine.io('add', body.WorldHandle, 'cylinder', body.TypeID, bsize, ax, body.Transform, body.Mass, body.Inertia);
                case "sphere"
                    eh = phx.engine.io('add', body.WorldHandle, 'sphere', body.TypeID, max(bsize), body.Transform, body.Mass, body.Inertia);
                case "convex"
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    eh = phx.engine.io('add', body.WorldHandle, 'convexhull', body.TypeID, vertices(:), size(vertices, 2), false, body.Transform, body.Mass, body.Inertia);
                    %vv = phx.engine.io('get', body.WorldHandle, eh, 'vertices');
                case {"concave", "concavef"}
                    ph = phx.internal.PrimitiveHelper(primitive);
                    vertices = ph.Vertices';
                    switch obj.Envelope
                        case "concave"
                            faces = int32(ph.LinearizedFaces - 1);
                        case "concavef"
                            faces = int32(fliplr(ph.LinearizedFaces) - 1);
                    end
                    sh_id = phx.engine.io('prepare', body.WorldHandle, uint64(0), 'concaveshape', vertices(:), numel(vertices)/3, faces, numel(faces)/3);
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'dynamictrimesh');
                    phx.engine.io('prepare', body.WorldHandle, sh_id, 'validation');
                    eh = phx.engine.io('add', body.WorldHandle, 'rigidbody', body.TypeID, sh_id, body.Transform, body.Mass, body.Inertia);
            end
        end

        function createComponent(obj, body)
        end
    end

    methods (Access = private)
        function ax = detectAxis(obj, vertices)
            v = vertices.^2;
            s = std([v(:, 2) + v(:, 3), v(:, 1) + v(:, 3), v(:, 1) + v(:, 2)]);
            [~, id] = max(s);
            ch = 'xyz';
            ax = ch(id);
        end
    end

end