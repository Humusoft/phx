classdef PlanarShadow < phx.base.Object
%phx.PlanarShadow Planar shadow
%
%   PlanarShadow draws the shadow of one or more bodies as a projection of
%   their visual geometry onto a plane. The plane is given by a point
%   (Position) and a normal (Normal), the light either by a direction
%   (LightDirection) or by a point source (LightPosition). The shadow is a
%   flat translucent silhouette which does not take part in the simulation.
%
%   Besides looking better, a shadow makes a scene easier to read: it shows
%   how high a body is above the surface and where it is going to land,
%   which a perspective view alone does not tell.
%
%   The plane can be attached to a body (Anchor). Position and Normal are
%   then taken in the local frame of that body, so the plane follows it as
%   it moves - a shadow cast on a tilting plate or a moving platform. The
%   light stays in world coordinates in either case.
%
%   phx.PlanarShadow(bodies) creates shadows of the given bodies on the
%   horizontal plane z = 0, lit from straight above.
%
%   phx.PlanarShadow(___, name, value, ...) creates shadows and sets
%   property values according to given name-value pairs.
%
%   Limitations:
%   - The shadow is a projection onto one plane, not a rendered shadow.
%     Bodies do not shadow each other, nothing shadows itself and nothing
%     blocks a shadow: it is drawn even where an obstacle stands between
%     the body and the plane. Use one object per plane and per light.
%   - Overlapping translucent triangles add up, so with Alpha below 1 the
%     interior of the silhouette shows edges where the projected front and
%     back faces (or the shadows of two bodies) cross. Alpha = 1 gives a
%     clean silhouette.
%   - Extent clamps the shadow to the given area instead of clipping it, so
%     a shadow reaching past the boundary is squashed against it and one
%     entirely outside collapses to nothing.
%   - The projected geometry is taken from the bodies when the simulation
%     pipeline is built. Replacing the Shape of a body afterwards is not
%     picked up until the pipeline is rebuilt.
%   - A body whose centre passes to the far side of the plane stops casting
%     a shadow, even if part of it is still on the lit side, and a body the
%     plane cuts through has its buried part projected to the wrong side.
%     Neither shows on a floor that bodies rest on rather than sink into.
%   - Every redraw re-projects the bodies that moved and uploads the whole
%     silhouette once, so the cost follows the total mesh size of the cast
%     bodies rather than their number. Shadowing a body with a large mesh
%     is what makes it expensive, and Detail is the way to bring that back
%     down. There is also a fixed cost per object and redraw that no
%     setting removes, so below a few thousand vertices Detail gains
%     little.
%
%   See also phx.Trace, phx.Camera, phx.extra.Viewer, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^

%#ok<*MCSUP> OK to access other properties in setters
%#ok<*INUSD> OK to see the full list of arguments for callbacks

    properties (Access = private, Transient)
        % The single primitive holding the shadows of all cast bodies. One
        % primitive means one vertex upload per redraw instead of one per
        % body, which is what the cost of a shadow is made of.
        hP = gobjects(0)

        % Per body: 3xN local vertices (single), the columns of Buf they
        % own, the centre of their geometry and the transform they were
        % last projected with (empty = never).
        VLocal = {}
        Cols = {}
        Centre = {}
        LastTransform = {}

        % Vertex buffer of the primitive, 3-by-(total vertices)
        Buf = single([])

        % Plane and light of the last update, to notice when they change
        LastSetup = []
    end

    properties
        % Body the shadow plane is attached to, empty for a fixed plane
        Anchor = phx.Body.empty

        % Point lying in the shadow plane, in the local frame of the anchor
        Position (1, 3) double {mustBeFinite} = [0 0 0]

        % Normal of the shadow plane, in the local frame of the anchor
        Normal (1, 3) double {mustBeFinite} = [0 0 1]

        % Direction of the parallel light, must not be parallel to the plane
        LightDirection (1, 3) double {mustBeFinite} = [0 0 -1]

        % Position of a point light source, empty for a parallel light
        LightPosition (1, :) double {mustBeFinite} = []

        % Distance of the shadow above the plane (to avoid z-fighting)
        Offset (1, 1) double = 0.002

        % Opacity of the shadow (0-1). Assigning Color resets it to opaque,
        % so set Alpha after Color, not before.
        Alpha (1, 1) double {mustBeInRange(Alpha, 0, 1)} = 0.4

        % Half-sizes of the shadow area within the plane (measured from
        % Position along the plane axes), Inf for unlimited
        Extent (1, 2) double {mustBePositive} = [Inf Inf]

        % Fraction of the faces kept when the geometry is cached, 1 for the
        % full mesh. A silhouette rarely needs every triangle of a detailed
        % body, and the projection cost follows the vertex count.
        Detail (1, 1) double {mustBeInRange(Detail, 0, 1)} = 1
    end

    methods
        function obj = PlanarShadow(Bodies, Options)
            arguments
                Bodies (1, :) {mustBeA(Bodies, "phx.Body")}
                Options.?phx.PlanarShadow
            end

            % Set default values
            obj.SimulationOrder = "none";
            obj.RedrawOrder = "after";
            obj.Color = [0.1 0.1 0.14];
            obj.ParentAxes = Bodies(1).ParentAxes;

            % Process input arguments
            obj.Parents = addChild(Bodies, obj);
            phx.internal.applyArguments(Options, obj);
        end

        function set.Alpha(obj, value)
            obj.Alpha = value;

            % Push the new opacity into the existing primitives
            for p = obj.hP(isgraphics(obj.hP))
                clr = p.ColorData;
                clr(4) = uint8(value*255);
                p.ColorData = clr;
            end
        end

        function set.Anchor(obj, value)
            if isempty(value)
                obj.Anchor = phx.Body.empty;
            else
                mustBeScalarOrEmpty(value);
                mustBeA(value, "phx.Body");
                obj.Anchor = value;
            end
        end
    end

    methods (Access = protected)
        function valid = initObject(obj, world)
            % Discard what the previous pipeline built
            delete(obj.hP(isgraphics(obj.hP)));
            obj.hP = gobjects(0);
            obj.VLocal = {};
            obj.Cols = {};
            obj.Centre = {};
            obj.LastTransform = {};
            obj.Buf = single([]);
            obj.LastSetup = [];

            % Without a target axes or a live anchor there is nothing to draw
            if ~isvalid(obj.Graphics) || isempty(obj.Graphics.Parent) || (~isempty(obj.Anchor) && ~isvalid(obj.Anchor))
                valid = false;
                return
            end

            % Collect the geometry of every cast body, keeping track of
            % which body each block of vertices belongs to
            vertices = {};
            faces = {};
            owner = [];
            for k = 1:numel(obj.Parents)
                body = obj.Parents{k};
                if ~isvalid(body) || ~isvalid(body.Graphics)
                    continue
                end
                for ch = body.Graphics.Children'
                    ph = phx.internal.PrimitiveHelper(ch);
                    V = ph.Vertices;
                    F = ph.LinearizedFaces;
                    if isempty(V) || isempty(F) || mod(numel(F), 3) ~= 0
                        continue
                    end

                    % Decimate once, here, so that every redraw carries the
                    % reduced vertex count
                    if obj.Detail < 1
                        [f, v] = reducepatch(reshape(F, 3, [])', V, obj.Detail);
                        if ~isempty(f)
                            V = v;
                            F = reshape(f', 1, []);
                        end
                    end

                    vertices{end + 1} = V;      %#ok<AGROW>
                    faces{end + 1} = F;         %#ok<AGROW>
                    owner(end + 1) = k;         %#ok<AGROW>
                end
            end
            if isempty(vertices)
                valid = false;
                return
            end

            % Concatenate everything into one vertex buffer and one index
            % list, with the face indices shifted to their block
            offsets = [0 cumsum(cellfun(@(v) size(v, 1), vertices))];
            indices = zeros(1, sum(cellfun(@numel, faces)), 'uint32');
            at = 0;
            for i = 1:numel(faces)
                indices(at + (1:numel(faces{i}))) = uint32(faces{i} + offsets(i));
                at = at + numel(faces{i});
            end
            obj.Buf = zeros(3, offsets(end), 'single');

            % Group the blocks by body, so each body is one matrix operation
            for k = 1:numel(obj.Parents)
                mine = find(owner == k);
                V = vertcat(vertices{mine});
                cols = arrayfun(@(i) offsets(i)+1:offsets(i+1), mine, "UniformOutput", false);
                obj.Cols{k} = [cols{:}];
                obj.VLocal{k} = single(V');
                [low, high] = bounds(V, 1);
                obj.Centre{k} = (low + high)/2;
                obj.LastTransform{k} = [];
            end

            obj.hP = matlab.graphics.primitive.world.TriangleStrip('Parent', obj.Graphics, 'VertexIndices', indices, 'VertexData', obj.Buf, 'ColorData', uint8([obj.Color obj.Alpha]'*255), 'ColorType', 'truecoloralpha', 'ColorBinding', 'object', 'FaceOffsetFactor', 0, 'FaceOffsetBias', -2e-4, 'PickableParts', 'none', 'HitTest', 'off');

            phx.PlanarShadow.updateView({obj});

            valid = all(cellfun(@isvalid, obj.Parents));
        end

        function destroyObject(obj)
        end
    end

    methods (Access = private)
        function n = unitNormal(obj)
            n = obj.Normal/norm(obj.Normal);
        end

        function B = planeBasis(obj, n)
        % Orthonormal in-plane basis (3x2) for the Extent clamping. The
        % normal may change at run time, so it is derived on every update.
            [~, k] = min(abs(n));
            e = zeros(1, 3);
            e(k) = 1;
            u = cross(n, e);
            u = u/norm(u);
            B = [u' cross(n, u)'];
        end

    end

    methods (Static, Access = protected)
        function resolveState(cellObjs, dt, time, world)
        end

        function updateView(cellObjs, dt, time, world)
            for i = 1:numel(cellObjs)
                obj = cellObjs{i};
                if isempty(obj.hP) || ~isgraphics(obj.hP)
                    continue
                end

                % Plane, given in the frame of the anchor if there is one
                n = obj.unitNormal;
                p = obj.Position;
                clamp = any(isfinite(obj.Extent));
                if clamp
                    B = obj.planeBasis(n);
                end
                if ~isempty(obj.Anchor)
                    A = obj.Anchor.Transform;
                    R = A(1:3, 1:3);
                    n = n*R';
                    p = p*R' + A(1:3, 4)';
                    if clamp
                        B = R*B;
                    end
                end
                p0 = p + n*obj.Offset;

                % A parallel light collapses into one affine map onto the
                % plane, P = V*S + c, which folds into the body transform
                % below. A point light needs the per-vertex divide instead.
                point = ~isempty(obj.LightPosition);
                if point
                    src = obj.LightPosition;
                    reach = (p0 - src)*n';
                    lightSide = -sign(reach);
                else
                    L = obj.LightDirection/norm(obj.LightDirection);
                    den = L*n';
                    if abs(den) < eps
                        error("phx:PlanarShadow:lightParallel", "LightDirection must not be parallel to the shadow plane.");
                    end
                    lightSide = -sign(den);
                    S = eye(3) - (n'*L)/den;
                    c = p0*(n'*L)/den;
                end

                % Anything else than the bodies moving invalidates the
                % whole buffer, so remember what it was computed with
                setup = [n p0 lightSide clamp obj.Extent];
                if point
                    setup = [setup src reach];         %#ok<AGROW>
                else
                    setup = [setup L];                 %#ok<AGROW>
                end
                if clamp
                    setup = [setup B(:)'];             %#ok<AGROW>
                end
                stale = ~isequal(setup, obj.LastSetup);
                obj.LastSetup = setup;

                dirty = false;
                for k = 1:numel(obj.Parents)
                    % Bodies that have not moved keep the vertices they
                    % already have in the buffer
                    M = obj.Parents{k}.Transform;
                    if ~stale && isequal(M, obj.LastTransform{k})
                        continue
                    end
                    obj.LastTransform{k} = M;
                    dirty = true;

                    Rk = M(1:3, 1:3);
                    tk = M(1:3, 4)';

                    % A body on the far side of the plane casts nothing:
                    % collapse its block to a point (degenerate triangles).
                    % Measured against the plane itself, not against the
                    % offset one, so that Offset cannot hide a body resting
                    % on the surface.
                    if (obj.Centre{k}*Rk' + tk - p)*n'*lightSide <= 0
                        obj.Buf(:, obj.Cols{k}) = repmat(single(p0'), 1, numel(obj.Cols{k}));
                        continue
                    end

                    if point
                        W = Rk*obj.VLocal{k} + tk';         % 3xN world
                        D = W - src';
                        P = src' + D.*(reach./(n*D));
                    else
                        P = (Rk'*S)'*obj.VLocal{k} + (tk*S + c)';
                    end

                    % Optional clamping to a finite shadow area
                    if clamp
                        uv = B'*(P - p0');
                        P = P - B*(uv - max(min(uv, obj.Extent'), -obj.Extent'));
                    end

                    obj.Buf(:, obj.Cols{k}) = P;
                end

                % One upload for the whole scene
                if dirty
                    obj.hP.VertexData = obj.Buf;
                end
            end
        end
    end

end
