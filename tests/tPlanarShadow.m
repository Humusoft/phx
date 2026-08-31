classdef tPlanarShadow < PhxTestCase
%tPlanarShadow Tests for phx.PlanarShadow projected shadows.
%
%   The construction and validation tests only build objects, so they need
%   graphics (a body owns an hgtransform) and no physics engine. The
%   projection tests need the shadow primitives, which are created when the
%   simulation pipeline is built, so they run a real phx.Simulation and
%   carry the "Engine" tag.
%
%   See also phx.PlanarShadow, phx.Trace

%   Copyright 2026 HUMUSOFT s.r.o.

    properties
        Ax
    end

    methods (TestMethodSetup)
        function setup(tc)
            tc.Ax = tc.prepareAxes;
        end
    end

    % --- Construction and property validation ---------------------------
    methods (Test, TestTags = {'Graphics'})
        function defaultsAreTheHorizontalGroundPlane(tc)
            s = phx.PlanarShadow(tc.mkBody([0 0 1]));
            tc.verifyEqual(s.Position, [0 0 0]);
            tc.verifyEqual(s.Normal, [0 0 1]);
            tc.verifyEqual(s.LightDirection, [0 0 -1]);
            tc.verifyEmpty(s.LightPosition);
            tc.verifyEmpty(s.Anchor);
            tc.verifyEqual(s.Extent, [Inf Inf]);
        end

        function shadowIsLinkedToEveryCaster(tc)
            b1 = tc.mkBody([0 0 1]);
            b2 = tc.mkBody([1 0 1]);
            s = phx.PlanarShadow([b1 b2]);
            tc.verifyNumElements(s.Parents, 2);
            tc.verifyTrue(b1.Children{1} == s);
            tc.verifyTrue(b2.Children{1} == s);
        end

        function anchorAcceptsBodyOrEmpty(tc)
            b = tc.mkBody([0 0 1]);
            plate = tc.mkBody([0 0 0], "static");
            s = phx.PlanarShadow(b, "Anchor", plate);
            tc.verifyTrue(s.Anchor == plate);

            s.Anchor = [];
            tc.verifyEmpty(s.Anchor);
            tc.verifyClass(s.Anchor, "phx.Body");
        end

        function invalidPropertiesAreRejected(tc)
            b = tc.mkBody([0 0 1]);
            s = phx.PlanarShadow(b);
            tc.verifyError(@() set(s, "Anchor", 42), "MATLAB:validators:mustBeA");
            tc.verifyError(@() set(s, "Anchor", [b b]), "MATLAB:validators:mustBeScalarOrEmpty");
            tc.verifyError(@() set(s, "Alpha", 1.5), "MATLAB:validators:mustBeInRange");
            tc.verifyError(@() set(s, "Extent", [0 1]), "MATLAB:validators:mustBePositive");
            tc.verifyError(@() phx.PlanarShadow("notABody"), "MATLAB:validators:mustBeA");
        end
    end

    % --- Projection (needs the pipeline, so a real simulation) ----------
    methods (Test, TestTags = {'Engine', 'Graphics'})
        function shadowLandsInThePlaneUnderTheBody(tc)
            b = tc.mkBox([0.4 -0.3 2]);
            s = phx.PlanarShadow(b, "Offset", 0);
            tc.mkSim();

            V = tc.shadowVertices(s);
            tc.verifyEqual(V(:, 3), zeros(size(V, 1), 1), "AbsTol", 1e-5);
            tc.verifyEqual(mean([min(V(:, 1)) max(V(:, 1))]), 0.4, "AbsTol", 1e-5);
            tc.verifyEqual(mean([min(V(:, 2)) max(V(:, 2))]), -0.3, "AbsTol", 1e-5);
        end

        function offsetLiftsTheShadowAboveThePlane(tc)
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Offset", 0.01);
            tc.mkSim();

            V = tc.shadowVertices(s);
            tc.verifyEqual(V(:, 3), 0.01*ones(size(V, 1), 1), "AbsTol", 1e-5);
        end

        function slantedLightShiftsTheShadowByTheHeight(tc)
            % A unit box centred 2 m up, lit at 45 degrees along +x: every
            % vertex slides by its own height, so the silhouette spans
            % [1.5 - 0.5, 2.5 + 0.5] in x.
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Offset", 0, "LightDirection", [1 0 -1]);
            tc.mkSim();

            V = tc.shadowVertices(s);
            tc.verifyEqual(min(V(:, 1)), 1.0, "AbsTol", 1e-4);
            tc.verifyEqual(max(V(:, 1)), 3.0, "AbsTol", 1e-4);
        end

        function pointLightSpreadsTheShadow(tc)
            % A lamp 4 m up over a unit box centred at 2 m. The widest part
            % of the silhouette comes from the top face, 1.5 m below the
            % lamp, magnified by 4/1.5 to a width of 8/3.
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Offset", 0, "LightPosition", [0 0 4]);
            tc.mkSim();

            V = tc.shadowVertices(s);
            tc.verifyEqual(max(V(:, 1)) - min(V(:, 1)), 8/3, "AbsTol", 1e-3);
        end

        function bodyBehindThePlaneCastsNothing(tc)
            b = tc.mkBox([0 0 -3], "static");     % below the shadow plane
            s = phx.PlanarShadow(b, "Offset", 0);
            tc.mkSim();

            % Its block collapses onto the plane point, so it has no area
            V = tc.shadowVertices(s);
            tc.verifyEqual(V, zeros(size(V)), "AbsTol", 1e-5);
        end

        function bodyRestingOnThePlaneStillCasts(tc)
            % Regression: the far-side test has to use the centre of the
            % geometry and to ignore Offset. Against a bounding-box corner,
            % or against the offset plane, a body resting on the surface
            % falls to the far side and its shadow blinks out.
            b = tc.mkBody([0 0 0.3], "static");
            b.Shape = {"Sphere", "Radius", 0.3};
            s = phx.PlanarShadow(b, "Offset", 0.05);
            tc.mkSim();

            V = tc.shadowVertices(s);
            tc.verifyGreaterThan(max(range(V(:, 1:2), 1)), 0.5);
        end

        function anchoredPlaneFollowsTheAnchor(tc)
            % The plane is the top face of a plate tilted about x, so every
            % shadow vertex must satisfy the plane equation of that face.
            plate = tc.mkBody([0 0 0], "static");
            plate.EulerAngles = [0.3 0 0];
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Anchor", plate, "Position", [0 0 0.1], "Offset", 0);
            tc.mkSim();

            R = plate.Orientation;
            n = R(:, 3)';
            p = plate.Position + n*0.1;
            V = tc.shadowVertices(s);
            tc.verifyEqual((V - p)*n', zeros(size(V, 1), 1), "AbsTol", 1e-4);
        end

        function extentKeepsTheShadowInsideTheArea(tc)
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Offset", 0, "Extent", [0.2 0.2]);
            tc.mkSim();

            V = tc.shadowVertices(s);
            tc.verifyLessThanOrEqual(max(abs(V(:, 1:2)), [], "all"), 0.2 + 1e-5);
        end

        function detailDecimatesTheCachedGeometry(tc)
            % A coarser silhouette still lands in the plane under the body
            b = tc.mkBody([0 0 2], "static");
            b.Shape = {"Sphere", "Radius", 0.5, "Division", 4};
            full = phx.PlanarShadow(b, "Offset", 0);
            coarse = phx.PlanarShadow(b, "Offset", 0, "Detail", 0.1);
            tc.mkSim();

            Vf = tc.shadowVertices(full);
            Vc = tc.shadowVertices(coarse);
            tc.verifyLessThan(size(Vc, 1), size(Vf, 1)/4);
            tc.verifyEqual(Vc(:, 3), zeros(size(Vc, 1), 1), "AbsTol", 1e-5);
            tc.verifyEqual(max(abs(Vc(:, 1:2)), [], "all"), 0.5, "AbsTol", 0.05);
        end

        function alphaWritesThroughToThePrimitives(tc)
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Alpha", 0.25);
            tc.mkSim();

            tc.verifyEqual(s.Graphics.Children(1).ColorData(4), uint8(64));
            s.Alpha = 0.8;
            tc.verifyEqual(s.Graphics.Children(1).ColorData(4), uint8(204));
        end

        function deletedAnchorSilencesTheShadow(tc)
            plate = tc.mkBody([0 0 0], "static");
            b = tc.mkBox([0 0 2]);
            s = phx.PlanarShadow(b, "Anchor", plate);
            tc.mkSim();
            tc.verifyNotEmpty(s.Graphics.Children);

            delete(plate);                        % rebuilds the pipeline
            tc.verifyEmpty(s.Graphics.Children);
        end
    end

    methods (Access = private)
        function b = mkBody(tc, pos, type)
            arguments
                tc
                pos (1, 3) double
                type (1, 1) string = "dynamic"
            end
            b = phx.Body(tc.Ax, "Position", pos, "Type", type);
        end

        function b = mkBox(tc, pos, type)
            arguments
                tc
                pos (1, 3) double
                type (1, 1) string = "dynamic"
            end
            b = phx.Body(tc.Ax, "Position", pos, "Type", type, ...
                "Shape", {"Box", "Size", [1 1 1]});
        end

        function sim = mkSim(tc)
            % Building the simulation creates the shadow primitives.
            tc.requireEngine;
            sim = phx.Simulation;
            tc.addTeardown(@() delete(sim));
        end

        function V = shadowVertices(~, s)
            V = phx.internal.PrimitiveHelper(s.Graphics.Children(1)).Vertices;
        end
    end

end
