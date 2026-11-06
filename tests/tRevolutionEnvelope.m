classdef tRevolutionEnvelope < PhxTestCase
%tRevolutionEnvelope The round collision envelopes of a revolution.
%
%   phx.shape.Revolution can wear envelopes that are round about its axis,
%   where a hull is faceted. Envelope "cylinder" fits a cylinder about the
%   revolution axis, sized from the profile (radius = its largest radius,
%   half height = its axial reach). Envelope "cones" revolves the profile
%   exactly, one truncated cone per segment, for a profile with one radius
%   per height. Neither needs a graphics primitive, so these tests run with
%   no axes at all.
%
%   Locked in here:
%     * a revolved rectangle with either envelope moves exactly like the
%       equivalent phx.shape.Cylinder, for every modeling axis,
%     * the cylinder rests at half the axial reach of its profile and is
%       solid: a hole in the profile is not a hole in it,
%     * the cones are round however the body is turned about its axis and
%       follow the profile into a waist,
%     * the cones refuse a profile that turns back.
%
%   See also phx.shape.Revolution, phx.shape.Cylinder

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (Constant)
        Radius = 0.15
        Height = 0.04
        HoleRatio = 0.7
    end

    methods (TestClassSetup)
        function engineIsPresent(tc)
            tc.requireEngine;
        end
    end

    methods (Test, TestTags = {'Engine'})
        function matchesPrimitiveCylinder(tc)
            % A revolved rectangle and a phx.shape.Cylinder of the same size
            % are the same collision shape, so with the same mass, inertia and
            % initial state they must follow the same trajectory. The drop is
            % tilted and spinning, so the rim rather than the flat face makes
            % the contact and a wrong radius or axis cannot pass unnoticed.
            % The closed rectangle runs back along the axis, which the cones
            % drop as enclosing nothing.
            for envelope = ["cylinder", "cones"]
                for axis = ["z", "x", "y"]
                    h = tc.Height;
                    r = tc.Radius;
                    rect = [-h/2 0; h/2 0; h/2 r; -h/2 r; -h/2 0];
                    revolved = tc.tiltedDrop(tc.revolutionBody(rect, axis, envelope), axis);
                    primitive = tc.tiltedDrop(phx.Body([], "Mass", 5, ...
                        "Inertia", [0.03 0.03 0.06], "Shape", {"Cylinder", ...
                        "Radius", r, "Height", h, "Axis", axis}), axis);
                    tc.verifyEqual(revolved, primitive, "AbsTol", 1e-9, ...
                        "Envelope " + envelope + " differs from phx.shape.Cylinder for Axis " + axis + ".");
                end
            end
        end

        function conesAreRoundAboutTheAxis(tc)
            % An hourglass lying on its two ends, turned about its axis by
            % half a facet of its 24-sided drawing, rests at its end radius:
            % the cones are round where a hull of the drawing would hold it
            % lower by 1 - cos(7.5 deg) of the radius (0.9 mm here).
            body = tc.revolutionBody(tc.hourglassProfile, "x", "cones");
            body.Position = [0 0 0.11];
            R = makehgtform("xrotate", deg2rad(7.5));
            body.Orientation = R(1:3, 1:3);
            z = tc.settle(body, phx.Body.empty);
            tc.verifyEqual(z, 0.1, "AbsTol", 1e-4, "The cones are not round about the axis.");
        end

        function conesFollowTheWaist(tc)
            % A ball dropped onto the waist of a static hourglass lying
            % along x rests in it, at the waist radius plus its own; a hull
            % would hold it at the end radius.
            hourglass = phx.Body([], "Type", "static", "Shape", ...
                phx.shape.Revolution("Profile", tc.hourglassProfile, "Axis", "x", ...
                "Envelope", "cones", "Segments", 24));
            ball = phx.Body([], "Position", [0.105 0 0.2], "Shape", {"Sphere", "Radius", 0.02});
            sim = tc.simulation([hourglass ball]);
            sim.step(1, 1000);
            tc.verifyEqual(ball.Position, [0.1 0 0.03 + 0.02*sqrt(1 + 0.7^2)], "AbsTol", 1e-3, ...
                "The ball does not rest in the waist.");
            delete(sim);
        end

        function conesNeedOneRadiusPerHeight(tc)
            % A cup returns along the axis at its rim, so its profile has
            % two radii at some heights: the cones refuse it, whether the
            % envelope comes with the profile or is set later.
            cup = [0 0; 0 0.1; 0.1 0.1; 0.1 0.08; 0.01 0.08; 0.01 0];
            tc.verifyError(@() phx.shape.Revolution("Profile", cup, "Envelope", "cones"), ...
                "phx:Revolution:profileNotMonotonic");
            shape = phx.shape.Revolution("Profile", cup);
            shape.Envelope = "cones";
            body = phx.Body([], "Shape", shape);
            tc.verifyError(@() phx.Simulation(body), "phx:Revolution:profileNotMonotonic");
        end

        function restsAtHalfTheProfileExtent(tc)
            % Dropped flat, the body must be held up at half of the axial
            % reach of its profile, not at its radius.
            body = tc.revolutionBody(tc.ringProfile, "z");
            body.Position = [0 0 3*tc.Height];
            z = tc.settle(body, phx.Body.empty);
            tc.verifyEqual(z, tc.Height/2, "AbsTol", 1e-3, ...
                "Resting height does not match the axial extent of the profile.");
        end

        function envelopeHasNoHole(tc)
            % The profile is an annulus, but the envelope is a full cylinder:
            % lowered onto a peg that fits through the hole, the body has to
            % come to rest on top of the peg instead of swallowing it.
            pegTop = 0.1;
            peg = phx.Body([], "Type", "static", "Position", [0 0 pegTop/2], ...
                "Shape", {"Cylinder", "Height", pegTop, ...
                "Radius", tc.HoleRatio*tc.Radius - 0.01});
            body = tc.revolutionBody(tc.ringProfile, "z");
            body.Position = [0 0 pegTop + tc.Height];
            z = tc.settle(body, peg);
            tc.verifyEqual(z, pegTop + tc.Height/2, "AbsTol", 1e-3, ...
                "The envelope swallowed the peg, so it is not a solid cylinder.");
        end
    end

    methods (Access = private)
        function p = ringProfile(tc)
            % Closed annulus cross-section, rows of [axial radial]
            h = tc.Height;
            ri = tc.HoleRatio*tc.Radius;
            p = [-h/2 ri; -h/2 tc.Radius; h/2 tc.Radius; h/2 ri; -h/2 ri];
        end

        function body = revolutionBody(~, profile, axis, envelope)
            % Mass and inertia are set explicitly, so a comparison of two
            % shapes cannot be thrown off by their mass computation
            arguments
                ~
                profile
                axis
                envelope = "cylinder"
            end
            body = phx.Body([], "Mass", 5, "Inertia", [0.03 0.03 0.06], ...
                "Shape", phx.shape.Revolution("Profile", profile, "Axis", axis, ...
                "Envelope", envelope, "Segments", 24));
        end

        function p = hourglassProfile(~)
            % Two cones of slope 0.7 meeting in a waist of radius 0.03 at
            % axial 0.1, flat ends of radius 0.1, rows of [axial radial]
            p = [0 0; 0 0.1; 0.1 0.03; 0.2 0.1; 0.2 0];
        end

        function T = tiltedDrop(tc, body, axis)
            % Final pose after a short tilted, spinning drop onto the table
            body.Position = [0 0 4*tc.Height + tc.Radius/2];
            body.EulerAngles = [0.35 0.2 0];
            body.AngularVelocity = 8*double(["x", "y", "z"] == axis);
            sim = tc.simulation([body tc.table]);
            sim.step(0.4, 800);
            T = body.Transform;
            delete(sim);
        end

        function z = settle(tc, body, extra)
            % Height the body comes to rest at
            sim = tc.simulation([body tc.table extra]);
            sim.step(0.5, 500);
            z = body.Position(3);
            delete(sim);
        end

        function t = table(~)
            t = phx.Body([], "Type", "static", "Position", [0 0 -0.05], ...
                "Shape", {"Box", "Size", [2 2 0.1]}, "Friction", 1);
        end

        function sim = simulation(~, objs)
            % Margin off: these tests are about the envelope geometry itself
            sim = phx.Simulation(objs, ...
                "EngineSettings", phx.engine.Settings("Margin", 0));
        end
    end

end
