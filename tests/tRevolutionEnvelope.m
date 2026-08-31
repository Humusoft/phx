classdef tRevolutionEnvelope < PhxTestCase
%tRevolutionEnvelope The bounding-cylinder collision envelope of a revolution.
%
%   phx.shape.Revolution can wear a primitive cylinder instead of a hull:
%   Envelope "cylinder" fits a cylinder about the revolution axis, sized from
%   the profile (radius = its largest radius, half height = its axial reach).
%   That is what a body rolling on a rim needs, since a hull is faceted, and
%   it is the only envelope of this shape that needs no graphics primitive,
%   so these tests run with no axes at all.
%
%   Locked in here:
%     * a revolved rectangle with this envelope moves exactly like the
%       equivalent phx.shape.Cylinder, for every modeling axis,
%     * the body rests at half the axial reach of its profile,
%     * the envelope is solid: a hole in the profile is not a hole in it.
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
            for axis = ["z", "x", "y"]
                h = tc.Height;
                r = tc.Radius;
                rect = [-h/2 0; h/2 0; h/2 r; -h/2 r; -h/2 0];
                revolved = tc.tiltedDrop(tc.revolutionBody(rect, axis), axis);
                primitive = tc.tiltedDrop(phx.Body([], "Mass", 5, ...
                    "Inertia", [0.03 0.03 0.06], "Shape", {"Cylinder", ...
                    "Radius", r, "Height", h, "Axis", axis}), axis);
                tc.verifyEqual(revolved, primitive, "AbsTol", 1e-9, ...
                    "Envelope differs from phx.shape.Cylinder for Axis " + axis + ".");
            end
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

        function body = revolutionBody(tc, profile, axis)
            % Mass and inertia are set explicitly, so a comparison of two
            % shapes cannot be thrown off by their mass computation
            body = phx.Body([], "Mass", 5, "Inertia", [0.03 0.03 0.06], ...
                "Shape", phx.shape.Revolution("Profile", profile, "Axis", axis, ...
                "Envelope", "cylinder", "Segments", 24));
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
                "Shape", {"Box", "Size", [2 2 0.1]}, "Friction", [1 0 0]);
        end

        function sim = simulation(~, objs)
            % Margin off: these tests are about the envelope geometry itself
            sim = phx.Simulation(objs, ...
                "EngineSettings", phx.engine.BulletSettings("Margin", 0));
        end
    end

end
