classdef tShapeMass < matlab.unittest.TestCase
%tShapeMass Analytic checks of shape mass and inertia.
%
%   computeMass is a pure method on the value-class shapes, so these tests
%   need neither the physics engine nor a graphics session. They lock in the
%   analytic formulas against textbook rigid-body values.
%
%   See also phx.shape.Box, phx.shape.Sphere, phx.shape.Cylinder

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function boxUnitMassAndInertia(tc)
            s = phx.shape.Box("Size", [1 1 1], "Density", 1000);
            [m, I] = s.computeMass;
            tc.verifyEqual(m, 1000, "RelTol", 1e-12);
            % I = m/12 * (b^2 + c^2) per axis, here all sides = 1
            tc.verifyEqual(I, [2 2 2]*1000/12, "RelTol", 1e-12);
        end

        function boxMassScalesWithVolume(tc)
            s = phx.shape.Box("Size", [2 3 4], "Density", 500);
            m = s.computeMass;
            tc.verifyEqual(m, 500*2*3*4, "RelTol", 1e-12);
        end

        function boxInertiaIsAnisotropic(tc)
            s = phx.shape.Box("Size", [1 2 3], "Density", 1000);
            [m, I] = s.computeMass;
            q = [1 4 9];
            tc.verifyEqual(I, [q(2)+q(3), q(1)+q(3), q(1)+q(2)]*m/12, "RelTol", 1e-12);
        end

        function sphereMassAndInertia(tc)
            s = phx.shape.Sphere("Diameter", 2, "Density", 1000); % r = 1
            [m, I] = s.computeMass;
            tc.verifyEqual(m, 1000*4/3*pi, "RelTol", 1e-12);
            tc.verifyEqual(I, 1^2*m*2/5, "RelTol", 1e-12); % 2/5 m r^2
        end

        function cylinderDefaultAxisInertia(tc)
            s = phx.shape.Cylinder("Diameter", 2, "Height", 4, "Density", 1000); % r=1 h=4
            [m, I] = s.computeMass;
            rq = 1; h = 4;
            tc.verifyEqual(m, 1000*pi*rq*h, "RelTol", 1e-12);
            ia = m*rq/2;             % about symmetry (z) axis
            io = m*(h^2 + 3*rq)/12;  % about transverse axes
            tc.verifyEqual(I, [io io ia], "RelTol", 1e-12);
        end

        function cylinderAxisPermutesInertia(tc)
            % Re-orienting the modeling axis must only permute the inertia,
            % not change the symmetry-axis value.
            base = phx.shape.Cylinder("Diameter", 2, "Height", 4, "Axis", "z");
            alt  = phx.shape.Cylinder("Diameter", 2, "Height", 4, "Axis", "x");
            [~, Iz] = base.computeMass;
            [~, Ix] = alt.computeMass;
            tc.verifyEqual(sort(Iz), sort(Ix), "RelTol", 1e-12);
            tc.verifyEqual(Ix(1), Iz(3), "RelTol", 1e-12); % symmetry axis moved to x
        end
    end

end
