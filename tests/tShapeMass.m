classdef tShapeMass < PhxTestCase
%tShapeMass Analytic checks of shape mass and inertia.
%
%   computeMass is a pure method on the value-class shapes, so these tests
%   need neither the physics engine nor a graphics session. They lock in the
%   analytic formulas against textbook rigid-body values.
%
%   See also phx.shape.Box, phx.shape.Sphere, phx.shape.Cylinder

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function boxMassAndInertia(tc)
            % m = density*volume and I = m/12*(b^2 + c^2) per axis, checked on
            % a unit cube (all three equal) and on a box with three different
            % sides (which is where a swapped axis would show).
            for size = {[1 1 1], [2 3 4], [1 2 3]}
                s = phx.shape.Box("Size", size{1}, "Density", 1000);
                [m, I] = s.computeMass;
                q = size{1}.^2;
                tc.verifyEqual(m, 1000*prod(size{1}), "RelTol", 1e-12);
                tc.verifyEqual(I, [q(2)+q(3), q(1)+q(3), q(1)+q(2)]*m/12, ...
                    "RelTol", 1e-12);
            end
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

        function coneMassAndInertia(tc)
            % A sharp cone about its centre of mass: m = rho pi r^2 h/3,
            % 3/10 m r^2 about the axis, 3/80 m h^2 + 3/20 m r^2 across
            s = phx.shape.Cone("Diameter", 2, "Height", 4, "Density", 1000); % r = 1, h = 4
            [m, I] = s.computeMass;
            tc.verifyEqual(m, 1000*pi*4/3, "RelTol", 1e-12);
            tc.verifyEqual(I, [3/80*16 + 3/20, 3/80*16 + 3/20, 3/10]*m, "RelTol", 1e-12);
        end

        function truncatedConeMassAndInertia(tc)
            % A frustum is a cone of height H minus its tip cone: radii 1 and
            % 0.5 over a height of 2 cut a cone of height 4 at half its
            % height. Mass, moments and centres of mass from the two cones.
            % Equal diameters give the cylinder's values.
            rho = 1000;
            s = phx.shape.Cone("Diameter", [2 1], "Height", 2, "Density", rho);
            [m, I] = s.computeMass;
            mBig = rho*pi*4/3;               % r = 1, h = 4, base at z = 0
            mTip = rho*pi*0.25*2/3;          % r = 0.5, h = 2, base at z = 2
            zBig = 1;                        % h/4 above its base
            zTip = 2 + 0.5;
            tc.verifyEqual(m, mBig - mTip, "RelTol", 1e-12);
            zc = (mBig*zBig - mTip*zTip)/m;
            ia = 3/10*(mBig*1 - mTip*0.25);
            io = (3/80*16 + 3/20)*mBig + mBig*(zBig - zc)^2 ...
                - (3/80*4 + 3/20*0.25)*mTip - mTip*(zTip - zc)^2;
            tc.verifyEqual(I, [io io ia], "RelTol", 1e-12);
            tc.verifyEqual(s.Radius, [1 0.5]);

            c = phx.shape.Cone("Diameter", [2 2], "Height", 4, "Density", rho);
            [mc, Ic] = c.computeMass;
            [my, Iy] = phx.shape.Cylinder("Diameter", 2, "Height", 4, "Density", rho).computeMass;
            tc.verifyEqual([mc Ic], [my Iy], "RelTol", 1e-12);
        end

        function coneSizeExpandsToTwoValues(tc)
            % A single diameter is a sharp cone, read back as [d 0]; the
            % radius follows; negative, three or all-zero values are refused
            s = phx.shape.Cone("Diameter", 0.6);
            tc.verifyEqual(s.Diameter, [0.6 0]);
            s.Radius = 0.2;
            tc.verifyEqual(s.Diameter, [0.4 0]);
            s.Radius = [0.2 0.1];
            tc.verifyEqual(s.Diameter, [0.4 0.2]);
            tc.verifyError(@() phx.shape.Cone("Diameter", [1 -1]), "phx:Cone:badSize");
            tc.verifyError(@() phx.shape.Cone("Diameter", [1 1 1]), "phx:Cone:badSize");
            tc.verifyError(@() phx.shape.Cone("Diameter", [0 0]), "phx:Cone:badSize");
        end
    end

end
