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
            tc.verifyEqual(I, [1 1 1]*1^2*m*2/5, "RelTol", 1e-12); % 2/5 m r^2
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
            % and the centre itself, a quarter of the height above the base
            s = phx.shape.Cone("Diameter", 2, "Height", 4, "Density", 1000); % r = 1, h = 4
            [m, I, c] = s.computeMass;
            tc.verifyEqual(m, 1000*pi*4/3, "RelTol", 1e-12);
            tc.verifyEqual(I, [3/80*16 + 3/20, 3/80*16 + 3/20, 3/10]*m, "RelTol", 1e-12);
            tc.verifyEqual(c, [0 0 -1], "AbsTol", 1e-12);
            [~, ~, c] = phx.shape.Cone("Height", 4, "Axis", "x").computeMass;
            tc.verifyEqual(c, [-1 0 0], "AbsTol", 1e-12);
        end

        function truncatedConeMassAndInertia(tc)
            % A frustum is a cone of height H minus its tip cone: radii 1 and
            % 0.5 over a height of 2 cut a cone of height 4 at half its
            % height. Mass, moments and centres of mass from the two cones.
            % Equal diameters give the cylinder's values.
            rho = 1000;
            s = phx.shape.Cone("Diameter", [2 1], "Height", 2, "Density", rho);
            [m, I, c] = s.computeMass;
            mBig = rho*pi*4/3;               % r = 1, h = 4, base at z = 0
            mTip = rho*pi*0.25*2/3;          % r = 0.5, h = 2, base at z = 2
            zBig = 1;                        % h/4 above its base
            zTip = 2 + 0.5;
            tc.verifyEqual(m, mBig - mTip, "RelTol", 1e-12);
            zc = (mBig*zBig - mTip*zTip)/m;
            tc.verifyEqual(c, [0 0 zc - 1], "AbsTol", 1e-12, "The origin is 1 above the base.");
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

        function symmetricShapesHaveTheCentreAtTheOrigin(tc)
            shapes = {phx.shape.Box("Size", [1 2 3]), phx.shape.Sphere, ...
                phx.shape.Cylinder("Axis", "x"), phx.shape.Capsule("Axis", "y"), ...
                phx.shape.Tube, phx.shape.Cone("Diameter", [1 1])};
            for k = 1:numel(shapes)
                [~, ~, c] = shapes{k}.computeMass;
                tc.verifyEqual(c, [0 0 0], "AbsTol", 1e-12, class(shapes{k}));
            end
        end

        function taperedTubeIsAFrustumLessItsBore(tc)
            % The mesh of a tapered tube approaches the difference of two
            % frusta: its centre of mass and its moments about that centre
            % follow from the two cones
            rho = 1000;
            h = 2;
            d = [0.9 0.3];
            t = 0.1;
            tube = phx.shape.Tube("InnerDiameter", d, "WallThickness", t, ...
                "Height", h, "Segments", 720, "Density", rho);
            [m, I, c] = tube.computeMass;
            [mo, Io, co] = phx.shape.Cone("Diameter", d + 2*t, "Height", h, "Density", rho).computeMass;
            [mi, Ii, ci] = phx.shape.Cone("Diameter", d, "Height", h, "Density", rho).computeMass;
            tc.verifyEqual(m, mo - mi, "RelTol", 1e-4);
            ce = (mo*co - mi*ci)/(mo - mi);
            tc.verifyEqual(c, ce, "AbsTol", 1e-4);
            io = Io(1) + mo*(co(3) - ce(3))^2 - Ii(1) - mi*(ci(3) - ce(3))^2;
            tc.verifyEqual(I, [io io Io(3) - Ii(3)], "RelTol", 1e-4);
        end

        function revolutionClosesAnOpenProfileToTheAxis(tc)
            % A profile with an end off the axis weighs as the solid closed
            % there by a disc square to the axis, wherever the origin lies:
            % the mantle of a cylinder as the cylinder, a cone without its
            % base as the cone. Enough segments that the faceted mesh
            % converges on the ideal solid.
            rho = 1000;
            revolution = @(p) phx.shape.Revolution("Profile", p, "Segments", 720, "Density", rho);
            [m, I, c] = revolution([-0.5 0.5; 0.5 0.5]).computeMass;
            [me, Ie] = phx.shape.Cylinder("Diameter", 1, "Height", 1, "Density", rho).computeMass;
            tc.verifyEqual([m I], [me Ie], "RelTol", 1e-4);
            tc.verifyEqual(c, [0 0 0], "AbsTol", 1e-9);

            [m, ~, c] = revolution([0 0.5; 1 0.5]).computeMass;
            tc.verifyEqual(m, me, "RelTol", 1e-4);
            tc.verifyEqual(c, [0 0 0.5], "AbsTol", 1e-6, "The mantle moved up weighs where it is.");

            [m, I, c] = revolution([-0.5 0.5; 0.5 0]).computeMass;
            [me, Ie, ce] = phx.shape.Cone("Diameter", 1, "Height", 1, "Density", rho).computeMass;
            tc.verifyEqual([m I], [me Ie], "RelTol", 1e-4);
            tc.verifyEqual(c, ce, "AbsTol", 1e-6);
        end

        function revolutionLoopIsHollow(tc)
            % A profile that returns to its first point keeps its bore: a
            % pipe of radii 0.4 and 0.5, and a cup (a bottom 0.05 thick and
            % a wall 0.05 thick, the loop running along the axis below)
            rho = 1000;
            pipe = phx.shape.Revolution("Profile", ...
                [-0.5 0.5; 0.5 0.5; 0.5 0.4; -0.5 0.4; -0.5 0.5], "Segments", 720, "Density", rho);
            [m, I] = pipe.computeMass;
            ro = 0.5; ri = 0.4; h = 1;
            tc.verifyEqual(m, rho*pi*(ro^2 - ri^2)*h, "RelTol", 1e-4);
            tc.verifyEqual(I, m*[(3*(ro^2 + ri^2) + h^2)/12, (3*(ro^2 + ri^2) + h^2)/12, (ro^2 + ri^2)/2], ...
                "RelTol", 1e-4);

            cup = phx.shape.Revolution("Profile", [0.5 0; -0.5 0; -0.5 0.5; 0.5 0.5; ...
                0.5 0.45; -0.45 0.45; -0.45 0; 0.5 0], "Segments", 720, "Density", rho);
            tc.verifyEqual(cup.computeMass, rho*pi*(0.5^2*1 - 0.45^2*0.95), "RelTol", 1e-4);
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

        function volumeIsTheGeometryOfTheMass(tc)
            % computeVolume gives the volume, the moments of the volume at
            % unit density (a row of three) and its centre; computeMass is
            % those at the shape's Density, every shape alike
            res = fullfile(fileparts(fileparts(mfilename("fullpath"))), "examples", "res");
            shapes = {
                phx.shape.Box("Size", [0.3 0.5 0.7], "Density", 2500)
                phx.shape.BoxShell("Size", [1 0.6 0.4], "WallThickness", 0.03, "Density", 7800)
                phx.shape.Capsule("Diameter", 0.4, "Height", 1.1, "Axis", "x", "Density", 900)
                phx.shape.CapsuleShell("Diameter", 0.5, "Height", 0.8, "WallThickness", 0.02, "Density", 2700)
                phx.shape.Cone("Radius", [0.3 0.1], "Height", 0.6, "Axis", "y", "Density", 1200)
                phx.shape.Cylinder("Diameter", 0.25, "Height", 0.9, "Density", 500)
                phx.shape.Extrusion("Profile", [0 0; 1 0; 1 0.5; 0 0.5], "Spine", [0 0 0; 0 0 1], "Density", 650)
                phx.shape.Mesh("Source", fullfile(res, "BuggyBody.stl"), "Scale", 0.001, "Envelope", "convex", "Density", 300)
                phx.shape.Revolution("Profile", [0.2 0; 0.3 0.5; 0.1 1], "Density", 1500)
                phx.shape.Rock("Diameter", 0.4, "Density", 2600)
                phx.shape.Sphere("Diameter", 0.33, "Density", 7800)
                phx.shape.SphereShell("Diameter", 1, "WallThickness", 0.05, "Density", 1100)
                phx.shape.Tube("Diameter", 0.3, "Height", 0.5, "WallThickness", 0.04, "Density", 4000)
                };
            for k = 1:numel(shapes)
                s = shapes{k};
                [V, J, c] = s.computeVolume;
                [m, I, cm] = s.computeMass;
                tc.verifySize(J, [1 3], class(s));
                tc.verifyGreaterThan([V J], 0, class(s));
                tc.verifyEqual(m, s.Density*V, "RelTol", 1e-14, class(s));
                tc.verifyEqual(I, s.Density*J, "RelTol", 1e-14, class(s));
                tc.verifyEqual(cm, c, class(s));
            end
        end

        function volumesOfTheSolids(tc)
            % Closed forms of the volume and of its moments per unit
            % density: box, sphere, cylinder, sphere shell
            [V, J] = phx.shape.Box("Size", [1 2 3]).computeVolume;
            tc.verifyEqual(V, 6, "RelTol", 1e-14);
            tc.verifyEqual(J, [4 + 9, 1 + 9, 1 + 4]*6/12, "RelTol", 1e-14);
            [V, J] = phx.shape.Sphere("Diameter", 2).computeVolume;
            tc.verifyEqual(V, 4/3*pi, "RelTol", 1e-14);
            tc.verifyEqual(J, [1 1 1]*2/5*4/3*pi, "RelTol", 1e-14);
            [V, J] = phx.shape.Cylinder("Diameter", 2, "Height", 4).computeVolume;
            tc.verifyEqual(V, 4*pi, "RelTol", 1e-14);
            tc.verifyEqual(J, [(16 + 3)/12, (16 + 3)/12, 1/2]*4*pi, "RelTol", 1e-14);
            [V, J] = phx.shape.SphereShell("Diameter", 2, "WallThickness", 0.5).computeVolume;
            tc.verifyEqual(V, 4/3*pi*(1 - 0.125), "RelTol", 1e-14);
            tc.verifyEqual(J, [1 1 1]*8/15*pi*(1 - 0.5^5), "RelTol", 1e-14);
        end

        function terrainHasPlaceholders(tc)
            % A height field encloses no volume: fixed placeholders, 1 m^3
            % with moments that give 1000 kg and 100 kg*m^2 at 1000 kg/m^3
            s = phx.shape.Terrain("Height", zeros(5));
            [V, J, c] = s.computeVolume;
            tc.verifyEqual([V J c], [1 0.1 0.1 0.1 0 0 0]);
            [m, I] = s.computeMass;
            tc.verifyEqual([m I], [1000 100 100 100]);
        end
    end

end
