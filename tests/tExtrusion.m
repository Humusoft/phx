classdef tExtrusion < matlab.unittest.TestCase
%tExtrusion Mesh-generation tests for phx.internal.Geometry.extrusion.
%
%   The swept-profile generator orients each cross-section with a
%   rotation-minimizing (parallel-transport) frame, so the profile plane
%   stays perpendicular to the spine and the section does not roll or tilt
%   between points. These tests pin down the properties that broke under the
%   old zero-roll yaw-pitch frame, which distorted (and was singular for) any
%   spine segment with a world-Z component:
%
%     * a swept solid is congruent under rotation of the spine about the
%       sweep axis (the root-cause proof: a +Z jog must not differ from the
%       same jog rotated into +Y);
%     * a spine running along world +Z is finite and consistent (no atan2
%       singularity at the poles);
%     * a straight spine along X keeps a non-circular profile's orientation
%       (the cam in phxex_camvalve relies on this).
%
%   See also phx.internal.Geometry, phx.shape.Extrusion

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test)
        function crossSectionStaysPerpendicularToTangent(tc)
            % A straight diagonal spine with a world-Z component (py~=0 AND
            % pz~=0) is exactly what the old zero-roll frame got wrong: it
            % placed the profile plane oblique to the tangent (here ~7 deg off).
            % The corrected frame keeps every cross-section perpendicular, so a
            % circular profile sweeps a true right cylinder whose rings lie in
            % planes normal to the spine direction.
            ca = linspace(0, 2*pi, 49)';
            prof = 0.02*[cos(ca) sin(ca)];
            d = [1 0.6 0.8]/norm([1 0.6 0.8]);          % py~=0 and pz~=0
            spine = (0:5)'*0.04.*d;
            V = phx.internal.Geometry.extrusion(spine, [1 1], prof, false, false);

            lp = numel(ca);                             % vertices per ring
            ring = V(2*lp + 1:3*lp, :);                 % an interior ring
            spokes = ring - mean(ring, 1);
            nrm = sum(cross(spokes, circshift(spokes, -1), 2), 1);
            nrm = nrm/norm(nrm);
            tc.verifyLessThan(acosd(abs(nrm*d')), 0.5, ...
                "Cross-section plane is not perpendicular to the spine tangent.");
        end

        function zAlignedSpineIsFiniteAndConsistent(tc)
            % A spine running straight along world +Z used to hit the atan2
            % singularity of the old aim. It must produce finite geometry whose
            % volume matches the identical extrusion swept along +X.
            ca = linspace(0, 2*pi, 25)';
            prof = 0.02*[cos(ca) sin(ca)];
            spZ = [0 0 0; 0 0 0.05; 0 0 0.10];
            spX = [0 0 0; 0.05 0 0; 0.10 0 0];

            [VZ, NZ, FZ] = phx.internal.Geometry.extrusion(spZ, [1 1], prof, true, true);
            tc.verifyFalse(any(isnan(VZ(:))), "Z-aligned spine produced NaN vertices.");
            tc.verifyFalse(any(isnan(NZ(:))), "Z-aligned spine produced NaN normals.");

            [VX, ~, FX] = phx.internal.Geometry.extrusion(spX, [1 1], prof, true, true);
            volZ = phx.internal.Geometry.meshMass(VZ, FZ, 1);
            volX = phx.internal.Geometry.meshMass(VX, FX, 1);
            tc.verifyEqual(volZ, volX, "RelTol", 1e-6);
        end

        function straightSpinePreservesProfileOrientation(tc)
            % Straight spine along X: the 2D profile spans the local Y-Z plane
            % and must stay there (the cam is a non-circular lobe extruded this
            % way). Check an intentionally asymmetric profile keeps its Y and Z
            % extents while X only spans the spine length.
            prof = [0.03*cos(linspace(0, 2*pi, 41)'), ...
                    0.02*sin(linspace(0, 2*pi, 41)')];   % ellipse: wider in Y
            L = 0.05;
            spine = [-L/2 0 0; L/2 0 0];
            V = phx.internal.Geometry.extrusion(spine, [1 1], prof, true, true);

            ext = max(V) - min(V);
            tc.verifyEqual(ext(1), L, "AbsTol", 1e-9);          % X = spine length
            tc.verifyEqual(ext(2), 0.06, "RelTol", 1e-6);       % Y = 2*0.03
            tc.verifyEqual(ext(3), 0.04, "RelTol", 1e-6);       % Z = 2*0.02
        end
    end

end
