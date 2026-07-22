% PHX
% Version 1.0.5 (R2026a) 22-Jul-2026
%
% PHX is an object-oriented physics toolbox for MATLAB, layered over the
% Bullet physics engine. Scenes are built from phx.* objects that draw into
% MATLAB axes, are stepped by a phx.Simulation, and can be driven from Simulink.
%
% Simulation
%   phx.Simulation         - Physical simulation
%   phx.Body               - Physical body
%
% Shapes
%   phx.shape.Box          - Box shape
%   phx.shape.Sphere       - Sphere shape
%   phx.shape.Cylinder     - Cylinder shape
%   phx.shape.Cone         - Cone shape
%   phx.shape.Capsule      - Capsule shape
%   phx.shape.Globe        - Globe shape
%   phx.shape.Mesh         - Custom shape with texture
%   phx.shape.Extrusion    - Extrusion shape
%   phx.shape.Revolution   - Revolution shape
%   phx.shape.Rock         - Rock shape
%   phx.shape.Terrain      - Terrain heightfield
%   phx.shape.OBJ          - OBJ imported shape
%   phx.shape.STL          - STL imported shape
%
% Assemblies
%   phx.assembly.arena     - Static arena - a floor plate enclosed by four walls
%   phx.assembly.chain     - Chain of rigid links along a polyline
%   phx.assembly.scatter   - Bodies scattered randomly inside a box region
%   phx.assembly.import    - Robot model imported from a URDF file
%
% Joints
%   phx.RevoluteJoint      - Revolute joint
%   phx.PrismaticJoint     - Prismatic joint
%   phx.SphericalJoint     - Spherical joint
%   phx.GearJoint          - Gear joint
%   phx.FixedJoint         - Fixed joint
%
% Springs and ropes
%   phx.Spring             - Spring
%   phx.Rope               - Rope routed over pulleys
%
% Forces and fields
%   phx.Thruster           - Thrust actuator
%   phx.Resistance         - Resistance
%   phx.Buoyancy           - Buoyancy and hydrodynamic damping
%   phx.Dipole             - Dipole interaction
%   phx.Monopole           - Monopole interaction
%
% Analysis and automation
%   phx.Trace              - Trace
%   phx.Measure            - Measure
%   phx.Logger             - Data logger
%   phx.Zone               - Spatial detection zone
%   phx.Camera             - Camera
%   phx.Script             - Automation script
%   phx.Function           - Custom computation in the simulation pipeline
%
% Visualization
%   phx.extra.Viewer       - Enhanced viewer
%
% Simulink
%   PhxModel               - Level-2 S-function backing the PHX library block
%
% See also phx.Simulation, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^
