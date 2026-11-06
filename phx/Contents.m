% PHX
% Version 1.0.16 (R2026b) 06-Nov-2026
%
% PHX is an object-oriented physics toolbox for MATLAB, layered over the
% Bullet physics engine. Scenes are built from phx.* objects that draw into
% MATLAB axes, are stepped by a phx.Simulation, and can be driven from Simulink.
%
% Core
%   phx.Simulation            - Scene root that steps the physics engine
%   phx.Body                  - Rigid body with pose, mass, friction and forces
%   phx.engine.BulletSettings - Initial settings of the Bullet physics engine
%
% Shapes
%   phx.shape.Box             - Box shape
%   phx.shape.Sphere          - Sphere shape
%   phx.shape.Cylinder        - Cylinder shape
%   phx.shape.Tube            - Tube, pipe or funnel shape
%   phx.shape.Cone            - Cone shape
%   phx.shape.Capsule         - Capsule shape
%   phx.shape.Mesh            - Triangular mesh shape imported from a file or built from data
%   phx.shape.Extrusion       - Profile extruded along a spine
%   phx.shape.Revolution      - Profile revolved about an axis
%   phx.shape.Rock            - Procedurally generated irregular rock
%   phx.shape.Terrain         - Terrain heightfield
%
% Joints
%   phx.RevoluteJoint         - Revolute joint (hinge, 1 DOF)
%   phx.PrismaticJoint        - Prismatic joint (slider, 1 DOF)
%   phx.CylindricalJoint      - Cylindrical joint (slide and spin about one axis, 2 DOF)
%   phx.SphericalJoint        - Spherical joint (ball joint, 3 DOF)
%   phx.GearJoint             - Gear joint (coupled rotation of two bodies)
%   phx.FixedJoint            - Fixed joint (rigid connection, 0 DOF)
%   phx.BushingJoint          - Bushing joint (compliant fixed joint)
%   phx.GenericJoint          - Generic joint (6 DOF with limits)
%
% Springs and ropes
%   phx.Spring                - Linear spring-damper between two bodies
%   phx.Rope                  - Rope routed over pulleys
%
% Forces and fields
%   phx.Thruster              - Body-fixed thrust actuator
%   phx.Resistance            - Environmental drag on bodies
%   phx.Buoyancy              - Buoyancy and hydrodynamic damping
%   phx.Monopole              - Monopole interaction between bodies
%   phx.Dipole                - Dipole interaction between bodies
%
% Assemblies
%   phx.assembly.arena        - Build a static arena - a floor plate enclosed by four walls
%   phx.assembly.chain        - Build a chain of rigid links along a polyline
%   phx.assembly.scatter      - Scatter bodies randomly inside a box region
%   phx.assembly.wall         - Build a wall of bricks laid in a running bond
%   phx.assembly.import       - Import a multi-body model from a file or a rigidBodyTree
%
% Measurement and automation
%   phx.Logger                - Data logger recording object properties over time
%   phx.Trace                 - Trail of a moving point of a body
%   phx.Measure               - Relative kinematics between two bodies
%   phx.Zone                  - Spatial zone detecting bodies inside a region
%   phx.Raycast               - Ray sensor probing the scene for bodies
%   phx.Script                - Automation script prescribing property profiles over time
%   phx.Function              - Custom computation in the simulation pipeline
%   phx.template.Interaction  - Template for a custom interaction between two bodies
%
% Simulink
%   PhxModel                  - PHX Model block: co-simulate a PHX scene from Simulink
%   PhxAction                 - PHX Action block: run MATLAB code against a running PHX scene
%
% Visualization
%   phx.extra.Viewer          - Interactive scene viewer
%   phx.Camera                - Camera following bodies of the scene
%   phx.PlanarShadow          - Shadow of bodies projected onto a plane
%
% Documentation
%   phxdoc                    - Open a PHX reference page in the Help browser
%
% See also phx.Simulation, phx.Body

%   Copyright 2026 HUMUSOFT s.r.o.
%   SPDX-License-Identifier: LicenseRef-PHX-Preview-1.0
%   Licensed under the PHX Preview License v1.0; see LICENSE and NOTICE.
%   ^..^
