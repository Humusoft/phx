classdef tPipelineRebuild < PhxTestCase
%tPipelineRebuild Tests that adding and deleting objects rebuilds the
%   execution pipelines of a live phx.Simulation.
%
%   phx.Simulation groups its objects by class into two pipelines
%   (compute-only and compute + redraw) and keeps a handle-to-body map for
%   distributing the poses the engine hands back. Both are rebuilt from
%   scratch by updatePipelines, which addObjects and delete(object) call.
%   The pipelines are private, so these tests drive the rebuild from the
%   outside: through the object counts the public dispPipelines display
%   reports, through bodies keeping their identity across a rebuild, and
%   through a body deleted mid-run leaving a steppable simulation behind
%   (the crash fixed by the 1.0.6 engine).
%
%   Bodies are headless ([] axes) except in the redraw tests, which need an
%   hgtransform to read the drawn pose from and therefore also carry the
%   "Graphics" tag.
%
%   See also phx.Simulation, phx.Simulation.addObjects, phx.base.Object

%   Copyright 2026 HUMUSOFT s.r.o.

    methods (Test, TestTags = {'Engine'})
        function addObjectsMidRunRegistersAndSimulatesTheBody(tc)
            % A body added to a running simulation is picked up by the
            % rebuilt pipeline and starts falling from where it was put.
            tc.requireEngine;
            first = tc.spawnBody([0 0 20]);
            sim = phx.Simulation(first);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.2, 20, -1);
            late = tc.spawnBody([5 0 20]);
            sim.addObjects(late);
            tc.verifyNumElements(sim.Children, 2);

            sim.step(0.2, 20, -1);

            % Both moved, the late one only for the second half.
            tc.verifyLessThan(first.Position(3), 20);
            tc.verifyLessThan(late.Position(3), 20);
            tc.verifyGreaterThan(late.Position(3), first.Position(3));
        end

        function deleteBodyMidRunLeavesSimulationSteppable(tc)
            % Regression guard for the memory corruption fixed by the 1.0.6
            % engine: deleting a body mid-run used to kill the whole MATLAB
            % process at the next teardown. Reaching the end of this test is
            % the assertion; the verifications only confirm that the
            % survivors keep simulating.
            tc.requireEngine;
            ground = tc.spawnBody([0 0 -0.5], "Type", "static");
            gate = tc.spawnBody([0 1 1], "Type", "static");
            ball = tc.spawnBody([0 0 5]);
            sim = phx.Simulation([ground gate ball]);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.2, 20, -1);
            delete(gate);
            tc.verifyNumElements(sim.Children, 2);

            sim.step(0.2, 20, -1);
            tc.verifyLessThan(ball.Position(3), 5);
            tc.verifyEqual(ground.Position, [0 0 -0.5], "AbsTol", 1e-9);

            delete(sim);            % the call this used to crash in
            tc.verifyFalse(isvalid(sim));
        end

        function jointedBodyDeletionLeavesSimulationSteppable(tc)
            % Deleting the middle body of a jointed chain leaves two joints
            % pointing at a dead parent. The rebuild has to survive that.
            tc.requireEngine;
            base = tc.spawnBody([0 0 0], "Type", "static");
            middle = tc.spawnBody([0 0 2]);
            tip = tc.spawnBody([0 0 4]);
            phx.RevoluteJoint(base, middle, "PointA", [0 0 1], "PointB", [0 0 -1]);
            phx.RevoluteJoint(middle, tip, "PointA", [0 0 1], "PointB", [0 0 -1]);
            sim = phx.Simulation([base middle tip]);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.2, 40, -1);
            delete(middle);
            sim.step(0.2, 40, -1);

            tc.verifyTrue(isvalid(tip));
            tc.verifyTrue(all(isfinite(tip.Position)));
            delete(sim);
            tc.verifyFalse(isvalid(sim));
        end

        function bodyIdentitySurvivesDeleteAndAdd(tc)
            % The handle-to-body map is rebuilt from scratch, so a rebuild
            % must not shuffle poses between bodies. Free fall keeps x
            % constant, which is what tells a mix-up apart.
            tc.requireEngine;
            bodies = phx.Body.empty;
            for i = 1:3
                bodies(i) = tc.spawnBody([i*2 0 10]);
            end
            sim = phx.Simulation(bodies);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.1, 10, -1);
            delete(bodies(2));
            late = tc.spawnBody([99 0 10]);
            sim.addObjects(late);

            sim.step(0.1, 10, -1);

            tc.verifyEqual(bodies(1).Position(1), 2, "AbsTol", 1e-9);
            tc.verifyEqual(bodies(3).Position(1), 6, "AbsTol", 1e-9);
            tc.verifyEqual(late.Position(1), 99, "AbsTol", 1e-9);
            % The survivors fell for twice as long as the late arrival.
            tc.verifyEqual(bodies(1).Position(3), bodies(3).Position(3), "AbsTol", 1e-9);
            tc.verifyLessThan(bodies(1).Position(3), late.Position(3));
        end

        function pipelineCountsFollowTheObjectSet(tc)
            % Bodies are batched into one phx.Body.updateView entry, whose
            % object count has to track additions and deletions.
            tc.requireEngine;
            bodies = [tc.spawnBody([0 0 10]) tc.spawnBody([2 0 10])];
            sim = phx.Simulation(bodies);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));
            tc.verifyEqual(tPipelineRebuild.redrawCount(sim), 2);

            sim.addObjects(tc.spawnBody([4 0 10]));
            tc.verifyEqual(tPipelineRebuild.redrawCount(sim), 3);

            delete(bodies(1));
            tc.verifyEqual(tPipelineRebuild.redrawCount(sim), 2);
        end

        function deletingTheLastBodyEmptiesThePipelines(tc)
            % An emptied simulation still has to be steppable.
            tc.requireEngine;
            b = tc.spawnBody([0 0 10]);
            sim = phx.Simulation(b);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.1, 10, -1);
            delete(b);

            tc.verifyEmpty(sim.Children);
            tc.verifyEqual(tPipelineRebuild.redrawCount(sim), 0);
            sim.step(0.1, 10, -1);
            tc.verifyEqual(sim.Time, 0.2, "AbsTol", 1e-9);
        end
    end

    methods (Test, TestTags = {'Graphics', 'Engine'})
        function invisibleBodyIsExcludedFromRedraw(tc)
            % With ExcludeInvisible (the default) an initially invisible
            % body is left out of the redraw pipeline: the engine advances
            % it, but its hgtransform stays where it was.
            tc.requireEngine;
            ax = tc.prepareAxes;
            shown = phx.Body(ax, "Position", [0 0 10]);
            hidden = phx.Body(ax, "Position", [3 0 10], "Visible", false);
            sim = phx.Simulation([shown hidden]);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.2, 20, 1);

            tc.verifyEqual(shown.Graphics.Matrix(3, 4), shown.Position(3), "AbsTol", 1e-9);
            tc.verifyLessThan(hidden.Position(3), 10);          % engine moved it
            tc.verifyEqual(hidden.Graphics.Matrix(3, 4), 10, "AbsTol", 1e-12);
        end

        function includingInvisibleBodiesRedrawsThem(tc)
            % ExcludeInvisible = false puts them back into the pipeline.
            tc.requireEngine;
            ax = tc.prepareAxes;
            hidden = phx.Body(ax, "Position", [3 0 10], "Visible", false);
            sim = phx.Simulation(hidden, "ExcludeInvisible", false);
            tc.addTeardown(@() tPipelineRebuild.deleteIfValid(sim));

            sim.step(0.2, 20, 1);

            tc.verifyEqual(hidden.Graphics.Matrix(3, 4), hidden.Position(3), "AbsTol", 1e-9);
        end
    end

    methods (Static, Access = private)
        function n = redrawCount(sim)
            % Number of bodies in the phx.Body.updateView entry of the
            % redraw pipeline, read off the public dispPipelines display.
            txt = evalc("sim.dispPipelines");
            n = regexp(txt, "phx\.Body\.updateView: (\d+) object", "tokens", "once");
            if isempty(n)
                n = 0;
            else
                n = str2double(n{1});
            end
        end

        function deleteIfValid(obj)
            % Teardown for the tests that delete the simulation themselves.
            if isvalid(obj)
                delete(obj);
            end
        end
    end

end
