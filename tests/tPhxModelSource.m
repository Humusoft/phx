classdef tPhxModelSource < PhxTestCase
%tPhxModelSource Tests of the scene source of the PHX Model block.
%
%   The Source of a PHX Model block is a scene function, given by its name
%   on the path or by the path to its .m file, or a MAT file with a saved
%   scene. Every kind gives the same scene, both to the edit-time interface
%   of the block (phx.simulink.BlockBackend.loadSource) and to a running
%   simulation, where the block builds or loads the scene at its start.
%
%   See also phx.simulink.BlockBackend, PhxModel

%   Copyright 2026 HUMUSOFT s.r.o.

    properties (TestParameter)
        Kind = {"name", "path", "mat"}
    end

    methods (Test)
        function everySourceGivesTheScene(tc, Kind)
            source = tc.writeSource(Kind);
            scene = phx.simulink.BlockBackend.loadSource(source);
            objs = phx.simulink.BlockBackend.collectObjects(scene);
            names = sort(string(cellfun(@(o) o.Name, objs, "UniformOutput", false)));
            tc.verifyEqual(names, ["Ball" "Ground"]);
        end

        function otherFilesAreRejected(tc)
            tc.verifyError(@() phx.simulink.BlockBackend.loadSource("scene.txt"), ...
                "phx:PhxModel:unsupportedSource");
        end
    end

    methods (Test, TestTags = {'Engine'})
        function everySourceRunsInTheBlock(tc, Kind)
            % The block builds or loads the scene at the start of the run:
            % the ball of the scene falls under gravity.
            tc.requireEngine;
            tc.assumeTrue(~isempty(ver("simulink")) && license("test", "Simulink"), ...
                "Simulink is not available.");

            source = tc.writeSource(Kind);
            model = "tPhxModelSource_" + Kind;
            new_system(model);
            tc.addTeardown(@() close_system(model, 0));
            load_system("PhxLibrary");

            add_block("PhxLibrary/PHX Model", model + "/PHX");
            set_param(model + "/PHX", "ModelSource", source, "OutputRefs", "Ball.Position(3)", ...
                "ShowViewer", "off", "SimulationStep", "0.005");
            add_block("built-in/ToWorkspace", model + "/Height", "VariableName", "height", ...
                "SaveFormat", "Array");
            add_line(model, "PHX/1", "Height/1");
            set_param(model, "Solver", "FixedStepDiscrete", "FixedStep", "0.005", "StopTime", "0.3");

            out = sim(model);
            height = out.get("height");
            tc.verifyEqual(height(1), 1, "AbsTol", 1e-9);
            tc.verifyLessThan(height(end), 0.7);
        end
    end

    methods (Access = private)
        function source = writeSource(tc, kind)
            % A scene function in a temporary folder, or a MAT file saved
            % from it: a static ground and a ball named Ball at height 1.
            file = tc.writeTextFile("phxtest_scene.m", join([
                "function bodies = phxtest_scene"
                "    bodies = [phx.Body([], ""Name"", ""Ground"", ""Type"", ""static"", ""Position"", [0 0 -2])"
                "              phx.Body([], ""Name"", ""Ball"", ""Position"", [0 0 1])];"
                "end"], newline));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(file)));
            switch kind
                case "name"
                    source = "phxtest_scene";
                case "path"
                    source = string(file);
                case "mat"
                    scene.bodies = phxtest_scene; %#ok<STRNU> saved below
                    source = fullfile(tc.tempFolder, "phxtest_scene.mat");
                    save(source, "-struct", "scene");
            end
        end
    end

end
