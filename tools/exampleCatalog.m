function catalog = exampleCatalog
%EXAMPLECATALOG Grouping and thumbnail timing of the phxex_* examples.
%
%   CATALOG = EXAMPLECATALOG returns a table with one row per example that
%   needs something the example file itself cannot say: the gallery section
%   it belongs to and, optionally, how deep into the run its thumbnail is
%   grabbed.
%
%   An example does not have to be listed here - unlisted phxex_*.m files fall
%   into the "Other" section with the default delay, and a "% Category: <name>"
%   line in the example header always wins over this table. A phxex_*.slx file
%   is in the gallery only when it is listed here with a Summary.
%
%   Columns:
%     Category - gallery section, must be one of the sections listed in
%                the CategoryOrder constant of buildExampleDocs
%     Delay    - seconds into the run when the thumbnail is captured;
%                NaN uses the default of buildExampleDocs
%     Summary  - gallery caption; needed only by Simulink examples,
%                which have no H1 line to read it from
%     Press    - buttons an interactive example needs pressed before
%                its thumbnail can show anything, comma separated
%
%   See also buildExampleDocs, captureExampleThumb

%   Copyright 2026 HUMUSOFT s.r.o.

    rows = [ ...
        entry("phxex_minimal",     "Basics and geometry",            3)
        entry("phxex_shapes",      "Basics and geometry",            6)
        entry("phxex_joints",      "Basics and geometry",            5)
        entry("phxex_springs",     "Basics and geometry",            4)
        entry("phxex_gravity",     "Basics and geometry",            6)
        entry("phxex_slide",       "Basics and geometry",            5)
        entry("phxex_terrain",     "Basics and geometry",            8)
        entry("phxex_textures",    "Basics and geometry",           10)

        entry("phxex_jenga",       "Contacts, stacking and granular", 6)
        entry("phxex_stack",       "Contacts, stacking and granular", 6)
        entry("phxex_galton",      "Contacts, stacking and granular", 25)
        entry("phxex_droptest",    "Contacts, stacking and granular", 6)
        entry("phxex_capsize",     "Contacts, stacking and granular", 20)
        entry("phxex_soil",        "Contacts, stacking and granular", 20)
        entry("phxex_grip",        "Contacts, stacking and granular", 10)
        entry("phxex_isolation",   "Contacts, stacking and granular", 22)

        entry("phxex_gears",       "Mechanisms and machines",         6)
        entry("phxex_wankel",      "Mechanisms and machines",        10)
        entry("phxex_vengine",     "Mechanisms and machines",        10)
        entry("phxex_camvalve",    "Mechanisms and machines",        10)
        entry("phxex_screwconv",   "Mechanisms and machines",        12)
        entry("phxex_conveyors",   "Mechanisms and machines",        15)
        entry("phxex_trainwheel",  "Mechanisms and machines",         8)
        entry("phxex_tackle",      "Mechanisms and machines",        10)

        entry("phxex_buggy",       "Vehicles, robots and control",   10)
        entry("phxex_segway",      "Vehicles, robots and control",   18)
        entry("phxex_drone",       "Vehicles, robots and control",   15)
        entry("phxex_rocket",      "Vehicles, robots and control",   12)
        entry("phxex_balance",     "Vehicles, robots and control",    4)
        entry("phxex_antisway",    "Vehicles, robots and control",   12)
        entry("phxex_swingover",   "Vehicles, robots and control",   16, "", "Build walls,Simulate")
        entry("phxex_sim_swingover", "Vehicles, robots and control", 24, "Simulink version: a scheduled trolley path swings the load over the wall")
        entry("phxex_gyrostab",    "Vehicles, robots and control",   20)
        entry("phxex_stairfall",   "Vehicles, robots and control",    8)

        entry("phxex_magnets",     "Fields, magnetism and fluids",   15)
        entry("phxex_magpendulum", "Fields, magnetism and fluids",   12)
        entry("phxex_charges",     "Fields, magnetism and fluids",   10)
        entry("phxex_crystal",     "Fields, magnetism and fluids",   25)
        entry("phxex_rotmagdip",   "Fields, magnetism and fluids",   10)
        entry("phxex_maglev",      "Fields, magnetism and fluids",   10)
        entry("phxex_sorter",      "Fields, magnetism and fluids",   12)
        entry("phxex_capacitor",   "Fields, magnetism and fluids",   25)
        entry("phxex_buoyancy",    "Fields, magnetism and fluids",   15)

        entry("phxex_optimize",    "Analysis and workflows",         40)
        entry("phxex_validate",    "Analysis and workflows",         20)
        entry("phxex_determinism", "Analysis and workflows",         15)
        entry("phxex_reversetime", "Analysis and workflows",          8)
        entry("phxex_noview",      "Analysis and workflows",         15)
        entry("phxex_multisim",    "Analysis and workflows",         10)
        entry("phxex_multiview",   "Analysis and workflows",         10)
        ];

    catalog = struct2table(rows);
    catalog.Properties.RowNames = catalog.Name;

end

function s = entry(name, category, delay, summary, press)
    arguments
        name, category, delay
        summary (1, 1) string = ""      % only Simulink examples need one
        press (1, 1) string = ""        % only interactive examples need one
    end
    s = struct("Name", string(name), "Category", string(category), ...
        "Delay", delay, "Summary", summary, "Press", press);
end
