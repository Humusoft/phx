function catalog = exampleCatalog
%EXAMPLECATALOG Grouping and thumbnail timing of the phxex_* examples.
%
%   CATALOG = EXAMPLECATALOG returns a table with one row per example that
%   needs something the example file itself cannot say: the gallery section
%   it belongs to and, optionally, how deep into the run its thumbnail is
%   grabbed.
%
%   An example does not have to be listed here. Unlisted examples fall into
%   the "Other" section and use the default capture delay, and a
%   "% Category: <name>" line in the example header always wins over this
%   table. The table exists so that adding a demo does not force an edit of
%   the generator itself - see tools/buildExampleDocs.m.
%
%   Columns:
%     Category - gallery section, must be one of the sections listed in
%                the CategoryOrder constant of buildExampleDocs
%     Delay    - seconds into the run when the thumbnail is captured;
%                NaN uses the default of buildExampleDocs
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
        entry("phxex_gyrostab",    "Vehicles, robots and control",   20)
        entry("phxex_stairfall",   "Vehicles, robots and control",    8)

        entry("phxex_magnets",     "Fields, magnetism and fluids",   15)
        entry("phxex_magpendulum", "Fields, magnetism and fluids",   12)
        entry("phxex_charges",     "Fields, magnetism and fluids",   10)
        entry("phxex_crystal",     "Fields, magnetism and fluids",   25)
        entry("phxex_rotmagdip",   "Fields, magnetism and fluids",   10)
        entry("phxex_maglev",      "Fields, magnetism and fluids",   10)
        entry("phxex_sorter",      "Fields, magnetism and fluids",   12)
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

function s = entry(name, category, delay)
    s = struct("Name", string(name), "Category", string(category), "Delay", delay);
end
