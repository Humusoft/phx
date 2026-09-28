function phxex_isolation
%PHXEX_ISOLATION  Roller base isolation against an earthquake.
%   Two identical dry-stacked towers (wall blocks + floor slabs, friction only)
%   stand side by side on a kinematic ground plate that is shaken horizontally
%   with a ramping amplitude. The left tower sits directly on the ground, the
%   right one on a base plate carried by two free rollers. The roof motion of
%   both towers is compared.
%
%   See also phx.Simulation, phx.Body, phx.extra.Viewer.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Default parameters
    P.wallT   = 0.25;    % wall block thickness (along shaking direction)
    P.wallD   = 1.20;    % wall block depth
    P.wallH   = 0.70;    % wall block height
    P.span    = 1.00;    % centre distance of the two walls of a storey
    P.slabX   = 1.50;
    P.slabY   = 1.40;
    P.slabZ   = 0.15;
    P.storeys = 4;
    P.mu      = 0.80;    % block-to-block sliding friction
    
    P.xRef    = -2.6;
    P.xIso    =  2.6;
    
    P.groundT   = 0.50;
    P.rollerR   = 0.15;
    P.rollerL   = 1.60;
    P.rollerGap = 2.20;
    P.plateX    = 4.00;
    P.plateY    = 1.70;
    P.plateZ    = 0.12;
    
    P.dt        = 0.002;
    P.tShake    = 10.0;
    P.tRamp     =  8.0;
    P.freq      = 1.1;
    P.ampMax    = 0.16;
    P.collapseDrop = 0.25;
    
    % Viewer
    figure(1);
    [~, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0 -10 3.2], "DefaultCameraTarget",   [0 0 1.9]);
    
    % Scene
    % shaking ground
    S.ground = phx.Body(ax, "Type", "kinematic", "Position", [0 0 -P.groundT/2], "Friction", [1.2 0 0], ...
        "Shape", {"Box", "Size", [12 6 P.groundT], "Color", [0.45 0.35 0.30]});
    
    % reference tower: straight on the ground
    [bodiesA, S.roofA] = buildTower(ax, P.xRef, 0, P);
    
    % isolation layer
    zR = P.rollerR;
    rollerShape = phx.shape.Cylinder("Radius", P.rollerR, "Height", P.rollerL, "Axis", "y", "Color", [0.80 0.62 0.20], "Texture", "checker", "TextureBlend", 0.25);
    S.roller1 = phx.Body(ax, "Position", [P.xIso - P.rollerGap/2, 0, zR], "Friction", [1.2 0.0015 0.0015], "Shape", rollerShape);
    S.roller2 = phx.Body(ax, "Position", [P.xIso + P.rollerGap/2, 0, zR], "Friction", [1.2 0.0015 0.0015], "Shape", rollerShape);
    
    zP = 2*P.rollerR + P.plateZ/2;
    S.plate = phx.Body(ax, "Position", [P.xIso 0 zP], "Friction", [1.2 0 0], "Shape", {"Box", "Size", [P.plateX P.plateY P.plateZ], "Color", [0.58 0.62 0.68]});
    
    zTop = 2*P.rollerR + P.plateZ;
    [bodiesB, S.roofB] = buildTower(ax, P.xIso, zTop, P);
    
    S.all = [S.ground, bodiesA, S.roller1, S.roller2, S.plate, bodiesB];

    
    % Simulation
    sim = phx.Simulation(S.all);
    nSteps  = round(P.tShake  / P.dt);
    
    t   = zeros(nSteps, 1);
    xg  = zeros(nSteps, 1);   % ground
    xrA = zeros(nSteps, 1);   % roof, reference tower
    xrB = zeros(nSteps, 1);   % roof, isolated tower
    zrA = zeros(nSteps, 1);
    zrB = zeros(nSteps, 1);

    pA0 = S.roofA.Position;  pB0 = S.roofB.Position;

    collTime = NaN;           % when the reference tower's roof has dropped

    for k = 1:nSteps
        tk  = k * P.dt;
        amp = P.ampMax * min(1, tk / P.tRamp);
        g   = amp * sin(2*pi*P.freq*tk);
        S.ground.Position = [g 0 -P.groundT/2];

        sim.step(P.dt, 1, 1);

        pA = S.roofA.Position;  pB = S.roofB.Position;
        t(k)   = tk;      xg(k)  = g;
        xrA(k) = pA(1);   zrA(k) = pA(3);
        xrB(k) = pB(1);   zrB(k) = pB(3);

        if isnan(collTime) && zrA(k) < pA0(3) - P.collapseDrop
            collTime = tk;
        end
    end

    delete(sim);

    % Plot results
    clf(figure(2));
    tl = tiledlayout(2, 1, "TileSpacing", "compact");
    
    nexttile; hold on; grid on;
    plot(t, 1000*xg, "Color", [.6 .6 .6], "LineWidth", 0.8);
    plot(t, 1000*(xrA-pA0(1)), "Color", [.85 .25 .15], "LineWidth", 1.1);
    plot(t, 1000*(xrB-pB0(1)), "Color", [.10 .40 .75], "LineWidth", 1.1);
    if ~isnan(collTime)
        xline(collTime, "k--", "collapse", "LineWidth", 1.2);
    end
    ylabel("horizontal displacement [mm]");
    legend("ground", "roof, un-isolated", "roof, roller-isolated", "Location", "northwest");
    title("Roof motion, dry-stacked towers under a ramped earthquake");
    
    nexttile; hold on; grid on;
    plot(t, 1000*(zrA-pA0(3)), "Color", [.85 .25 .15], "LineWidth", 1.1);
    plot(t, 1000*(zrB-pB0(3)), "Color", [.10 .40 .75], "LineWidth", 1.1);
    if ~isnan(collTime)
        xline(collTime, "k--", "LineWidth", 1.2);
    end
    xlabel("time since start of shaking [s]");
    ylabel("roof height change [mm]");
    legend("un-isolated", "roller-isolated", "Location", "southwest");
    title("Roof height (collapse indicator)");
    
    xlabel(tl, "");

end

% =========================================================================
function [bodies, roof] = buildTower(ax, x0, zBase, P)
    bodies = phx.Body.empty(1, 0);
    h = P.wallH + P.slabZ;
    for s = 0:P.storeys-1
        zw = zBase + s*h + P.wallH/2;
        for sgn = [-1 1]
            b = phx.Body(ax, "Position", [x0 + sgn*P.span/2, 0, zw], "Friction", [P.mu 0 0], ...
                "Shape", {"Box", "Size", [P.wallT P.wallD P.wallH], "Color", [0.66 0.67 0.68]});
            bodies(end+1) = b; %#ok<AGROW>
        end
        zs = zBase + s*h + P.wallH + P.slabZ/2;
        roof = phx.Body(ax, "Position", [x0 0 zs], "Friction", [P.mu 0 0], ...
            "Shape", {"Box", "Size", [P.slabX P.slabY P.slabZ], "Color", [0.44 0.49 0.56]});
        bodies(end+1) = roof; %#ok<AGROW>
    end
end
