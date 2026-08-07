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
    xpl = zeros(nSteps, 1);   % isolated base plate
    xr1 = zeros(nSteps, 1);   % roller 1
    
    pA0 = S.roofA.Position;  pB0 = S.roofB.Position;
    pl0 = S.plate.Position;  r10 = S.roller1.Position;
    
    kColl = NaN;
    
    for k = 1:nSteps
        tk = k * P.dt;
        ts  = k * P.dt;
        amp = P.ampMax * min(1, ts / P.tRamp);
        g   = amp * sin(2*pi*P.freq*ts);
        S.ground.Position = [g 0 -P.groundT/2];
    
        sim.step(P.dt, 1, 1);
    
        pA = S.roofA.Position;  pB = S.roofB.Position;
        t(k)   = tk;      xg(k)  = g;
        xrA(k) = pA(1);   zrA(k) = pA(3);
        xrB(k) = pB(1);   zrB(k) = pB(3);
        xpl(k) = S.plate.Position(1);
        xr1(k) = S.roller1.Position(1);
    
        if isnan(kColl) && k > nSteps && zrA(k) < pA0(3) - P.collapseDrop
            kColl = k;
        end
    end
    
    delete(sim);
    
    % Analysis
    if isnan(kColl)
        kEnd = nSteps;
        collTime = NaN;
    else
        kEnd = kColl;
        collTime = t(kColl);
    end
    win = (nSteps+1):kEnd;
    
    dA = xrA(win) - pA0(1);
    dB = xrB(win) - pB0(1);
    dG = xg(win);
    
    peakA = max(abs(dA));  rmsA = sqrt(mean(dA.^2));
    peakB = max(abs(dB));  rmsB = sqrt(mean(dB.^2));
    peakG = max(abs(dG));  rmsG = sqrt(mean(dG.^2));
    
    driftPlate  = xpl(kEnd) - pl0(1);
    driftPlateE = xpl(end)  - pl0(1);
    rollerRel   = (xr1(end) - r10(1)) - driftPlateE;   % roller travel along the plate
    zDropB      = pB0(3) - zrB(kEnd);
    zDropBend   = pB0(3) - zrB(end);
    
    fprintf("\n=== PHX roller base isolation ===\n");
    fprintf("shake: %.2f Hz, amplitude ramp 0 -> %.0f mm over %.1f s\n", P.freq, 1000*P.ampMax, P.tRamp);
    fprintf("timestep %.0f ms, %d storeys, %d bodies\n\n", 1000*P.dt, P.storeys, numel(S.all));
    
    if isnan(collTime)
        fprintf("Un-isolated tower did NOT collapse within %.1f s of shaking.\n", P.tShake);
    else
        fprintf("Un-isolated tower collapsed at t = %.2f s of shaking ", collTime);
        fprintf("(ground amplitude then %.0f mm).\n", 1000*P.ampMax*min(1, collTime/P.tRamp));
    end
    
    fprintf("  horizontal motion      peak [mm]   RMS [mm]\n");
    fprintf("  ground                 %8.1f   %8.1f\n", 1000*peakG, 1000*rmsG);
    fprintf("  roof, un-isolated      %8.1f   %8.1f\n", 1000*peakA, 1000*rmsA);
    fprintf("  roof, roller-isolated  %8.1f   %8.1f\n", 1000*peakB, 1000*rmsB);
    fprintf("\n  transmission ratio (isolated / un-isolated):  peak %.3f, RMS %.3f\n", ...
        peakB/peakA, rmsB/rmsA);
    fprintf("  amplification vs ground: un-isolated %.2f, isolated %.2f (peak)\n", ...
        peakA/peakG, peakB/peakG);
    
    % The isolated roof's ABSOLUTE motion is dominated by the rigid drift of the
    % whole isolated assembly (there is no restoring force). Racking = roof motion
    % relative to whatever the tower stands on, i.e. the shaking actually felt.
    rA = dA - dG;                             % reference tower: roof vs ground
    rB = dB - (xpl(win) - pl0(1));            % isolated tower: roof vs base plate
    pkrA = max(abs(rA)); rmrA = sqrt(mean(rA.^2));
    pkrB = max(abs(rB)); rmrB = sqrt(mean(rB.^2));
    fprintf("\n  drift-removed (roof relative to its own base = racking):\n");
    fprintf("  racking, un-isolated   %8.1f   %8.1f\n", 1000*pkrA, 1000*rmrA);
    fprintf("  racking, roller-isolat %8.1f   %8.1f\n", 1000*pkrB, 1000*rmrB);
    fprintf("  racking transmission ratio: peak %.3f, RMS %.3f\n", pkrB/pkrA, rmrB/rmrA);
    
    fprintf("\n  isolated base plate drift: %+.0f mm at collapse, %+.0f mm at end of run\n", ...
        1000*driftPlate, 1000*driftPlateE);
    fprintf("  roller travel along the plate (end): %+.0f mm (plate half-length %.0f mm)\n", ...
        1000*rollerRel, 1000*P.plateX/2);
    fprintf("  isolated roof settling: %.0f mm at collapse, %.0f mm at end ", ...
        1000*zDropB, 1000*zDropBend);
    if zDropBend < P.collapseDrop
        fprintf("-> STILL STANDING\n");
    else
        fprintf("-> COLLAPSED (after the reference tower)\n");
    end
    fprintf("\n");
    
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
