function phxex_vengine(opts)
%PHXEX_VENGINE V-configuration multi-cylinder piston engine (slider-crank)
%
% A torque-driven crankshaft (phx.shape.Extrusion) on two main bearings spins a
% bank of slider-crank cylinders: crank pin -> rod (phx.RevoluteJoint) -> wrist
% pin (phx.SphericalJoint) -> piston on a phx.PrismaticJoint, in two banks opened
% by the bank angle. The block is dynamic, connected to a static pedestal by one
% compliant phx.BushingJoint, so the demo reads the engine's vibration from the
% block motion and the joint reaction (mount.ForceA). Cylinder count, bank angle,
% stroke, bore and rod length are settable.
%
% phxex_vengine("NumCylinders", 6, "BankAngle", 60) runs a 60-degree V6.
%
% See also phx.RevoluteJoint, phx.SphericalJoint, phx.PrismaticJoint, phx.BushingJoint

%   Copyright 2026 HUMUSOFT s.r.o.

    arguments
        opts.NumCylinders (1, 1) double {mustBeInteger, mustBePositive} = 8
        opts.BankAngle    (1, 1) double = 90      % angle between the two banks (deg)
        opts.Stroke       (1, 1) double = 0.09    % piston stroke (m)
        opts.Bore         (1, 1) double = 0.086   % cylinder bore = piston diameter (m)
        opts.RodLength    (1, 1) double = 0.15    % connecting-rod length (m)
        opts.Rpm          (1, 1) double = 600     % crankshaft speed (rev/min)
    end

    if mod(opts.NumCylinders, 2) ~= 0
        error("phx:vengine:oddCylinders", ...
            "NumCylinders must be even (two banks); got %d.", opts.NumCylinders);
    end
    if opts.RodLength <= opts.Stroke/2
        error("phx:vengine:shortRod", ...
            "RodLength (%.3f) must exceed the crank radius Stroke/2 (%.3f).", ...
            opts.RodLength, opts.Stroke/2);
    end

    % --- Derived geometry -----------------------------------------------
    halfV  = deg2rad(opts.BankAngle)/2;   % half bank angle
    r      = opts.Stroke/2;               % crank radius
    L      = opts.RodLength;              % connecting-rod length
    bore   = opts.Bore;
    nPair  = opts.NumCylinders/2;         % cylinders per bank = crank throws
    zc     = 0;                           % crankshaft-axis height
    throwSpc = bore*1.35;                 % crank-throw spacing along the shaft
    rodOff   = bore*0.32;                 % X offset of the two rods on one throw

    % --- Scene ----------------------------------------------------------
    [viewer, ax] = phx.extra.Viewer("clear", "DefaultCameraPosition", [0.9 0.6 0.3], "Texture", "defaultChecker");

    [crank, block, mount, cat, cyl] = buildScene(ax, nPair, halfV, r, L, bore, zc, throwSpc, rodOff);

    reach = L + r;
    xL = (nPair-1)/2*throwSpc + rodOff + 1.5*bore;
    yL = reach*sin(halfV) + bore;
    axis([-xL xL, -yL yL, zc-r-7*bore, zc + reach*cos(halfV) + bore]);

    % Loggers record the run at 500 Hz (the 10 Hz default is far too coarse for
    % the ~10 Hz engine orders and their harmonics).
    Lspeed = phx.Logger(crank, "Parameters", "AngularVelocity", "Frequency", 500);
    Lblock = phx.Logger(block, "Parameters", "Position", "Frequency", 500);
    Lreact = phx.Logger(mount, "Parameters", "ForceA", "Frequency", 500);
    Lcat   = phx.Logger(cat,   "Parameters", "Position", "Frequency", 500);

    % Centimetre-scale parts -> zero the default collision margin, as in
    % phxex_camvalve (the jointed parts pass through each other anyway).
    sim = phx.Simulation(ax, "EngineSettings", phx.engine.BulletSettings("Margin", 0));

    % --- Run: drive the crankshaft with a torque, governed to target rpm --
    dt = 1e-3; subFrame = 10;
    tEnd = 3.0;                              % spin-up + steady running
    nStep = round(tEnd/dt);

    wTarget = 2*pi*opts.Rpm/60;              % target spin rate (rad/s)
    Kp = 0.12;                               % governor gain (N*m per rad/s error)
    tauMax = 30;                             % drive-torque clamp (N*m)

    for i = 1:nStep
        w = crank.AngularVelocity(1);        % spin rate about the shaft axis
        tau = max(min(Kp*(wTarget - w), tauMax), -tauMax);
        crank.applyTorque([tau 0 0], false); % drive torque about world X
        sim.step(dt, 1, mod(i, subFrame) == 0);
        if mod(i, subFrame) == 0
            viewer.displayText(sprintf("V%d   bank %.0f%c   %.0f / %.0f rpm   drive %.1f Nm", ...
                opts.NumCylinders, opts.BankAngle, char(176), w*60/(2*pi), opts.Rpm, tau), ...
                "replace", 1, 16, [0.8 0.4 0]);
        end
    end

    % Pull the recorded channels from the loggers, then tear the engine down.
    tL  = Lblock.Time;
    avc = Lspeed.getChannel(1);  rpm = avc(:, 1)*60/(2*pi);   % shaft speed (rpm)
    blk = Lblock.getChannel(1);                               % block position (m)
    rec = Lreact.getChannel(1);                               % joint reaction (N)
    ktp = Lcat.getChannel(1);                                 % cat position (m)
    delete(sim);

    % Vibration read two ways, each about its own steady-state mean (spin-up out)
    steady = tL > tEnd/2;
    vib = (blk - mean(blk(steady, :)))*1e3;        % block motion, mm
    frc = rec - mean(rec(steady, :));              % dynamic mount reaction, N
    pkV = max(abs(vib(steady, :)), [], 1);
    pkF = max(abs(frc(steady, :)), [], 1);
    fprintf("V%d, bank %.0f deg: reached %.0f rpm; block motion +-[%.3f %.3f %.3f] mm, mount reaction +-[%.1f %.1f %.1f] N (XYZ).\n", ...
        opts.NumCylinders, opts.BankAngle, rpm(end), pkV(1), pkV(2), pkV(3), pkF(1), pkF(2), pkF(3));

    % Did the vibration move the cat? Measure its drift over steady running.
    catS = ktp(steady, :);
    catDrift = catS(end, :) - catS(1, :);          % net move while running
    catPP = max(catS, [], 1) - min(catS, [], 1);   % peak-to-peak
    fprintf("cat on plate corner: net drift [%.2f %.2f %.2f] mm, peak-to-peak [%.2f %.2f %.2f] mm.\n", ...
        catDrift*1e3, catPP*1e3);

    % --- Spin-up, the two vibration readouts, and a reaction spectrogram -
    clf(figure(2));
    subplot(4, 1, 1);
    plot(tL, rpm, "LineWidth", 1.3); grid on; hold on;
    yline(opts.Rpm, ":", "target");
    ylabel("crank speed [rpm]");
    title(sprintf("V%d spins up to %.0f rpm under drive torque", opts.NumCylinders, opts.Rpm));
    subplot(4, 1, 2);
    plot(tL, vib, "LineWidth", 1.0); grid on;
    ylabel("block motion [mm]"); legend("X", "Y", "Z", "Location", "northwest");
    title("Vibration from block motion");
    subplot(4, 1, 3);
    plot(tL, frc, "LineWidth", 1.0); grid on;
    ylabel("mount reaction [N]"); legend("X", "Y", "Z", "Location", "northwest");
    title("Vibration from bushing-joint reaction force");

    % Spectrogram of the reaction force over the whole run: the engine orders
    % sweep up during spin-up and settle at 1x/2x/3x of the shaft speed (marked).
    subplot(4, 1, 4);
    fs = 1/mean(diff(tL));
    [tc, fSpec, S] = localSpectrogram(frc, fs, 0.35, 0.9, 40, 4);
    imagesc(tc, fSpec, 20*log10(S/max(S(:)) + 1e-4)); axis("xy");
    colormap(gca, "turbo"); clim([-40 0]);
    cb = colorbar; cb.Label.String = "dB";
    hold on;
    f0 = median(rpm(steady))/60;                 % fundamental (Hz) at steady speed
    for h = 1:3
        plot(tc(end), h*f0, "wo", "MarkerFaceColor", "w", "MarkerSize", 5);
        text(tc(end), h*f0, sprintf("%d%c ", h, 215), "Color", "w", ...
            "HorizontalAlignment", "right", "VerticalAlignment", "bottom", "FontWeight", "bold");
    end
    xlabel("time [s]"); ylabel("frequency [Hz]");
    title("Reaction-force spectrogram (orders rise with rpm, settle at 1/2/3x)");
end

% ---------------------------------------------------------------------------
function [crank, block, mount, cat, cyl] = buildScene(ax, nPair, halfV, r, L, bore, zc, throwSpc, rodOff)
% Build the crankshaft, the block plate on its mount, and every slider-crank loop

    crankR = r*0.25;
    pistH  = bore*0.85;
    rodTX  = bore*0.12;                   % rod thickness across the shaft
    rodTZ  = bore*0.28;                   % rod thickness in the swing plane

    % Resources directory
    resdir = fullfile(fileparts(mfilename("fullpath")), "res", " ");

    % Crankshaft: one multiply-cranked shaft (phx.shape.Extrusion) whose spine
    % jogs from the bearing axis out to each rod journal and back, passing through
    % every pin. Collisions OFF (joints constrain it; its hull would hit the rods).
    crankLen = (nPair-1)*throwSpc + 2*rodOff + bore;
    jR   = crankR;                          % journal / pin radius
    web  = throwSpc*0.11;                    % axial length of each diagonal crank web
    jl   = throwSpc*0.06;                    % short axial lead-in onto the pin
    na   = 24; ca = linspace(0, 2*pi, na+1)';
    profile = jR*[cos(ca) sin(ca)];          % circular shaft cross-section (closed)
    spine = [-crankLen/2, 0, 0];             % left main-journal end (on the axis)
    for k = 1:nPair
        xk  = (k - (nPair+1)/2)*throwSpc;
        phi = (k-1)*2*pi/nPair;
        py  = -r*sin(phi); pz = r*cos(phi);  % pin offset (matches the rod revolute PointA)
        % Lead-in points at the pin offset give both pin points an axial incoming
        % segment, so the extrusion renders the pin as a clean short cylinder.
        spine = [spine
                 xk - rodOff - web - jl, 0,  0     % main-journal shoulder (on axis)
                 xk - rodOff - jl,       py, pz    % crank-web landing at the pin offset
                 xk - rodOff,            py, pz    % --- crank pin: clean short cylinder
                 xk + rodOff,            py, pz    %     from these two points ---
                 xk + rodOff + jl,       py, pz    % crank-web landing
                 xk + rodOff + web + jl, 0,  0];   % main-journal shoulder     %#ok<AGROW>
    end
    spine = [spine; crankLen/2, 0, 0];       % right main-journal end (on the axis)
    crankShape = phx.shape.Extrusion("Spine", spine, "Profile", profile, ...
        "Material", "metal", "Color", 1, "Texture", resdir+"checker4.png", "TextureBlend", 0.5);
    crank = phx.Body(ax, "Type", "dynamic", "Position", [0 0 zc], "Shape", crankShape);
    crank.Collisions = false;               % visual-only shape; joints do the constraining
    crank.Mass    = 3;                       % lumped shaft + flywheel
    crank.Inertia = [0.03 0.03 0.03];        % modest flywheel (bearings lock the tilt DOFs)

    % Engine block: a dynamic plate carrying the whole engine, held only by the
    % single central mount below (partner for the bearings and piston sliders).
    blkT   = 0.02;
    blkTop = zc - r - 1.0*bore;
    blkPos = [0 0 blkTop - blkT/2];
    Lx = crankLen + 2*bore; Ly = 3*bore;
    block  = phx.Body(ax, "Type", "dynamic", "Position", blkPos, ...
        "Shape", {"Box", "Size", [Lx, Ly, blkT], "Color", [0.8 0.32 0.36]});

    % Two main bearings: a revolute joint at each shaft end ties the crankshaft
    % to the block, leaving it free only to spin about its own axis (world X).
    for e = [-1 +1]
        phx.RevoluteJoint(block, crank, "PointA", [e*crankLen/2, 0, zc] - blkPos, "PointB", [e*crankLen/2, 0, 0], ...
            "AxisA", [1 0 0], "AxisB", [1 0 0], "Visible", false);
    end

    % Engine mount: one static pedestal under the block, connected by a phx.BushingJoint
    % whose small compliance lets the block micro-vibrate - read from
    % block.Position and the joint reaction (mount.ForceA / mount.TorqueA).
    pedH     = 4*bore;
    mountTop = blkPos(3) - blkT/2;
    pedestal = phx.Body(ax, "Type", "static", "Position", [0 0 mountTop - pedH/2], ...
        "Shape", {"Box", "Size", [3*bore, 2*bore, pedH], "Color", [0.22 0.23 0.26]});
    mount = phx.BushingJoint(pedestal, block, "PointA", [0 0 pedH/2], "PointB", [0 0 -blkT/2], ...
        "LinearStiffness", 5e5, "LinearDamping", 5e3, "AngularStiffness", 1e3, "AngularDamping", 1e2, ...
        "Visible", false);

    % A small cat on a plate corner: does the block vibration move it? An ordinary
    % dynamic body resting through contact + friction, so real shaking would slide it.
    cat = phx.Body(ax, "EulerAngles", [0 0 pi/2], "Friction", [0.6 0 0], ...
        "Position", [Lx/2 - 0.05, Ly/2 - 0.02, blkTop + 0.1], ...
        "Shape", {resdir+"cat.stl", "Scale", 0.002, "Envelope", "box", "Material", "matte", "Color", 1});

    rodColor  = [0.85 0.6 0.38];
    pistColor = [0.9 0.9 0.9];

    % Piston: a revolved "H" cross-section - two skirt walls joined by a crown web
    % pushed high, leaving a shallow dish in the crown. Profile is [axial, radial],
    % closed, on the axis only at the web; mass/inertia come from the mesh (Density).
    Rp  = bore/2;                            % piston radius
    pw  = Rp*0.25;                           % skirt-wall / crown-ring thickness
    ptw = pistH*0.22;                        % crown-web thickness
    pd  = pistH*0.12;                        % crown recess depth (small dish)
    pistProfile = [-pistH/2,        Rp-pw
                   -pistH/2,        Rp
                    pistH/2,        Rp
                    pistH/2,        Rp-pw
                    pistH/2-pd,     Rp-pw
                    pistH/2-pd,     0
                    pistH/2-pd-ptw, 0
                    pistH/2-pd-ptw, Rp-pw
                   -pistH/2,        Rp-pw];

    cyl = struct("piston", {}, "O", {}, "u", {});

    for k = 1:nPair
        xThrow = (k - (nPair+1)/2)*throwSpc;
        phase  = (k-1)*2*pi/nPair;                 % crank-throw phasing
        d = [0, -sin(phase), cos(phase)];          % crank-pin radial dir at theta = 0

        for bank = [+1 -1]                         % +1 = right (+Y) bank, -1 = left (-Y)
            xRod = xThrow + bank*rodOff;
            u = [0, bank*sin(halfV), cos(halfV)];  % bore axis (unit)
            O = [xRod, 0, zc];                     % crank centre in this cylinder plane
            P0 = O + r*d;                          % crank pin (world, theta = 0)
            udotd = dot(u, d);
            s = r*udotd + sqrt(L^2 - r^2*(1 - udotd^2));   % piston distance along bore
            W0 = O + s*u;                          % wrist pin (world, theta = 0)

            rodDir = (W0 - P0)/L;
            beta  = atan2(rodDir(3), rodDir(2));   % rod long axis (local Y) -> rodDir
            gamma = -bank*halfV;                   % piston local Z -> bore axis u

            rod = phx.Body(ax, "Position", (P0+W0)/2, "EulerAngles", [beta 0 0], ...
                "Shape", {"Box", "Size", [rodTX L rodTZ], "Material", "metal", "Color", rodColor});
            piston = phx.Body(ax, "Position", W0, "EulerAngles", [gamma 0 0], ...
                "Shape", phx.shape.Revolution("Axis", "z", "Profile", pistProfile, ...
                         "Material", "metal", "Color", pistColor, "Density", 2700));

            % Crank pin: revolute between the crankshaft and the rod, axis = X.
            phx.RevoluteJoint(crank, rod, "PointA", P0 - [0 0 zc], "PointB", [0 -L/2 0], ...
                "AxisA", [1 0 0], "AxisB", [1 0 0], "Visible", false);

            % Wrist pin: spherical (not revolute) so the closed loop is not
            % over-constrained by redundant out-of-plane hinge locks.
            phx.SphericalJoint(rod, piston, "PointA", [0 L/2 0], "PointB", [0 0 0], "Visible", false);

            % Piston slides in its bore: prismatic vs the block, sliding axis
            % aimed along the bore u. The piston was built with its own axis Z
            % along the bore, so on its side the default axis already fits.
            phx.PrismaticJoint(block, piston, "PointA", O - blkPos, "AxisA", u, ...
                "PointB", [0 0 0], "Visible", false);

            cyl(end+1) = struct("piston", piston, "O", O, "u", u); %#ok<AGROW>
        end
    end
end

% ---------------------------------------------------------------------------
function [tc, f, S] = localSpectrogram(X, fs, winSec, overlap, fmax, pad)
% Short-time Fourier transform of a signal X (columns summed in power, so a
% 3-axis force gives one order-content map), without a toolbox dependency.
% The true frequency resolution is 1/winSec; each window is zero-padded to
% pad*Nw so the plotted frequency grid is pad-times finer (interpolated, i.e.
% visually smoother - it does not separate components closer than 1/winSec).
    Nw   = max(16, round(winSec*fs));
    hop  = max(1, round(Nw*(1 - overlap)));
    Nfft = pad*Nw;                                   % zero-pad for a finer grid
    win  = 0.5 - 0.5*cos(2*pi*(0:Nw-1)'/(Nw-1));     % Hann window
    starts = 1:hop:(size(X, 1) - Nw + 1);
    nf = floor(Nfft/2) + 1;
    f  = (0:nf-1)'*fs/Nfft;
    S  = zeros(nf, numel(starts));
    for k = 1:numel(starts)
        seg = X(starts(k):starts(k)+Nw-1, :);
        seg = (seg - mean(seg, 1)).*win;             % detrend + window each column
        F = fft(seg, Nfft, 1);                       % zero-padded FFT
        S(:, k) = sqrt(sum(abs(F(1:nf, :)).^2, 2));  % combine axes in power
    end
    tc = (starts + (Nw-1)/2)/fs;                      % window-centre times
    keep = f <= fmax;  f = f(keep);  S = S(keep, :);
end