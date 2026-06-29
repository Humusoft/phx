function phxex_trainwheel
% PHXEX_TRAINWHEEL Revolved wheel rolling on cylindrical rails
%
% A wheel built with phx.shape.Revolution rolls along static cylindrical rails;
% its lateral (hunting) motion is recorded and plotted live.

%   Copyright 2026 HUMUSOFT s.r.o.

    % Figure setup
    clf;
    ax = subplot(3, 1, 1:2);
    set(ax, "Projection", "perspective", "CameraPosition", [-115 -45 20], "Clipping", false);
    axis("equal"); camlight("headlight"); grid("on");
    xlabel("X"); ylabel("Y"); zlabel("Z");
    title("Train wheel motion");
    
    % Reset color order for shapes
    phx.base.ShapeMesh.resetColorOrder;
    
    % Create two static cylinders forming rails
    phx.Body(ax, "Type", "static", "Position", [0 +1 0], "EulerAngles", [0 pi/2.05 0], "Shape", {"Cylinder", "Diameter", 0.2, "Height", 30}, "Friction", [1 0 0]);
    phx.Body(ax, "Type", "static", "Position", [0 -1 0], "EulerAngles", [0 pi/2.05 0], "Shape", {"Cylinder", "Diameter", 0.2, "Height", 30}, "Friction", [1 0 0]);
    
    % Create wheel shape by revolving profile around y-axis
    halfProfile = [-1.25 0; -1.2 0.6; -0.8 0.8; -0.7 1; -0.6 1; -0.6 0.2];
    wheelShape = phx.shape.Revolution("Axis", "y", "Profile", [halfProfile; flipud(abs(halfProfile))], "Envelope", "concave", "Style", "flat", "SkeletPoints", [0 0.6 0; 0 0.6 1], "SkeletColor", 1, "SkeletStyle", "line");
    
    % Create dynamic wheel body with mass and inertia
    wheel = phx.Body(ax, "Position", [14 0.05 1.5], "Shape", wheelShape, "Friction", [1 0 0]);
    % phx.Trace(wheel, "Point", [0 -1.25 0], "Color", wheel.Color, "TracePoints", 1000);
    
    % Prepare second axes for the lateral movement curve
    xlim([-15 15]); ylim([-3 3]); zlim([-1 3]);
    subplot(3, 1, 3);
    ln = plot(NaN, NaN, "Color", wheel.Color);
    xlim([0 20]); ylim([-0.2 0.2]);
    xlabel("Time (s)"); ylabel("Y (m)");
    title("Lateral wheel movement")
    grid on;

    % Setup simulation with specific options
    opt = phx.engine.BulletSettings("Margin", 0, "AutoActivated", false);
    sim = phx.Simulation(ax, "EngineSettings", opt);

    % video = VideoWriter("phxex_trainwheel.mp4", "MPEG-4");
    % video.open;
    
    % Run the simulation, record lateral wheel position each step
    for i = 1:200
        sim.step(0.1, 10, 10);
        ln.XData(end + 1) = sim.Time;
        ln.YData(end + 1) = wheel.Position(2);
        % video.writeVideo(getframe(gcf));
    end
    
    delete(sim);
    % video.close;

end