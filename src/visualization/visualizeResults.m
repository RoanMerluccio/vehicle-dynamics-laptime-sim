function visualizeResults(results)
%VISUALIZERESULTS Produce standard plots from simulation results.
%
%   visualizeResults(results)
%
%   Produces 4 figures:
%     1. Track map coloured by speed (2D top-down)
%     2. Performance plots (tiled): speed, lateral g, longitudinal g, G-G diagram
%     3. 3D track ribbon (height = speed)
%     4. Sector time bar chart

g     = 9.81;
track = results.track;
speed = results.speed_profile_mps;
latG  = results.lateral_acc_mps2      / g;
lonG  = results.longitudinal_acc_mps2 / g;
s     = track.s;
x     = track.x;
y     = track.y;

% ---- Figure 1: Track map (2D) -------------------------------------------
figure('Name','Track Map','NumberTitle','off');
scatter(x, y, 20, speed, 'filled');
colormap(jet); cb = colorbar; cb.Label.String = 'Speed (m/s)';
axis equal; grid on;
title('Track map — coloured by speed');
xlabel('x (m)'); ylabel('y (m)');

% ---- Figure 2: Performance plots (2x2 tiled) ----------------------------
figure('Name','Performance','NumberTitle','off');
tl = tiledlayout(2, 2, 'TileSpacing','compact','Padding','compact');
title(tl, 'Performance plots');

nexttile;
plot(s, speed, 'b-', 'LineWidth', 1.5);
grid on; title('Speed'); xlabel('s (m)'); ylabel('m/s');

nexttile;
plot(s, latG, 'r-', 'LineWidth', 1.5);
grid on; title('Lateral acceleration'); xlabel('s (m)'); ylabel('g');

nexttile;
plot(s, lonG, 'Color',[0 0.6 0], 'LineWidth', 1.5);
grid on; title('Longitudinal acceleration'); xlabel('s (m)'); ylabel('g');

nexttile;
plot(lonG, latG, '.k', 'MarkerSize', 6);
grid on; axis equal;
title('G-G diagram'); xlabel('Longitudinal (g)'); ylabel('Lateral (g)');

% ---- Figure 3: 3D track ribbon ------------------------------------------
figure('Name','3D Track Map','NumberTitle','off');

halfWidth = 2.0;
dxv = gradient(x);   dyv = gradient(y);
len = hypot(dxv, dyv) + 1e-12;
nx  = -dyv ./ len;   ny = dxv ./ len;

XS = [x + halfWidth*nx, x - halfWidth*nx]';
YS = [y + halfWidth*ny, y - halfWidth*ny]';
ZS = [speed, speed]';

surf(XS, YS, ZS, ZS, 'EdgeColor','none', 'FaceAlpha',0.95);
colormap(jet); shading interp;
cb3 = colorbar; cb3.Label.String = 'Speed (m/s)';
hold on;
plot3(x, y, speed, 'k-', 'LineWidth', 0.8);

% Mark fastest and slowest points
[~, iMax] = max(speed);
plot3(x(iMax), y(iMax), speed(iMax), 'w^','MarkerFaceColor','w','MarkerSize',8);
text(x(iMax), y(iMax), speed(iMax), sprintf('  %.1f m/s', speed(iMax)), ...
     'Color','w','FontWeight','bold','FontSize',9);

interior = 5:numel(speed)-5;
[~, iMinRel] = min(speed(interior));
iMin = iMinRel + 4;
plot3(x(iMin), y(iMin), speed(iMin), 'rv','MarkerFaceColor','r','MarkerSize',8);
text(x(iMin), y(iMin), speed(iMin), sprintf('  %.1f m/s', speed(iMin)), ...
     'Color','r','FontSize',9);

axis equal; grid on; box on;
title('3D track ribbon — height = speed');
xlabel('x (m)'); ylabel('y (m)'); zlabel('Speed (m/s)');
view(45, 35);
hold off;

% ---- Figure 4: Sector times ---------------------------------------------
if isfield(results, 'sectors')
    sec = results.sectors;
    figure('Name','Sector Times','NumberTitle','off');
    b = bar(sec.times_s, 'FaceColor','flat');
    cdata = jet(numel(sec.times_s));
    for k = 1:numel(sec.times_s)
        b.CData(k,:) = cdata(k,:);
    end
    set(gca, 'XTickLabel', sec.labels);
    grid on; box on;
    title('Sector times'); xlabel('Sector'); ylabel('Time (s)');
    for k = 1:numel(sec.times_s)
        text(k, sec.times_s(k), sprintf('  %.1f m/s', sec.avg_speed_mps(k)), ...
             'VerticalAlignment','bottom','FontSize',9);
    end
end
end
