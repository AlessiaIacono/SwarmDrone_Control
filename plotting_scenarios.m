
%% PLOTTING SCENARIOS
% Post-processing script for figures.
% Usage:
%   1) Run Parameters_scenarios.m
%   2) Run Simulink (must create variable 'out' in workspace)
%   3) Run this file

if ~exist('out', 'var')
    error('Missing variable ''out'': run the Simulink simulation first.');
end

disp('Generating plots...');

%% =========================
%  0) PLOT SELECTION
% =========================
DO_FIG1_TRAJ     = true;   % 3D trajectories + obstacles + goals
DO_FIG2_SAFETY   = true;   % per-agent minimum clearance dashboard
DO_FIG3_INPUTS   = true;   % estimated accel + estimated roll/pitch
DO_FIG4_TOPOLOGY = true;   % active links vs non-connected pairs
DO_FIG5_ANIM     = true;   % 3D animation of the swarm

% Saving / naming
SAVE_FIGS   = false;
SAVE_VIDEO  = false;
FILE_TAG    = '';

if exist('scenario_id', 'var') && ~isempty(scenario_id)
    SCENARIO_TAG = scenario_id;
else
    warning('scenario_id not found in workspace. Using fallback tag "Unknown".');
    SCENARIO_TAG = 'Unknown';
end

% Save folder near this script
script_path = fileparts(mfilename('fullpath'));
SAVE_DIR = fullfile(script_path, 'figures');

disp(['SAVE_DIR = ', SAVE_DIR]);

%% =========================
%  1) GLOBAL LATEX STYLE
% =========================
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');

%% =========================
%  2) RAW DATA EXTRACTION
% =========================
t_raw = out.x_robot.Time;
x_data_raw = out.x_robot.Data;

% Ensure shape [N_samples x (3*N_robots)]
if ndims(x_data_raw) == 3
    x_data_full = squeeze(x_data_raw)';
elseif size(x_data_raw, 1) == 3*n_robots && size(x_data_raw, 2) ~= 3*n_robots
    x_data_full = x_data_raw';
else
    x_data_full = x_data_raw;
end

if size(x_data_full, 2) ~= 3*n_robots
    error('Unexpected x_robot data shape: expected %d columns, found %d.', ...
        3*n_robots, size(x_data_full, 2));
end

% Basic plot parameters
colors = lines(n_robots);
LW = 1.5;
FS = 13;

%% =========================
%  3) TIME WINDOW
% =========================
start_idx = 10;
time_cutoff = min(25.0, t_raw(end));
[~, end_idx] = min(abs(t_raw - time_cutoff));

if length(t_raw) > start_idx && end_idx > start_idx
    t_plot = t_raw(start_idx:end_idx);
    x_plot = x_data_full(start_idx:end_idx, :);
else
    warning('Simulation too short: using full data.');
    t_plot = t_raw;
    x_plot = x_data_full;
end

if numel(t_plot) < 2
    error('Not enough samples in t_plot.');
end

%% =========================
%  FIGURE 1: 3D TRAJECTORIES
% =========================
if DO_FIG1_TRAJ
    f1 = figure('Name', 'Fig1_Trajectories', 'Color', 'w', 'Position', [50 100 900 700]);
    ax1 = axes('Parent', f1);
    hold(ax1, 'on');
    grid(ax1, 'on');
    axis(ax1, 'equal');
    view(ax1, 3);

    xlabel(ax1, 'X $[m]$', 'FontSize', FS);
    ylabel(ax1, 'Y $[m]$', 'FontSize', FS);
    zlabel(ax1, 'Z $[m]$', 'FontSize', FS);

    % Obstacles
    if ~exist('x_objects', 'var') || ~exist('radii_obstacles', 'var')
        warning('Missing x_objects / radii_obstacles in workspace: obstacles will not be drawn.');
    else
        [X_sph, Y_sph, Z_sph] = sphere(40);
        n_obs = size(x_objects, 2);
        for o = 1:n_obs
            xc = x_objects(1,o);
            yc = x_objects(2,o);
            zc = x_objects(3,o);
            r  = radii_obstacles(o);

            if o == 1
                obstacle_visibility = 'on';
            else
                obstacle_visibility = 'off';
            end

            surf(ax1, r*X_sph + xc, r*Y_sph + yc, r*Z_sph + zc, ...
                'FaceColor', [0.6 0.6 0.6], ...
                'FaceAlpha', 0.5, ...
                'EdgeColor', 'none', ...
                'DisplayName', 'Static Obstacle', ...
                'HandleVisibility', obstacle_visibility);
        end
    end

    % Trajectories + start + goal
    for i = 1:n_robots
        idx = (i-1)*3 + (1:3);
        pos = x_plot(:, idx);

        plot3(ax1, pos(:,1), pos(:,2), pos(:,3), ...
            'Color', colors(i,:), 'LineWidth', 2, ...
            'DisplayName', sprintf('UAV %d', i));

        plot3(ax1, pos(1,1), pos(1,2), pos(1,3), 'o', ...
            'Color', colors(i,:), ...
            'MarkerFaceColor', 'w', ...
            'MarkerSize', 8, ...
            'HandleVisibility', 'off');

        if exist('xg', 'var')
            if size(xg, 2) == n_robots
                goal_i = xg(:,i);
            else
                goal_i = xg(i,:)';
            end

            plot3(ax1, goal_i(1), goal_i(2), goal_i(3), 'x', ...
                'Color', colors(i,:), ...
                'LineWidth', 3, ...
                'MarkerSize', 10, ...
                'HandleVisibility', 'off');
        end
    end

    legend(ax1, 'show', 'Location', 'northeastoutside', 'FontSize', 11);
    title(ax1, '\textbf{Swarm Navigation in Cluttered Environment}', 'FontSize', FS+2);
    view(ax1, -35, 25);
end

%% =========================
%  FIGURE 2: SAFETY DASHBOARD
% =========================
if DO_FIG2_SAFETY
    if ~exist('x_objects', 'var') || ~exist('radii_obstacles', 'var') || ~exist('radius_robots', 'var')
        error('Safety dashboard needs x_objects, radii_obstacles, radius_robots in workspace.');
    end

    f2 = figure('Name', 'Fig2_Safety_Dashboard', 'Color', 'w', 'Position', [100 50 900 700]);
    sgtitle(f2, '\textbf{Individual Drone Safety Monitor}', 'Interpreter', 'latex', 'FontSize', FS+4);

    y_limit_plot = 2;
    ax2 = gobjects(n_robots, 1);

    nrows = ceil(sqrt(n_robots));
    ncols = ceil(n_robots / nrows);

    for i = 1:n_robots
        ax2(i) = subplot(nrows, ncols, i, 'Parent', f2);
        hold(ax2(i), 'on');
        grid(ax2(i), 'on');
        box(ax2(i), 'on');

        % Background zones
        patch(ax2(i), ...
            [t_plot(1) t_plot(end) t_plot(end) t_plot(1)], ...
            [0 0 -0.5 -0.5], ...
            [1 0.8 0.8], ...
            'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');

        patch(ax2(i), ...
            [t_plot(1) t_plot(end) t_plot(end) t_plot(1)], ...
            [0 0 0.3 0.3], ...
            [1 1 0.8], ...
            'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');

        % Min clearance over time
        min_dist_i = inf(size(t_plot));
        pos_i = x_plot(:, (i-1)*3 + (1:3));

        n_obs = size(x_objects, 2);
        for k = 1:length(t_plot)
            current_p = pos_i(k, :);
            for o = 1:n_obs
                d = norm(current_p - x_objects(:,o)') - radii_obstacles(o) - radius_robots;
                if d < min_dist_i(k)
                    min_dist_i(k) = d;
                end
            end
        end

        plot(ax2(i), t_plot, min_dist_i, 'Color', colors(i,:), 'LineWidth', 2);
        yline(ax2(i), 0, 'r-', 'LineWidth', 2);

        title(ax2(i), sprintf('\\textbf{UAV %d}', i), 'Interpreter', 'latex', 'FontSize', FS);

        if i > ncols
            xlabel(ax2(i), 'Time $[s]$', 'Interpreter', 'latex', 'FontSize', FS);
        end
        if mod(i-1, ncols) == 0
            ylabel(ax2(i), 'Clearance $[m]$', 'Interpreter', 'latex', 'FontSize', FS);
        end

        xlim(ax2(i), [t_plot(1) t_plot(end)]);
        ylim(ax2(i), [-0.1 y_limit_plot]);

        text(ax2(i), t_plot(1)+0.5, 0.15, '\textbf{Warning Zone}', ...
            'Color', [0.6 0.4 0], 'FontSize', 8, 'Interpreter', 'latex');

        text(ax2(i), t_plot(1)+0.5, -0.05, '\textbf{COLLISION}', ...
            'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold', 'Interpreter', 'latex');
    end
end

%% =========================
%  FIGURE 3: CONTROL INPUTS (UAV 1)
% =========================
if DO_FIG3_INPUTS
    if ~exist('max_acc', 'var')
        warning('max_acc not found: limits will not be drawn correctly.');
        max_acc_local = NaN;
    else
        max_acc_local = max_acc;
    end

    f3 = figure('Name', 'Fig3_Control_Feasibility', 'Color', 'w', 'Position', [150 100 700 600]);

    dt = t_plot(2) - t_plot(1);
    v_num = [zeros(1, size(x_plot,2)); diff(x_plot)] / dt;
    a_num = [zeros(1, size(x_plot,2)); diff(v_num)] / dt;
    acc_d1 = a_num(:, 1:3);

    % Smoother without weird axis side effects
    acc_d1_filt = movmean(acc_d1, 15, 1);

    ax31 = subplot(2,1,1, 'Parent', f3);
    hold(ax31, 'on');
    grid(ax31, 'on');

    plot(ax31, t_plot, acc_d1_filt(:,1), 'LineWidth', 1, 'DisplayName', '$a_x$');
    plot(ax31, t_plot, acc_d1_filt(:,2), 'LineWidth', 1, 'DisplayName', '$a_y$');
    plot(ax31, t_plot, acc_d1_filt(:,3), 'LineWidth', 1, 'DisplayName', '$a_z$');

    if ~isnan(max_acc_local)
        yline(ax31, max_acc_local, 'k--', 'Label', 'Limit');
        yline(ax31, -max_acc_local, 'k--');
    end

    ylabel(ax31, 'Acc. $[m/s^2]$', 'FontSize', FS);
    title(ax31, '\textbf{Control Inputs (UAV 1)}', 'FontSize', FS);
    legend(ax31, 'show');
    xlim(ax31, [t_plot(1) t_plot(end)]);

    g = 9.81;
    phi_est = -acc_d1_filt(:,2) / g;
    theta_est = acc_d1_filt(:,1) / g;

    ax32 = subplot(2,1,2, 'Parent', f3);
    hold(ax32, 'on');
    grid(ax32, 'on');

    plot(ax32, t_plot, rad2deg(phi_est), 'LineWidth', 1, 'DisplayName', 'Roll ($\phi$)');
    plot(ax32, t_plot, rad2deg(theta_est), 'LineWidth', 1, 'DisplayName', 'Pitch ($\theta$)');
    yline(ax32, 30, 'k:', 'Label', 'Small Angle Approx.');
    yline(ax32, -30, 'k:');

    ylabel(ax32, 'Attitude $[deg]$', 'FontSize', FS);
    xlabel(ax32, 'Time $[s]$', 'FontSize', FS);
    title(ax32, '\textbf{Estimated Attitude}', 'FontSize', FS);
    xlim(ax32, [t_plot(1) t_plot(end)]);
    legend(ax32, 'show');
end

%% =========================
%  FIGURE 4: TOPOLOGY (Active vs Unconnected)
% =========================
if DO_FIG4_TOPOLOGY
    if ~exist('Adj_mat', 'var') || ~exist('Rc', 'var')
        error('Topology plot needs Adj_mat and Rc in workspace.');
    end

    f4 = figure('Name', 'Fig4_Topology_Split', 'Color', 'w', 'Position', [150 50 800 800]);

    % Active links
    ax41 = subplot(2,1,1, 'Parent', f4);
    hold(ax41, 'on');
    grid(ax41, 'on');
    box(ax41, 'on');

    yline(ax41, Rc, 'r--', 'LineWidth', 2, 'DisplayName', 'Max Range $R_c$');

    for i = 1:n_robots
        for j = i+1:n_robots
            if Adj_mat(i,j) == 1
                dist_ij = sqrt(sum((x_plot(:, (i-1)*3+(1:3)) - x_plot(:, (j-1)*3+(1:3))).^2, 2));
                plot(ax41, t_plot, dist_ij, 'LineWidth', 2, 'DisplayName', sprintf('Link D%d-D%d', i, j));
            end
        end
    end

    ylabel(ax41, 'Distance $[m]$', 'Interpreter', 'latex', 'FontSize', FS);
    title(ax41, '\textbf{Active Comm. Links (Must be $< R_c$)}', 'Interpreter', 'latex', 'FontSize', FS);
    legend(ax41, 'Location', 'eastoutside', 'FontSize', 10);
    xlim(ax41, [t_plot(1) t_plot(end)]);
    ylim(ax41, [0 Rc*1.5]);

    % Unconnected pairs
    ax42 = subplot(2,1,2, 'Parent', f4);
    hold(ax42, 'on');
    grid(ax42, 'on');
    box(ax42, 'on');

    yline(ax42, Rc, 'g--', 'LineWidth', 2, 'DisplayName', 'Ref. Range $R_c$');

    for i = 1:n_robots
        for j = i+1:n_robots
            if Adj_mat(i,j) == 0
                dist_ij = sqrt(sum((x_plot(:, (i-1)*3+(1:3)) - x_plot(:, (j-1)*3+(1:3))).^2, 2));
                plot(ax42, t_plot, dist_ij, 'LineWidth', 1.5, 'LineStyle', '-', 'DisplayName', sprintf('Pair D%d-D%d', i, j));
            end
        end
    end

    xlabel(ax42, 'Time $[s]$', 'Interpreter', 'latex', 'FontSize', FS);
    ylabel(ax42, 'Distance $[m]$', 'Interpreter', 'latex', 'FontSize', FS);
    title(ax42, '\textbf{Unconnected Pairs (Allowed to Break Formation)}', 'Interpreter', 'latex', 'FontSize', FS);
    legend(ax42, 'Location', 'eastoutside', 'FontSize', 10);
    xlim(ax42, [t_plot(1) t_plot(end)]);
end

%% =========================
%  FIGURE 5: 3D ANIMATION / VIDEO
% =========================
if DO_FIG5_ANIM
    disp('Generazione animazione in corso...');

    f5 = figure('Name', 'Fig5_Animation', 'Color', 'w', 'Position', [200 150 900 700]);
    ax5 = axes('Parent', f5);
    hold(ax5, 'on');
    grid(ax5, 'on');
    axis(ax5, 'equal');
    view(ax5, -35, 25);

    xlabel(ax5, 'X $[m]$', 'FontSize', FS);
    ylabel(ax5, 'Y $[m]$', 'FontSize', FS);
    zlabel(ax5, 'Z $[m]$', 'FontSize', FS);

    % --- 1. Draw obstacles and goals ---
    if exist('x_objects', 'var') && exist('radii_obstacles', 'var')
        [X_sph, Y_sph, Z_sph] = sphere(40);
        n_obs = size(x_objects, 2);
        for o = 1:n_obs
            xc = x_objects(1,o);
            yc = x_objects(2,o);
            zc = x_objects(3,o);
            r  = radii_obstacles(o);

            surf(ax5, r*X_sph + xc, r*Y_sph + yc, r*Z_sph + zc, ...
                'FaceColor', [0.6 0.6 0.6], ...
                'FaceAlpha', 0.5, ...
                'EdgeColor', 'none');
        end
    else
        warning('x_objects or radii_obstacles not found: obstacles will not be drawn.');
    end

    if exist('xg', 'var')
        for i = 1:n_robots
            if size(xg, 2) == n_robots
                goal_i = xg(:,i);
            else
                goal_i = xg(i,:)';
            end

            plot3(ax5, goal_i(1), goal_i(2), goal_i(3), 'x', ...
                'Color', colors(i,:), ...
                'LineWidth', 3, ...
                'MarkerSize', 10);
        end
    else
        warning('xg not found: goals will not be drawn.');
    end

    % --- 2. Fix axis limits ---
    x_all = x_plot(:,1:3:end);
    y_all = x_plot(:,2:3:end);
    z_all = x_plot(:,3:3:end);

    xmin = min(x_all(:));
    xmax = max(x_all(:));
    ymin = min(y_all(:));
    ymax = max(y_all(:));
    zmax = max(z_all(:));

    xlim(ax5, [xmin - 0.5, xmax + 0.5]);
    ylim(ax5, [ymin - 0.5, ymax + 0.5]);
    zlim(ax5, [0, max(zmax, 2) + 0.5]);

    % --- 3. Init plot objects ---
    h_drones = gobjects(n_robots, 1);
    h_tails  = gobjects(n_robots, 1);

    for i = 1:n_robots
        idx = (i-1)*3 + (1:3);
        tail_color = 0.5 * colors(i,:) + 0.5 * [1 1 1];

        h_tails(i) = plot3(ax5, ...
            x_plot(1, idx(1)), x_plot(1, idx(2)), x_plot(1, idx(3)), ...
            '-', 'Color', tail_color, 'LineWidth', 1.5);

        h_drones(i) = plot3(ax5, ...
            x_plot(1, idx(1)), x_plot(1, idx(2)), x_plot(1, idx(3)), ...
            'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 8);
    end

    % --- 4. Setup VideoWriter ---
    video_enabled = false;

    if SAVE_VIDEO
        if ~exist(SAVE_DIR, 'dir')
            mkdir(SAVE_DIR);
        end

        timestamp = datestr(now, 'yyyymmdd_HHMMSS');
        video_filename = fullfile(SAVE_DIR, ...
            ['Animation_', SCENARIO_TAG, '_', timestamp, FILE_TAG, '.mp4']);

        try
            v = VideoWriter(video_filename, 'MPEG-4');
            v.FrameRate = 30;
            open(v);
            video_enabled = true;
        catch ME
            warning('Cannot create video file:\n%s\nReason: %s', video_filename, ME.message);
            video_enabled = false;
        end
    end

    % --- 5. Animation parameters ---
    dt = t_plot(2) - t_plot(1);
    target_fps = 30;
    step = max(1, round(1 / (target_fps * dt)));

    % --- 6. Animation loop ---
    try
        for k = 1:step:length(t_plot)
            for i = 1:n_robots
                idx = (i-1)*3 + (1:3);

                set(h_tails(i), ...
                    'XData', x_plot(1:k, idx(1)), ...
                    'YData', x_plot(1:k, idx(2)), ...
                    'ZData', x_plot(1:k, idx(3)));

                set(h_drones(i), ...
                    'XData', x_plot(k, idx(1)), ...
                    'YData', x_plot(k, idx(2)), ...
                    'ZData', x_plot(k, idx(3)));
            end

            title(ax5, sprintf('\\textbf{Swarm Navigation} (t = %.2f s)', t_plot(k)), ...
                'FontSize', FS+2);

            drawnow;

            if video_enabled
                frame = getframe(f5);
                writeVideo(v, frame);
            end
        end

        % Force exact last frame
        for i = 1:n_robots
            idx = (i-1)*3 + (1:3);

            set(h_tails(i), ...
                'XData', x_plot(:, idx(1)), ...
                'YData', x_plot(:, idx(2)), ...
                'ZData', x_plot(:, idx(3)));

            set(h_drones(i), ...
                'XData', x_plot(end, idx(1)), ...
                'YData', x_plot(end, idx(2)), ...
                'ZData', x_plot(end, idx(3)));
        end

        title(ax5, sprintf('\\textbf{Swarm Navigation} (t = %.2f s)', t_plot(end)), ...
            'FontSize', FS+2);
        drawnow;

        if video_enabled
            frame = getframe(f5);
            writeVideo(v, frame);
        end

    catch ME
        if video_enabled
            try
                close(v);
            catch
            end
        end
        rethrow(ME);
    end

    % --- 7. Close video ---
    if video_enabled
        close(v);
        disp(['Video salvato con successo in: ', video_filename]);
    end
end

%% =========================
%  SAVING
% =========================
if SAVE_FIGS
    if ~exist(SAVE_DIR, 'dir')
        mkdir(SAVE_DIR);
    end

    try
        if exist('f1', 'var') && isgraphics(f1)
            exportgraphics(f1, fullfile(SAVE_DIR, ...
                ['Fig1_Traj_', SCENARIO_TAG, FILE_TAG, '.pdf']), ...
                'ContentType', 'vector');
        end

        if exist('f2', 'var') && isgraphics(f2)
            exportgraphics(f2, fullfile(SAVE_DIR, ...
                ['Fig2_Safety_', SCENARIO_TAG, FILE_TAG, '.pdf']), ...
                'ContentType', 'vector');
        end

        if exist('f3', 'var') && isgraphics(f3)
            exportgraphics(f3, fullfile(SAVE_DIR, ...
                ['Fig3_Inputs_', SCENARIO_TAG, FILE_TAG, '.pdf']), ...
                'ContentType', 'vector');
        end

        if exist('f4', 'var') && isgraphics(f4)
            exportgraphics(f4, fullfile(SAVE_DIR, ...
                ['Fig4_Topology_', SCENARIO_TAG, FILE_TAG, '.pdf']), ...
                'ContentType', 'vector');
        end

        if exist('f5', 'var') && isgraphics(f5)
            exportgraphics(f5, fullfile(SAVE_DIR, ...
                ['Fig5_AnimationFrame_', SCENARIO_TAG, FILE_TAG, '.png']), ...
                'Resolution', 300);
        end

        disp(['Saved figures in ', SAVE_DIR]);

    catch ME
        warning('Error while saving figures in %s\nReason: %s', SAVE_DIR, ME.message);
    end
end




