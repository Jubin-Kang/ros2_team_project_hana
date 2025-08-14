%% Clear workspace, close all figures, and clear command window
clear; close all; clc;

%% --- Configuration ---
% Set the directory where your CSV files are located
data_dir = 'data/';

% List of estimation files to process
est_files = {
  'raw_output.csv',
  'clahe_output.csv',
  'bilateral_output.csv',
  'clahe_bilateral_output.csv'
};

% Define plot styles and legends for better visualization
plot_styles = {'-ko', '-rx', '-g.', '-b*'};
plot_legends = {'Raw', 'CLAHE', 'Bilateral', 'CLAHE+Bilateral'};

% Define plot styles for Figure 1 (no lines, only markers)
plot_styles_figure1 = {'ko', 'rx', 'g.', 'b*'};

%% --- Load Data ---

% Load all estimation data files (timestamp, x, y, yaw)
est_data = {};
for i = 1:length(est_files)
  file_path = fullfile(data_dir, est_files{i});
  if (exist(file_path, 'file'))
    printf("Loading estimation data from: %s\n", file_path);
    est_data{i} = dlmread(file_path, ',', 1, 0);

  else
    printf('Warning: Estimation file not found: %s\n', file_path);
    est_data{i} = []; % Use an empty matrix as a placeholder
  end
end

%% --- Calculate Standard Deviations and Means ---
printf('\n--- Statistics (Mean and Standard Deviation) ---\n');

best_x_std = inf;
best_y_std = inf;
best_yaw_std = inf;
best_x_dataset = '';
best_y_dataset = '';
best_yaw_dataset = '';

% Store means and stds for plotting ellipses
all_x_means = zeros(1, length(est_data));
all_y_means = zeros(1, length(est_data));
all_x_stds = zeros(1, length(est_data));
all_y_stds = zeros(1, length(est_data));

for i = 1:length(est_data)
  if (~isempty(est_data{i}))
    current_data = est_data{i};

    x_mean = mean(current_data(:,2));
    y_mean = mean(current_data(:,3));
    yaw_mean = mean(current_data(:,4));

    x_std = std(current_data(:,2));
    y_std = std(current_data(:,3));
    yaw_std = std(current_data(:,4));

    printf('Dataset: %s\n', plot_legends{i});
    printf('  X Mean: %.6f, X Std Dev: %.6f\n', x_mean, x_std);
    printf('  Y Mean: %.6f, Y Std Dev: %.6f\n', y_mean, y_std);
    printf('  Yaw Mean: %.6f, Yaw Std Dev: %.6f\n', rad2deg(yaw_mean), rad2deg(yaw_std));
    printf('\n');

    % Store for plotting
    all_x_means(i) = x_mean;
    all_y_means(i) = y_mean;
    all_x_stds(i) = x_std;
    all_y_stds(i) = y_std;

    % Update best precision
    if x_std < best_x_std
      best_x_std = x_std;
      best_x_dataset = plot_legends{i};
    end
    if y_std < best_y_std
      best_y_std = y_std;
      best_y_dataset = plot_legends{i};
    end
    if yaw_std < best_yaw_std
      best_yaw_std = yaw_std;
      best_yaw_dataset = plot_legends{i};
    end

  else
    printf('Dataset: %s - No data available for statistics calculation.\n\n', plot_legends{i});
  end
end

printf('--- Highest Precision (Lowest Standard Deviation) ---');
printf('  X-axis: %s (Std Dev: %.6f)\n', best_x_dataset, best_x_std);
printf('  Y-axis: %s (Std Dev: %.6f)\n', best_y_dataset, best_y_std);
printf('  Yaw: %s (Std Dev: %.6f)\n', best_yaw_dataset, rad2deg(best_yaw_std));
printf('\n');

%% --- Normalize Data for Plotting ---
for i = 1:length(est_data)
    if ~isempty(est_data{i})
      est_data{i}(:,2) = est_data{i}(:,2) - mean(est_data{i}(:,2)); % Normalize X
      est_data{i}(:,3) = est_data{i}(:,3) - mean(est_data{i}(:,3)); % Normalize Y
      est_data{i}(:,4) = est_data{i}(:,4) - mean(est_data{i}(:,4)); % Normalize Yaw
    end
end

%% --- Plotting Results ---


% Plot 1: 2D Trajectory (X-Y Plane)
figure('Name', '2D Trajectory');
hold on;
for i = 1:length(est_data)
  if (~isempty(est_data{i}))
    if i == 3
      plot(est_data{i}(:,2), est_data{i}(:,3), plot_styles_figure1{i}, 'LineWidth', 1.5, 'MarkerSize', 18);
    else
      plot(est_data{i}(:,2), est_data{i}(:,3), plot_styles_figure1{i}, 'LineWidth', 1.5);
    end
  end
end



hold off;
title('2D Trajectory Comparison', 'FontSize', 30);
xlabel('X Position (m)', 'FontSize', 20);
ylabel('Y Position (m)', 'FontSize', 20);
set(gca, 'FontSize', 15);
legend(plot_legends, 'Location', 'northeast');
axis([-0.002 0.005 -0.04 0.02])
grid on;

% Plot 2: X Position vs. N
figure('Name', 'X Position vs. Image Frame');
hold on;
for i = 1:length(est_data)
  if (~isempty(est_data{i}))
    n_points = size(est_data{i}, 1);
    if i == 3
      plot(0:n_points-1, est_data{i}(:,2), plot_styles{i}, 'LineWidth', 1.5, 'MarkerSize', 18);
    else
      plot(0:n_points-1, est_data{i}(:,2), plot_styles{i}, 'LineWidth', 1.5);
    end
  end
end
hold off;
title('X Position vs. Image Frame', 'FontSize', 30);
xlabel('Image Frame (N)', 'FontSize', 20);
ylabel('X Position (m)', 'FontSize', 20);
set(gca, 'FontSize', 15);
legend(plot_legends, 'Location', 'northeast');
axis([0 300 -0.002 0.005])
grid on;

% Plot 3: Y Position vs. N
figure('Name', 'Y Position vs. Image Frame');
hold on;
for i = 1:length(est_data)
  if (~isempty(est_data{i}))
    n_points = size(est_data{i}, 1);
    if i == 3
      plot(0:n_points-1, est_data{i}(:,3), plot_styles{i}, 'LineWidth', 1.5, 'MarkerSize', 18);
    else
      plot(0:n_points-1, est_data{i}(:,3), plot_styles{i}, 'LineWidth', 1.5);
    end
  end
end
hold off;
title('Y Position vs. Image Frame', 'FontSize', 30);
xlabel('Image Frame (N)', 'FontSize', 20);
ylabel('Y Position (m)', 'FontSize', 20);
set(gca, 'FontSize', 15);
legend(plot_legends, 'Location', 'northeast');
axis([0 300 -0.04 0.02])
grid on;

% Plot 4: Yaw vs. N
figure('Name', 'Yaw vs. Image Frame');
hold on;
for i = 1:length(est_data)
  if (~isempty(est_data{i}))
    n_points = size(est_data{i}, 1);
    if i == 3
      plot(0:n_points-1, rad2deg(est_data{i}(:,4)), plot_styles{i}, 'LineWidth', 1.5, 'MarkerSize', 18);
    else
      plot(0:n_points-1, rad2deg(est_data{i}(:,4)), plot_styles{i}, 'LineWidth', 1.5);
    end
  end
end
hold off;
title('Yaw vs. Image Frame', 'FontSize', 30);
xlabel('Image Frame (N)', 'FontSize', 20);
ylabel('Yaw (rad)', 'FontSize', 20);
set(gca, 'FontSize', 15);
legend(plot_legends, 'Location', 'northeast'); % Now plot_legends only contains the estimation legends
axis([0 300 -3 6])
grid on;

printf("\nPlotting complete.\n");
