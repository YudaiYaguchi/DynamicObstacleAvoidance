% ロボットが障害物に向かって移動し、距離をログに記録するプログラム

clear;
clc;

% センサーの設定
range_sensor = rangeSensor;
range_sensor.Range = [0.0 5.0]; % 距離範囲（0m～5m）
% range_sensor.HorizontalAngle = [-pi/4 pi/4]; % 視野角（-45度～45度）
range_sensor.HorizontalAngle = [-pi pi]; % 視野角 360°
% マップ生成（障害物を含む）
occupancy_map = binaryOccupancyMap(10, 10, 10);
occupancy_map.GridLocationInWorld = [0 0];
setOccupancy(occupancy_map, [5 5], 1); % 障害物を配置
inflate(occupancy_map, 0.2); % 障害物を膨張させる
% show(occupancy_map); % ← 障害物の黒塗り表示はしない
hold on;

% 描画範囲を 0～8 に固定
axis([0 8 0 8]);
grid on; % グリッドを表示
xlabel('X [m]');
ylabel('Y [m]');

% ロボットの初期位置
robot_x = 2;
robot_y = 2;
robot_heading = atan2(5 - robot_y, 5 - robot_x); % 障害物に向かう角度を計算

% 初期位置を青丸で表示（塗りつぶしなし）
plot(robot_x, robot_y, 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);

% ゴール（障害物の位置）をオレンジ丸で表示（塗りつぶしなし）
plot(5, 5, 'o', 'Color', [1, 0.5, 0], 'MarkerSize', 8, 'LineWidth', 1.5);

% ログファイルの準備
log_file = fopen('lidar_distance_log.txt', 'w');
fprintf(log_file, 'Time(s), Distance(m)\n');

% シミュレーション時間
simulation_time = 20; % 秒
time_step = 0.1; % 時間ステップ

% ロボット軌跡用の記録
trajectory_x = [];
trajectory_y = [];

for t = 0:time_step:simulation_time
  % ロボットの現在位置と向き
  truePose = [robot_x robot_y robot_heading];
  
  % LiDARセンサーでスキャン
  [ranges, angles] = range_sensor(truePose, occupancy_map);
  scan = lidarScan(ranges, angles);
  
  % 障害物との距離を取得
  distance_to_obstacle = min(ranges); % 最短距離を取得
  
  % 距離をログに記録
  if mod(t,1) == 0 % 1秒ごとにログを記録
      fprintf(log_file, 'Time: %.2f s, Distance to obstacle: %.2f m\n', t, distance_to_obstacle);
  end
  
  % 障害物に到達したら停止
  if distance_to_obstacle <= 0.05 % 障害物までの距離が0.1m未満なら停止
      disp('障害物に到達しました。');
      break;
  end
  
  % ロボットの移動（障害物に向かって直進）
  robot_x = robot_x + 0.05 * cos(robot_heading);
  robot_y = robot_y + 0.05 * sin(robot_heading);
  
  % 軌跡を記録
  trajectory_x(end+1) = robot_x;
  trajectory_y(end+1) = robot_y;
  
  % 描画更新（ロボットの軌跡）
  plot(trajectory_x, trajectory_y, 'b-', 'LineWidth', 1); % 軌跡（細い青線）
  pause(time_step);
end

% ログファイルを閉じる
fclose(log_file);
disp('ログ記録が完了しました。');

% ログファイルをExcel形式で保存
log_data = readtable('lidar_distance_log.txt'); % テキストファイルを読み込む
writetable(log_data, 'lidar_distance_log.xlsx'); % Excel形式で保存
disp('ログをExcel形式で保存しました。');

% 終了後10秒間ウィンドウを保持
pause(10);
