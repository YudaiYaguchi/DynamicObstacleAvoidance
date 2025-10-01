function [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(ob_mv, prev_ob_pos, prev_ob_velocity, dt, steps)
%
% 入力:
%   ob_mv:       現在の障害物の座標群 (センサで得られた座標点群など)
%   prev_ob_pos: 前回の障害物位置
%   prev_ob_velocity: 前回の障害物速度
%   dt:          シミュレーションの時間刻み (1ステップの経過時間)
%   steps:       予測したい未来ステップ数
%
% 出力:
%   current_pos:       現在の障害物位置 [x; y]
%   predicted_pos:     steps ステップ後に予測される障害物位置 [x; y]
%   current_velocity:  最新の障害物速度 [vx; vy]
%   current_acceleration: 最新の障害物加速度 [ax; ay]

% --- 現在位置を代表値で取得 (重心など)
current_pos = [mean(ob_mv(1,:)); mean(ob_mv(2,:))];

if ~isempty(prev_ob_pos)
  % 現在の速度を計算
  current_velocity = (current_pos - prev_ob_pos) / dt;

  % 現在の加速度を計算
  current_acceleration = (current_velocity - prev_ob_velocity) / dt;

  % 任意ステップ後の時間
  T = steps * dt;

  % 等加速度運動の式で未来位置を予測
  predicted_pos = current_pos + current_velocity * T + 0.5 * current_acceleration * T^2;

  fprintf('現在位置: (%.2f, %.2f)\n', current_pos(1), current_pos(2));
  fprintf('予測位置 (steps=%d): (%.2f, %.2f)\n', steps, predicted_pos(1), predicted_pos(2));
  fprintf('速度: (%.4f, %.4f)\n', current_velocity(1), current_velocity(2));
  fprintf('加速度: (%.4f, %.4f)\n\n', current_acceleration(1), current_acceleration(2));
else
  % 初回は予測できないので現位置をそのまま返す
  fprintf('初回は予測できない\n');
  current_velocity = [0; 0];
  current_acceleration = [0; 0];
  predicted_pos = current_pos;
  fprintf('現在位置: (%.2f, %.2f)\n', current_pos(1), current_pos(2));
  fprintf('予測位置 (steps=%d): (%.2f, %.2f)\n', steps, predicted_pos(1), predicted_pos(2));
  fprintf('速度: (%.4f, %.4f)\n', current_velocity(1), current_velocity(2));
  fprintf('加速度: (%.4f, %.4f)\n\n', current_acceleration(1), current_acceleration(2));
end
end
