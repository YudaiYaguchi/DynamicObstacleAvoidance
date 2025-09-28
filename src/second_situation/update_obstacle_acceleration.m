function [prev_ob_pos, prev_ob_velocity] = update_obstacle_acceleration(ob_mv, prev_ob_pos, prev_ob_velocity, time_step, time)
%
% 入力:
%   ob_mv: 移動後の障害物座標
%   prev_ob_pos: 前の時点の障害物位置
%   prev_ob_velocity: 前の時点の障害物速度
%   time_step: 時間ステップ
%   time: 現在の時間
%   acceleration_log: ログファイルハンドル
%
% 出力:
%   prev_ob_pos: 更新された障害物位置（次のステップ用）
%   prev_ob_velocity: 更新された障害物速度（次のステップ用）

current_ob_pos = [mean(ob_mv(1,:)); mean(ob_mv(2,:))];
if ~isempty(prev_ob_pos)
  [accel_x, accel_y] = calculate_obstacle_acceleration(current_ob_pos, prev_ob_pos, time_step, prev_ob_velocity);

  % 速度を更新
  prev_ob_velocity(1) = (current_ob_pos(1) - prev_ob_pos(1)) / time_step;
  prev_ob_velocity(2) = (current_ob_pos(2) - prev_ob_pos(2)) / time_step;
end
prev_ob_pos = current_ob_pos;
end
