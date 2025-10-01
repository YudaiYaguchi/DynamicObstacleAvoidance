function [prev_ob_pos, prev_ob_velocity, ob_acceleration] = update_obstacle_acceleration(ob_mv, prev_ob_pos, prev_ob_velocity, dt)
%
% 入力:
%   ob_mv: 移動後の障害物座標
%   prev_ob_pos: 前の時点の障害物位置
%   prev_ob_velocity: 前の時点の障害物速度
%   dt: 経過時間
%
% 出力:
%   prev_ob_pos: 更新された障害物位置（次のステップ用）
%   prev_ob_velocity: 更新された障害物速度（次のステップ用）
%   ob_acceleration: 計算された加速度 [ax, ay]

current_ob_pos = [mean(ob_mv(1,:)); mean(ob_mv(2,:))];
if ~isempty(prev_ob_pos)
  % 現在の速度を計算
  current_velocity = (current_ob_pos - prev_ob_pos) / dt;

  % 加速度を計算
  ob_acceleration = (current_velocity - prev_ob_velocity) / dt;

  % 速度を更新
  prev_ob_velocity = current_velocity;
else
  ob_acceleration = [0; 0];
end
prev_ob_pos = current_ob_pos;
end
