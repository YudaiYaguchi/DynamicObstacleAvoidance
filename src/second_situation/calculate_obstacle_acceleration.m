function [acceleration_x, acceleration_y] = calculate_obstacle_acceleration(current_ob_pos, prev_ob_pos, time_step, prev_velocity)
    % この関数は、動的障害物の加速度を計算します。
    %
    % 入力:
    %   current_ob_pos: 現在の障害物の座標 [x, y]
    %   prev_ob_pos: 前の時点の障害物の座標 [x, y]
    %   time_step: 時間ステップ（サンプリング間隔）。ここでは1サンプルごとに1とします。
    %   prev_velocity: 前の時点の障害物の速度ベクトル [vx, vy]
    %
    % 出力:
    %   acceleration_x: 障害物のx方向の加速度
    %   acceleration_y: 障害物のy方向の加速度

    if isempty(prev_ob_pos) || isempty(prev_velocity)
        acceleration_x = 0;
        acceleration_y = 0;
        return;
    end

    % 速度を計算
    current_velocity_x = (current_ob_pos(1) - prev_ob_pos(1)) / time_step;
    current_velocity_y = (current_ob_pos(2) - prev_ob_pos(2)) / time_step;

    % 加速度を計算
    acceleration_x = (current_velocity_x - prev_velocity(1)) / time_step;
    acceleration_y = (current_velocity_y - prev_velocity(2)) / time_step;
end