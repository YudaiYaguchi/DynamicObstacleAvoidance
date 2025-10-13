function [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(o, prev_ob_pos, prev_ob_velocity, dt, steps)
%
% 入力:
%   o:       検知した障害物の座標群 [N×2]
%   prev_ob_pos: 前回の障害物位置 [1×2]
%   prev_ob_velocity: 前回の障害物速度 [1×2]
%   dt:          シミュレーションの時間刻み
%   steps:       予測したい未来ステップ数
%
% 出力:
%   current_pos:       現在の障害物位置 [1×2]（重心）
%   predicted_pos:     予測位置 [1×2]
%   current_velocity:  速度 [1×2]
%   current_acceleration: 加速度 [1×2]

    if isempty(o)
      fprintf('障害物データが空です。予測できません。\n');
      current_pos = [NaN, NaN];
      predicted_pos = [NaN, NaN];
      current_velocity = [0, 0];
      current_acceleration = [0, 0];
      return;
    end
    disp("前回速度");
    disp(prev_ob_velocity);
    % disp("前回位置");
    % disp(prev_ob_pos);

    % --- 現在位置を重心で代表させる ---
    current_pos = mean(o, 1);  % 各列(x,y)の平均 → [1×2]

    if ~isempty(prev_ob_pos)
        % --- 速度・加速度を計算 ---
        current_velocity = (current_pos - prev_ob_pos) / dt;
        current_acceleration = (current_velocity - prev_ob_velocity) / dt;

        % --- 未来位置を等加速度運動で予測 ---
        T = steps * dt;
        predicted_pos = current_pos + current_velocity * T + 0.5 * current_acceleration * T^2;

        % --- ログ出力 ---
        fprintf('現在位置: (%.4f, %.4f)\n', current_pos(1), current_pos(2));
        fprintf('予測位置 (steps=%d): (%.4f, %.4f)\n', steps, predicted_pos(1), predicted_pos(2));
        fprintf('速度: (%.4f, %.4f)\n', current_velocity(1), current_velocity(2));
        fprintf('加速度: (%.4f, %.4f)\n\n', current_acceleration(1), current_acceleration(2));
    else
        % --- 初回時 ---
        fprintf('初回は予測できない\n');
        current_velocity = [0, 0];
        current_acceleration = [0, 0];
        predicted_pos = current_pos;

        fprintf('現在位置: (%.2f, %.2f)\n', current_pos(1), current_pos(2));
        fprintf('予測位置 (steps=%d): (%.2f, %.2f)\n', steps, predicted_pos(1), predicted_pos(2));
    end
end
