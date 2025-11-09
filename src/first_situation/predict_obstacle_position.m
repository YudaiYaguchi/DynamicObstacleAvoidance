function predicted_pos = predict_obstacle_position(latest_obstacle, dt, current_velocity, current_acceleration)
%
% 入力:
%   latest_obstacle:   ロボットと最も近い障害物の座標群 [N×2] または [1×2]
%   dt:                時間刻み
%   current_velocity:  現在速度 [2×1]
%   current_acceleration: 現在加速度 [2×1]
%
% 出力:
%   predicted_pos:     予測位置 [steps×2]
%   current_velocity:  現在速度 [1×2]
%   current_acceleration: 現在加速度 [1×2]

if any(abs(current_acceleration) > 1e-6)
    % どれか1つでも加速度がある
    steps = 20;
else % すべてほぼ0 → 等速
    steps = 50; % 障害物が目の前を通過する場合
end

if isempty(latest_obstacle)
    return;
end

% --- 入力を [1×2] に整形 ---
latest_obstacle = mean(latest_obstacle, 1);
current_pos = latest_obstacle(:).'; % 行ベクトルに統一

% --- 速度・加速度も行ベクトルに統一 ---
current_velocity = current_velocity(:).';
current_acceleration = current_acceleration(:).';

idx = 1;
% --- 等加速度運動モデルによる予測 ---
for k = 1:steps
    if mod(k, 2) == 0 % 計算量を減らす為 steps の半分だけ等間隔で予測
        T = k * dt;
        predicted_pos(idx, :) = current_pos + current_velocity * T + 0.5 * current_acceleration * (T ^ 2);
        idx = idx + 1;
    end
end
% --- ログ出力 ---
fprintf('障害物の現在位置: (%.4f, %.4f)\n', current_pos(1), current_pos(2));
fprintf('障害物の速度: (%.4f, %.4f)\n', current_velocity(1), current_velocity(2));
fprintf('障害物の加速度: (%.4f, %.4f)\n\n', current_acceleration(1), current_acceleration(2));
fprintf('実際の予測している点の数: %d \n', length(predicted_pos));
fprintf('--- %d ステップ先までの予測位置 ---\n', steps);
disp(predicted_pos);

end
