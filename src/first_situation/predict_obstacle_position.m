% function [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(o, prev_ob_pos, prev_ob_velocity, dt, steps, current_velocity, current_acceleration)
% %
% % 入力:
% %   o:                 検知した障害物の座標群 [N×2] または [1×2]
% %   prev_ob_pos:       前回の障害物位置 [1×2]
% %   prev_ob_velocity:  前回の障害物速度 [1×2]
% %   dt:                時間刻み
% %   steps:             予測ステップ数
% %
% % 出力:
% %   current_pos:       現在位置 [1×2]
% %   predicted_pos:     予測位置 [steps×2]
% %   current_velocity:  現在速度 [1×2]
% %   current_acceleration: 現在加速度 [1×2]

%     if isempty(o)
%         current_pos = [NaN, NaN];
%         predicted_pos = NaN(steps, 2);
%         current_velocity = [0, 0];
%         current_acceleration = [0, 0];
%         return;
%     end

%     % --- 入力を [1×2] ベクトルに整形 ---
%     if size(o, 2) == 1
%         o = o';
%     end

%     % --- 現在位置を重心で代表 ---
%     current_pos = mean(o, 1);  % [1×2]

%     % --- 結果格納用 ---
%     predicted_pos = zeros(steps, 2);

%     if ~isempty(prev_ob_pos)
%         % --- 等加速度運動モデルによる予測 ---
%         for k = 1:steps
%             T = k * dt;
%             predicted_pos(k, :) = current_pos + current_velocity * T + 0.5 * current_acceleration * (T^2);
%         end

%         % --- ログ出力 ---
%         fprintf('現在位置: (%.4f, %.4f)\n', current_pos(1), current_pos(2));
%         fprintf('--- %d ステップ先までの予測位置 ---\n', steps);
%         disp(predicted_pos);

%         fprintf('速度: (%.4f, %.4f)\n', current_velocity(1), current_velocity(2));
%         fprintf('加速度: (%.4f, %.4f)\n\n', current_acceleration(1), current_acceleration(2));
%     else
%         % --- 初回時 ---
%         fprintf('初回は予測できない\n');
%         current_velocity = [0, 0];
%         current_acceleration = [0, 0];
%         for k = 1:steps
%             predicted_pos(k, :) = current_pos; % 全ステップ同じ位置
%         end
%         fprintf('現在位置: (%.4f, %.4f)\n', current_pos(1), current_pos(2));
%     end
% end

function [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(o, prev_ob_pos, prev_ob_velocity, dt, steps, current_velocity, current_acceleration)
    %
    % 入力:
    %   o:                 検知した障害物の座標群 [N×2] または [1×2]
    %   prev_ob_pos:       前回の障害物位置 [1×2]
    %   prev_ob_velocity:  前回の障害物速度 [1×2]
    %   dt:                時間刻み
    %   steps:             予測ステップ数
    %
    % 出力:
    %   current_pos:       現在位置 [1×2]
    %   predicted_pos:     予測位置 [steps×2]
    %   current_velocity:  現在速度 [1×2]
    %   current_acceleration: 現在加速度 [1×2]
    
        if isempty(o)
            current_pos = [NaN, NaN];
            predicted_pos = NaN(steps, 2);
            current_velocity = [0, 0];
            current_acceleration = [0, 0];
            return;
        end
    
        % --- 入力を [1×2] に整形 ---
        o = mean(o, 1);
        current_pos = o(:).'; % 行ベクトルに統一
    
        % --- 出力用 ---
        predicted_pos = zeros(steps, 2);
    
        % --- 速度・加速度も行ベクトルに統一 ---
        current_velocity = current_velocity(:).';
        current_acceleration = current_acceleration(:).';
        prev_ob_pos = prev_ob_pos(:).';
        prev_ob_velocity = prev_ob_velocity(:).';
    
        if ~isempty(prev_ob_pos)
            % --- 等加速度運動モデルによる予測 ---
            for k = 1:steps
                T = k * dt;
                predicted_pos(k, :) = current_pos + current_velocity * T + 0.5 * current_acceleration * (T^2);
            end
    
            % --- ログ出力 ---
            fprintf('現在位置: (%.4f, %.4f)\n', current_pos(1), current_pos(2));
            fprintf('--- %d ステップ先までの予測位置 ---\n', steps);
            disp(predicted_pos);
    
            fprintf('速度: (%.4f, %.4f)\n', current_velocity(1), current_velocity(2));
            fprintf('加速度: (%.4f, %.4f)\n\n', current_acceleration(1), current_acceleration(2));
        else
            % --- 初回時 ---
            fprintf('初回は予測できない\n');
            current_velocity = [0, 0];
            current_acceleration = [0, 0];
            predicted_pos(:,:) = repmat(current_pos, steps, 1); % 全ステップ同じ位置
            fprintf('現在位置: (%.4f, %.4f)\n', current_pos(1), current_pos(2));
        end
    end
    