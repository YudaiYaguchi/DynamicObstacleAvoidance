function detected_obstacles = update_obstacle_dynamic_status(detected_obstacles, obstacles, robot_x, robot_y)
    % update_obstacle_dynamic_status - 検出された障害物のisDynamicプロパティを更新
    %
    % 入力:
    %   detected_obstacles - 検出された障害物の構造体
    %                        .position: [N x 2] の検出位置行列
    %   obstacles - 障害物のリスト（構造体配列）
    %               各要素は .position, .isDynamic を持つ
    %   robot_x, robot_y - ロボットの座標
    %
    % 出力:
    %   detected_obstacles - 更新された detected_obstacles 構造体
    %
    % 処理内容:
    %   全ての検出点の中で最も近い障害物を見つけ、
    %   その障害物のisDynamicプロパティをdetected_obstacles.isDynamicに設定
    
    % 検出された障害物がない場合は何もしない
    if isempty(detected_obstacles.position)
        return;
    end
    
    % 全検出点と全障害物の中での最小距離を探索
    global_min_distance = inf;
    closest_isDynamic = false;
    closest_obstacle_idx = -1;
    closest_detected_pos = [];
    closest_obstacle_point = [];
    
    % detected_obstacles.position の行数を取得
    num_detected = size(detected_obstacles.position, 1);
    
    fprintf('\n[DEBUG] ========== 障害物判定開始 ==========\n');
    fprintf('[DEBUG] 検出点数: %d\n', num_detected);
    
    % 各検出された点について処理
    for i = 1:num_detected
        detected_pos = detected_obstacles.position(i, :);  % 検出位置 [x, y]
        fprintf('[DEBUG] 検出点#%d: (%.4f, %.4f)\n', i, detected_pos(1), detected_pos(2));
        
        % obstacles リスト内の各障害物との距離を計算
        for j = 1:length(obstacles)
            obstacle = obstacles(j);

            % obstacle.position は [2 x M] の形式（各列が障害物の頂点）
            % ロボットに最も近い頂点を用いて距離を評価
            if ~isempty(obstacle.position)
                robot_distances = sqrt((obstacle.position(1, :) - robot_x).^2 + ...
                                       (obstacle.position(2, :) - robot_y).^2);
                [~, min_robot_idx] = min(robot_distances);
                obstacle_point = [obstacle.position(1, min_robot_idx), obstacle.position(2, min_robot_idx)];

                distance_to_detected = norm(obstacle_point - detected_pos);

                fprintf('[DEBUG]   障害物#%d: ロボット最近点=(%.4f, %.4f), 距離=%.4f, isDynamic=%d\n', ...
                        j, obstacle_point(1), obstacle_point(2), distance_to_detected, obstacle.isDynamic);

                % これまでのグローバル最小距離より小さい場合は更新
                if distance_to_detected < global_min_distance
                    global_min_distance = distance_to_detected;
                    closest_isDynamic = obstacle.isDynamic;
                    closest_obstacle_idx = j;
                    closest_detected_pos = detected_pos;
                    closest_obstacle_point = obstacle_point;
                end
            end
        end
    end
    
    if ~isempty(closest_obstacle_point)
        fprintf('[DEBUG] 結果: 障害物#%dを選択, 距離=%.4f, 障害物最近点=(%.4f, %.4f), isDynamic=%d\n', ...
                closest_obstacle_idx, global_min_distance, ...
                closest_obstacle_point(1), closest_obstacle_point(2), closest_isDynamic);
    else
        fprintf('[DEBUG] 結果: 障害物#%dを選択, 距離=%.4f, isDynamic=%d\n', ...
                closest_obstacle_idx, global_min_distance, closest_isDynamic);
    end
    fprintf('[DEBUG] ========================================\n\n');
    
    % 最も近い障害物の isDynamic を設定（単一のbool値）
    detected_obstacles.isDynamic = closest_isDynamic;
end
