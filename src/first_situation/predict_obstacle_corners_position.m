function [predict_obstacle_corners, connected_corners] = predict_obstacle_corners_position(corners, dt, steps, current_velocity, current_acceleration, prev_predict_corners)
% predict_obstacle_corners_position
% corners: 4x2 (任意順でも可)
% dt, steps, current_velocity(1x2), current_acceleration(1x2)
% prev_predict_corners: 前回の予測角 [(4*steps)×2] (オプション)
% predict_obstacle_corners: (4*steps) x 2
% connected_corners: 9x2 （最初と最後の矩形を順番に並べて閉ループ）

% --- 前回の予測位置が存在するかチェック ---
if nargin < 6 || isempty(prev_predict_corners)
  % 初回呼び出し：全ステップを計算
  prev_predict_corners = [];
  is_first_call = true;
else
  is_first_call = false;
end

% --- 安全化 ---
corners = reshape(corners, [], 2);

% --- 外接矩形を作成 ---
minX = min(corners(:,1));
maxX = max(corners(:,1));
minY = min(corners(:,2));
maxY = max(corners(:,2));
rect = [
  minX, minY;   % 左下
  maxX, minY;   % 右下
  maxX, maxY;   % 右上
  minX, maxY    % 左上
  ];

if is_first_call
  % --- 初回：全ステップを計算 ---
  predict_obstacle_corners = zeros(4 * steps, 2);
  idx = 1;

  for k = 1:steps
    T = k * dt;

    % この時刻での重心の移動量（全体を平行移動）
    move = current_velocity * T + 0.5 * current_acceleration * (T^2);

    % 各角を同じだけ移動
    predicted_rect = rect + move;

    predict_obstacle_corners(idx:idx+3, :) = predicted_rect;
    idx = idx + 4;
  end
else
  % --- 2回目以降：最古の4点（steps=1）を削除し、最新の予測（steps=20）から1ステップ追加 ---
  % 前回の予測から最初の4点（steps=1、最も古い予測）を削除
  if size(prev_predict_corners, 1) >= 4 * steps
    % 最初の4点（steps=1）を削除
    predict_obstacle_corners = prev_predict_corners(5:end, :);

    % 最新の予測位置（最後の4点 = steps=20の予測）を取得
    if size(predict_obstacle_corners, 1) >= 4
      last_rect = predict_obstacle_corners(end-3:end, :);

      % steps=20の予測から、さらに1ステップ先（steps=21）を計算
      % 1ステップ分の移動量を計算
      move_step = current_velocity * dt + 0.5 * current_acceleration * (dt^2);

      % 最新の矩形（steps=20）から1ステップ分移動させた新しい矩形（steps=21）
      new_rect = last_rect + move_step;

      % 新しい予測を追加
      predict_obstacle_corners = [predict_obstacle_corners; new_rect];
    end
  else
    % 予期しないサイズの場合は初回として扱う
    predict_obstacle_corners = zeros(4 * steps, 2);
    idx = 1;
    for k = 1:steps
      T = k * dt;
      move = current_velocity * T + 0.5 * current_acceleration * (T^2);
      predicted_rect = rect + move;
      predict_obstacle_corners(idx:idx+3, :) = predicted_rect;
      idx = idx + 4;
    end
  end
end

% --- 最初と最後の矩形を取得 ---
first_rect = predict_obstacle_corners(1:4, :);
last_rect  = predict_obstacle_corners(end-3:end, :);

% --- 最初と最後を結んだ閉ループを構築 ---
connected_corners = [
  first_rect;
  first_rect(1,:);   % 最初の矩形を閉じる
  NaN NaN;
  last_rect;
  last_rect(1,:)     % 最後の矩形を閉じる
  ];
end
