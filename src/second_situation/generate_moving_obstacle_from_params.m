function ob_points = generate_moving_obstacle_from_params(time, x1, x2, y1, y2)
%
% 既存の矩形定義 [x1, x2, y1, y2] のサイズを保持しつつ、
% 時刻 time に基づいて指定された放物線軌道上の点群を計算します。
%
% 入力:
%   time: 現在の時刻
%   x1, x2, y1, y2: 元の矩形を定義する座標（サイズ抽出に使用）
%
% 出力:
%   ob_points: 時刻 time における移動後の矩形の点群座標 [x; y]

    % --- 1. 既存の情報から障害物サイズを抽出 ---
    width = x2 - x1;
    height = y2 - y1;
    half_width = width / 2;
    half_height = height / 2;
    
    % --- 2. 軌道パラメータ ---
    % 始点(0, 6)、頂点(2, 8)、終点(6, 6)
    a = -0.5; % 係数
    h = 4;    % 頂点のx座標
    k = 10;    % 頂点のy座標
    
    % --- 3. 時刻 time から障害物の中心位置 (xc, yc) を計算 ---
    
    % X座標を時間に線形に依存させる (x(time) = time)
    obstacle_speed_ratio = 0.15; % 速度調整用の係数
    xc = time * obstacle_speed_ratio;
    
    % X座標 (xc) に基づいてY座標を計算（放物線）
    yc = a * (xc - h)^2 + k;
    
    % --- 4. 移動後の矩形の境界座標を計算 ---
    
    % 新しい境界座標は、新しい中心位置から半分の幅/高さを引くことで求まる
    new_x1 = xc - half_width; % 移動後の左端
    new_x2 = xc + half_width; % 移動後の右端
    new_y1 = yc - half_height; % 移動後の下端
    new_y2 = yc + half_height; % 移動後の上端
    
    % --- 5. 点群生成 ---
    
    resolution = 0.01; 
    
    % 移動後の x, y の配列を生成
    x_array = new_x1 : resolution : new_x2;
    y_array = new_y1 : resolution : new_y2;

    % 障害物の境界（ob 変数に相当する点群データ）を構成
    % 順序: [底辺, 右辺, 上辺(逆順), 左辺(逆順)]
    ob_points = [
        % X座標
        x_array, ...                            % 底辺のX
        new_x2 .* ones(1, length(y_array)), ... % 右辺のX
        fliplr(x_array), ...                    % 上辺のX (逆順)
        new_x1 .* ones(1, length(y_array))      % 左辺のX
        ;
        % Y座標
        new_y1 .* ones(1, length(x_array)), ... % 底辺のY
        y_array, ...                            % 右辺のY
        new_y2 .* ones(1, length(x_array)), ... % 上辺のY
        fliplr(y_array)                         % 左辺のY (逆順)
    ];
end