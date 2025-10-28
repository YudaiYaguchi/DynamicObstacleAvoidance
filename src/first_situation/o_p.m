function z_o = o_p(a,b,o)
if isempty(o)
    z_o=0 ;
else
    % oの行数（障害物点の数）を取得
    num_obstacles = size(o, 1);

    % ロボット位置 [a b] を障害物点の数だけ縦に複製
    % repmat([a b], num_obstacles, 1) は、[a b]をnum_obstacles行に複製した行列を作成
    robot_pos_matrix = repmat([a b], num_obstacles, 1);

    % 行列演算で距離の逆数を一気に計算
    % (point - o) は、シンボリック変数であっても要素ごとに差分が計算される
    distance_sq = sum((robot_pos_matrix - o).^2, 2); % 距離の2乗
    distance = sqrt(distance_sq); % 距離
    
    z_o = sum(1 ./ distance); % 距離の逆数の総和 (斥力ポテンシャル)
end
end