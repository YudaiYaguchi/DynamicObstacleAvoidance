function z_o = new1_o_p(a,b,o)
if isempty(o)
    z_o=0 ;
else
    num_obstacles = size(o, 1);
    robot_pos_matrix = repmat([a b], num_obstacles, 1);

    % 行列演算で指数関数の斥力ポテンシャルを一気に計算
    distance_sq = sum((o - robot_pos_matrix).^2, 2);
    distance = sqrt(distance_sq);
    
    % o-point と point-o の順序が逆転していますが、2乗するため結果は同じです。
    z_o = 20 * sum(1.4 .^ (1 ./ (distance * 1.0)) - 1); % 距離係数を1.0に、斥力係数を20に増やして早期回避
end
end