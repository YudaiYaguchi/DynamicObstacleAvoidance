function visualize_potential_field(d_x, d_y, o, wo, wd)
  % 可視化範囲（マップ全体などに合わせて調整）
  [X, Y] = meshgrid(-2:0.2:15, -2:0.2:15);

  % 各点のポテンシャル値を計算
  Z = zeros(size(X));

  for i = 1:size(X,1)
      for j = 1:size(X,2)
          a = X(i,j);
          b = Y(i,j);

          % ゴールへの引力
          att = wd * new1_d_p(a,b,d_x,d_y);

          % 障害物への斥力
          rep = wo * (new1_o_p(a,b,o) + o_p(a,b,o));

          % 合成ポテンシャル
          Z(i,j) = att + rep;
      end
  end

  % --- 新しい図ウィンドウで表示 ---
  figure(2); % ← Mainとは別の図を固定的に使用
  clf;          % このウィンドウだけクリア

  contourf(X, Y, Z, 30, 'LineColor', 'none'); % ポテンシャル分布
  colorbar;
  hold on;

  % ゴール位置
  plot(d_x, d_y, 'r*', 'MarkerSize', 10, 'LineWidth', 2);

  % 障害物
  if ~isempty(o)
      plot(o(:,1), o(:,2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);
  end

  title('Potential Field (Separate Figure)');
  xlabel('X');
  ylabel('Y');
  axis equal;
  drawnow; % 更新
end
