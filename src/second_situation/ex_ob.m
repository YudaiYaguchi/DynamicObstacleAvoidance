clear;
% マップ1の作成
map1 = binaryOccupancyMap(10, 10, 10);
obstacles1 = [1 1; 2 2; 3 3];
setOccupancy(map1, obstacles1, 1);

% マップ2の作成
map2 = binaryOccupancyMap(10, 10, 10);
obstacles2 = [5 5; 6 6; 7 7];
setOccupancy(map2, obstacles2, 1);

% マップの表示
figure;
subplot(2, 3, 4);   %  1 2 3
show(map1);         % ➃ 5 6
title('Map 1');

subplot(2, 3, [2,3,5,6]);
show(map2);
title('Map 2');

