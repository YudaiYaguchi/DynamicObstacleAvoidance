clear;
dx = 6;
dy = 5;
mx = 0;
my = 0;
map = binaryOccupancyMap(15,15,30);
%グリッド点設定（ｘ、ｙ）(mapの左下の点)
map.GridLocationInWorld = [-2 -2];
%ob = [1:1:5; 1:2:10];%障害物
        %%四角
        x1 = 4;   % 障害物の左辺のx座標
        x2 = 6;   % 障害物の右辺のx座標
        y1 = 1;   % 障害物の底辺のy座標
        y2 = 4;   % 障害物の上辺のy座標

        x = x1:0.01:x2;   % x座標の範囲
        y = y1:0.01:y2;   % y座標の範囲

        ob = [x, x2.*ones(size(y)), fliplr(x), x1.*ones(size(y)); y1.*ones(size(x)), y, y2.*ones(size(x)), fliplr(y)];

        setOccupancy(map,ob.',1);

        show(map); %mapの描画
        hold on   %mapの固定
        title('Field');
        
        plot(dx,dy,'ro'); %ゴール
        plot(mx,my,'bo'); %スタート(ロボットの初期位置)