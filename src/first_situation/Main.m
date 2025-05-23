clear;
% %斜め
% dx = 11; %　ゴールの初期位置
% dy = 11;
% mx = -1; %　ロボットの初期位置
% my = -1;
% ob_velocity = [0.035; -0.035];
% %斜め障害物
% x1 = 2;   % 障害物の左辺のx座標
% x2 = 4;   % 障害物の右辺のx座標
% y1 = 8;   % 障害物の底辺のy座標
% y2 = 10;   % 障害物の上辺のy座標

% %横
dx = 6;
dy = 12;
mx = 6;
my = 0;
ob_velocity = [0.035; 0];
%横向き
x1 = 3;   % 障害物の左辺のx座標
x2 = 5;   % 障害物の右辺のx座標
y1 = 6;   % 障害物の底辺のy座標
y2 = 8;   % 障害物の上辺のy座標

%縦
% dx = 6;
% dy = 12;
% mx = 6;
% my = 0;
% ob_velocity = [0; -0.035];
% %縦の確認用
% x1 = 5;   % 障害物の左辺のx座標
% x2 = 7;   % 障害物の右辺のx座標
% y1 = 7;   % 障害物の底辺のy座標
% y2 = 9;   % 障害物の上辺のy座標

dx_tmp = 0;
dy_tmp = 0;

mx1 = mx;
my1 = my;
hr = 0;
o = [];     %障害物検出点の格納用
so = [0 0]; %障害物の検出数の格納用
wo=0.05;    %小さいほど小さなステップで進む。
wd=2;
point=[0 0];
r=36;
flag=0;     %1:障害物回避が必要。　0:通常の走行状態（目的地にまっすぐ進む）
flag_ob = 0;%0:障害物が検出されていない状態。　-1:障害物を検出して、障害物の座標が格納された状態。
%frag_mv = 0;
time=1;
i = 0;
data = 0;


%距離センサ生成
rb1 = rangeSensor;
rb2 = rangeSensor;
rb3 = rangeSensor;
rb4 = rangeSensor;
rb5 = rangeSensor;
%センサの感知範囲
rb1.Range = [0.0 0.7];%距離
rb2.Range = [0.0 0.7];
rb3.Range = [0.0 0.7];
rb4.Range = [0.0 0.7];
rb5.Range = [0.0 0.7];
rb1.HorizontalAngle = [(pi/6)-pi/360 (pi/6)+pi/360]; %角度
rb2.HorizontalAngle = [(pi/180)-pi/360 (pi/180)+pi/360];
rb3.HorizontalAngle = [(-pi/6)-pi/360 (-pi/6)+pi/360];
rb4.HorizontalAngle = [(4*pi/9)-pi/360 (4*pi/9)+pi/360];
rb5.HorizontalAngle = [(-4*pi/9)-pi/360 (-4*pi/9)+pi/360];

%マップ生成（ｘ、ｙ、幅[1マスの幅]）
map = binaryOccupancyMap(15,15,20);
%グリッド点設定（ｘ、ｙ）(mapの左下の点)
map.GridLocationInWorld = [-2 -2];


        x = x1:0.01:x2;   % x座標の範囲
        y = y1:0.01:y2;   % y座標の範囲

        ob = [x, x2.*ones(size(y)), fliplr(x), x1.*ones(size(y)); y1.*ones(size(x)), y, y2.*ones(size(x)), fliplr(y)];




frag_mv = 1; %move

g = [mx,my];%ロボットの軌跡

%マップに障害物を割り当て（map, 障害物[転置(.')してn行２列にしている],確率占有値[?][0で消える、１で追加できる]）
setOccupancy(map,ob.',1);
%s=ones(size(ob.')); %.'は転置 %ob.'と同じサイズで要素がすべて１の行列
%setOccupancy(map,ob.',s(:,1));%上のコードと同じ、要素それぞれに１をかけるか、１をまとめてかけるかの違い
%inflate(map,0.1); %障害物を膨らませる

show(map); %mapの描画
hold on   %mapの固定
title('Field');

plot(dx,dy,'ro'); %ゴール
plot(mx,my,'bo'); %スタート(ロボットの初期位置)

%シンボリックとして定義（文字式の計算ができるようになる）
syms x;
syms y;
po=double(subs(grad(dx,dy,o,wo,wd),{x y},{mx,my})); %subs(s,old,new) シンボリックsのoldの各要素をそれぞれnewに対応する要素で書き換え。

while norm([mx,my]-[dx,dy])>0.10 %ロボットが目的地に着くまでループ (2点間のユークリッド距離)

    o_rb1 = [0 0];
    o_rb2 = [0 0];
    o_rb3 = [0 0];
    o_rb4 = [0 0];
    o_rb5 = [0 0];
    o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
    flag_rb = [0;0;0;0;0;0];    %   左前　前　右前　左　右　どの方向かなど
                                %frag_rb(6)=0,1,4,5,7
                                %0角度変更なし、1予測線の描画。４角度を左へ、5角度を右へ、7特別な角度変更
    %ロボットの現在の状況把握
    truePose = [mx my hr];                  %ロボットの位置x,yと向きhr
    [ranges, angles] = rb1(truePose, map);  %rb1（rangeSensor）にセンサーの姿勢とマップ情報(truePose, map)を代入、出力[ranges, angles]
    scan1 = lidarScan(ranges, angles);      %障害物をスキャンする。liberScan
    [ranges, angles] = rb2(truePose, map);
    scan2 = lidarScan(ranges, angles);
    [ranges, angles] = rb3(truePose, map);
    scan3 = lidarScan(ranges, angles);
    [ranges, angles] = rb4(truePose, map);
    scan4 = lidarScan(ranges, angles);
    [ranges, angles] = rb5(truePose, map);
    scan5 = lidarScan(ranges, angles);

    %向きhrの範囲を-piからpiに
    if hr > pi
       hr = hr - 2*pi;
    end
    if hr < -pi
       hr = hr + 2*pi;
    end
%%障害物検知
    %rb1での障害物の反応
    if ~isnan(scan1.Cartesian)  % 障害物を読み取った時「isnan：NaN（読み取りがない）を１、それ以外を０」「scan1.Cartesian：rb1で読み取った値の直行座標」
        s1d=norm(scan1.Cartesian);                       %ロボットから障害物点までの距離
        s1r=atan(scan1.Cartesian(2)/scan1.Cartesian(1)); %障害物点がロボットを向いている方向からどの角度にあるか
         so=size(o);
         if flag_ob == -1 %障害物の検出が完了し、障害物の座標が追加された
             o_rb1 = [s1d*cos(hr+s1r)+mx,s1d*sin(hr+s1r)+my];%ロボットから見た情報から座標系における位置を計算
             o=[o;o_rb1]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(1) = 1;
             flag_ob = 0;
         elseif so==[0 0] %oが空の時
             o_rb1 = [s1d*cos(hr+s1r)+mx,s1d*sin(hr+s1r)+my];
             o=[o;o_rb1]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(1) = 1;
         elseif  min(sum(([(s1d*cos(hr+s1r)+mx).*ones([so(1),1]),(s1d*sin(hr+s1r)+my).*ones([so(1),1])]-o).^2,2)) > [0.2^2 0.2^2] 
             %検知した障害物の限りなく近い箇所で障害物をまた検出して一か所に何個も障害物を検出するのを回避するための条件  %
             o_rb1 = [s1d*cos(hr+s1r)+mx,s1d*sin(hr+s1r)+my];    
             o=[o;o_rb1]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(1) = 1;
         end
         plot(o_rb1(1),o_rb1(2),'sr');%正方形表示
         %     rb1のx　rb1のy
    end

    %rb2での障害物の反応
    if ~isnan(scan2.Cartesian)
        s2d=norm(scan2.Cartesian);
        s2r=atan(scan2.Cartesian(2)/scan2.Cartesian(1));
        so=size(o);
           if flag_ob == -1
             o_rb2 = [s2d*cos(hr+s2r)+mx,s2d*sin(hr+s2r)+my];
             o=[o;o_rb2]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(2) = 1;
             flag_ob = 0;
           elseif so==[0 0]
             o_rb2 = [s2d*cos(hr+s2r)+mx,s2d*sin(hr+s2r)+my];
             o=[o;o_rb2]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
            flag_rb(2) = 1;
           elseif min(sum(([(s2d*cos(hr+s2r)+mx).*ones([so(1),1] ),(s2d*sin(hr+s2r)+my).*ones([so(1),1])]-o).^2,2))>[0.2^2 0.2^2]
             o_rb2 = [s2d*cos(hr+s2r)+mx,s2d*sin(hr+s2r)+my];
             o=[o;o_rb2]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
            flag_rb(2) = 1;
           end    
        plot(o_rb2(1),o_rb2(2),'s');
    end

    %rb3での障害物の反応
    if ~isnan(scan3.Cartesian)
        s3d=norm(scan3.Cartesian);
        s3r=atan(scan3.Cartesian(2)/scan3.Cartesian(1));
        so=size(o);
        if flag_ob == -1
             o_rb3 = [s3d*cos(hr+s3r)+mx,s3d*sin(hr+s3r)+my];
             o=[o;o_rb3]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(3) = 1;
             flag_ob = 0;
        elseif so==[0 0]
             o_rb3 = [s3d*cos(hr+s3r)+mx,s3d*sin(hr+s3r)+my];
             o=[o;o_rb3]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(3) = 1;
        elseif min(sum(([(s3d*cos(hr+s3r)+mx).*ones([so(1),1]),(s3d*sin(hr+s3r)+my).*ones([so(1),1])]-o).^2,2))>[0.2^2 0.2^2]
             o_rb3 = [s3d*cos(hr+s3r)+mx,s3d*sin(hr+s3r)+my];
             o=[o;o_rb3]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
            flag_rb(3) = 1;
        end
        plot(o_rb3(1),o_rb3(2),'sr');
    end

    %rb4での障害物の反応
    if ~isnan(scan4.Cartesian)
        s4d=norm(scan4.Cartesian);
        s4r=atan(scan4.Cartesian(2)/scan4.Cartesian(1));
        so=size(o);
        if flag_ob == -1
             o_rb4 = [s4d*cos(hr+s4r)+mx,s4d*sin(hr+s4r)+my];
             o=[o;o_rb4]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(4) = 1;
             flag_ob = 0;
        elseif so == [0 0]
             o_rb4 = [s4d*cos(hr+s4r)+mx,s4d*sin(hr+s4r)+my];
             o=[o;o_rb4]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(4) = 1;
        elseif min(sum(([(s4d*cos(hr+s4r)+mx).*ones([so(1),1]),(s4d*sin(hr+s4r)+my).*ones([so(1),1])]-o).^2,2))>[0.2^2 0.2^2]
             o_rb4 = [s4d*cos(hr+s4r)+mx,s4d*sin(hr+s4r)+my];
             o=[o;o_rb4]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
            flag_rb(4) = 1;
        end 
        if (o_rb4(1) == 0 && o_rb4(2) == 0) == false
            plot(o_rb4(1),o_rb4(2),'sr');
        end
    end

    %rb5での障害物の反応
    if ~isnan(scan5.Cartesian)
        s5 = scan5.Cartesian;
        s5d=norm(scan5.Cartesian);
        s5r=atan(scan5.Cartesian(2)/scan5.Cartesian(1));
        so=size(o);
        if flag_ob == -1
             o_rb5 = [s5d*cos(hr+s5r)+mx,s5d*sin(hr+s5r)+my];
             o=[o;o_rb5]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(5) = 1;
             flag_ob = 0;
        elseif so==[0 0] 
             o_rb5 = [s5d*cos(hr+s5r)+mx,s5d*sin(hr+s5r)+my];
             o=[o;o_rb5]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(5) = 1;
        elseif min(sum(([(s5d*cos(hr+s5r)+mx).*ones([so(1),1]),(s5d*sin(hr+s5r)+my).*ones([so(1),1])]-o).^2,2))>[0.2^2 0.2^2]
             o_rb5 = [s5d*cos(hr+s5r)+mx,s5d*sin(hr+s5r)+my];
             o=[o;o_rb5]; %障害物の座標を格納
             o_tmp = [o_rb1;o_rb2;o_rb3;o_rb4;o_rb5];
             flag_rb(5) = 1;
        end
        if (o_rb5(1) == 0 && o_rb5(2) == 0) == false
            plot(o_rb5(1),o_rb5(2),'sr');
        end
    end

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        kokomade        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if nnz(flag_rb) >= 2 && norm([mx,my]-[dx,dy]) > 0.5 %障害物が2個以上検知した。かつロボットとゴールの距離が0.5以上離れている（ゴールしていない）。
        flag_ob = 0;
        i = 0;
        if mx == dx %ゴールとロボットのx座標が同じとき
            %plot([mx mx],[my dy]);
            tilt_goal = 0;
            if my < dy  %ゴールの真下にいるから
                hr = pi/2;%真上を向く
            else        %ゴールの真上にいるから
                hr = -pi/2;%真下を向く
            end
        else        %ゴールとロボットのx座標が異なるとき
            [f_goal,~,tilt_goal] = Forecast_line(dx,mx,dy,my);%ゴールとロボットを結ぶ直線を作る%f_goalゴールのy座標、tilt_goal傾き %目的x,現在x,目的y,現在y
            if my < dy  %ゴールより下
                if tilt_goal < 0 %傾きが-(ゴールより右側)
                    hr = pi + atan(tilt_goal);
                else             %傾きが0以上(ゴールより左側)
                    hr = atan(tilt_goal);
                end
            else%ゴールより上
                if tilt_goal < 0%傾きが-(ゴールより左側)
                    hr = atan(tilt_goal);
                else            %傾きが0以上(ゴールより右側)
                    hr = pi + atan(tilt_goal);
                end
            end
        end

        [flag_rb,hr,theta] = Change_angle(flag_rb,o_tmp,hr,mx,my,dx,dy);%調整した向きを反映、予測線を描画する。
    elseif nnz(flag_rb) == 1 %障害物をセンサー１本のみで検知したとき %向きを変えずに
        flag_ob = -1;        %
        i = i + 1;           %
    end

    if i == 3
            flag_ob = 0;
            i = 0;
    end

    if flag_rb(6) == 0
        %wo = 0.05;
        %plot(mx,my,'rx');%redのx
    end

    while flag_rb(6) ~= 0%角度変更または予測線分の描画の必要あり。
        data = 0;
        %[hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,~,time] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data);
        wo = 0.01;
        flag_d = 0;
        flag_rb = [0;0;0;0;0;0];
    end

 if flag_ob == -1          %ロボットを前に進める。
    mx = mx + cos(hr) * 0.1;
    my = my + sin(hr) * 0.1;
    g = [g; mx, my];
    i = i + 1;
    if ~isempty(o)
        o(end, :) = [];
    end
    % ゴールの方向を再確認
    goal_direction = atan2(dy - my, dx - mx);
    hr = goal_direction;
 else

% 障害物回避後にゴールの方向を再確認するロジック

    if flag==1  %障害物回避の必要あり

        temp=normr(double(subs(grad(dx_tmp,dy_tmp,o,wo,wd),{x y},{mx,my}))).*0.12;%勾配ベクトルを正規化して0.12を乗算 %ベクトルの正規化は向きはそのままに大きさを1にすること

         if temp(2)>0   %勾配ベクトルが上向き
             hr=acos(temp(1)/0.12);
         else           %勾配ベクトルが下向き
             hr=-acos(temp(1)/0.12);
         end
         mx=mx+temp(1); %勾配ベクトル方向に進める
         my=my+temp(2);
         g=[g;mx,my];   %gの末尾に移動先の座標を追加
          po=double(subs(grad(dx_tmp,dy_tmp,o,wo,wd),{x y},{mx,my}));%勾配の再計算
         time=time+1;   %時間を進める
         plot(g(:,1),g(:,2),'r');%軌跡を描画
         drawnow;                %mapに描画
         if time>300    %時間が300を超えたら停留したとみなしてプログラムを止める
             break; 
         end

         if norm([mx,my]-[dx_tmp,dy_tmp])<=0.50 %仮想ゴールとの距離が0.5以下の時
             wo = 0;                            %重みを0にする
         end

         if norm([mx,my]-[dx_tmp,dy_tmp])<=0.10 %仮想ゴールとの距離が0.1以下の時
             %o=[];
             flag=0;                            %障害物を回避した
             wo = 0.05;                         %重みを0.05にする
         end
         continue;                              %uhileの最初
    else            %通常の走行

        if norm([mx,my]-[dx,dy])<=1.0           %ゴールとの距離が1以下の時
            o=[];                               %障害物をリセット
        end
       %勾配ベクトルに沿って進める
        temp=normr(double(subs(grad(dx,dy,o,wo,wd),{x y},{mx,my}))).*0.12;

        if temp(2)>0
            hr=acos(temp(1)/0.12);
        else
            hr=-acos(temp(1)/0.12);
        end 

            mx=mx+temp(1);
            my=my+temp(2);
            g=[g;mx,my]; 
            po=double(subs(grad(dx,dy,o,wo,wd),{x y},{mx,my}));
            time=time+1;        

        if norm([mx,my]-[dx,dy]) <= 1   %ゴールとの距離が1以下の時
             o = [];                    %障害物をリセット
        end

        if time>500
            break; 
        end
    end
 end
plot(g(:,1),g(:,2),'b');   %軌跡を描画
drawnow;

%%%障害物を動かす
if frag_mv == 1
    %障害物を動かす幅

    ob_mv = ob + ob_velocity; %障害物を動かす

    setOccupancy(map, ob.', 0);     %障害物を動かす前のobを消す
    setOccupancy(map, ob_mv.', 1);  %障害物を動した後のobを追加する

    % マップを再描画
    show(map);

    ob = ob_mv; % 障害物位置を更新

    % 軌跡や目標位置の描画などの処理
    hold on;
    plot(g(:, 1), g(:, 2), 'b');
    plot(dx, dy, 'ro'); % ゴール
    plot(mx1, my1, 'bo'); % スタート

    plot(mx, my, 'bo'); % robot
end

drawnow;
end
time

