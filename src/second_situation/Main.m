%何サンプル後か考えて向かってくるやつ通れるか
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

% % %横
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

% %縦
% dx = 6;
% dy = 12;
% mx = 6;
% my = 0;
% ob_velocity = [0; -0.035];
%縦の確認用
% x1 = 5; % 障害物の左辺のx座標
% x2 = 7; % 障害物の右辺のx座標
% y1 = 10; % 障害物の底辺のy座標
% y2 = 12; % 障害物の上辺のy座標

dx_tmp = 0;
dy_tmp = 0;

mx1 = mx;
my1 = my;

hr = 0; % ロボットの角度
o = []; %障害物検出点の格納用
so = [0 0]; %障害物の検出数の格納用
wo = 0.05; %小さいほど小さなステップで進む。
wd = 2;
point = [0 0];
r = 36;
flag = 0; %1:障害物回避が必要。　0:通常の走行状態（目的地にまっすぐ進む）
flag_ob = 0; %0:障害物が検出されていない状態。　-1:障害物を検出して、障害物の座標が格納された状態。
%frag_mv = 0;
time = 1;
i = 0;
j = 0;
data = 0;
new_x2 = 0;
new_y1 = 0;

% 加速度計算用の変数を初期化
ob_acceleration = [0.0010; -0.0000]; % 障害物の加速度 (例: y方向に-0.005)
ob_prev = []; % 前回の障害物位置
prev_ob_pos = [];
prev_ob_velocity = [0; 0];
steps = 8; % Nステップ後の予測に使う
dt = 1;

% 予測した位置を保存する変数
predicted_pos = [];

%距離センサ生成
[rb1, rb2, rb3, rb4, rb5] = createSensors();

%マップ生成（ｘ、ｙ、幅[1マスの幅]）
map = binaryOccupancyMap(15, 15, 20);
%グリッド点設定（ｘ、ｙ）(mapの左下の点)
map.GridLocationInWorld = [-2 -2];

%%四角

%奥
x3 = 3; % 障害物の左辺のx座標
x4 = 12; % 障害物の右辺のx座標
y3 = 9.5; % 障害物の底辺のy座標
y4 = 9.5; % 障害物の上辺のy座標

% x3 = 4.5;   % 障害物の左辺のx座標
% x4 = 8;   % 障害物の右辺のx座標
% y3 = 3.5;   % 障害物の底辺のy座標
% y4 = 3.5;   % 障害物の上辺のy座標

%真横の確認用
% x1 = 0.75;   % 障害物の左辺のx座標
% x2 = 2.75;   % 障害物の右辺のx座標
% y1 = 5;   % 障害物の底辺のy座標
% y2 = 7;   % 障害物の上辺のy座標

x = x1:0.01:x2; % x座標の範囲
y = y1:0.01:y2; % y座標の範囲
x5 = x3:0.01:x4;
y5 = y3:0.01:y4;

ob = [x, x2 .* ones(size(y)), fliplr(x), x1 .* ones(size(y)); y1 .* ones(size(x)), y, y2 .* ones(size(x)), fliplr(y)];
% ob = generate_moving_obstacle_from_params(time, x1, x2, y1, y2); %カーブを描く障害物
ob2_mv = ob; %障害物が動いているかの判定　and　1つ後の動的障害物の出力に使う
% ob5 = [x5, x4.*ones(size(y5)), fliplr(x5), x3.*ones(size(y5)); y3.*ones(size(x5)), y5, y4.*ones(size(x5)), fliplr(y5)];

%マップに障害物を割り当て（map, 障害物[転置(.')してn行２列にしている],確率占有値[?][0で消える、１で追加できる]）
setOccupancy(map, ob.', 1);
% setOccupancy(map,ob5.',1);

g = [mx, my]; %ロボットの軌跡
figure(1);
show(map); %mapの描画
hold on %mapの固定
plot(dx, dy, 'ro'); %ゴール
plot(mx, my, 'bo'); %スタート(ロボットの初期位置)

%シンボリックとして定義（文字式の計算ができるようになる）
syms x;
syms y;
po = double(subs(grad(dx, dy, o, wo, wd), {x y}, {mx, my})); %subs(s,old,new) シンボリックsのoldの各要素をそれぞれnewに対応する要素で書き換え。

o_rb1 = [0 0];
o_rb2 = [0 0];
o_rb3 = [0 0];
o_rb4 = [0 0];
o_rb5 = [0 0];
o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
prev_o_tmp = o_tmp; %センサに反応があれば変化する
flag_rb = [0; 0; 0; 0; 0; 0]; %   左前　前　右前　左　右　どの方向かなど
%frag_rb(6)=0,1,4,5,7
%0角度変更なし、1予測線の描画。４角度を左へ、5角度を右へ、7特別な角度変更

while norm([mx, my] - [dx, dy]) > 0.10 %ロボットが目的地に着くまでループ (2点間のユークリッド距離)
    % --- カーブの動的障害物のとき ---
    % ob = generate_moving_obstacle_from_params(time, x1, x2, y1, y2);
    % setOccupancy(map, ob.', 1);
    %     % --- 消去 ---
    %     if ~isempty(ob_prev)
    %         setOccupancy(map, ob_prev.', 0); % 前回描画した障害物を消去
    %     end

    %　動的障害物と認識してからの動作
    if nnz(flag_rb) >= 2 && norm([mx, my] - [ob(1), ob(2)]) < 2.5 %センサが反応する位置で動的障害物と認識する(障害物の座標はここで与えられるものとする)
        while any(flag_rb == 1) %どれかのセンサに障害物が反応してる間ループ

            %ロボットの現在の状況把握
            truePose = [mx my hr]; %ロボットの位置x,yと向きhr
            [ranges, angles] = rb1(truePose, map); %rb1（rangeSensor）にセンサーの姿勢とマップ情報(truePose, map)を代入、出力[ranges, angles]
            scan1 = lidarScan(ranges, angles); %障害物をスキャンする。liberScan
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
                hr = hr - 2 * pi;
            end

            if hr < -pi
                hr = hr + 2 * pi;
            end

            %rb1での障害物の反応
            if ~isnan(scan1.Cartesian) % 障害物を読み取った時「isnan：NaN（読み取りがない）を１、それ以外を０」「scan1.Cartesian：rb1で読み取った値の直行座標」
                s1d = norm(scan1.Cartesian); %ロボットから障害物点までの距離
                s1r = atan(scan1.Cartesian(2) / scan1.Cartesian(1)); %障害物点がロボットを向いている方向からどの角度にあるか
                so = size(o);

                if flag_ob == -1 %障害物の検出が完了し、障害物の座標が追加された
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my]; %ロボットから見た情報から座標系における位置を計算
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                    flag_ob = 0;
                elseif so == [0 0] %oが空の時
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                elseif min(sum(([(s1d * cos(hr + s1r) + mx) .* ones([so(1), 1]), (s1d * sin(hr + s1r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    %検知した障害物の限りなく近い箇所で障害物をまた検出して一か所に何個も障害物を検出するのを回避するための条件  %
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                end

                plot(o_rb1(1), o_rb1(2), 'sr'); %正方形表示
                %     rb1のx　rb1のy
            end

            %rb2での障害物の反応
            if ~isnan(scan2.Cartesian)
                s2d = norm(scan2.Cartesian);
                s2r = atan(scan2.Cartesian(2) / scan2.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                elseif min(sum(([(s2d * cos(hr + s2r) + mx) .* ones([so(1), 1]), (s2d * sin(hr + s2r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                end

                plot(o_rb2(1), o_rb2(2), 's');
            end

            %rb3での障害物の反応
            if ~isnan(scan3.Cartesian)
                s3d = norm(scan3.Cartesian);
                s3r = atan(scan3.Cartesian(2) / scan3.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                elseif min(sum(([(s3d * cos(hr + s3r) + mx) .* ones([so(1), 1]), (s3d * sin(hr + s3r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                end

                plot(o_rb3(1), o_rb3(2), 'sr');
            end

            %rb4での障害物の反応
            if ~isnan(scan4.Cartesian)
                s4d = norm(scan4.Cartesian);
                s4r = atan(scan4.Cartesian(2) / scan4.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                elseif min(sum(([(s4d * cos(hr + s4r) + mx) .* ones([so(1), 1]), (s4d * sin(hr + s4r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                end

                if (o_rb4(1) == 0 && o_rb4(2) == 0) == false
                    plot(o_rb4(1), o_rb4(2), 'sr');
                end

            end

            %rb5での障害物の反応
            if ~isnan(scan5.Cartesian)
                s5 = scan5.Cartesian;
                s5d = norm(scan5.Cartesian);
                s5r = atan(scan5.Cartesian(2) / scan5.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                elseif min(sum(([(s5d * cos(hr + s5r) + mx) .* ones([so(1), 1]), (s5d * sin(hr + s5r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                end

                if (o_rb5(1) == 0 && o_rb5(2) == 0) == false
                    plot(o_rb5(1), o_rb5(2), 'sr');
                end

            end

            % 障害物の速度を更新
            ob_velocity = ob_velocity + ob_acceleration * dt;

            % 障害物の位置を更新
            ob_mv = ob + ob_velocity * dt;

            last_point = ob_mv(:, end)';   % ← 転置して [1×2] ベクトルに
            disp(last_point);
            % Nステップ後の障害物位置を予測
            if prev_ob_pos == current_pos
                [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(last_point, prev_ob_pos, prev_ob_velocity, dt, steps);
            else
                [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(o, prev_ob_pos, prev_ob_velocity, dt, steps);
            end

            % === 次回に使うため更新 ===
            prev_ob_pos = current_pos;
            fprintf("😂😂😂どれかのセンサに障害物が反応している間のループ😂😂😂　prev_ob_pos:"); disp(current_pos);
            prev_ob_velocity = current_velocity;

            ob2_mv_velocity = ob_mv - ob;
            ob3_mv = ob2_mv; %1つあとの障害物の座標
            ob2_mv = ob_mv + 10 * ob2_mv_velocity;
            setOccupancy(map, ob3_mv.', 0);
            setOccupancy(map, ob2_mv.', 1);

            %現時点の障害物の位置を保存

            setOccupancy(map, ob.', 0); %障害物を動かす前のobを消す
            setOccupancy(map, ob_mv.', 1); %障害物を動した後のobを追加する

            ob_prev = ob_mv; % 次回消去するために現在の障害物位置を保存
            % マップを再描画
            show(map);

            ob = ob_mv; % 障害物位置を更新

            x_1 = max(ob(1, :)); % ob の x の最大値
            y_1 = min(ob(2, :)); % ob の y の最小値

            x_2 = max(ob2_mv(1, :)); % ob2_mv の x の最大値
            y_2 = min(ob2_mv(2, :)); % ob2_mv の y の最小値

            [opf, ~, opa, ~] = Forecast_line(x_1, x_2, y_1, y_2);
            [f_goal, ~, tilt_goal, ~] = Forecast_line(dx, mx, dy, my);

            m1 = opa; % 障害物の直線の傾き
            m2 = tilt_goal; % ゴールとロボットを結ぶ直線の傾き

            theta = atan(abs(m2 - m1) / (1 + m1 * m2)); % ラジアン単位のなす角
            fprintf('Theta: %f radians\n', theta); % theta の値を表示

            % theta が 0° に近いかどうかをチェック
            %仮ゴールを設定
            if new_x2 == 0 && new_y1 == 0

                if isnan(theta) || abs(theta) < 1e-2

                    if (ob2_mv_velocity(1) <= 0 && ob2_mv_velocity(2) <= 0) || (ob2_mv_velocity(1) > 0 && ob2_mv_velocity(2) > 0)
                        ob2_mv_velocity = [ob2_mv_velocity(2); -ob2_mv_velocity(1)];
                        t = -60; % 時間 t 秒
                        displacement = ob2_mv_velocity * t;
                        new_x2 = x_2 + displacement(1);
                        new_y1 = y_2 + displacement(2);
                    end

                    if (ob2_mv_velocity(1) > 0 && ob2_mv_velocity(2) < 0) || (ob2_mv_velocity(1) < 0 && ob2_mv_velocity(2) > 0)
                        ob2_mv_velocity = [-ob2_mv_velocity(2); ob2_mv_velocity(1)];
                        t = 60; % 時間 t 秒
                        displacement = ob2_mv_velocity * t;
                        new_x2 = x_2 + displacement(1);
                        new_y1 = y_2 + displacement(2);
                    end

                else
                    t = -120; % 時間 t 秒
                    displacement = ob2_mv_velocity * t;
                    new_x2 = (x_1 + x_2) / 2 + displacement(1);
                    new_y1 = y_1 + 1.0 + displacement(2);
                end

            end

            % 軌跡や目標位置の描画などの処理
            plot(ob2_mv(1, :), ob2_mv(2, :), 'b.-');
            plot(g(:, 1), g(:, 2), 'b');
            plot(dx, dy, 'ro'); % ゴール
            plot(mx1, my1, 'bo'); % スタート
            plot(mx, my, 'bo'); % robot
            plot(new_x2, new_y1, 'go');
            drawnow;
            show(map);

            if hr > pi
                hr = hr - 2 * pi;
            end

            if hr < -pi
                hr = hr + 2 * pi;
            end

            if time > 300
                break;
            end

            truePose = [mx my hr]; %ロボットの位置と向きがtruePose (hr = 0→右方向,pi/2→上方向,piと-pi→左方向,-pi/2→下方向)

            if nnz(flag_rb) >= 2 && norm([mx, my] - [dx, dy]) > 0.5 %障害物が2個以上検知した。かつロボットとゴールの距離が0.5以上離れている（ゴールしていない）。
                flag_ob = 0;
                flag = 1;
                i = 0;

                if mx == dx %ゴールとロボットのx座標が同じとき
                    tilt_goal = 0;

                    if my < dy %ゴールの真下にいるから
                        hr = pi / 2; %真上を向く
                    else %ゴールの真上にいるから
                        hr = -pi / 2; %真下を向く
                    end

                else %ゴールとロボットのx座標が異なるとき
                    [f_goal, ~, tilt_goal, ~] = Forecast_line(dx, mx, dy, my); %ゴールとロボットを結ぶ直線を作る %f_goalゴールのy座標、tilt_goal傾き %目的x,現在x,目的y,現在y

                    if my < dy %ゴールより下

                        if tilt_goal < 0 %傾きが-(ゴールより右側)
                            hr = pi + atan(tilt_goal);
                        else %傾きが0以上(ゴールより左側)
                            hr = atan(tilt_goal);
                        end

                    else %ゴールより上

                        if tilt_goal < 0 %傾きが-(ゴールより左側)
                            hr = atan(tilt_goal);
                        else %傾きが0以上(ゴールより右側)
                            hr = pi + atan(tilt_goal);
                        end

                    end

                end

                [flag_rb, hr, theta] = Change_angle(flag_rb, o_tmp, hr, mx, my, dx, dy); %調整した向きを反映、予測線を描画する。

            elseif nnz(flag_rb) == 1 %障害物をセンサー１本のみで検知したとき %向きを変えずに
                flag_ob = -1; %
                i = i + 1; %
            end

            if i == 3
                flag_ob = 0;
                i = 0;
            end

            if flag_ob == -1 %ロボットを前に進める。
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

                if flag == 1 %障害物回避の必要あり
                    %勾配ベクトルに沿って進める
                    temp = normr(double(subs(grad(new_x2, new_y1, o, wo, wd), {x y}, {mx, my}))) .* 0.12;

                    if temp(2) > 0 %勾配ベクトルが上向き
                        hr = acos(temp(1) / 0.12);
                    else %勾配ベクトルが下向き
                        hr = -acos(temp(1) / 0.12);
                    end

                    mx = mx + temp(1); %勾配ベクトル方向に進める
                    my = my + temp(2);
                    g = [g; mx, my]; %gの末尾に移動先の座標を追加

                    if norm([mx, my] - [new_x2, new_y1]) <= 0.50 %仮想ゴールとの距離が0.5以下の時
                        wo = 0; %重みを0にする
                    end

                    if norm([mx, my] - [new_x2, new_y1]) <= 0.10 %仮想ゴールとの距離が0.1以下の時
                        flag = 0; %障害物を回避した
                        wo = 0.05; %重みを0.05にする
                        flag_rb = 0;
                    end

                end

            end

            j = j + 1;
            % すべてのセンサーが障害物を検出しない場合、ループを抜ける
            if all(isnan(scan1.Cartesian)) && all(isnan(scan2.Cartesian)) && all(isnan(scan3.Cartesian)) && all(isnan(scan4.Cartesian)) && all(isnan(scan5.Cartesian)) && j > 30
                flag = 0;
                flag_rb = 0;
                j = 0;
                break;
            end

        end

    elseif nnz(flag_rb) >= 2

        rb1 = rangeSensor; rb2 = rangeSensor; rb3 = rangeSensor; rb4 = rangeSensor; rb5 = rangeSensor;
        rb1.Range = [0.0 0.7]; rb2.Range = [0.0 0.7]; rb3.Range = [0.0 0.7]; rb4.Range = [0.0 0.7]; rb5.Range = [0.0 0.7];
        rb1.HorizontalAngle = [(pi / 6) - pi / 360 (pi / 6) + pi / 360];
        rb2.HorizontalAngle = [(pi / 180) - pi / 360 (pi / 180) + pi / 360];
        rb3.HorizontalAngle = [(-pi / 6) - pi / 360 (-pi / 6) + pi / 360];
        rb4.HorizontalAngle = [(4 * pi / 9) - pi / 360 (4 * pi / 9) + pi / 360];
        rb5.HorizontalAngle = [(-4 * pi / 9) - pi / 360 (-4 * pi / 9) + pi / 360];
        flag_rb = 0;

        % flag_rbのどれかが1になるまで繰り返す
        while all(flag_rb == 0)
            %ロボットの現在の状況把握
            truePose = [mx my hr]; %ロボットの位置x,yと向きhr
            [ranges, angles] = rb1(truePose, map); %rb1（rangeSensor）にセンサーの姿勢とマップ情報(truePose, map)を代入、出力[ranges, angles]
            scan1 = lidarScan(ranges, angles); %障害物をスキャンする。liberScan
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
                hr = hr - 2 * pi;
            end

            if hr < -pi
                hr = hr + 2 * pi;
            end

            %rb1での障害物の反応
            if ~isnan(scan1.Cartesian) % 障害物を読み取った時「isnan：NaN（読み取りがない）を１、それ以外を０」「scan1.Cartesian：rb1で読み取った値の直行座標」
                s1d = norm(scan1.Cartesian); %ロボットから障害物点までの距離
                s1r = atan(scan1.Cartesian(2) / scan1.Cartesian(1)); %障害物点がロボットを向いている方向からどの角度にあるか
                so = size(o);

                if flag_ob == -1 %障害物の検出が完了し、障害物の座標が追加された
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my]; %ロボットから見た情報から座標系における位置を計算
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                    flag_ob = 0;
                elseif so == [0 0] %oが空の時
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                elseif min(sum(([(s1d * cos(hr + s1r) + mx) .* ones([so(1), 1]), (s1d * sin(hr + s1r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    %検知した障害物の限りなく近い箇所で障害物をまた検出して一か所に何個も障害物を検出するのを回避するための条件  %
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                end

                plot(o_rb1(1), o_rb1(2), 'sr'); %正方形表示
                %     rb1のx　rb1のy
            end

            %rb2での障害物の反応
            if ~isnan(scan2.Cartesian)
                s2d = norm(scan2.Cartesian);
                s2r = atan(scan2.Cartesian(2) / scan2.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                elseif min(sum(([(s2d * cos(hr + s2r) + mx) .* ones([so(1), 1]), (s2d * sin(hr + s2r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                end

                plot(o_rb2(1), o_rb2(2), 's');
            end

            %rb3での障害物の反応
            if ~isnan(scan3.Cartesian)
                s3d = norm(scan3.Cartesian);
                s3r = atan(scan3.Cartesian(2) / scan3.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                elseif min(sum(([(s3d * cos(hr + s3r) + mx) .* ones([so(1), 1]), (s3d * sin(hr + s3r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                end

                plot(o_rb3(1), o_rb3(2), 'sr');
            end

            %rb4での障害物の反応
            if ~isnan(scan4.Cartesian)
                s4d = norm(scan4.Cartesian);
                s4r = atan(scan4.Cartesian(2) / scan4.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                elseif min(sum(([(s4d * cos(hr + s4r) + mx) .* ones([so(1), 1]), (s4d * sin(hr + s4r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                end

                if (o_rb4(1) == 0 && o_rb4(2) == 0) == false
                    plot(o_rb4(1), o_rb4(2), 'sr');
                end

            end

            %rb5での障害物の反応
            if ~isnan(scan5.Cartesian)
                s5 = scan5.Cartesian;
                s5d = norm(scan5.Cartesian);
                s5r = atan(scan5.Cartesian(2) / scan5.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                elseif min(sum(([(s5d * cos(hr + s5r) + mx) .* ones([so(1), 1]), (s5d * sin(hr + s5r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                end

                if (o_rb5(1) == 0 && o_rb5(2) == 0) == false
                    plot(o_rb5(1), o_rb5(2), 'sr');
                end

            end

            % 障害物の速度を更新
            ob_velocity = ob_velocity + ob_acceleration * dt;

            % 障害物の位置を更新
            ob_mv = ob + ob_velocity * dt;

            % Nステップ後の障害物位置を予測
            [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(o, prev_ob_pos, prev_ob_velocity, dt, steps);

            % === 次回に使うため更新 ===
            prev_ob_pos = current_pos;
            prev_ob_velocity = current_velocity;

            ob2_mv_velocity = ob_mv - ob;
            ob3_mv = ob2_mv; %1つあとの障害物の座標
            ob2_mv = ob_mv + 10 * ob2_mv_velocity;
            setOccupancy(map, ob3_mv.', 0);
            setOccupancy(map, ob2_mv.', 1);

            %現時点の障害物の位置を保存

            setOccupancy(map, ob.', 0); %障害物を動かす前のobを消す
            setOccupancy(map, ob_mv.', 1); %障害物を動した後のobを追加する

            ob_prev = ob_mv; % 次回消去するために現在の障害物位置を保存
            % マップを再描画
            show(map);

            ob = ob_mv; % 障害物位置を更新

            % 軌跡や目標位置の描画などの処理

            plot(g(:, 1), g(:, 2), 'b');
            plot(dx, dy, 'ro'); % ゴール
            plot(mx1, my1, 'bo'); % スタート
            plot(mx, my, 'bo'); % robot

            drawnow;
            show(map);

            if hr > pi
                hr = hr - 2 * pi;
            end

            if hr < -pi
                hr = hr + 2 * pi;
            end

            if time > 300
                break;
            end

            truePose = [mx my hr]; %ロボットの位置と向きがtruePose (hr = 0→右方向,pi/2→上方向,piと-pi→左方向,-pi/2→下方向)

            o = [];
            %勾配ベクトルに沿って進める
            temp = normr(double(subs(grad(dx, dy, o, wo, wd), {x y}, {mx, my}))) .* 0.12;

            if temp(2) > 0 %勾配ベクトルが上向き
                hr = acos(temp(1) / 0.12);
            else %勾配ベクトルが下向き
                hr = -acos(temp(1) / 0.12);
            end

            mx = mx + temp(1); %勾配ベクトル方向に進める
            my = my + temp(2);
            g = [g; mx, my]; %gの末尾に移動先の座標を追加

            if norm([mx, my] - [dx, dy]) <= 0.50 %仮想ゴールとの距離が0.5以下の時
                wo = 0; %重みを0にする
            end

            if norm([mx, my] - [dx, dy]) <= 0.10 %仮想ゴールとの距離が0.1以下の時
                flag = 0; %障害物を回避した
                wo = 0.05; %重みを0.05にする
                flag_rb = 0;
            end

        end

        while any(flag_rb == 1)
            o_rb1 = [0 0];
            o_rb2 = [0 0];
            o_rb3 = [0 0];
            o_rb4 = [0 0];
            o_rb5 = [0 0];
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb = [0; 0; 0; 0; 0; 0]; %flag_rb1;flag_rb2;flag_rb3;flag_rb4;flag_rb5;flag_pre;flag_WhichLR
            %　左前，　　　前，　　右前，　　左，　　　　右，　　　　どの方向か
            truePose = [mx my hr];
            [ranges, angles] = rb1(truePose, map);
            scan1 = lidarScan(ranges, angles);
            [ranges, angles] = rb2(truePose, map);
            scan2 = lidarScan(ranges, angles);
            [ranges, angles] = rb3(truePose, map);
            scan3 = lidarScan(ranges, angles);
            [ranges, angles] = rb4(truePose, map);
            scan4 = lidarScan(ranges, angles);
            [ranges, angles] = rb5(truePose, map);
            scan5 = lidarScan(ranges, angles);

            %hr
            if hr > pi
                hr = hr - 2 * pi;
            end

            if hr < -pi
                hr = hr + 2 * pi;
            end

            if ~isnan(scan1.Cartesian)
                s1d = norm(scan1.Cartesian); %ロボットから障害物点までの距離
                s1r = atan(scan1.Cartesian(2) / scan1.Cartesian(1)); %障害物点がロボットを向いている方向からどの角度にあるか
                so = size(o);

                if flag_ob == -1
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                elseif min(sum(([(s1d * cos(hr + s1r) + mx) .* ones([so(1), 1]), (s1d * sin(hr + s1r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    %検知した障害物の限りなく近い箇所で障害物をまた検出して一か所に何個も障害物を検出するのを回避するための条件
                    o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                    o = [o; o_rb1]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(1) = 1;
                end

                plot(o_rb1(1), o_rb1(2), 's'); %正方形表示
            end

            if ~isnan(scan2.Cartesian)
                s2d = norm(scan2.Cartesian);
                s2r = atan(scan2.Cartesian(2) / scan2.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                elseif min(sum(([(s2d * cos(hr + s2r) + mx) .* ones([so(1), 1]), (s2d * sin(hr + s2r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                    o = [o; o_rb2]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(2) = 1;
                end

                plot(o_rb2(1), o_rb2(2), 'rx'); %センサの角度が前の時に障害物を検知するとプロット
            end

            if ~isnan(scan3.Cartesian)
                s3d = norm(scan3.Cartesian);
                s3r = atan(scan3.Cartesian(2) / scan3.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                elseif min(sum(([(s3d * cos(hr + s3r) + mx) .* ones([so(1), 1]), (s3d * sin(hr + s3r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                    o = [o; o_rb3]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(3) = 1;
                end

                plot(o_rb3(1), o_rb3(2), 'bs'); %センサが右前
            end

            if ~isnan(scan4.Cartesian)
                s4d = norm(scan4.Cartesian);
                s4r = atan(scan4.Cartesian(2) / scan4.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                elseif min(sum(([(s4d * cos(hr + s4r) + mx) .* ones([so(1), 1]), (s4d * sin(hr + s4r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                    o = [o; o_rb4]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(4) = 1;
                end

                if (o_rb4(1) == 0 && o_rb4(2) == 0) == false
                    plot(o_rb4(1), o_rb4(2), 'om'); %センサが左　円
                end

            end

            if ~isnan(scan5.Cartesian)
                s5 = scan5.Cartesian;
                s5d = norm(scan5.Cartesian);
                s5r = atan(scan5.Cartesian(2) / scan5.Cartesian(1));
                so = size(o);

                if flag_ob == -1
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                    flag_ob = 0;
                elseif so == [0 0]
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                elseif min(sum(([(s5d * cos(hr + s5r) + mx) .* ones([so(1), 1]), (s5d * sin(hr + s5r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                    o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                    o = [o; o_rb5]; %障害物の座標を格納
                    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                    flag_rb(5) = 1;
                end

                if (o_rb5(1) == 0 && o_rb5(2) == 0) == false
                    plot(o_rb5(1), o_rb5(2), 'b*'); %センサが右
                end

            end

            if nnz(flag_rb) >= 2
                flag_ob = 0;
                i = 0;

                if mx == dx
                    %plot([mx mx],[my dy]);
                    tilt_goal = 0;

                    if my < dy
                        hr = pi / 2;
                    else
                        hr = -pi / 2;
                    end

                else
                    [f_goal, ~, tilt_goal, ~] = Forecast_line(dx, mx, dy, my); %目的x,現在x,目的y,現在y

                    if my < dy

                        if tilt_goal < 0
                            hr = pi + atan(tilt_goal);
                        else
                            hr = atan(tilt_goal);
                        end

                    else

                        if tilt_goal < 0
                            hr = atan(tilt_goal);
                        else
                            hr = pi + atan(tilt_goal);
                        end

                    end

                end

                [flag_rb, hr, theta] = Change_angle(flag_rb, o_tmp, hr, mx, my, dx, dy);

            elseif nnz(flag_rb) == 1
                flag_ob = -1;
                i = i + 1;
            end

            if i == 3
                flag_ob = 0;
                i = 0;
            end

            if flag_rb(6) ~= 0
                %[hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time] = RunningAlongTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time);
                data = 0;

                [hr, g, o, o_tmp, mx, my, dx_tmp, dy_tmp, flag_d, time, ob] = GoBehindTheWall(flag_rb, o_tmp, hr, mx, my, dx, dy, g, o, rb1, rb2, rb3, rb4, rb5, map, time, data, ob_velocity, ob, mx1, my1);
                wo = 0.001;
                flag_d = 0;
                flag_rb = [0; 0; 0; 0; 0; 0];

                if flag_d == 1
                    flag = 1;
                    plot(dx_tmp, dy_tmp, 'rx'); %redのx
                end

                break;
            end

            if flag_ob == -1
                mx = mx + cos(hr) * 0.1;
                my = my + sin(hr) * 0.1;
                g = [g; mx, my];
                i = i + 1;

                if ~isempty(o)
                    o(end, :) = [];
                end

            else

                if flag == 1

                    %temp=normr(double(subs(grad(dx_tmp,dy_tmp,o,wo,wd),{x y},{mx,my}))).*0.12;%勾配ベクトルを正規化して0.12を乗算 %ベクトルの正規化は向きはそのままに大きさを1にすること

                    temp = normr(double(subs(grad(dx_tmp, dy_tmp, o, wo, wd), {x y}, {mx, my}))) .* 0.12;

                    if temp(2) > 0 %勾配ベクトルが上向き
                        hr = acos(temp(1) / 0.12);
                    else %勾配ベクトルが下向き
                        hr = -acos(temp(1) / 0.12);
                    end

                    mx = mx + temp(1); %勾配ベクトル方向に進める
                    my = my + temp(2);
                    g = [g; mx, my];
                    % o = [o; predicted_pos]; %予測した障害物の位置を障害物として追加
                    fprintf('\n\n\n\ポテンシャル場を構築\n');
                    fprintf('障害物の数：%d\n', size(o, 1));
                    disp('障害物の座標：');
                    disp(o);
                    % error("障害物回避のの必要あり　勾配ベクトル上向き");
                    fprintf('\n\n\n');
                    po = double(subs(grad(dx_tmp, dy_tmp, o, wo, wd), {x y}, {mx, my}));
                    time = time + 1;
                    plot(g(:, 1), g(:, 2), 'b');
                    drawnow;

                    if time > 300
                        break;
                    end

                    if norm([mx, my] - [dx_tmp, dy_tmp]) <= 0.50
                        wo = 0;
                    end

                    if norm([mx, my] - [dx_tmp, dy_tmp]) <= 0.10
                        flag = 0;
                        wo = 0.05;
                    end

                    continue;
                else

                    if norm([mx, my] - [dx, dy]) <= 1.0
                        o = [];
                    end

                    temp = normr(double(subs(grad(dx, dy, o, wo, wd), {x y}, {mx, my}))) .* 0.12;

                    if temp(2) > 0
                        hr = acos(temp(1) / 0.12);
                    else
                        hr = -acos(temp(1) / 0.12);
                    end

                    mx = mx + temp(1);
                    my = my + temp(2);
                    g = [g; mx, my];
                    fprintf('\n\n\n\ポテンシャル場を構築\n');
                    fprintf('障害物の数：%d\n', size(o, 1));
                    disp('障害物の座標：');
                    disp(o);
                    % error("障害物回避のの必要あり　勾配ベクトル下向き");
                    fprintf('\n\n\n')
                    po = double(subs(grad(dx, dy, o, wo, wd), {x y}, {mx, my}));
                    time = time + 1;

                    if norm([mx, my] - [dx, dy]) <= 1
                        o = [];
                    end

                    if time > 500
                        break;
                    end

                end

            end

            plot(g(:, 1), g(:, 2), 'b');
            drawnow;
        end

    else

        %距離センサ生成
        [rb1, rb2, rb3, rb4, rb5] = createSensors();
        o_rb1 = [0 0];
        o_rb2 = [0 0];
        o_rb3 = [0 0];
        o_rb4 = [0 0];
        o_rb5 = [0 0];
        o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
        prev_o_tmp = o_tmp; %センサに反応があれば変化する
        flag_rb = [0; 0; 0; 0; 0; 0]; %   左前　前　右前　左　右　どの方向かなど
        %frag_rb(6)=0,1,4,5,7
        %0角度変更なし、1予測線の描画。４角度を左へ、5角度を右へ、7特別な角度変更
        %ロボットの現在の状況把握
        truePose = [mx my hr]; %ロボットの位置x,yと向きhr
        [ranges, angles] = rb1(truePose, map); %rb1（rangeSensor）にセンサーの姿勢とマップ情報(truePose, map)を代入、出力[ranges, angles]
        scan1 = lidarScan(ranges, angles); %障害物をスキャンする。liberScan
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
            hr = hr - 2 * pi;
        end

        if hr < -pi
            hr = hr + 2 * pi;
        end

        %%障害物検知
        %rb1での障害物の反応
        if ~isnan(scan1.Cartesian) % 障害物を読み取った時「isnan：NaN（読み取りがない）を１、それ以外を０」「scan1.Cartesian：rb1で読み取った値の直行座標」
            s1d = norm(scan1.Cartesian); %ロボットから障害物点までの距離
            s1r = atan(scan1.Cartesian(2) / scan1.Cartesian(1)); %障害物点がロボットを向いている方向からどの角度にあるか
            so = size(o);

            if flag_ob == -1 %障害物の検出が完了し、障害物の座標が追加された
                o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my]; %ロボットから見た情報から座標系における位置を計算
                o = [o; o_rb1]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(1) = 1;
                flag_ob = 0;
            elseif so == [0 0] %oが空の時
                o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                o = [o; o_rb1]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(1) = 1;
            elseif min(sum(([(s1d * cos(hr + s1r) + mx) .* ones([so(1), 1]), (s1d * sin(hr + s1r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                %検知した障害物の限りなく近い箇所で障害物をまた検出して一か所に何個も障害物を検出するのを回避するための条件  %
                o_rb1 = [s1d * cos(hr + s1r) + mx, s1d * sin(hr + s1r) + my];
                o = [o; o_rb1]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(1) = 1;
            end

            plot(o_rb1(1), o_rb1(2), 'sr'); %正方形表示
            %     rb1のx　rb1のy
        end

        %rb2での障害物の反応
        if ~isnan(scan2.Cartesian)
            s2d = norm(scan2.Cartesian);
            s2r = atan(scan2.Cartesian(2) / scan2.Cartesian(1));
            so = size(o);

            if flag_ob == -1
                o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                o = [o; o_rb2]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(2) = 1;
                flag_ob = 0;
            elseif so == [0 0]
                o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                o = [o; o_rb2]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(2) = 1;
            elseif min(sum(([(s2d * cos(hr + s2r) + mx) .* ones([so(1), 1]), (s2d * sin(hr + s2r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                o_rb2 = [s2d * cos(hr + s2r) + mx, s2d * sin(hr + s2r) + my];
                o = [o; o_rb2]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(2) = 1;
            end

            plot(o_rb2(1), o_rb2(2), 's');
        end

        %rb3での障害物の反応
        if ~isnan(scan3.Cartesian)
            s3d = norm(scan3.Cartesian);
            s3r = atan(scan3.Cartesian(2) / scan3.Cartesian(1));
            so = size(o);

            if flag_ob == -1
                o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                o = [o; o_rb3]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(3) = 1;
                flag_ob = 0;
            elseif so == [0 0]
                o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                o = [o; o_rb3]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(3) = 1;
            elseif min(sum(([(s3d * cos(hr + s3r) + mx) .* ones([so(1), 1]), (s3d * sin(hr + s3r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                o_rb3 = [s3d * cos(hr + s3r) + mx, s3d * sin(hr + s3r) + my];
                o = [o; o_rb3]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(3) = 1;
            end

            plot(o_rb3(1), o_rb3(2), 'sr');
        end

        %rb4での障害物の反応
        if ~isnan(scan4.Cartesian)
            s4d = norm(scan4.Cartesian);
            s4r = atan(scan4.Cartesian(2) / scan4.Cartesian(1));
            so = size(o);

            if flag_ob == -1
                o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                o = [o; o_rb4]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(4) = 1;
                flag_ob = 0;
            elseif so == [0 0]
                o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                o = [o; o_rb4]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(4) = 1;
            elseif min(sum(([(s4d * cos(hr + s4r) + mx) .* ones([so(1), 1]), (s4d * sin(hr + s4r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                o_rb4 = [s4d * cos(hr + s4r) + mx, s4d * sin(hr + s4r) + my];
                o = [o; o_rb4]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(4) = 1;
            end

            if (o_rb4(1) == 0 && o_rb4(2) == 0) == false
                plot(o_rb4(1), o_rb4(2), 'sr');
            end

        end

        %rb5での障害物の反応
        if ~isnan(scan5.Cartesian)
            s5 = scan5.Cartesian;
            s5d = norm(scan5.Cartesian);
            s5r = atan(scan5.Cartesian(2) / scan5.Cartesian(1));
            so = size(o);

            if flag_ob == -1
                o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                o = [o; o_rb5]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(5) = 1;
                flag_ob = 0;
            elseif so == [0 0]
                o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                o = [o; o_rb5]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(5) = 1;
            elseif min(sum(([(s5d * cos(hr + s5r) + mx) .* ones([so(1), 1]), (s5d * sin(hr + s5r) + my) .* ones([so(1), 1])] - o) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
                o_rb5 = [s5d * cos(hr + s5r) + mx, s5d * sin(hr + s5r) + my];
                o = [o; o_rb5]; %障害物の座標を格納
                o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
                flag_rb(5) = 1;
            end

            if (o_rb5(1) == 0 && o_rb5(2) == 0) == false
                plot(o_rb5(1), o_rb5(2), 'sr');
            end

        end

        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%        kokomade        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if nnz(flag_rb) >= 2 && norm([mx, my] - [dx, dy]) > 0.5 %障害物が2個以上検知した。かつロボットとゴールの距離が0.5以上離れている（ゴールしていない）。
            flag_ob = 0;
            i = 0;

            if mx == dx %ゴールとロボットのx座標が同じとき
                tilt_goal = 0;

                if my < dy %ゴールの真下にいるから
                    hr = pi / 2; %真上を向く
                else %ゴールの真上にいるから
                    hr = -pi / 2; %真下を向く
                end

            else %ゴールとロボットのx座標が異なるとき
                [f_goal, ~, tilt_goal, ~] = Forecast_line(dx, mx, dy, my); %ゴールとロボットを結ぶ直線を作る %f_goalゴールのy座標、tilt_goal傾き %目的x,現在x,目的y,現在y

                if my < dy %ゴールより下

                    if tilt_goal < 0 %傾きが-(ゴールより右側)
                        hr = pi + atan(tilt_goal);
                    else %傾きが0以上(ゴールより左側)
                        hr = atan(tilt_goal);
                    end

                else %ゴールより上

                    if tilt_goal < 0 %傾きが-(ゴールより左側)
                        hr = atan(tilt_goal);
                    else %傾きが0以上(ゴールより右側)
                        hr = pi + atan(tilt_goal);
                    end
                end

            end

            [flag_rb, hr, theta] = Change_angle(flag_rb, o_tmp, hr, mx, my, dx, dy); %調整した向きを反映、予測線を描画する。
        elseif nnz(flag_rb) == 1 %障害物をセンサー１本のみで検知したとき %向きを変えずに
            flag_ob = -1; %
            i = i + 1; %
        end

        if i == 3
            flag_ob = 0;
            i = 0;
        end

        if flag_ob == -1 %ロボットを前に進める。
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

            if flag == 1 %障害物回避の必要あり

                temp = normr(double(subs(grad(dx_tmp, dy_tmp, o, wo, wd), {x y}, {mx, my}))) .* 0.12; %勾配ベクトルを正規化して0.12を乗算 %ベクトルの正規化は向きはそのままに大きさを1にすること

                if temp(2) > 0 %勾配ベクトルが上向き
                    hr = acos(temp(1) / 0.12);
                else %勾配ベクトルが下向き
                    hr = -acos(temp(1) / 0.12);
                end

                mx = mx + temp(1); %勾配ベクトル方向に進める
                my = my + temp(2);
                g = [g; mx, my]; %gの末尾に移動先の座標を追加
                fprintf('\n\n\n\ポテンシャル場を構築\n');
                fprintf('障害物の数：%d\n', size(o, 1));
                disp('障害物の座標：');
                disp(o);
                fprintf('\n\n\n')
                % error('動的障害物検知　停止');
                po = double(subs(grad(dx_tmp, dy_tmp, o, wo, wd), {x y}, {mx, my})); %勾配の再計算
                time = time + 1; %時間を進める
                plot(g(:, 1), g(:, 2), 'r'); %軌跡を描画
                drawnow; %mapに描画

                if time > 300 %時間が300を超えたら停留したとみなしてプログラムを止める
                    break;
                end

                if norm([mx, my] - [dx_tmp, dy_tmp]) <= 0.50 %仮想ゴールとの距離が0.5以下の時
                    wo = 0; %重みを0にする
                end

                if norm([mx, my] - [dx_tmp, dy_tmp]) <= 0.10 %仮想ゴールとの距離が0.1以下の時
                    %o=[];
                    flag = 0; %障害物を回避した
                    wo = 0.05; %重みを0.05にする
                end

                continue; %uhileの最初
            else %通常の走行

                if norm([mx, my] - [dx, dy]) <= 1.0 %ゴールとの距離が1以下の時
                    o = []; %障害物をリセット
                end

                %勾配ベクトルに沿って進める
                temp = normr(double(subs(grad(dx, dy, o, wo, wd), {x y}, {mx, my}))) .* 0.12;

                if temp(2) > 0
                    hr = acos(temp(1) / 0.12);
                else
                    hr = -acos(temp(1) / 0.12);
                end

                mx = mx + temp(1);
                my = my + temp(2);
                g = [g; mx, my];
                % o = [o; predicted_pos;]; %予測した障害物の位置を障害物として追加
                fprintf('\n\n\nポテンシャル場を構築\n');
                fprintf('障害物の数：%d\n', size(o, 1));
                disp('予測障害物の座標：');
                disp(predicted_pos);
                disp('障害物の座標：');
                disp(o);
                if size(o,1) > 1
                    % error("障害物回避の必要あり");
                    fprintf('\n\n\n')
                end

                
                po = double(subs(grad(dx, dy, o, wo, wd), {x y}, {mx, my}));
                time = time + 1;

                if norm([mx, my] - [dx, dy]) <= 1 %ゴールとの距離が1以下の時
                    o = []; %障害物をリセット
                end

                if time > 500
                    break;
                end

            end

        end

        plot(g(:, 1), g(:, 2), 'b'); %軌跡を描画
        drawnow;
    end

    % 障害物の速度を更新
    ob_velocity = ob_velocity + ob_acceleration * dt;

    % 障害物の位置を更新
    ob_mv = ob + ob_velocity * dt;

    % Nステップ後の障害物の予測位置
    [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(o, prev_ob_pos, prev_ob_velocity, dt, steps);

    % === 次回に使うため更新 ===
    prev_ob_pos = current_pos;
    prev_ob_velocity = current_velocity;

    %現時点の障害物の位置を保存

    setOccupancy(map, ob.', 0); %障害物を動かす前のobを消す
    setOccupancy(map, ob_mv.', 1); %障害物を動した後のobを追加する

    ob_prev = ob_mv; % 次回消去するために現在の障害物位置を保存
    % マップを再描画
    show(map);

    ob = ob_mv; % 障害物位置を更新

    % 軌跡や目標位置の描画などの処理

    plot(g(:, 1), g(:, 2), 'b');
    plot(dx, dy, 'ro'); % ゴール
    plot(mx1, my1, 'bo'); % スタート
    plot(mx, my, 'bo'); % robot

    drawnow;
    % disp(flag_rb);
end

time
