clear;
% %斜め
goal_x = 11; %　ゴールの初期位置
goal_y = 11;
robot_x = -1; %　ロボットの初期位置
robot_y = -1;
obstacle_velocity = [0.035; -0.035];
%斜め障害物
obstacle_left_x = -1;   % 障害物の左辺のx座標
obstacle_right_x = 1;   % 障害物の右辺のx座標
obstacle_bottom_y = 8;   % 障害物の底辺のy座標
obstacle_top_y = 10;   % 障害物の上辺のy座標

% % %横
% goal_x = 6;
% goal_y = 12;
% robot_x = 6;
% robot_y = 0;
% obstacle_velocity = [0.035; 0];
% %横向き
% obstacle_left_x = 0; % 障害物の左辺のx座標
% obstacle_right_x = 2; % 障害物の右辺のx座標
% obstacle_bottom_y = 6; % 障害物の底辺のy座標
% obstacle_top_y = 8; % 障害物の上辺のy座標

%縦
% goal_x = 6;
% goal_y = 12;
% robot_x = 6;
% robot_y = 0;
% obstacle_velocity = [0; -0.035];
% %縦の確認用
% obstacle_left_x = 5;   % 障害物の左辺のx座標
% obstacle_right_x = 7;   % 障害物の右辺のx座標
% obstacle_bottom_y = 7;   % 障害物の底辺のy座標
% obstacle_top_y = 9;   % 障害物の上辺のy座標

temp_goal_x = 0;
temp_goal_y = 0;

initial_robot_x = robot_x;
initial_robot_y = robot_y;
robot_heading = 0;
detected_obstacles = []; %障害物検出点の格納用
obstacle_detection_count = [0 0]; %障害物の検出数の格納用
obstacle_weight = 0.05; %小さいほど小さなステップで進む。 初期値
% obstacle_weight = 0.0001; %小さいほど小さなステップで進む。
distance_weight = 300;
point = [0 0];
radius = 36;
avoidance_flag = 0; %1:障害物回避が必要。　0:通常の走行状態（目的地にまっすぐ進む）
obstacle_detection_flag = 0; %0:障害物が検出されていない状態。　-1:障害物を検出して、障害物の座標が格納された状態。
%frag_mv = 0;
time_step = 1;
detection_counter = 0;
data = 0;

% 加速度計算用の変数を初期化
obstacle_acceleration = [0.0020; -0.0020]; % 障害物の加速度 (例: y方向に-0.005)
ob_prev = []; % 前回の障害物位置
prev_ob_pos = [];
prev_ob_velocity = [0; 0];
steps = 20; % Nステップ後の予測に使う
dt = 1;

% 予測した位置を保存する変数
predicted_pos = [];

h_predicted = []; % 前回の予測点プロットを保持

%距離センサ生成
range_sensor_1 = rangeSensor;
range_sensor_2 = rangeSensor;
range_sensor_3 = rangeSensor;
range_sensor_4 = rangeSensor;
range_sensor_5 = rangeSensor;
% センサの感知範囲
% range_sensor_1.Range = [0.0 0.7]; %距離
% range_sensor_2.Range = [0.0 0.7];
% range_sensor_3.Range = [0.0 0.7];
% range_sensor_4.Range = [0.0 0.7];
% range_sensor_5.Range = [0.0 0.7];
range_sensor_1.Range = [0.0 3.0]; %距離
range_sensor_2.Range = [0.0 3.0];
range_sensor_3.Range = [0.0 3.0];
range_sensor_4.Range = [0.0 3.0];
range_sensor_5.Range = [0.0 3.0];
range_sensor_1.HorizontalAngle = [(pi / 6) - pi / 360 (pi / 6) + pi / 360]; %角度
range_sensor_2.HorizontalAngle = [(pi / 180) - pi / 360 (pi / 180) + pi / 360];
range_sensor_3.HorizontalAngle = [(-pi / 6) - pi / 360 (-pi / 6) + pi / 360];
range_sensor_4.HorizontalAngle = [(4 * pi / 9) - pi / 360 (4 * pi / 9) + pi / 360];
range_sensor_5.HorizontalAngle = [(-4 * pi / 9) - pi / 360 (-4 * pi / 9) + pi / 360];

%マップ生成（ｘ、ｙ、幅[1マスの幅]）
occupancy_map = binaryOccupancyMap(15, 15, 20);
%グリッド点設定（ｘ、ｙ）(mapの左下の点)
occupancy_map.GridLocationInWorld = [-2 -2];

x = obstacle_left_x:0.01:obstacle_right_x; % x座標の範囲
y = obstacle_bottom_y:0.01:obstacle_top_y; % y座標の範囲

obstacle = [x, obstacle_right_x .* ones(size(y)), fliplr(x), obstacle_left_x .* ones(size(y));
    obstacle_bottom_y .* ones(size(x)), y, obstacle_top_y .* ones(size(x)), fliplr(y)];
% new_obstacle = [x+6, obstacle_right_x.*ones(size(y))+6, fliplr(x)+6, obstacle_left_x.*ones(size(y))+6;
%   (obstacle_bottom_y+2).*ones(size(x)), y+2, (obstacle_top_y+2).*ones(size(x)), fliplr(y)+2];

move_flag = 1; %move

robot_trajectory = [robot_x, robot_y]; %ロボットの軌跡

%マップに障害物を割り当て（map, 障害物[転置(.')してn行２列にしている],確率占有値[?][0で消える、１で追加できる]）
setOccupancy(occupancy_map, obstacle.', 1);
% setOccupancy(occupancy_map,new_obstacle.',1);
%s=ones(size(obstacle.')); %.'は転置 %obstacle.'と同じサイズで要素がすべて１の行列
%setOccupancy(occupancy_map,obstacle.',s(:,1));%上のコードと同じ、要素それぞれに１をかけるか、１をまとめてかけるかの違い
%inflate(occupancy_map,0.1); %障害物を膨らませる

show(occupancy_map); %mapの描画
hold on %mapの固定
title('Field');

plot(goal_x, goal_y, 'ro'); %ゴール
plot(robot_x, robot_y, 'bo'); %スタート(ロボットの初期位置)

%シンボリックとして定義（文字式の計算ができるようになる）
syms x;
syms y;
% po = double(subs(grad(goal_x, goal_y, detected_obstacles, obstacle_weight, distance_weight), {x y}, {robot_x, robot_y})); %subs(s,old,new) シンボリックsのoldの各要素をそれぞれnewに対応する要素で書き換え。

while norm([robot_x, robot_y] - [goal_x, goal_y]) > 0.10 %ロボットが目的地に着くまでループ (2点間のユークリッド距離)
    o_rb1 = [0 0];
    o_rb2 = [0 0];
    o_rb3 = [0 0];
    o_rb4 = [0 0];
    o_rb5 = [0 0];
    o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
    flag_rb = [0; 0; 0; 0; 0; 0]; %   左前　前　右前　左　右　どの方向かなど
    %frag_rb(6)=0,1,4,5,7
    %0角度変更なし、1予測線の描画。４角度を左へ、5角度を右へ、7特別な角度変更
    %ロボットの現在の状況把握
    truePose = [robot_x robot_y robot_heading]; %ロボットの位置x,yと向きhr
    [ranges, angles] = range_sensor_1(truePose, occupancy_map); %rb1（rangeSensor）にセンサーの姿勢とマップ情報(truePose, map)を代入、出力[ranges, angles]
    scan1 = lidarScan(ranges, angles); %障害物をスキャンする。liberScan
    [ranges, angles] = range_sensor_2(truePose, occupancy_map);
    scan2 = lidarScan(ranges, angles);
    [ranges, angles] = range_sensor_3(truePose, occupancy_map);
    scan3 = lidarScan(ranges, angles);
    [ranges, angles] = range_sensor_4(truePose, occupancy_map);
    scan4 = lidarScan(ranges, angles);
    [ranges, angles] = range_sensor_5(truePose, occupancy_map);
    scan5 = lidarScan(ranges, angles);

    %向きhrの範囲を-piからpiに
    if robot_heading > pi
        robot_heading = robot_heading - 2 * pi;
    end

    if robot_heading < -pi
        robot_heading = robot_heading + 2 * pi;
    end

    %%障害物検知
    %rb1での障害物の反応
    if ~isnan(scan1.Cartesian) % 障害物を読み取った時「isnan：NaN（読み取りがない）を１、それ以外を０」「scan1.Cartesian：rb1で読み取った値の直行座標」
        s1d = norm(scan1.Cartesian); %ロボットから障害物点までの距離
        s1r = atan(scan1.Cartesian(2) / scan1.Cartesian(1)); %障害物点がロボットを向いている方向からどの角度にあるか
        so = size(detected_obstacles);

        if obstacle_detection_flag == -1 %障害物の検出が完了し、障害物の座標が追加された
            o_rb1 = [s1d * cos(robot_heading + s1r) + robot_x, s1d * sin(robot_heading + s1r) + robot_y]; %ロボットから見た情報から座標系における位置を計算
            detected_obstacles = [detected_obstacles; o_rb1]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(1) = 1;
            obstacle_detection_flag = 0;
        elseif so == [0 0] %oが空の時
            o_rb1 = [s1d * cos(robot_heading + s1r) + robot_x, s1d * sin(robot_heading + s1r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb1]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(1) = 1;
        elseif min(sum(([(s1d * cos(robot_heading + s1r) + robot_x) .* ones([so(1), 1]), (s1d * sin(robot_heading + s1r) + robot_y) .* ones([so(1), 1])] - detected_obstacles) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
            %検知した障害物の限りなく近い箇所で障害物をまた検出して一か所に何個も障害物を検出するのを回避するための条件  %
            o_rb1 = [s1d * cos(robot_heading + s1r) + robot_x, s1d * sin(robot_heading + s1r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb1]; %障害物の座標を格納
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
        so = size(detected_obstacles);

        if obstacle_detection_flag == -1
            o_rb2 = [s2d * cos(robot_heading + s2r) + robot_x, s2d * sin(robot_heading + s2r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb2]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(2) = 1;
            obstacle_detection_flag = 0;
        elseif so == [0 0]
            o_rb2 = [s2d * cos(robot_heading + s2r) + robot_x, s2d * sin(robot_heading + s2r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb2]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(2) = 1;
        elseif min(sum(([(s2d * cos(robot_heading + s2r) + robot_x) .* ones([so(1), 1]), (s2d * sin(robot_heading + s2r) + robot_y) .* ones([so(1), 1])] - detected_obstacles) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
            o_rb2 = [s2d * cos(robot_heading + s2r) + robot_x, s2d * sin(robot_heading + s2r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb2]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(2) = 1;
        end

        plot(o_rb2(1), o_rb2(2), 's');
    end

    %rb3での障害物の反応
    if ~isnan(scan3.Cartesian)
        s3d = norm(scan3.Cartesian);
        s3r = atan(scan3.Cartesian(2) / scan3.Cartesian(1));
        so = size(detected_obstacles);

        if obstacle_detection_flag == -1
            o_rb3 = [s3d * cos(robot_heading + s3r) + robot_x, s3d * sin(robot_heading + s3r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb3]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(3) = 1;
            obstacle_detection_flag = 0;
        elseif so == [0 0]
            o_rb3 = [s3d * cos(robot_heading + s3r) + robot_x, s3d * sin(robot_heading + s3r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb3]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(3) = 1;
        elseif min(sum(([(s3d * cos(robot_heading + s3r) + robot_x) .* ones([so(1), 1]), (s3d * sin(robot_heading + s3r) + robot_y) .* ones([so(1), 1])] - detected_obstacles) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
            o_rb3 = [s3d * cos(robot_heading + s3r) + robot_x, s3d * sin(robot_heading + s3r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb3]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(3) = 1;
        end

        plot(o_rb3(1), o_rb3(2), 'sr');
    end

    %rb4での障害物の反応
    if ~isnan(scan4.Cartesian)
        s4d = norm(scan4.Cartesian);
        s4r = atan(scan4.Cartesian(2) / scan4.Cartesian(1));
        so = size(detected_obstacles);

        if obstacle_detection_flag == -1
            o_rb4 = [s4d * cos(robot_heading + s4r) + robot_x, s4d * sin(robot_heading + s4r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb4]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(4) = 1;
            obstacle_detection_flag = 0;
        elseif so == [0 0]
            o_rb4 = [s4d * cos(robot_heading + s4r) + robot_x, s4d * sin(robot_heading + s4r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb4]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(4) = 1;
        elseif min(sum(([(s4d * cos(robot_heading + s4r) + robot_x) .* ones([so(1), 1]), (s4d * sin(robot_heading + s4r) + robot_y) .* ones([so(1), 1])] - detected_obstacles) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
            o_rb4 = [s4d * cos(robot_heading + s4r) + robot_x, s4d * sin(robot_heading + s4r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb4]; %障害物の座標を格納
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
        so = size(detected_obstacles);

        if obstacle_detection_flag == -1
            o_rb5 = [s5d * cos(robot_heading + s5r) + robot_x, s5d * sin(robot_heading + s5r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb5]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(5) = 1;
            obstacle_detection_flag = 0;
        elseif so == [0 0]
            o_rb5 = [s5d * cos(robot_heading + s5r) + robot_x, s5d * sin(robot_heading + s5r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb5]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(5) = 1;
        elseif min(sum(([(s5d * cos(robot_heading + s5r) + robot_x) .* ones([so(1), 1]), (s5d * sin(robot_heading + s5r) + robot_y) .* ones([so(1), 1])] - detected_obstacles) .^ 2, 2)) > [0.2 ^ 2 0.2 ^ 2]
            o_rb5 = [s5d * cos(robot_heading + s5r) + robot_x, s5d * sin(robot_heading + s5r) + robot_y];
            detected_obstacles = [detected_obstacles; o_rb5]; %障害物の座標を格納
            o_tmp = [o_rb1; o_rb2; o_rb3; o_rb4; o_rb5];
            flag_rb(5) = 1;
        end

        if (o_rb5(1) == 0 && o_rb5(2) == 0) == false
            plot(o_rb5(1), o_rb5(2), 'sr');
        end

    end


    if nnz(flag_rb) >= 1
        latest_obstacle = obstacle(:, end)'; % ←ここで転置
        % [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(detected_obstacles, prev_ob_pos, prev_ob_velocity, dt, steps,obstacle_velocity,obstacle_acceleration);
        [current_pos, predicted_pos, current_velocity, current_acceleration] = predict_obstacle_position(latest_obstacle, prev_ob_pos, prev_ob_velocity, dt, steps, obstacle_velocity, obstacle_acceleration);
        prev_ob_pos = current_pos;
        prev_ob_velocity = current_velocity;

        if all(~isnan(predicted_pos))
            detected_obstacles = [detected_obstacles; predicted_pos]; %予測した位置を障害物リストに追加

            % 前回の予測点を削除
            if ~isempty(h_predicted) && isvalid(h_predicted)
                delete(h_predicted);
            end

            % 新しい予測位置を描画
            h_predicted = plot(predicted_pos(:,1), predicted_pos(:,2), ...
                'r-', ...       % 赤色の実線
                'LineWidth', 2);% 太さ
        end

        % if ~isempty(detected_obstacles)
        %     display('detected_obstacles:');
        %     disp(detected_obstacles);
        % end
    end

    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%        kokomade        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if nnz(flag_rb) >= 2 && norm([robot_x, robot_y] - [goal_x, goal_y]) > 0.5 %障害物が2個以上検知した。かつロボットとゴールの距離が0.5以上離れている（ゴールしていない）。
        obstacle_detection_flag = 0; %0:障害物が検出されていない状態。　-1:障害物を検出して、障害物の座標が格納された状態。
        detection_counter = 0;

        if robot_x == goal_x %ゴールとロボットのx座標が同じとき
            %plot([robot_x robot_x],[robot_y goal_y]);
            tilt_goal = 0;

            if robot_y < goal_y %ゴールの真下にいるから
                robot_heading = pi / 2; %真上を向く
            else %ゴールの真上にいるから
                robot_heading = -pi / 2; %真下を向く
            end

        else %ゴールとロボットのx座標が異なるとき
            [f_goal, ~, tilt_goal] = Forecast_line(goal_x, robot_x, goal_y, robot_y); %ゴールとロボットを結ぶ直線を作る %f_goalゴールのy座標、tilt_goal傾き %目的x,現在x,目的y,現在y

            if robot_y < goal_y %ゴールより下

                if tilt_goal < 0 %傾きが-(ゴールより右側)
                    robot_heading = pi + atan(tilt_goal);
                else %傾きが0以上(ゴールより左側)
                    robot_heading = atan(tilt_goal);
                end

            else %ゴールより上

                if tilt_goal < 0 %傾きが-(ゴールより左側)
                    robot_heading = atan(tilt_goal);
                else %傾きが0以上(ゴールより右側)
                    robot_heading = pi + atan(tilt_goal);
                end

            end

        end

        [flag_rb, robot_heading, theta] = Change_angle(flag_rb, o_tmp, robot_heading, robot_x, robot_y, goal_x, goal_y); %調整した向きを反映、予測線を描画する。
    elseif nnz(flag_rb) == 1 %障害物をセンサー１本のみで検知したとき %向きを変えずに
        obstacle_detection_flag = -1; %
        detection_counter = detection_counter + 1; %
    end

    if detection_counter == 3
        obstacle_detection_flag = 0;
        detection_counter = 0;
    end

    if flag_rb(6) == 0
        %obstacle_weight = 0.05;
        %plot(robot_x,robot_y,'rx');%redのx
    end

    while flag_rb(6) ~= 0 %角度変更または予測線分の描画の必要あり。
        fprintf("----- flag_rb reset -----\n")
        data = 0;
        %[robot_heading,robot_trajectory,detected_obstacles,o_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,~,time_step] = GoBehindTheWall(flag_rb,o_tmp,robot_heading,robot_x,robot_y,goal_x,goal_y,robot_trajectory,detected_obstacles,range_sensor_1,range_sensor_2,range_sensor_3,range_sensor_4,range_sensor_5,occupancy_map,time_step,data);
        obstacle_weight = 0.1; %障害物の重みを大きくする
        flag_d = 0;
        flag_rb = [0; 0; 0; 0; 0; 0];
    end

    if obstacle_detection_flag == -1 %ロボットを前に進める。 0:障害物が検出されていない状態。　-1:障害物を検出して、障害物の座標が格納された状態。
        %frag_mv = 0;

        robot_x = robot_x + cos(robot_heading) * 0.1;
        robot_y = robot_y + sin(robot_heading) * 0.1;
        robot_trajectory = [robot_trajectory; robot_x, robot_y];
        detection_counter = detection_counter + 1;

        % ゴールの方向を再確認
        goal_direction = atan2(goal_y - robot_y, goal_x - robot_x);
        robot_heading = goal_direction;
    else

        % 障害物回避後にゴールの方向を再確認するロジック

        if avoidance_flag == 1 %障害物回避の必要あり
            error("障害物回避の必要あり");

            temp = normr(double(subs(grad(temp_goal_x, temp_goal_y, detected_obstacles, obstacle_weight, distance_weight), {x y}, {robot_x, robot_y}))) .* 0.12; %勾配ベクトルを正規化して0.12を乗算 %ベクトルの正規化は向きはそのままに大きさを1にすること

            if temp(2) > 0 %勾配ベクトルが上向き
                robot_heading = acos(temp(1) / 0.12);
            else %勾配ベクトルが下向き
                robot_heading = -acos(temp(1) / 0.12);
            end

            robot_x = robot_x + temp(1); %勾配ベクトル方向に進める
            robot_y = robot_y + temp(2);
            robot_trajectory = [robot_trajectory; robot_x, robot_y]; %gの末尾に移動先の座標を追加
            % po = double(subs(grad(temp_goal_x, temp_goal_y, detected_obstacles, obstacle_weight, distance_weight), {x y}, {robot_x, robot_y})); %勾配の再計算
            time_step = time_step + 1; %時間を進める
            plot(robot_trajectory(:, 1), robot_trajectory(:, 2), 'r'); %軌跡を描画
            drawnow; %mapに描画

            if time_step > 300 %時間が300を超えたら停留したとみなしてプログラムを止める
                break;
            end

            if norm([robot_x, robot_y] - [temp_goal_x, temp_goal_y]) <= 0.50 %仮想ゴールとの距離が0.5以下の時
                obstacle_weight = 0; %重みを0にする
            end

            if norm([robot_x, robot_y] - [temp_goal_x, temp_goal_y]) <= 0.10 %仮想ゴールとの距離が0.1以下の時
                %detected_obstacles=[];
                avoidance_flag = 0; %障害物を回避した
                obstacle_weight = 0.05; %重みを0.05にする
            end

            continue; %uhileの最初
        else %通常の走行
            %勾配ベクトルに沿って進める


            if ~isempty(detected_obstacles)
                fprintf('\n\n--------------  detected_obstacles: --------------\n');
                disp(detected_obstacles);
                fprintf('---------------------------------------------------\n\n\n')
            end
            temp = normr(double(subs(grad(goal_x, goal_y, detected_obstacles, obstacle_weight, distance_weight), {x y}, {robot_x, robot_y}))) .* 0.12;

            if temp(2) > 0
                robot_heading = acos(temp(1) / 0.12);
            else
                robot_heading = -acos(temp(1) / 0.12);
            end

            robot_x = robot_x + temp(1);
            robot_y = robot_y + temp(2);
            robot_trajectory = [robot_trajectory; robot_x, robot_y];
            % po = double(subs(grad(goal_x, goal_y, detected_obstacles, obstacle_weight, distance_weight), {x y}, {robot_x, robot_y}));
            time_step = time_step + 1;

            if norm([robot_x, robot_y] - [goal_x, goal_y]) <= 1 %ゴールとの距離が1以下の時
                detected_obstacles = []; %障害物をリセット
            end

            if time_step > 500
                break;
            end

        end

    end

    plot(robot_trajectory(:, 1), robot_trajectory(:, 2), 'b'); %軌跡を描画
    drawnow;

    if nnz(flag_rb) <= 1
        detected_obstacles = [];
    end

    %%%障害物を動かす
    if move_flag == 1
        %障害物を動かす幅
        obstacle_velocity = obstacle_velocity + obstacle_acceleration * dt; %障害物の速度を更新
        obstacle_mv = obstacle + obstacle_velocity * dt; %障害物を動かす

        setOccupancy(occupancy_map, obstacle.', 0); %障害物を動かす前のobstacleを消す
        setOccupancy(occupancy_map, obstacle_mv.', 1); %障害物を動した後のobstacleを追加する

        % マップを再描画
        show(occupancy_map);

        obstacle = obstacle_mv; % 障害物位置を更新
        % fprintf('Obstacle moved to new position: (%.4f,%.4f) \n', obstacle(1), obstacle(2)); % 障害物の新しい位置を表示
        % disp(obstacle);

        % 軌跡や目標位置の描画などの処理
        hold on;
        plot(robot_trajectory(:, 1), robot_trajectory(:, 2), 'b');
        plot(goal_x, goal_y, 'ro'); % ゴール
        plot(initial_robot_x, initial_robot_y, 'bo'); % スタート

        plot(robot_x, robot_y, 'bo'); % robot
    end

    drawnow;
end

time_step
