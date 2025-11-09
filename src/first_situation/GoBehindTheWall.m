function [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1)
%547
a = 0; %変数a
b = 0; %変数b
p = 1;
flag_wall = 0;
flag_d = 0;
flag = 0;
temp_goal_x = 0;
temp_goal_y = 0;
o_midx = (detected_obstacles(1,1)+detected_obstacles(2,1))/2;%検出した障害物上の2点の中心x座標
o_midy = (detected_obstacles(1,2)+detected_obstacles(2,2))/2;%検出した障害物上の2点の中心y座標
omx = robot_x;
omy = robot_y;
distancex = o_midx-robot_x;
distancey = o_midy-robot_y;
distance = sqrt(power((distancex),2)+power((distancey),2));
d = 0;

while flag_rb(6) ~= 0 %どれかのセンサに障害物が反応してる間ループ

    ob_mv = obstacle + obstacle_velocity; %障害物を動かす

    %現時点の障害物の位置を保存

    setOccupancy(map, obstacle.', 0);     %障害物を動かす前のobを消す
    setOccupancy(map, ob_mv.', 1);  %障害物を動した後のobを追加する

    % マップを再描画
    show(map);


    obstacle = ob_mv; % 障害物位置を更新


    % 軌跡や目標位置の描画などの処理


    plot(robot_trajectory(:, 1), robot_trajectory(:, 2), 'b');
    plot(dx, dy, 'ro'); % ゴール
    plot(mx1, my1, 'bo'); % スタート
    plot(robot_x, robot_y, 'go'); % robot

    drawnow;

    if robot_heading > pi
        robot_heading = robot_heading - 2*pi;
    end

    if robot_heading < -pi
        robot_heading = robot_heading + 2*pi;
    end

    if time > 300 && data > 0
        break;
    end

    if abs(distancex) > 0.7 || abs(distancey) > 0.7
        distancex = 0.7;
        distancey = 0.7;
    end
    if abs(distancex) < 0.1
        distancex = 0.1;
    end
    if abs(distancey) < 0.1
        distancey = 0.1;
    end

    robot_x = robot_x + cos(robot_heading)*0.12;%ロボット位置の更新
    robot_y = robot_y + sin(robot_heading)*0.12;
    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新

    if a == 0 %最初にm_tmpにロボット座標を入れる
        m_tmp = [robot_x,robot_y];
    end

    truePose = [robot_x robot_y robot_heading];%ロボットの位置と向きがtruePose (hr = 0→右方向,pi/2→上方向,piと-pi→左方向,-pi/2→下方向)


    if flag_rb(6) == 4 %左に障害物があったら
        [ranges, angles] = rb4(truePose, map);
        scan4 = lidarScan(ranges, angles);
        if isnan(scan4.Cartesian)
            flag_rb(6) = 0;
            flag = 4;
            if data == 0
                z=1
                truePose
                %o_midx
                %o_midy
                %mx
                %my
                %omx
                %omy
                distancex
                distancey
                %distance
                robot_heading
            end

            if pi/2 > robot_heading && robot_heading >= 0 %右or上
                e = 1;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                hrb = robot_heading;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb1(truePose, map);
                    scan1 = lidarScan(ranges, angles);
                    if ~isnan(scan1.Cartesian)
                        %c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end
                    if (tan(robot_heading) > c && c > tan(hrb)) || (tan(robot_heading) < c && c < tan(hrb))
                        break;
                    end
                    hrb = robot_heading;

                    robot_heading = robot_heading + pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新
                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                    truePose = [robot_x robot_y robot_heading];
                end

            elseif pi >= robot_heading && robot_heading >= pi/2 %上or左
                e = 2;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                hrb = robot_heading;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb1(truePose, map);
                    scan1 = lidarScan(ranges, angles);
                    if ~isnan(scan1.Cartesian)
                        %c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end
                    if (tan(robot_heading) > c && c > tan(hrb)) || (tan(robot_heading) < c && c < tan(hrb))
                        break;
                    end
                    hrb = robot_heading;

                    robot_heading = robot_heading + pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新

                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                    truePose = [robot_x robot_y robot_heading];
                end

            elseif -pi/2 < robot_heading && robot_heading < 0 %右or下
                e = 3;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                hrb = robot_heading;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb1(truePose, map);
                    scan1 = lidarScan(ranges, angles);
                    if ~isnan(scan1.Cartesian)
                        %c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end

                    if tan(robot_heading) > c && c > tan(hrb)
                        break;
                    end
                    hrb = robot_heading;
                    robot_heading = robot_heading + pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新
                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);

                    if data == 0
                        [tan(robot_heading) c]
                    end
                    truePose = [robot_x robot_y robot_heading];
                end

            elseif (-pi <= robot_heading && robot_heading < -pi/2) || robot_heading > pi %下or左
                e = 4;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                hrb = robot_heading;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb1(truePose, map);
                    scan1 = lidarScan(ranges, angles);
                    if ~isnan(scan1.Cartesian)
                        %c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end

                    if tan(robot_heading) > c && c > tan(hrb)
                        break;
                    end
                    hrb = robot_heading;
                    robot_heading = robot_heading + pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新

                    [f_goal,~,c] = Forecast_line(dx,robot_x,dy,robot_y);
                    truePose = [robot_x robot_y robot_heading];
                end
            else
                e = 10;
                return
            end
            if data == 0
                c
                e
            end

        else
            if flag_d == 1 %もし仮想ゴールポイントがあったら
                if norm([robot_x,robot_y]-m_tmp) < 0.1
                    s4d=norm(scan4.Cartesian);
                    s4r=atan(scan4.Cartesian(2)/scan4.Cartesian(1));
                    if rem(a,2) == 0
                        detected_obstacles = [detected_obstacles;s4d*cos(robot_heading+s4r)+robot_x,s4d*sin(robot_heading+s4r)+robot_y];
                    end
                end
            else
                s4d=norm(scan4.Cartesian);
                s4r=atan(scan4.Cartesian(2)/scan4.Cartesian(1));
                if rem(a,2) == 0
                    detected_obstacles = [detected_obstacles;s4d*cos(robot_heading+s4r)+robot_x,s4d*sin(robot_heading+s4r)+robot_y];
                end
            end
        end

    elseif flag_rb(6) == 5 %右に障害物があったら
        [ranges, angles] = rb5(truePose, map);
        scan5 = lidarScan(ranges, angles);
        plot(robot_x,robot_y,'yo');%yellowのO
        drawnow;
        if isnan(scan5.Cartesian)
            flag_rb(6) = 0;
            flag = 5;
            if data == 0
                truePose
                %o_midx
                %o_midy
                %mx
                %my
                %omx
                %omy
                distancex
                distancey
                %distance
            end

            if pi/2 > robot_heading && robot_heading >= 0 %右or上
                d = 1;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                hrb = robot_heading;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb3(truePose, map);
                    scan3 = lidarScan(ranges, angles);
                    if ~isnan(scan3.Cartesian)
                        %c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end

                    if tan(robot_heading) < c && c < tan(hrb)
                        break;
                    end
                    hrb = robot_heading;
                    robot_heading = robot_heading - pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新
                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                    %hr
                    truePose = [robot_x robot_y robot_heading];
                end

            elseif pi >= robot_heading && robot_heading >= pi/2 %上or左
                d = 2;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                hrb = robot_heading;
                rb3.Range;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb3(truePose, map);
                    scan3 = lidarScan(ranges, angles);
                    if ~isnan(scan3.Cartesian)
                        c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end

                    if tan(robot_heading) < c && c < tan(hrb)
                        break;
                    end
                    hrb = robot_heading;
                    robot_heading = robot_heading - pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey ok
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex ok
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新
                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                    truePose = [robot_x robot_y robot_heading];
                end

            elseif -pi/2 < robot_heading && robot_heading < 0 %右or下
                d = 3;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);

                hrb = robot_heading;
                while robot_heading - c < 0
                    [ranges, angles] = rb3(truePose, map);
                    scan3 = lidarScan(ranges, angles);
                    if ~isnan(scan3.Cartesian)
                        %c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end

                    hrb = robot_heading;
                    robot_heading = robot_heading - pi/360;
                    if abs(distancex) < abs(distancey)
                        robot_x = robot_x + cos(robot_heading)*abs(distancey)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancey)/120;%distancey
                    else
                        robot_x = robot_x + cos(robot_heading)*abs(distancex)/120;%ちょっと進ませる
                        robot_y = robot_y + sin(robot_heading)*abs(distancex)/120;%distancex
                    end
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新
                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                    truePose = [robot_x robot_y robot_heading];
                end

            elseif (-pi <= robot_heading && robot_heading < -pi/2) || robot_heading > pi %下or左
                d = 4;
                [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);%c=目的地現在地直線の傾き
                hrb = robot_heading;
                while abs(tan(robot_heading) - c) > 0.01
                    [ranges, angles] = rb3(truePose, map);
                    scan3 = lidarScan(ranges, angles);
                    if ~isnan(scan3.Cartesian)
                        c = 100;
                        if data == 0
                            c
                        end
                        if p == 1
                            plot(robot_x,robot_y,'rx');
                        end
                        [robot_heading,robot_trajectory,detected_obstacles,detected_obstacles_tmp,robot_x,robot_y,temp_goal_x,temp_goal_y,flag_d,time,obstacle] = GoBehindTheWall(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy,robot_trajectory,detected_obstacles,rb1,rb2,rb3,rb4,rb5,map,time,data,obstacle_velocity,obstacle,mx1, my1);
                        break;
                    end
                    if tan(robot_heading) < c && c < tan(hrb)
                        break;
                    end
                    hrb = robot_heading;
                    [robot_x,robot_y,robot_heading] = CircleRunning(robot_x,robot_y,robot_heading,distancex,distancey);
                    robot_trajectory = [robot_trajectory;robot_x,robot_y];%ロボット軌跡の更新
                    [~,~,c,~] = Forecast_line(dx,robot_x,dy,robot_y);
                    truePose = [robot_x robot_y robot_heading];
                end
            else
                d = 10;
                return
            end
            if data == 0
                c
                d
            end


        else
            if flag_d == 1
                if norm([robot_x,robot_y]-m_tmp) < 0.1
                    s5d=norm(scan5.Cartesian);
                    s5r=atan(scan5.Cartesian(2)/scan5.Cartesian(1));
                    if rem(a,2) == 0
                        detected_obstacles = [detected_obstacles;s5d*cos(robot_heading+s5r)+robot_x,s5d*sin(robot_heading+s5r)+robot_y];
                    end
                end
            else
                s5d=norm(scan5.Cartesian);
                s5r=atan(scan5.Cartesian(2)/scan5.Cartesian(1));
                if rem(a,2) == 0
                    detected_obstacles = [detected_obstacles;s5d*cos(robot_heading+s5r)+robot_x,s5d*sin(robot_heading+s5r)+robot_y];
                end
            end
        end
    end
    %truePose

    if a ~= 0 %壁沿い走行をしてる途中で障害物を見つけたら
        [ranges, angles] = rb2(truePose, map); %ロボットの前
        scan2 = lidarScan(ranges, angles);
        if ~isnan(scan2.Cartesian)
            s2d = norm(scan2.Cartesian);
            s2r = atan(scan2.Cartesian(2)/scan2.Cartesian(1));
            flag_rb(2) = 1;
            detected_obstacles_tmp(2,1) = s2d*cos(robot_heading+s2r)+robot_x;
            detected_obstacles_tmp(2,2) = s2d*sin(robot_heading+s2r)+robot_y;
            detected_obstacles = [detected_obstacles;detected_obstacles_tmp(2,1),detected_obstacles_tmp(2,2)];
            plot(detected_obstacles_tmp(2,1),detected_obstacles_tmp(2,2),'*r'); %square 壁沿い走行後に対面の障害物にsquareつける 壁沿い走行中に正面に障害物を発見した時にプロット
            flag_d = 1;
            if flag_wall ~= 1
                flag_wall = 1;
                if robot_heading > 360*pi
                    robot_heading = robot_heading - pi;
                elseif robot_heading < 0
                    robot_heading = robot_heading + pi;
                else
                    robot_heading = robot_heading + pi;
                end
                if flag_rb(6) == 4
                    flag_rb(6) = 5;
                elseif flag_rb(6) == 5
                    flag_rb(6) = 4;
                end
            else
                if flag_rb(6) == 4
                    rb3.Range = [0.0 2.0];
                    [ranges, angles] = rb3(truePose, map);
                    scan3 = lidarScan(ranges, angles);
                    if ~isnan(scan3.Cartesian)
                        s3d = norm(scan3.Cartesian);
                        s3r = atan(scan3.Cartesian(2)/scan3.Cartesian(1));
                        detected_obstacles_tmp(3,1) = s3d*cos(robot_heading+s3r)+robot_x;
                        detected_obstacles_tmp(3,2) = s3d*sin(robot_heading+s3r)+robot_y;
                        plot(detected_obstacles_tmp(3,1),detected_obstacles_tmp(3,2),'s'); %square

                        detected_obstacles = [detected_obstacles;detected_obstacles_tmp(3,1),detected_obstacles_tmp(3,2)];
                        flag_rb = [0;1;1;0;0;0];
                        [~,robot_heading,~] = Change_angle(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy);
                        flag_rb(6) = 5;
                        flag_wall = 0;
                        flag_d = 1;
                        rb3.Range = [0.0 0.75];
                    else
                        flag_d = 0;
                        flag_rb(6) = 0;

                    end
                elseif flag_rb(6) == 5
                    rb1.Range = [0.0 2.0];
                    [ranges, angles] = rb1(truePose, map);
                    scan1 = lidarScan(ranges, angles);
                    if ~isnan(scan1.Cartesian)
                        s1d = norm(scan1.Cartesian);
                        s1r = atan(scan1.Cartesian(2)/scan1.Cartesian(1));
                        detected_obstacles_tmp(1,1) = s1d*cos(robot_heading+s1r)+robot_x;
                        detected_obstacles_tmp(1,2) = s1d*sin(robot_heading+s1r)+robot_y;
                        plot(detected_obstacles_tmp(1,1),detected_obstacles_tmp(1,2),'s');

                        detected_obstacles = [detected_obstacles;detected_obstacles_tmp(1,1),detected_obstacles_tmp(1,2)];
                        flag_rb = [1;1;0;0;0;0];
                        [~,robot_heading,~] = Change_angle(flag_rb,detected_obstacles_tmp,robot_heading,robot_x,robot_y,dx,dy);
                        flag_rb(6) = 5;
                        flag_wall = 0;
                        flag_d = 1;
                        rb1.Range = [0.0 0.75];
                    else
                        flag_d = 0;
                        flag_rb(6) = 0;

                    end
                end
            end
        end
    end

    a = a + 1;

    if flag_rb(6) == 0

        robot_trajectory = [robot_trajectory;robot_x,robot_y];
        a = 0;
        if flag == 4
            temp_goal_x = robot_x + 3*cos(robot_heading + 2*pi/3);
            temp_goal_y = robot_y + 3*sin(robot_heading + 2*pi/3);
            %                plot(dx_tmp,dy_tmp,'--bo');
            %plot(dx_tmp,dy_tmp,'--rx');
        elseif flag == 5
            temp_goal_x = robot_x + 3*cos(robot_heading - 2*pi/3);
            temp_goal_y = robot_y + 3*sin(robot_heading - 2*pi/3);
            %                plot(dx_tmp,dy_tmp,'--bo');
            %plot(dx_tmp,dy_tmp,'--rx');
        end
    end
    time = time + 1;
end
detected_obstacles = [];
end
