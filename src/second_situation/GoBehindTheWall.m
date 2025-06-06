function [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1)
%547
a = 0; %変数a
b = 0; %変数b
p = 1;
flag_wall = 0;
flag_d = 0;
flag = 0;
dx_tmp = 0;
dy_tmp = 0;
o_midx = (o(1,1)+o(2,1))/2;%検出した障害物上の2点の中心x座標
o_midy = (o(1,2)+o(2,2))/2;%検出した障害物上の2点の中心y座標
omx = mx;
omy = my;
distancex = o_midx-mx;
distancey = o_midy-my;
distance = sqrt(power((distancex),2)+power((distancey),2));
d = 0;

 while flag_rb(6) ~= 0 %どれかのセンサに障害物が反応してる間ループ

         ob_mv = ob + ob_velocity; %障害物を動かす

    %現時点の障害物の位置を保存

    setOccupancy(map, ob.', 0);     %障害物を動かす前のobを消す
    setOccupancy(map, ob_mv.', 1);  %障害物を動した後のobを追加する

    % マップを再描画
    show(map);


    ob = ob_mv; % 障害物位置を更新


    % 軌跡や目標位置の描画などの処理


    plot(g(:, 1), g(:, 2), 'b');
    plot(dx, dy, 'ro'); % ゴール
    plot(mx1, my1, 'bo'); % スタート
    plot(mx, my, 'go'); % robot

    drawnow;

        if hr > pi
            hr = hr - 2*pi;
        end

        if hr < -pi
            hr = hr + 2*pi;
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

        mx = mx + cos(hr)*0.2;%ロボット位置の更新
        my = my + sin(hr)*0.2;
        g = [g;mx,my];%ロボット軌跡の更新
      
        if a == 0 %最初にm_tmpにロボット座標を入れる
            m_tmp = [mx,my];
        end
        
        truePose = [mx my hr];%ロボットの位置と向きがtruePose (hr = 0→右方向,pi/2→上方向,piと-pi→左方向,-pi/2→下方向)
        

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
                    hr
                end

                if pi/2 > hr && hr >= 0 %右or上
                    e = 1;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    hrb = hr;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb1(truePose, map);
                        scan1 = lidarScan(ranges, angles);
                        if ~isnan(scan1.Cartesian)
                            %c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                        if (tan(hr) > c && c > tan(hrb)) || (tan(hr) < c && c < tan(hrb))
                            break;
                        end
                        hrb = hr;
                
                        hr = hr + pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex
                        end
                        g = [g;mx,my];%ロボット軌跡の更新
                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        truePose = [mx my hr];
                    end

                elseif pi >= hr && hr >= pi/2 %上or左
                    e = 2;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    hrb = hr;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb1(truePose, map);
                        scan1 = lidarScan(ranges, angles);
                        if ~isnan(scan1.Cartesian)
                            %c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                        if (tan(hr) > c && c > tan(hrb)) || (tan(hr) < c && c < tan(hrb))
                            break;
                        end
                        hrb = hr;
                
                        hr = hr + pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex
                        end
                        g = [g;mx,my];%ロボット軌跡の更新

                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        truePose = [mx my hr];
                    end

                elseif -pi/2 < hr && hr < 0 %右or下
                    e = 3;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    hrb = hr;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb1(truePose, map);
                        scan1 = lidarScan(ranges, angles);
                        if ~isnan(scan1.Cartesian)
                            %c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                       
                        if tan(hr) > c && c > tan(hrb)
                            break;
                        end
                        hrb = hr;
                        hr = hr + pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex
                        end
                        g = [g;mx,my];%ロボット軌跡の更新
                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        
                        if data == 0
                            [tan(hr) c]
                        end
                        truePose = [mx my hr];
                    end

                elseif (-pi <= hr && hr < -pi/2) || hr > pi %下or左
                    e = 4;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    hrb = hr;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb1(truePose, map);
                        scan1 = lidarScan(ranges, angles);
                        if ~isnan(scan1.Cartesian)
                            %c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                
                        if tan(hr) > c && c > tan(hrb)
                            break;
                        end
                        hrb = hr;
                        hr = hr + pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex
                        end
                        g = [g;mx,my];%ロボット軌跡の更新

                        [f_goal,~,c] = Forecast_line(dx,mx,dy,my);
                        truePose = [mx my hr];
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
                    if norm([mx,my]-m_tmp) < 0.1
                        s4d=norm(scan4.Cartesian);
                        s4r=atan(scan4.Cartesian(2)/scan4.Cartesian(1));
                        if rem(a,2) == 0
                            o = [o;s4d*cos(hr+s4r)+mx,s4d*sin(hr+s4r)+my];
                        end
                    end
                else
                        s4d=norm(scan4.Cartesian);
                        s4r=atan(scan4.Cartesian(2)/scan4.Cartesian(1));
                        if rem(a,2) == 0
                            o = [o;s4d*cos(hr+s4r)+mx,s4d*sin(hr+s4r)+my];
                        end
                end
            end
         
        elseif flag_rb(6) == 5 %右に障害物があったら
            [ranges, angles] = rb5(truePose, map);
            scan5 = lidarScan(ranges, angles);
            plot(mx,my,'yo');%yellowのO
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
                
                if pi/2 > hr && hr >= 0 %右or上
                    d = 1;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    hrb = hr;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb3(truePose, map);
                        scan3 = lidarScan(ranges, angles);
                        if ~isnan(scan3.Cartesian)
                            %c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                
                        if tan(hr) < c && c < tan(hrb)
                            break;
                        end
                        hrb = hr;
                        hr = hr - pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex
                        end
                        g = [g;mx,my];%ロボット軌跡の更新
                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        %hr
                        truePose = [mx my hr];
                    end

                elseif pi >= hr && hr >= pi/2 %上or左
                    d = 2;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    hrb = hr;
                    rb3.Range;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb3(truePose, map);
                        scan3 = lidarScan(ranges, angles);
                        if ~isnan(scan3.Cartesian)
                            c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                
                        if tan(hr) < c && c < tan(hrb)
                            break;
                        end
                        hrb = hr;
                        hr = hr - pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey ok
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex ok
                        end
                        g = [g;mx,my];%ロボット軌跡の更新
                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        truePose = [mx my hr];
                    end

                elseif -pi/2 < hr && hr < 0 %右or下
                    d = 3;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                    
                    hrb = hr;
                    while hr - c < 0
                        [ranges, angles] = rb3(truePose, map);
                        scan3 = lidarScan(ranges, angles);
                        if ~isnan(scan3.Cartesian)
                            %c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                            [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                
                        hrb = hr;
                        hr = hr - pi/360;
                        if abs(distancex) < abs(distancey)
                            mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancey)/120;%distancey
                        else
                            mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
                            my = my + sin(hr)*abs(distancex)/120;%distancex
                        end
                        g = [g;mx,my];%ロボット軌跡の更新
                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        truePose = [mx my hr];
                    end

                elseif (-pi <= hr && hr < -pi/2) || hr > pi %下or左
                    d = 4;
                    [~,~,c,~] = Forecast_line(dx,mx,dy,my);%c=目的地現在地直線の傾き
                    hrb = hr;
                    while abs(tan(hr) - c) > 0.01
                        [ranges, angles] = rb3(truePose, map);
                        scan3 = lidarScan(ranges, angles);
                        if ~isnan(scan3.Cartesian)
                            c = 100;
                            if data == 0
                                c
                            end
                            if p == 1
                                plot(mx,my,'rx');
                            end
                           [hr,g,o,o_tmp,mx,my,dx_tmp,dy_tmp,flag_d,time,ob] = GoBehindTheWall(flag_rb,o_tmp,hr,mx,my,dx,dy,g,o,rb1,rb2,rb3,rb4,rb5,map,time,data,ob_velocity,ob,mx1, my1);
                            break;
                        end
                        if tan(hr) < c && c < tan(hrb)
                            break;
                        end
                        hrb = hr;
                        [mx,my,hr] = CircleRunning(mx,my,hr,distancex,distancey);
                        g = [g;mx,my];%ロボット軌跡の更新
                        [~,~,c,~] = Forecast_line(dx,mx,dy,my);
                        truePose = [mx my hr];
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
                    if norm([mx,my]-m_tmp) < 0.1
                        s5d=norm(scan5.Cartesian);
                        s5r=atan(scan5.Cartesian(2)/scan5.Cartesian(1));
                        if rem(a,2) == 0
                            o = [o;s5d*cos(hr+s5r)+mx,s5d*sin(hr+s5r)+my];
                        end
                    end
                else
                   s5d=norm(scan5.Cartesian);
                   s5r=atan(scan5.Cartesian(2)/scan5.Cartesian(1));
                   if rem(a,2) == 0
                      o = [o;s5d*cos(hr+s5r)+mx,s5d*sin(hr+s5r)+my];
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
                o_tmp(2,1) = s2d*cos(hr+s2r)+mx;
                o_tmp(2,2) = s2d*sin(hr+s2r)+my;
                o = [o;o_tmp(2,1),o_tmp(2,2)];
                plot(o_tmp(2,1),o_tmp(2,2),'*r'); %square 壁沿い走行後に対面の障害物にsquareつける 壁沿い走行中に正面に障害物を発見した時にプロット
                flag_d = 1;
                if flag_wall ~= 1
                    flag_wall = 1;
                    if hr > 360*pi
                        hr = hr - pi;
                    elseif hr < 0
                        hr = hr + pi;
                    else
                        hr = hr + pi;
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
                            o_tmp(3,1) = s3d*cos(hr+s3r)+mx;
                            o_tmp(3,2) = s3d*sin(hr+s3r)+my;
                            plot(o_tmp(3,1),o_tmp(3,2),'s'); %square

                            o = [o;o_tmp(3,1),o_tmp(3,2)];
                            flag_rb = [0;1;1;0;0;0];
                            [~,hr,~] = Change_angle(flag_rb,o_tmp,hr,mx,my,dx,dy);
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
                            o_tmp(1,1) = s1d*cos(hr+s1r)+mx;
                            o_tmp(1,2) = s1d*sin(hr+s1r)+my;
                            plot(o_tmp(1,1),o_tmp(1,2),'s');

                            o = [o;o_tmp(1,1),o_tmp(1,2)];
                            flag_rb = [1;1;0;0;0;0];
                            [~,hr,~] = Change_angle(flag_rb,o_tmp,hr,mx,my,dx,dy);
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
           
            g = [g;mx,my];
            a = 0;
            if flag == 4
                dx_tmp = mx + 3*cos(hr + 2*pi/3);
                dy_tmp = my + 3*sin(hr + 2*pi/3);
%                plot(dx_tmp,dy_tmp,'--bo');
                %plot(dx_tmp,dy_tmp,'--rx');
            elseif flag == 5
                dx_tmp = mx + 3*cos(hr - 2*pi/3);
                dy_tmp = my + 3*sin(hr - 2*pi/3);
%                plot(dx_tmp,dy_tmp,'--bo');
                %plot(dx_tmp,dy_tmp,'--rx');
            end
        end
        time = time + 1;
 end
 o = [];
end
 