function [flag_rb,hr,theta] = Change_angle(flag_rb,o_tmp,hr,mx,my,dx,dy)

    theta = 0;
    [~,~,tilt_goal,~] = Forecast_line(dx,mx,dy,my);
    %予測線の描画
        if flag_rb(1) == 1 && flag_rb(4) == 1
            flag_rb(6) = 1;
            if o_tmp(1) == o_tmp(4)
                theta = atan((mx - dx)/(my - dy));
                %plot([o_tmp(1) o_tmp(1)],[o_tmp(1,2) o_tmp(4,2)],'--');
                if my > dy
                    hr = -pi/2;
                    if o_tmp(1) > mx
                        flag_rb(6) = 4;
                    elseif o_tmp(1) < mx
                        flag_rb(6) = 5;
                    end
                else
                    hr = pi/2;
                    if o_tmp(1) > mx
                        flag_rb(6) = 5;
                    elseif o_tmp(1) < mx
                        flag_rb(6) = 4;
                    end
                end
            elseif mx == dx
                theta = -atan((o_tmp(1) - o_tmp(4))/(o_tmp(1,2) - o_tmp(4,2)));
                [~,~,tilt_L,intercept_L] = Forecast_line(o_tmp(1),o_tmp(4),o_tmp(1,2),o_tmp(4,2));
                if mx*tilt_L+intercept_L > my && dx*tilt_L+intercept_L > dy
                    flag_rb(6) = 7;
                elseif mx*tilt_L+intercept_L < my && dx*tilt_L+intercept_L < dy
                    flag_rb(6) = 7;
                end
            else
                [f_L,x,tilt_L,intercept_L] = Forecast_line(o_tmp(1),o_tmp(4),o_tmp(1,2),o_tmp(4,2));
                %plot(x,f_L,'--');
                theta = calAngle(tilt_goal,tilt_L);
                if mx*tilt_L+intercept_L > my && dx*tilt_L+intercept_L > dy
                    flag_rb(6) = 7;
                elseif mx*tilt_L+intercept_L < my && dx*tilt_L+intercept_L < dy
                    flag_rb(6) = 7;
                end
            end
       elseif flag_rb(1) == 1 && flag_rb(2) == 1
            flag_rb(6) = 1;
            if o_tmp(1) == o_tmp(2)
                theta = atan((mx - dx)/(my - dy));
                %plot([o_tmp(1) o_tmp(1)],[o_tmp(1,2) o_tmp(2,2)],'--');
                if my > dy
                    hr = -pi/2;
                    if o_tmp(1) > mx
                        flag_rb(6) = 4;
                    elseif o_tmp(1) < mx
                        flag_rb(6) = 5;
                    end
                else
                    hr = pi/2;
                    if o_tmp(1) > mx
                        flag_rb(6) = 5;
                    elseif o_tmp(1) < mx
                        flag_rb(6) = 4;
                    end
                end
            elseif mx == dx
                theta = -atan((o_tmp(2) - o_tmp(1))/(o_tmp(2,2) - o_tmp(1,2)));
                [~,~,tilt_L,intercept_L] = Forecast_line(o_tmp(1),o_tmp(2),o_tmp(1,2),o_tmp(2,2));
                if mx*tilt_L+intercept_L > my && dx*tilt_L+intercept_L > dy
                    flag_rb(6) = 7;
                elseif mx*tilt_L+intercept_L < my && dx*tilt_L+intercept_L < dy
                    flag_rb(6) = 7;
                end
            else
                [f_L,x,tilt_L,intercept_L] = Forecast_line(o_tmp(1),o_tmp(2),o_tmp(1,2),o_tmp(2,2));
                %plot(x,f_L,'--');
                theta = calAngle(tilt_goal,tilt_L);
                if mx*tilt_L+intercept_L > my && dx*tilt_L+intercept_L > dy
                    flag_rb(6) = 7;
                elseif mx*tilt_L+intercept_L < my && dx*tilt_L+intercept_L < dy
                    flag_rb(6) = 7;
                end
            end
        end
        if flag_rb(6) == 0
            if flag_rb(3) == 1 && flag_rb(5) == 1
                flag_rb(6)= 1;
                if o_tmp(3) == o_tmp(5)
                    theta = atan((mx - dx)/(my - dy));
                    %plot([o_tmp(3) o_tmp(3)],[o_tmp(3,2) o_tmp(5,2)],'--');
                    if my > dy
                        hr = -pi/2;
                        if o_tmp(1) > mx
                            flag_rb(6) = 4;
                        elseif o_tmp(1) < mx
                            flag_rb(6) = 5;
                        end
                    else
                        hr = pi/2;
                        if o_tmp(1) > mx
                            flag_rb(6) = 5;
                        elseif o_tmp(1) < mx
                            flag_rb(6) = 4;
                        end
                    end
                elseif mx == dx
                    theta = -atan((o_tmp(5) - o_tmp(3))/(o_tmp(5,2) - o_tmp(3,2)));
                    [~,~,tilt_R,intercept_R] = Forecast_line(o_tmp(3),o_tmp(5),o_tmp(3,2),o_tmp(5,2));
                    if mx*tilt_R+intercept_R > my && dx*tilt_R+intercept_R > dy
                        flag_rb(6) = 7;
                    elseif mx*tilt_R+intercept_R < my && dx*tilt_R+intercept_R < dy
                        flag_rb(6) = 7;
                    end
                else
                    [f_R,x,tilt_R,intercept_R] = Forecast_line(o_tmp(3),o_tmp(5),o_tmp(3,2),o_tmp(5,2));
                    %plot(x,f_R,'--');
                    theta = calAngle(tilt_goal,tilt_R);
                    if mx*tilt_R+intercept_R > my && dx*tilt_R+intercept_R > dy
                        flag_rb(6) = 7;
                    elseif mx*tilt_R+intercept_R < my && dx*tilt_R+intercept_R < dy
                        flag_rb(6) = 7;
                    end
                end
            elseif flag_rb(2) == 1 && flag_rb(3) == 1
                flag_rb(6) = 1;
                if o_tmp(2) == o_tmp(3)
                    theta = atan((mx - dx)/(my - dy));
                    %plot([o_tmp(2) o_tmp(2)],[o_tmp(3,2)-2 o_tmp(2,2)+2],'--');
                    if my > dy
                        hr = -pi/2;
                        if o_tmp(1) > mx
                            flag_rb(6) = 4;
                        elseif o_tmp(1) < mx
                            flag_rb(6) = 5;
                        end
                    else
                        hr = pi/2;
                        if o_tmp(1) > mx
                            flag_rb(6) = 5;
                        elseif o_tmp(1) < mx
                            flag_rb(6) = 4;
                        end
                    end
                elseif mx == dx
                    theta = -atan((o_tmp(3) - o_tmp(2))/(o_tmp(3,2) - o_tmp(2,2)));
                    [~,~,tilt_R,intercept_R] = Forecast_line(o_tmp(2),o_tmp(3),o_tmp(2,2),o_tmp(3,2));
                    if mx*tilt_R+intercept_R > my && dx*tilt_R+intercept_R > dy
                        flag_rb(6) = 7;
                    elseif mx*tilt_R+intercept_R < my && dx*tilt_R+intercept_R < dy
                        flag_rb(6) = 7;
                    end
                else
                    [f_R,x,tilt_R,intercept_R] = Forecast_line(o_tmp(2),o_tmp(3),o_tmp(2,2),o_tmp(3,2));
                    %plot(x,f_R,'--');
                    theta = calAngle(tilt_goal,tilt_R);
                    if mx*tilt_R+intercept_R > my && dx*tilt_R+intercept_R > dy
                        flag_rb(6) = 7;
                    elseif mx*tilt_R+intercept_R < my && dx*tilt_R+intercept_R < dy
                        flag_rb(6) = 7;
                    end
                end
            end
        end 
    
    if theta ~= 0 && flag_rb(6) == 1
        hr = hr + theta;
        if theta < 0
           flag_rb(6) = 4;
        elseif theta > 0
            flag_rb(6) = 5;
        end
    end
    
    if flag_rb(6) == 7 && mx < dx
        if nnz(flag_rb([1,2,4])) >= 2
            hr = atan(tilt_L);
            if mx*tilt_L+intercept_L > my
                flag_rb(6) = 4;
            elseif mx*tilt_L+intercept_L < my
                flag_rb(6) = 5;
            end
        else
            hr = atan(tilt_R);
            if mx*tilt_R+intercept_R > my
                flag_rb(6) = 4;
            elseif mx*tilt_R+intercept_R < my
                flag_rb(6) = 5;
            end
        end
    elseif flag_rb(6) == 7 && mx > dx
        if nnz(flag_rb([1,2,4])) >= 2
            hr = atan(tilt_L) + pi;
            if mx*tilt_L+intercept_L > my
                flag_rb(6) = 5;
            elseif mx*tilt_L+intercept_L < my
                flag_rb(6) = 4;
            end
        else
            hr = atan(tilt_R) + pi;
            if mx*tilt_R+intercept_R > my
                flag_rb(6) = 5;
            elseif mx*tilt_R+intercept_R < my
                flag_rb(6) = 4;
            end
        end
    end
            

end