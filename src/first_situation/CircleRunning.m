function [mx,my,hr] = CircleRunning(mx,my,hr,distancex,distancey)%

hr = hr - pi/360;
if abs(distancex) < abs(distancey)
    mx = mx + cos(hr)*abs(distancey)/120;%ちょっと進ませる
    my = my + sin(hr)*abs(distancey)/120;%distancey
else
    mx = mx + cos(hr)*abs(distancex)/120;%ちょっと進ませる
    my = my + sin(hr)*abs(distancex)/120;%distancex
end
%hr


%mx = mx + cos(hr)*0.5;%ちょっと進ませる
%my = my + sin(hr)*0.5;
end

