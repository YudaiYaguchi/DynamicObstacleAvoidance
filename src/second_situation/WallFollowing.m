function [hr, mx, my] = WallFollowing(map, mx, my, dx, dy, hr, flag_rb, wo)
    % 壁沿い走行を行う関数

    % 進行方向を調整
    if flag_rb(1) == 1 || flag_rb(2) == 1 || flag_rb(3) == 1
        hr = hr + pi/4;  % 左側の障害物を避けるため右に回転
    elseif flag_rb(4) == 1 || flag_rb(5) == 1
        hr = hr - pi/4;  % 右側の障害物を避けるため左に回転
    end

    % 新しい位置を設定
    mx = mx + wo * cos(hr);
    my = my + wo * sin(hr);
end