function [d_Rline] = dis_Rline(x, y ,theta, abs2Ax, abs2Ay)
%右側面と壁の距離の計算

% abs2Ax = 830;
% abs2Ay = 480;

[RF] = CarBody_RF(x, y, theta);
RFx = RF(1,1);
RFy = RF(1,2);
[RR] = CarBody_RR(x, y, theta);
RRx = RR(1,1);
RRy = RR(1,2);
[a, b] = ST_line(RFx, RFy, RRx, RRy);
d_Rline = Point_d(a, b, abs2Ax, abs2Ay);

end

