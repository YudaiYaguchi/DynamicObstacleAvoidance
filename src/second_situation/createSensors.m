% 距離センサ生成関数
function [rb1, rb2, rb3, rb4, rb5] = createSensors()
    rb1 = rangeSensor; rb2 = rangeSensor; rb3 = rangeSensor; rb4 = rangeSensor; rb5 = rangeSensor;
    rb1.Range = [0.0 2.0]; rb2.Range = [0.0 2.0]; rb3.Range = [0.0 2.0]; rb4.Range = [0.0 2.0]; rb5.Range = [0.0 2.0];
    rb1.HorizontalAngle = [(pi/6)-pi/360 (pi/6)+pi/360];
    rb2.HorizontalAngle = [(pi/180)-pi/360 (pi/180)+pi/360];
    rb3.HorizontalAngle = [(-pi/6)-pi/360 (-pi/6)+pi/360];
    rb4.HorizontalAngle = [(4*pi/9)-pi/360 (4*pi/9)+pi/360];
    rb5.HorizontalAngle = [(-4*pi/9)-pi/360 (-4*pi/9)+pi/360];
end