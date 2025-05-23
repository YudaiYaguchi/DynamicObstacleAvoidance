function [f,x,a,b] = Forecast_line(x_1,x_2,y_1,y_2)

        a = (y_2 - y_1) / (x_2 - x_1);
        b = y_1 - a*x_1;
        if x_1 < x_2
            x = x_1-2:1:x_2+2;
        else
            x = x_2-2:1:x_1+2;
        end
        f = a*x + b;
end