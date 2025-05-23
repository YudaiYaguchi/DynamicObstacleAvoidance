function z=grad(d_x,d_y,o,wo,wd)%勾配
sym o_p;
sym d_o;
syms x;
syms y;

    fx = wo * (diff(new1_o_p(x, y, o), x) + diff(o_p(x, y, o), x)) + wd * diff(new1_d_p(x, y, d_x, d_y), x);
    fy = wo * (diff(new1_o_p(x, y, o), y) + diff(o_p(x, y, o), y)) + wd * diff(new1_d_p(x, y, d_x, d_y), y);
    z=-[fx fy];

end