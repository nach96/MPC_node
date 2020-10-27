syms f(x,y,K,d)

f(x,y) = K*((x)^2 + (y)^2 - d^2)^(2)
gradf_x = diff(f,x)
gradf_y = diff(f,y)

gradf_xx = diff(f,x,2)
gradf_yy = diff(f,y,2)
gradf_xy = diff(f,x,y)
gradf_yx = diff(f,y,x)