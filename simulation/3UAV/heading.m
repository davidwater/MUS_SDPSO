function theta = heading(hx,hy)

if hx > 0
    theta = atan(hy/hx);
elseif hx < 0
    theta = atan(hy/hx);
    if theta > 0
        theta = -pi + theta;
    elseif theta <= 0
        theta = pi + theta;
    end
elseif hx == 0 && hy > 0
    theta = pi/2;
elseif hx == 0 && hy < 0
    theta = -pi/2;
elseif hx == 0 && hy ==0
    theta = 0;
end

end