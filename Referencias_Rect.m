x = 0;
y = 0;
r = 5;
t = 0;
array_counter = 2;
while(t<29)
    if t>=0 && t<7
        x(array_counter) = x(array_counter - 1) + 1;
        y(array_counter) = y(array_counter-1);
    elseif t>=7 && t<14
        y(array_counter) = y(array_counter - 1) + 1;
        x(array_counter) = x(array_counter-1);
    elseif t>=14 && t<21
        x(array_counter) = x(array_counter -1) - 1;
        y(array_counter) = y(array_counter-1);
    elseif t>=21 && t<28
        y(array_counter) = y(array_counter -1) - 1;
        x(array_counter) = x(array_counter-1);
    end
    t = t + 1;
    array_counter = array_counter + 1;
end
figure; plot(x,y);
