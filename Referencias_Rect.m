x = 0;
y = 0;
r = 5;
t = 0;
array_counter = 2;
while(t<29)
    switch t
        case 0
            x(array_counter) = x(array_counter - 1) + 1;
            y(array_counter) = y(array_counter-1);
            if x(array_counter)==r
               t = t + 1; 
            end
        case 1
            y(array_counter) = y(array_counter - 1) + 1;
            x(array_counter) = x(array_counter-1);
            if y(array_counter)==r
               t = t + 1; 
            end
        case 2
            x(array_counter) = x(array_counter -1) - 1;
            y(array_counter) = y(array_counter-1);
            if x(array_counter)==0
                t = t+1;
            end
        case 3
            y(array_counter) = y(array_counter -1) - 1;
            x(array_counter) = x(array_counter-1);
            if y(array_counter) == 0
                t = t + 1;
            end
        case 4
            break;
    end
    array_counter = array_counter + 1;
end
figure; plot(x,y);
