

function plotorigin (T,l, label)


if size(T,1)==4
    origin = T * [0,0,0,1]';

    axis_x = T * [l,0,0,1]';
    axis_y = T * [0,l,0,1]';
    axis_z = T * [0,0,l,1]';
    
    plot3 ([origin(1), axis_x(1)],[origin(2), axis_x(2)],[origin(3), axis_x(3)],'-r>')
    plot3 ([origin(1), axis_y(1)],[origin(2), axis_y(2)],[origin(3), axis_y(3)],'-g>')
    plot3 ([origin(1), axis_z(1)],[origin(2), axis_z(2)],[origin(3), axis_z(3)],'-b>')
    if exist('label','var')
        text(origin(1),origin(2),origin(3),label)
    end
end

if size(T,1)==3
    origin= T*[0;0;1];
    axis_x =T*[l;0;1];
    axis_y =T*[0;l;1];
    plot([origin(1), axis_x(1)],[origin(2), axis_x(2)],'-r>')
    plot([origin(1), axis_y(1)],[origin(2), axis_y(2)],'-g>')
    if exist('label','var')
        text(origin(1),origin(2),label)
    end
end

