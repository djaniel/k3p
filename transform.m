function A=transform (X)

Rz= [cos(X(end)), -sin(X(end)),0;...
     sin(X(end)),  cos(X(end)),0;
     0,0,1];
switch length(X) 
    case 3
        A = eye(3);
        A(1:2,3)  = X(1:2);
        A(1:2,1:2)= Rz(1:2,1:2);
    case 4
        A = eye(4);
        A(1:3,4)  = X(1:3);
        A(1:2,1:2)= Rz(1:2,1:2);
    case 6
        A = eye(4);
        A(1:3,4) = X(1:3);
        ct = cos(X(4));
        st = sin(X(4));
        Rx= [1,  0,  0;...
             0, ct,-st;...
             0, st, ct];
        cp = cos(X(5));
        sp = sin(X(5));
        Ry = [ cp,  0, sp;...
                0,  1,  0;...
              -sp,  0, cp];
        A(1:3,1:3)=Rz*Ry*Rx;
    otherwise
        msg='invalid size of vector X';
        error(msg)
end