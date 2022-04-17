function Rz = rotationZ(phi)

Rz = [cos(phi), -sin(phi),0,0; ...
          sin(phi), cos(phi),0,0; ...
          0,0,1,0;
          0,0,0,1];
    