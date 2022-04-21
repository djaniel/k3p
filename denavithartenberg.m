function T = denavithartenberg(d, theta, r,alpha)
ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

T=[ct , -st*ca, st*sa, r*ct;...
   st , ct*ca, -ct*sa, r*st;...
   0 , sa, ca, d;...
   0 , 0, 0, 1];