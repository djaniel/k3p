function [odds, even]=hexagon(r,height)

odds = [];
even = [];

if exist('height','var')
    h = height;
else
    h = 0;
end

for ind= 0:5
    ang = ind*pi/3;
    x = r * cos(ang);
    y = r * sin(ang);
    z = h;
    if mod(ind,2)==0    %even legs
        odds = [odds, [x;y;z;1]];
    else                %odd legs
        even = [even, [x;y;z;1]];
    end
end


odds = [odds, odds(:,1)];
even = [even, even(:,1)];

if ~exist('height','var')
    odds = [odds(1:2,:);odds(4,:)];
    even = [even(1:2,:);even(4,:)];
end

%plot(r*odds(1,:),r*odds(2,:),'r')
%plot(r*even(1,:),r*even(2,:),'g')