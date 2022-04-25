function color = get_color(selector)



strings = {
'turquoise','green sea','emerald','nephritis','peter river', ...
'belize hole','amethyst','wisteria','wet asphalt','midnight blue', ...
'sun flower','orange','carrot','pumpkin','alizarin','pomegranate', ...
'clouds','silver','concrete','asbestos'};

if nargin==0
    for ind=1:length(strings)
        disp(strings(ind))
    end
    return 
end
    

colors =[...
26,188,156;... %1 Turquoise
22,160,133;... %2 green sea
46,204,113;... %3 Emerald
39,174,96;...  %4 Nephritis
52,152,219;... %5 peter river
41,128,185;... %6 belize hole
155,89,182;... %7 amethyst
142,68,173;... %8 wisteria
52,73,94;...   %9 wet asphalt
44,62,80;...   %10 midnight blue
241,196,15;... %11 sun flower
243,156,18;... %12 Orange
230,126,34;... %13 Carrot
211,84,0;...   %14 Pumpkin
231,76,60;...  %15 Alizarin
192,57,43;...  %16 Pomegranate
236,240,241;...%17 Clouds
189,195,199;...%18 Silver
149,165,166;...%19 Concrete
127,140,141;...%20 Asbestos
]';

if ischar(selector)
    for ind= 1:length(strings)
        if strcmp(strings(ind),selector)
            break
        end
    end
else
    ind = mod(selector,20);
    if ind==0
        ind = 20;
    end
        
end
r =  colors(1,ind) ;
g =  colors(2,ind) ;
b =  colors(3,ind) ;


color =[r,g,b]/255;


