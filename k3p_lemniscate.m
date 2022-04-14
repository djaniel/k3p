close all
clear all

record_movie= false;
iter_step = 100;
t_end_graphic =  550;
%addpath ('../tools','./terrain_generation/')
%addpath( '../tools/matlab2tikz-816f875/src/')
%addpath( '../tools/matlab2tikz-816f875/src/dev')
%addpath( '../tools/matlab2tikz-816f875/src/private')
v  = 0.02;           %speed [m/s]
%dt = 0.05;
FPS = 30;
dt  = 1/FPS;          % 30 fps
diamBase= 0.26*2;   %diametral base of the walking machine
walkHeight = 0.15;   % mean height of walking
gaitHeightPC = 80;   % per centage value of walkHeight
sat  = 0.25;
wmax = deg2rad(10); % maximum angular velocity [rad/s]
phase_shift = -1;  % Three state variable
                   % -1   legs are taking off
                   %  0   legs swinging
                   %  >=1 legs landing


%Parameters of the lemniscate
a=3*diamBase; b=2*diamBase; k=30; cb=0;
[Xt, dXt,smax] = lemniscate('a',a,b,cb,k);


scrsz = get(groot,'ScreenSize');
%fig=figure('position',[1800 200 5*scrsz(3)/8 3*scrsz(4)/4],'name','Trajectory');

fig=figure('position',[200 200 542 402],'name','Trajectory');
hold on; grid on; axis equal	
axis([-2,2, -1.5, 1.5,-0.5,1])
view(3)
%view(-20,30)
AZ = -37.5;
EL = 30;
view(AZ,EL)
%view(0,0)

% Ground
%[X, Y, H, Hm, Xm, Ym] = generate_terrain(7, 513, 0, 0.1, -0.15);
%save('terrain1.mat','X', 'Y', 'H', 'Hm', 'Xm', 'Ym');
Terrain = load('terrain2.mat');
Terrain.Xm=2*Terrain.Xm;
Terrain.Ym=2*Terrain.Ym;
Terrain.Hm=  Terrain.Hm;% - 0.25*min(min(Terrain.Hm));

% Reduce the number of points for creating the graphic
skep = 4;
TerrainPrint.Ym = Terrain.Ym(1:skep:size(Terrain.Ym,1), 1:skep:size(Terrain.Ym,2));
TerrainPrint.Xm = Terrain.Xm(1:skep:size(Terrain.Xm,1), 1:skep:size(Terrain.Xm,2));
TerrainPrint.Hm = Terrain.Hm(1:skep:size(Terrain.Hm,1), 1:skep:size(Terrain.Hm,2));

validYm = and(TerrainPrint.Ym(:,1)>=-1.5, TerrainPrint.Ym(:,1)<=1.5);
Yrange  = find(validYm==1, 1, 'first'):find(validYm==1, 1, 'last');
TerrainPrint.Ym= TerrainPrint.Ym(Yrange,:);
TerrainPrint.Xm = TerrainPrint.Xm(1:size(TerrainPrint.Ym,1),:);
TerrainPrint.Hm = TerrainPrint.Hm(Yrange,:);

ground1  = [0.5,0.5, -0.5, -0.5;...
           -1.5,1.5,1.5,-1.5;...
           0,0,0,0];
planes_n  = zeros(3,3);
planes_r0 = zeros(3,3);
n1=[0;0;1];
planes_n(:,1) = n1;

% The positions for the six legs, an hexagon
[odds, evens]=hexagon(diamBase/2,0); 
odds  =rotationZ(deg2rad(30)) * odds;
evens =rotationZ(deg2rad(30)) * evens;

odds0 = odds(:,1:3); 
odds0(3,1:3) = interp2(Terrain.Xm,Terrain.Ym,Terrain.Hm, odds(1,1:3),odds(2,1:3));
evens0 = evens(:,1:3);
evens0(3,1:3)= interp2(Terrain.Xm,Terrain.Ym,Terrain.Hm, evens(1,1:3),evens(2,1:3));

Z_height = mean([odds0(3,:), evens0(3,:)])+ walkHeight;

X_BW = [0;0; Z_height;0;0;0];
X_OB = [0;0;-walkHeight;0;0;0];  % wrt B
X_EB = [0;0;-walkHeight;0;0;0];  % wrt B 
A_BW = transform(X_BW);
A_EB = transform(X_EB); 
A_OB = transform(X_OB);

oddsB = A_BW \ odds0;
evensB = A_BW \ evens0;
Spts =[oddsB(:,1), evensB(:,1),oddsB(:,2), evensB(:,2),oddsB(:,3), evensB(:,3)];

X_BW_last= X_BW;
gaitHeight  = -walkHeight * (1-gaitHeightPC*.01);

phi  = atan2(dXt(2),dXt(1));
X_DW = [Xt(1,1); Xt(2,1); Z_height; 0;0;phi];

Xs   = [];
XBs  = [];
Torques=[];
tk   = 0;   %Execution time

dXz  = zeros(6,1);
dLz  = zeros(6,1);
Kp   = diag([2.00,2.00,2.50,0.0,0.0,0.9]);
Kd   = diag([0.05,0.05,0.1,0.0,0.0,0.0]);

L_last= X_OB;

X_MPs = zeros(4,6);
for ind = 0:5
    ang = ind * pi/3 + pi/6;
    iMP = rotationZ(ang)*[0.07;0;-0.03;1];
    X_MPs(:,ind+1) = [iMP(1:3);ang];
end

robot = struct( 'X' ,X_BW,...           %Initial position
                'X_MPs',X_MPs,...       %Mounting points of all limbs
                'mass',1.6,...          %Kilograms
                'mass_distribution',[0.40,0.3,0.2,0.1],...
                'lc',0.06,...           %coxa's length
                'lf',0.16,...           %femur's length
                'lt',0.16,...           %tibia's length
                'lp',0.14,...           %
                'hm',0.04,...           %
                'SPs',Spts,...            %Puntos de apoyo
                'min_ang', 0.07/(diamBase*0.5) ,... %minimum angle before collision
                'max_gait',0,...        %Maximum length for the gait
                'sensors_odd',ones(3,1),...
                'sensors_even',ones(3,1),...
                'Swings',zeros(6,1),... %Record of the DoF
                'Lifts',zeros(6,1),...  
                'Knees',zeros(6,1),...
                'A_CBs',zeros(4,4,6),...  %Rigid body transformation wrt B % COXA 
                'A_pBs',zeros(4,4,6),...  %motor top / propeller
                'A_mBs',zeros(4,4,6),...  %motor mount
                'A_KBs',zeros(4,4,6),...  %KNEE
                'A_SpBs',zeros(4,4,6),... %SUPPORT POINT
                'Torque_S',zeros(3,6),...
                'Torque_L',zeros(3,6),... %The torque of Mass of the Femur, wrt L
                'Torque_K',zeros(3,6),... %The torque of Mass of the Tibia, wrt K
                'parity',1);   
%Three state variable
% 0 even and odd legs are stopped
% 1 odd legs are moving
% 2 even legs are moving
parity = 1;

xgait= robot.X_MPs(1)+robot.lc+sqrt((robot.lt^2+robot.lf^2)-walkHeight^2); %The two parameters for the gait
robot.max_gait = xgait * 0.7;
robot=inverse_kinematics(robot);

%trisurf(delaunay(Terrain.X, Terrain.Y), Terrain.X, Terrain.Y, Terrain.H);
surf(Terrain.Xm, Terrain.Ym, Terrain.Hm);
colormap parula;                    % Default color map.
set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
axis equal vis3d;                % Set aspect ratio.
shading interp;                  % Interpolate color across faces.
camlight left;                   % Add a light over to the left somewhere.
lighting gouraud;                % Use decent lighting.
draw_hexapodopter_rf(robot)
axis([-0.3,0.3, -0.3, 0.3,-0.1,0.45])
%pause


axis([-2,2, -1.5, 1.5,-0.5,1])



vars =[];
changes=[];


angs_knee=[];
% error_height = [];
e_L =zeros(6,1);
leveling_legs =[0,0,0];

leg_trayectories=zeros(3,2650,6);
even_legs_supporting = zeros(3,200,6);
odd_legs_supporting = zeros(3,200,6);


iter_odd_support= 1;
odd_legs_supporting(:,iter_odd_support,1) = odds0(1:3,1);
odd_legs_supporting(:,iter_odd_support,2) = odds0(1:3,2);
odd_legs_supporting(:,iter_odd_support,3) = odds0(1:3,3);
iter_odd_support=iter_odd_support+1;

iter_even_support = 1;
even_legs_supporting(:,iter_even_support,1) = evens0(1:3,1);
even_legs_supporting(:,iter_even_support,2) = evens0(1:3,2);
even_legs_supporting(:,iter_even_support,3) = evens0(1:3,3);
iter_even_support=iter_even_support+1;

s=0;
iter = 1;
%Movie generation
if record_movie
    nFrames = 2600;
    vidObj = VideoWriter('gait_pattern.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = FPS;
    open(vidObj);
end

v_cmd =[];
shifts = 0;
while s<smax
    
    % t execution time
    % vm mean speed
    % ta tiempo de aceleracion
    % vamp amplitud de velocidad
    % periodo
    
    v=0.02;
    v_cmd =[v_cmd,v];
    
    if tk >(t_end_graphic +10)
        break
    end
    
    if parity ==1
        A_mB   = transform(X_OB);
        A_fB   = transform(X_EB);
        Lf     = X_EB;
        Lm     = X_OB;                  % State vector for the moving legs
        supportPointsB = A_fB*evens;
    elseif parity == 2
        A_mB   = transform(X_EB);
        A_fB   = transform(X_OB);
        Lf     = X_OB;                  % State vector fro the fixed legs
        Lm     = X_EB;                  % State vector for the moving legs
        supportPointsB = A_fB*odds;
    end
    robot.parity = parity;
    
    heightZ =mean (supportPointsB(3,1:3));
      
    %% Compute the next step on the trajectory
    A_DW    = transform(X_DW);
    if phase_shift == 0     %Compute step only if the robot is moving
        [~,dX]  = lemniscate(s,a,b,cb,k);
        cossd   = dX/norm(dX);      % Get dcosines
        [~,ind] = max(abs(cossd));  % get the biggest from the dcosines
        cosd    = cossd(ind);
        ds      = v*dt*cosd/dX(ind);% Compute delta S
        s       = s+ds;
        [X,dX]  = lemniscate(s,a,b,cb,k);
        phi     = atan2(dX(2),dX(1));
        X(end)  = walkHeight;
        
        X_DW    = [X; Lf(4:5); phi];
        dXd     = [dX;0;0;1/(1+tan(phi)^2)];
        dXd(end)= min ( wmax,dXd(end));    %The desired angular velocity may go crazy
        dXd(end)= max (-wmax,dXd(end));
    else                    %When the legs are takin off or landing
        X_DW    = robot.X;
        dXd     = zeros(6,1);
    end
    
    Xs      = [Xs, X_DW];
    
    terrain_height=interp2(Terrain.Xm,Terrain.Ym,Terrain.Hm,...
                      X_BW(1),X_BW(2));
    %% Command the the CoG to the desired location
    e_global  = X_DW - X_BW;
    de_global = dXd - dXz;
    while abs(e_global(end))>pi
        e_global(end) = e_global(end)-2*pi;
    end
    
    R_BW    = transform([0;0;0;X_BW(4:end)]);
    e_pose  = [ R_BW(1:3,1:3)' *  e_global(1:3) ;  e_global(4:end)]; %Local errors
    e_pose(3) = walkHeight+heightZ;
%     error_height= [error_height walkHeight+heightZ];
    e_dL    = [ R_BW(1:3,1:3)' * de_global(1:3) ; de_global(4:end)];
    U_b     = Kp * e_pose + Kd * e_dL;
    U_b(1:3)= min ( sat,U_b(1:3));
    U_b(1:3)= max (-sat,U_b(1:3));
    X_BW    = X_BW + [R_BW(1:3,1:3)*U_b(1:3); U_b(4:end)] *dt;        % Command execution
    dXz     = (X_BW-X_BW_last)/dt;          % compute speed of the vehicle
    rho     = norm(dXz(1:3))/dXz(end);      % Turning radius
    
    XBs = [XBs, [X_BW;terrain_height]];
    %% Gait forward math
    A_BW_pas = transform(X_BW_last);
    A_BW_fut = transform(X_BW);
    
    A_fB = A_BW_fut \ A_BW_pas * A_fB;
    A_fW = A_BW_fut * A_fB;
    A_BW = A_BW_fut;
       
    A_fm = A_mB \ A_fB;
    vec  = transform2vec(A_fm);
    gait = norm(vec(1:2));
    height = A_fB(3,4);
    
    %% Determine the desired position for the legs
    if phase_shift <= 0                         %swing
        Ld   = [robot.max_gait/2;0;gaitHeight;0;0;0];
        theta= gait/rho;
        if  abs(rho) < 0.8 
            Ld = [               robot.max_gait/2 * cos(theta); 
                   sign(theta) * robot.max_gait/2 * sin(theta);...
                                                    gaitHeight;...
                                                   0; 0; theta];
        end
        dLd  = [2*v*cos(phi);2*v*sin(phi); 0;0;0;0];
    elseif phase_shift < 0                      %Taking off
        Ld   = [Lm(1:2);gaitHeight;0;0;0];
        dLd  = zeros(6,1);
    elseif phase_shift > 0                      %Landing
        Ld   = [Lm(1:2);-walkHeight*2;0;0;0];
        dLd  = zeros(6,1);
    end
    
    
    
    %% Command the moving set of legs to the desired location

    A_LB = transform(Ld);
    A_LW = A_BW*A_LB;
    
    e_L  = Ld - Lm;
    e_dL = dLd - dLz;
    U_l  = Kp * e_L + Kd * e_dL;
    U_l(1:3) = min ( 2*sat,U_l(1:3));
    U_l(1:3) = max (-2*sat,U_l(1:3));
    
    L_last = Lm;
    Lm   = Lm + U_l*dt;
    dLm  = Lm - L_last;
    dLz  = dLm/dt;
    
    %A_Lk1_Lk = transform(dLm);

    %% Update all variables after time step 
    if parity == 1
        A_EB = A_fB;
        X_OB = Lm;
        X_EB = transform2vec(A_EB,6);
        A_OB = transform(X_OB);
    elseif parity ==2 
        A_OB = A_fB;
        X_EB = Lm;
        X_OB = transform2vec(A_OB,6);
        A_EB = transform(X_EB);
    end
    
    tk = tk+dt;
    X_BW_last = X_BW;
    
    oddsB  = A_OB * odds;
    evensB = A_EB * evens;
    
    oddsW  = A_BW * oddsB;
    evensW = A_BW * evensB;
    
    
    leg_trayectories(:,iter,1) = oddsW (1:3,1);
    leg_trayectories(:,iter,2) = evensW(1:3,1);
    leg_trayectories(:,iter,3) = oddsW (1:3,2);
    leg_trayectories(:,iter,4) = evensW(1:3,2);
    leg_trayectories(:,iter,5) = oddsW (1:3,3);
    leg_trayectories(:,iter,6) = evensW(1:3,3);
    
    %% Simulate the sensors mounted on the Support Points of every leg    
    oddsW_h = interp2(Terrain.Xm,Terrain.Ym,Terrain.Hm,...
                      oddsW(1,1:3),oddsW(2,1:3));
    evensW_h= interp2(Terrain.Xm,Terrain.Ym,Terrain.Hm,...
                      evensW(1,1:3),evensW(2,1:3));
    
    robot.sensors_odd = oddsW(3,1:3) <= oddsW_h;
    robot.sensors_even= evensW(3,1:3)<= evensW_h;
    
    
    if phase_shift > 0
        %LANDING SET OF LEGS
        if and(parity==1, any(robot.sensors_odd))
            for ind = 1:length(robot.sensors_odd)
                if ~robot.sensors_odd(ind)
                    continue
                end
                odds(3,ind) = odds(3,ind) - dLm(3);
            end
        elseif and(parity==2, any(robot.sensors_even))
            for ind = 1:length(robot.sensors_even)
                if ~robot.sensors_even(ind)
                    continue
                end
                evens(3,ind) = evens(3,ind) - dLm(3);
            end
        end
    elseif phase_shift < 0
        % TAKING OFF SET OF LEGS
        if and(parity == 1, any(leveling_legs))
            for ind = 1:length(leveling_legs)
                if leveling_legs(ind)
                    if oddsB(3,ind) > Z_max 
                        leveling_legs(ind)=0;
                    end                    
                else
                    odds(3,ind) = odds(3,ind) - dLm(3);
                end
            end
        elseif and( parity==2, any(leveling_legs) )
            for ind= 1:length(leveling_legs)
                if leveling_legs(ind)
                    if evensB(3,ind) > Z_max
                        leveling_legs(ind)=0;
                    end 
                else
                    evens(3,ind) = evens(3,ind) - dLm(3);
                end
            end
        end
        
        
        if and(parity == 1, all(~leveling_legs))
           odds(3,:)=zeros(1,4);
           even_legs_supporting(:,iter_even_support,1) = evensW(1:3,1);
           even_legs_supporting(:,iter_even_support,2) = evensW(1:3,2);
           even_legs_supporting(:,iter_even_support,3) = evensW(1:3,3);
           iter_even_support =iter_even_support+1;
           phase_shift=0;  % Odd legs have taken off
        elseif and(parity == 2, all(~leveling_legs))
           evens(3,:)=zeros(1,4);
           odd_legs_supporting(:,iter_odd_support,1) = oddsW(1:3,1);
           odd_legs_supporting(:,iter_odd_support,2) = oddsW(1:3,2);
           odd_legs_supporting(:,iter_odd_support,3) = oddsW(1:3,3);
           iter_odd_support = iter_odd_support+1;
           phase_shift=0;  % Even legs have taken off
        end
        
    end
    odds(:,end) = odds(:,1);
    evens(:,end)= evens(:,1);
    
    % compute the angles from B to the SP of each leg
    Spts =zeros(4,6);
    Spts(:,1:2) = [oddsB(:,1), evensB(:,1)];
    Spts(:,3:4) = [oddsB(:,2), evensB(:,2)];
    Spts(:,5:6) = [oddsB(:,3), evensB(:,3)];
    angs  = atan2(Spts(2,:),Spts(1,:));
    angsL = circshift(angs,-1,2)-angs;
    
    ind   = abs(angsL)> pi;
    angsL(ind) = angsL(ind)+2*pi;
    
    %% Inverse kinematics
    robot.X  = X_BW;
    robot.SPs= Spts;
    robot=inverse_kinematics(robot);
    angs_knee=[angs_knee, robot.Knees];
    
    Torques =[Torques, [robot.Torque_S; robot.Torque_L;robot.Torque_K]];
    
    %% Test for change in parity
    if phase_shift == 0
        
        gait_err= norm(e_L(1:2));
        test1   = and(gait>=robot.max_gait, gait_err< 0.5*robot.max_gait);
        test2   = any(abs(angsL)<= robot.min_ang); 
        test3   = or (any(robot.Knees< deg2rad(40)), any(robot.Lifts< deg2rad(-35)));
    
        if test1 || test2 || test3
            shifts = shifts +1;
            if test1
                disp(['t_k: ',num2str(tk) ,'. Triggered by test 1'])
                phase_shift = 1;
            elseif test2
                disp(['t_k: ',num2str(tk) ,'. Triggered by test 2'])
                phase_shift = 2;
            elseif test3
                disp(['t_k: ',num2str(tk) ,'. Triggered by test 3'])
                phase_shift = 3;
            end
            
            changes=[changes, [tk; phase_shift;tk;parity]];
        end
    end
    
    % we only change parity after the phase shift ends
    % i.e. the moving legs have landed
    if parity == 1
        sensors_m = robot.sensors_odd;
    else
        sensors_m = robot.sensors_even;
    end
   
    if and(all(sensors_m),phase_shift>0)
        phase_shift = -1;
        if parity == 1
            [~,highest_leg] = min(abs(evensB(3,1:3)));
            Z_max = evensB(3,highest_leg);
            leveling_legs = evensB(3,1:3) < Z_max;
            parity = 2;
        elseif parity ==2
            [~,highest_leg] = min(abs(oddsB(3,1:3)));
            Z_max = oddsB(3,highest_leg);
            leveling_legs = oddsB(3,1:3)  < Z_max;
            parity = 1;
        end
    end
        
            %set(0,'CurrentFigure',fig3);
            %plot(oddsp(1,:), oddsp(2,:),'color',get_color('carrot'))
            %plot(evenp(1,:), evenp(2,:),'color',get_color('wisteria'))
            %plot(evenp(1,:), evenp(2,:),'color',get_color('wisteria'))
            %text(evenp(1,3), evenp(2,3)-0.1,['t_{', num2str(tk), '}'],'color',get_color('wisteria'))
            %plot(evenp(1,3), evenp(2,3),'*','color',get_color('wisteria'))
            %plot(oddsp(1,2), oddsp(2,2),'*','color',get_color('carrot'))
            %text(oddsp(1,2)-0.13,oddsp(2,2)+0.12,['t_{', num2str(odds_instants), '}'],'color',get_color('carrot'))
            
            
            %set(0,'CurrentFigure',fig2);
            %plot(oddsp(1,:), oddsp(2,:),'color',get_color('pumpkin'))
            %text(oddsp(1,3), oddsp(2,3)-0.12,['t_{', num2str(tk), '}'],'color',get_color('pumpkin'))
            %plot(oddsp(1,3), oddsp(2,3),'*','color',get_color('pumpkin'))
            %plot(evenp(1,:), evenp(2,:),'color',get_color('emerald'))
            
        
        
        
    
   
    
    %% Create graphic
    P_B = A_BW*[0;0;0;1];
    if parity ==1 %even are fixed
        spolygon = evensW(:,1:3);
    elseif parity ==2 %odds are fixed
        spolygon = oddsW(:,1:3);
    end
    vars    = [vars, [gait; min(angsL)]]; %dLd(end)]];
    
    if mod(iter,iter_step)~=0
       iter = iter+1;
       continue
    end
    
    set(0,'CurrentFigure',fig);
    cla    
    xlabel('X axis'), ylabel('Y axis'), zlabel('Z axis')
    
    plot3(leg_trayectories(1,1:iter,1), leg_trayectories(2,1:iter,1),leg_trayectories(3,1:iter,1),'color', get_color('turquoise') )
    plot3(leg_trayectories(1,1:iter,3), leg_trayectories(2,1:iter,3),leg_trayectories(3,1:iter,3),'color', get_color('emerald'))
    plot3(leg_trayectories(1,1:iter,5), leg_trayectories(2,1:iter,5),leg_trayectories(3,1:iter,5),'color', get_color('peter river'))
     
    plot3(leg_trayectories(1,1:iter,2), leg_trayectories(2,1:iter,2),leg_trayectories(3,1:iter,2),'color', get_color('amethyst'))
    plot3(leg_trayectories(1,1:iter,4), leg_trayectories(2,1:iter,4),leg_trayectories(3,1:iter,4),'color', get_color('wet asphalt'))
    plot3(leg_trayectories(1,1:iter,6), leg_trayectories(2,1:iter,6),leg_trayectories(3,1:iter,6),'color', get_color('orange'))
    
    
    if iter_even_support > 1
        plot3(even_legs_supporting(1,1:(iter_even_support-1),1), ...
              even_legs_supporting(2,1:(iter_even_support-1),1), ...
              even_legs_supporting(3,1:(iter_even_support-1),1),'s', 'color', get_color('amethyst') )
        plot3(even_legs_supporting(1,1:(iter_even_support-1),2), ...
              even_legs_supporting(2,1:(iter_even_support-1),2), ...
              even_legs_supporting(3,1:(iter_even_support-1),2),'s' , 'color', get_color('wet asphalt') )
        plot3(even_legs_supporting(1,1:(iter_even_support-1),3), ...
              even_legs_supporting(2,1:(iter_even_support-1),3), ...
              even_legs_supporting(3,1:(iter_even_support-1),3),'s', 'color', get_color('orange') ) 
    end
    
    if iter_odd_support > 1
        plot3(odd_legs_supporting(1,1:(iter_even_support-1),1), ...
              odd_legs_supporting(2,1:(iter_even_support-1),1), ...
              odd_legs_supporting(3,1:(iter_even_support-1),1),'s', 'color', get_color('turquoise') )
        plot3(odd_legs_supporting(1,1:(iter_even_support-1),2), ...
              odd_legs_supporting(2,1:(iter_even_support-1),2), ...
              odd_legs_supporting(3,1:(iter_even_support-1),2),'s', 'color', get_color('emerald') )
        plot3(odd_legs_supporting(1,1:(iter_even_support-1),3), ...
              odd_legs_supporting(2,1:(iter_even_support-1),3), ...
              odd_legs_supporting(3,1:(iter_even_support-1),3),'s', 'color', get_color('peter river') )
    end
    
    %trisurf(delaunay(Terrain.X, Terrain.Y), Terrain.X, Terrain.Y, Terrain.H);
    surf(TerrainPrint.Xm, TerrainPrint.Ym, TerrainPrint.Hm);
    colormap parula;                    % Default color map.
    set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
    axis equal vis3d;                % Set aspect ratio.
    shading interp;                  % Interpolate color across faces.
    camlight left;                   % Add a light over to the left somewhere.
    lighting gouraud;                % Use decent lighting.
    %alpha(0.5)
    
    plot3(Xt(1,:),Xt(2,:),Xt(3,:),'color','b')
    plot3(oddsW(1,:), oddsW(2,:), oddsW(3,:),'color',get_color('sun flower'))
    plot3(evensW(1,:), evensW(2,:), evensW(3,:),'color',get_color('pomegranate'))
    
    plotorigin(A_BW,0.25,'B')
    plotorigin(A_LW,0.04,'L')
    %plot(Xs(1,:),Xs(2,:),'ok')
    %plotorigin(A_DW,0.5,'D')
    %plotorigin(A_OW,0.04,'O')
    %plotorigin(A_EW,0.04,'E')
    %plot(U_vec(1,:),U_vec(2,:),'b')
    %plot(t_BE_W_vec(1,:),t_BE_W_vec(2,:),'m')
    %plot(t_BO_W_vec(1,:),t_BO_W_vec(2,:),'k')
    draw_hexapodopter(robot)
    %[legh,objh,outh,outm] = legend('Leg 1', 'Leg 2', 'Leg 3', 'Leg 4', 'Leg 5','Leg 6','location','best');
    
    %set(objh,'linewidth',2) % this will also work on a vector of handles
    axis([-2,2, -1.5, 1.5,-0.5,1])
    %AZ = AZ - 0.05;
    %view(AZ,EL)
    drawnow
    
    if record_movie
        writeVideo(vidObj, getframe(gca));
    end
    %mov(iter) = getframe(gca);
    iter = iter+1;
end

iter = iter-1;
if record_movie
    close(vidObj);
end
%%
set(0,'CurrentFigure',fig);
    cla    
    xlabel('X axis'), ylabel('Y axis'), zlabel('Z axis')
    
    surf(TerrainPrint.Xm, TerrainPrint.Ym, TerrainPrint.Hm);
    %colormap parula;                    % Default color map.
    set(gca, 'Position', [0 0 1 1]); % Fill the figure window.
    axis equal vis3d;                % Set aspect ratio.
    shading interp;                  % Interpolate color across faces.
    %camlight left;                   % Add a light over to the left somewhere.
    
    %lighting gouraud;                % Use decent lighting.
    cb=colorbar('FontSize',12,'AxisLocation','in');
    cbpos = cb.Position;
    set(cb,'position',[cbpos(1)-0.03,cbpos(2)+0.25, cbpos(3),0.5]);
    cb.Label.String = 'Elevation [m]';
    
    plot3(Xt(1,:),Xt(2,:),Xt(3,:),'color','b')
    skep=25;
    hbody = plot3(leg_trayectories(1,1:skep:iter,1), leg_trayectories(2,1:skep:iter,1),leg_trayectories(3,1:skep:iter,1),'-','color', get_color('turquoise') );
    set(hbody,'linewidth',1)
    hbody = plot3(leg_trayectories(1,1:skep:iter,3), leg_trayectories(2,1:skep:iter,3),leg_trayectories(3,1:skep:iter,3),'-','color', get_color('emerald'));
    set(hbody,'linewidth',1)
    hbody = plot3(leg_trayectories(1,1:skep:iter,5), leg_trayectories(2,1:skep:iter,5),leg_trayectories(3,1:skep:iter,5),'-','color', get_color('peter river'));
     set(hbody,'linewidth',1)
    hbody = plot3(leg_trayectories(1,1:skep:iter,2), leg_trayectories(2,1:skep:iter,2),leg_trayectories(3,1:skep:iter,2),'-','color', get_color('amethyst'));
    set(hbody,'linewidth',1)
    hbody = plot3(leg_trayectories(1,1:skep:iter,4), leg_trayectories(2,1:skep:iter,4),leg_trayectories(3,1:skep:iter,4),'-','color', get_color('wet asphalt'));
    set(hbody,'linewidth',1)
    hbody = plot3(leg_trayectories(1,1:skep:iter,6), leg_trayectories(2,1:skep:iter,6),leg_trayectories(3,1:skep:iter,6),'-','color', get_color('orange'));
    set(hbody,'linewidth',1)
    
    if iter_even_support > 1
        plot3(even_legs_supporting(1,1:(iter_even_support-1),1), ...
              even_legs_supporting(2,1:(iter_even_support-1),1), ...
              even_legs_supporting(3,1:(iter_even_support-1),1),'s', 'color', get_color('amethyst') )
        plot3(even_legs_supporting(1,1:(iter_even_support-1),2), ...
              even_legs_supporting(2,1:(iter_even_support-1),2), ...
              even_legs_supporting(3,1:(iter_even_support-1),2),'s' , 'color', get_color('wet asphalt') )
        plot3(even_legs_supporting(1,1:(iter_even_support-1),3), ...
              even_legs_supporting(2,1:(iter_even_support-1),3), ...
              even_legs_supporting(3,1:(iter_even_support-1),3),'s', 'color', get_color('orange') ) 
    end
    
    if iter_odd_support > 1
        plot3(odd_legs_supporting(1,1:(iter_even_support-1),1), ...
              odd_legs_supporting(2,1:(iter_even_support-1),1), ...
              odd_legs_supporting(3,1:(iter_even_support-1),1),'s', 'color', get_color('turquoise') )
        plot3(odd_legs_supporting(1,1:(iter_even_support-1),2), ...
              odd_legs_supporting(2,1:(iter_even_support-1),2), ...
              odd_legs_supporting(3,1:(iter_even_support-1),2),'s', 'color', get_color('emerald') )
        plot3(odd_legs_supporting(1,1:(iter_even_support-1),3), ...
              odd_legs_supporting(2,1:(iter_even_support-1),3), ...
              odd_legs_supporting(3,1:(iter_even_support-1),3),'s', 'color', get_color('peter river') )
    end
    
    plot3(oddsW(1,:), oddsW(2,:), oddsW(3,:),'color',get_color('sun flower'))
    plot3(evensW(1,:), evensW(2,:), evensW(3,:),'color',get_color('pomegranate'))
    
    plotorigin(A_BW,0.25,'B')
    plotorigin(A_LW,0.04,'L')
    %plot(Xs(1,:),Xs(2,:),'ok')
    %plotorigin(A_DW,0.5,'D')
    %plotorigin(A_OW,0.04,'O')
    %plotorigin(A_EW,0.04,'E')
    %plot(U_vec(1,:),U_vec(2,:),'b')
    %plot(t_BE_W_vec(1,:),t_BE_W_vec(2,:),'m')
    %plot(t_BO_W_vec(1,:),t_BO_W_vec(2,:),'k')
    draw_hexapodopter(robot)
    %[legh,objh,outh,outm] = legend('Leg 1', 'Leg 2', 'Leg 3', 'Leg 4', 'Leg 5','Leg 6','location','best');
    %legend ('Lemniscate','Limb 1','Limb 2','Limb 3','Limb 4','Limb 5','Limb 6')
    %set(objh,'linewidth',2) % this will also work on a vector of handles
    
    
    
    axis([-2,2, -1.5, 1.5,-0.5,0.5])

    %matlab2tikz('./TEX/gait_uneven.tex','floatFormat','%.3f')
%%
fig3= figure();
%% Graphic phase shifts

cla

t_line = 0:dt:t_end_graphic; 
yyaxis left
cla
plot(t_line,vars(1,1:length(t_line)))%,'color',get_color('orange'))
hold on; grid on;
plot (t_line, robot.max_gait*ones(1,length(t_line)))
ylim([0 0.25])
ylabel('Gait distance [m]')

yyaxis right
cla
plot(t_line,vars(2,1:length(t_line)),'-')
ylim([0 1.25])
xlim([0,t_end_graphic])
ylabel('Minimum angle between legs [rad]')
plot (t_line, robot.min_ang*ones(1,length(t_line)))

%lgnd= legend('\Theta_{thd}', 'd_B','Location','SouthEast');
%set(lgnd,'Fontsize',10)

start = 0;
for ind =1:length(changes)
    end_ = changes(3,ind)+dt;
    if changes(4,ind) == 1
        start = end_;
        continue
    end
    
    
    patch([start, end_, end_, start], ...
          [    -3,  -3, end_,   100], ...
          get_color('silver'),'EdgeColor','none','FaceAlpha',0.4)
    start = end_;
end

for ind= 1:length(changes)
    %text(start,1,num2str(changes(2,ind)) ,'color',get_color(''))
    %text(end_+0.09, 1,num2str(changes(2,ind+1)),'color',get_color('carrot'))
    %[tk; test;tk;parity]]
    tkn = changes(1,ind);
    
    if tkn > t_end_graphic
        break
    end
    
    if tkn< (t_end_graphic-1)
        if changes(2,ind)==1
            plot(tkn, 1.1,'s', 'color',get_color('belize hole'),'MarkerFaceColor',get_color('belize hole'))
            text(changes(3,ind)-10*dt,1.14,num2str(changes(2,ind)),'color',get_color('belize hole'))
        else
            plot(tkn, 1.1,'s', 'color',get_color('carrot'),'MarkerFaceColor',get_color('carrot'))
            text(changes(3,ind)-10*dt,1.14,num2str(changes(2,ind)),'color',get_color('carrot'))
        end
    end
end

xlabel('Time [s]','Fontsize',12);

figure(4)

plot(t_line,v_cmd(1:length(t_line)))
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

%%
%[px,py] = gradient(z)

skep = 15;

TerrainPrint2.Ym = Terrain.Ym(1:skep:size(Terrain.Ym,1), 1:skep:size(Terrain.Ym,2));
TerrainPrint2.Xm = Terrain.Xm(1:skep:size(Terrain.Xm,1), 1:skep:size(Terrain.Xm,2));
TerrainPrint2.Hm = Terrain.Hm(1:skep:size(Terrain.Hm,1), 1:skep:size(Terrain.Hm,2));

xx = TerrainPrint2.Xm(1,:);
yy = TerrainPrint2.Ym(:,1);

[px,py]=gradient(TerrainPrint2.Hm);

figure
contour(xx,yy,TerrainPrint2.Hm)
hold on
quiver(xx,yy,px,py)
plot(Xt(1,:),Xt(2,:),'color','b')
xlabel('X axis [m]')
ylabel('Y axis [m]')
hold off


%%

maxgrad = zeros(3,1);

[pxm, pym] = gradient(TerrainPrint2.Hm);
grad = sqrt(pxm.^2 + pym.^2);
disp(max(max(abs(grad))))

for indx = 1:length(pxm)
    for indy = 1:length(pym)
        gradx = pxm(indy,indx);
        grady = pym(indy,indx);
        mag = sqrt(gradx *gradx +grady*grady);
        if abs(mag)>abs(maxgrad(3))
            maxgrad(1) =indx;
            maxgrad(2) =indy;
            maxgrad(3) =mag;
        end
        
    end
end

disp(maxgrad)


%save('torques_lemniscate_3.mat','Torques')

%save('changes_lemniscate_3.mat','changes')

%save('state_B_lemniscate_3.mat','Xs','XBs')
