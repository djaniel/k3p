function robot = inverse_kinematics(robot)
origin=[0;0;0;1];
g=9.81;


fReaction = robot.mass/3 * g;
mCoxa  = (robot.mass_distribution(2) * robot.mass)/6;
fCoxa  = g*mCoxa;
mFemur = (robot.mass_distribution(3) * robot.mass)/6;
fFemur = g*mFemur;
mTibia = (robot.mass_distribution(4) * robot.mass)/6;
fTibia = g*mTibia;
    
for patai = 1:6
    A_MpB= transform (robot.X_MPs(:,patai));
    
    %desired position for i-th leg wrt Mounting point
    xd_mp = A_MpB \ robot.SPs(:,patai); 
    
    %INVERSE KINEMATICS
    swing_ang= atan2(xd_mp(2),xd_mp(1));
    A_SMp = denavithartenberg(0,swing_ang, 0, 0);     % Angle of the swing axis
    A_LS  = denavithartenberg(0,0,robot.lc,pi/2);     % The length of the coxa

    A_LMp = A_SMp * A_LS; 
    xd_coxa = A_LMp \ xd_mp;
    px=xd_coxa(1); 
    py=xd_coxa(2);

    ch = ( px^2+py^2 - robot.lf^2 - robot.lt^2 )/( 2* robot.lf * robot.lt );
%     disp([patai, ch,sqrt(1-ch^2) ])
    knee_ang = atan2(sqrt(1-ch^2),ch);

    lift_ang = atan2(py,px) + atan2(sqrt(py^2+px^2-(robot.lt*cos(knee_ang)+robot.lf)^2),robot.lt*cos(knee_ang)+robot.lf);

    
    robot.Swings(patai) = swing_ang;
    robot.Lifts(patai) = lift_ang;
    robot.Knees(patai) = knee_ang;
    
    %ADDITIONAL TRANSFORMS TO DISPLAY THE POSITION OF THE VEHICLE
    A_mL  = denavithartenberg(0,lift_ang,robot.lp,-pi/2); %angle of the lift axis to the mounting point of the motor
    A_m   = denavithartenberg(robot.hm,0,0,0); %vertical translation due to the height of the motor
    A_KL  = denavithartenberg(0,lift_ang,robot.lf,0); %angle of the lift axis to the knee point (length of the femur)
    A_SpK = denavithartenberg(0,-knee_ang,robot.lt,0); %Tip of the tibia and support point
    
    A_SB  = A_MpB * A_SMp;
    A_LB  = A_MpB * A_LMp;
    A_mB  = A_MpB * A_SMp * A_LS * A_mL;
    A_pB  = A_mB  * A_m; %The mounting point of the propeller
    A_KB  = A_MpB * A_SMp * A_LS * A_KL;
    A_SpB = A_MpB * A_SMp * A_LS * A_KL * A_SpK; %The support point transformation
    
    Cog_tibiaK =    denavithartenberg(0,-knee_ang,robot.lt/2,0) * origin;
    
    Cog_femurL =    denavithartenberg(0, lift_ang,robot.lf/2,0) * origin;
    Cog_tibiaL = A_KL * Cog_tibiaK;
    
    Cog_CoxaS  = A_SMp * denavithartenberg(0,0,robot.lc/2,pi/2) * origin;
    Cog_femurS = A_LS * Cog_femurL;
    Cog_tibiaS = A_LS * Cog_tibiaL;
    
    robot.A_CBs(:,:,patai) = A_LB;
    robot.A_mBs(:,:,patai) = A_mB;
    robot.A_pBs(:,:,patai) = A_pB;
    robot.A_KBs(:,:,patai) = A_KB;
    robot.A_SpBs(:,:,patai)= A_SpB;
    
    wCoxa   = A_SB \ [0;0; fCoxa;1];
    wFemur  = A_LB \ [0;0; fFemur;1];
    wTibiaK = A_KB \ [0;0; fTibia;1];
    %     odd legs moving, patai is odd         OR   even legs moving, patai is even
    %if or(and (robot.parity == 1, mod(patai,2)==1), and (robot.parity == 2, mod(patai,2)==0))
    robot.Torque_S (:,patai) = cross(  Cog_CoxaS(1:3),wCoxa(1:3)) + cross(Cog_femurS(1:3),wCoxa(1:3)) + cross(Cog_tibiaS(1:3),wCoxa(1:3));
    robot.Torque_L (:,patai) = cross( Cog_femurL(1:3),wFemur(1:3))+ cross(Cog_tibiaL(1:3),wFemur(1:3));
    robot.Torque_K (:,patai) = cross( Cog_tibiaK(1:3),wTibiaK(1:3));
        
    %         odd legs moving, patai is even    OR          even legs moving, patai is odd
    if or(and (robot.parity == 1, mod(patai,2)==0),and (robot.parity == 2, mod(patai,2)==1)) 
        FyS  = A_SB \ [0;0;fReaction;1];   %Reaction Force;
        FyL  = A_LB \ [0;0;fReaction;1];
        FyK  = A_KB \ [0;0;fReaction;1];
        
        r_SPK= A_SpK * origin;       %Torque arm of the SP wrt K
        r_SPL= A_KL  * r_SPK;
        r_SPS= A_LS  * r_SPL;
        
        robot.Torque_S(:,patai) = robot.Torque_S(:,patai) + cross(r_SPS(1:3),FyS(1:3));
        robot.Torque_L(:,patai) = robot.Torque_L(:,patai) + cross(r_SPL(1:3),FyL(1:3));
        robot.Torque_K(:,patai) = robot.Torque_K(:,patai) + cross(r_SPK(1:3),FyK(1:3));
    end
    
end