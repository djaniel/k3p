function draw_hexapodopter(robot, legs_to_plot,color_to_plot)

% draw robot

A_BW = transform(robot.X);
if ~exist('legs_to_plot','var')
    plotorigin(A_BW,0.04,'B')
end
origin = [0;0;0;1];
MPs_W = zeros(4,6);
show_rf = false ;


if exist('color_to_plot','var')
    if length(color_to_plot)==1
        color1 = get_color(color_to_plot);
        color2 = get_color(color_to_plot);
        color3 = get_color(color_to_plot);
        color4 = get_color(color_to_plot);
    elseif length(color_to_plot)==3;
        color1 = color_to_plot;
        color2 = color_to_plot;
        color3 = color_to_plot;
        color4 = color_to_plot;
    end
        
else
    color1 = get_color('orange');
    color2 = get_color('wet asphalt');
    color3 = get_color('wisteria');
    color4 = [0,0,0];
end


for ind= 1:6
    
    if exist('legs_to_plot','var')
        if ~any( legs_to_plot == ind )
            continue
        end
        
    end
    A_MpB = transform(robot.X_MPs(:,ind));
    MPs_W(:,ind) = A_BW * A_MpB * origin;
    
    A_CB =robot.A_CBs (:,:,ind);
    A_mB =robot.A_mBs (:,:,ind);
    A_pB =robot.A_pBs (:,:,ind);
    A_KB =robot.A_KBs (:,:,ind);
    A_SpB=robot.A_SpBs(:,:,ind);
    
    coxa  = A_BW * A_CB *origin;
    femur = A_BW * A_KB *origin;
    spoint= A_BW * A_SpB*origin;
    motor = A_BW * A_mB *origin;
    prop  = A_BW * A_pB *origin;
    
    
    hcoxa =plot3([MPs_W(1,ind), coxa(1)], [MPs_W(2,ind), coxa(2)], [MPs_W(3,ind), coxa(3)],'-','color',color1);
    set(hcoxa, 'LineWidth',2)
    plot3(coxa(1), coxa(2), coxa(3),'>','color',color1)
    %set (hcoxa, 'linewidth',10)
    
    
    hfemur=plot3([coxa(1), femur(1)],[coxa(2), femur(2)],[coxa(3), femur(3)],'color', color2);
    set(hfemur, 'LineWidth',2)
    plot3(femur(1), femur(2), femur(3),'o','color', color2)
    htibia = plot3([femur(1), spoint(1)],[femur(2), spoint(2)],[femur(3), spoint(3)],'color', color3);
    set(htibia, 'LineWidth',2)
    plot3(spoint(1),spoint(2), spoint(3), 'x','color', color3)
    plot3([motor(1),prop(1)],[motor(2),prop(2)],[motor(3),prop(3)],'-','color',color1)
    plot3(prop(1),prop(2),prop(3),'o','color',color1)
    if show_rf 
        if ind==1
            plotorigin(A_BW * A_MpB ,0.04,['MP_', num2str(ind)])
            plotorigin(A_BW * A_CB  ,0.04,['C_', num2str(ind)])
            plotorigin(A_BW * A_pB  ,0.04,['m_', num2str(ind)])
            plotorigin(A_BW * A_KB  ,0.04,['K_', num2str(ind)])
        end
        plotorigin(A_BW * A_SpB ,0.04,['Sp_', num2str(ind)])
    end
end

if exist('legs_to_plot','var')
    B_W = A_BW * origin;
    %plot3(B_W(1,:), B_W(2,:), B_W(3,:),'ks-','color',[0,0,0],'MarkerFaceColor',[0,0,0]);
else
    hbody = plot3(MPs_W(1,:), MPs_W(2,:), MPs_W(3,:),'*-','color', color4);
    set(hbody,'linewidth',2)
end

