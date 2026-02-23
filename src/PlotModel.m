% Plot the terrain model and threats
function PlotModel(model)
    mesh(model.X,model.Y,model.H);   % Plot the data
    colormap summer;                 
    set(gca, 'Position', [0 0 1 1]); 
    axis equal vis3d on;             
    shading interp;                  
    material dull;                   
    camlight left;                   
    lighting gouraud;                
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    hold on
    
    % Threats as cylinders
    threats = model.threats;
    threat_num = size(threats,1);
    
    % 【删除】原来的全局固定高度 h=250; 删掉这行
    
    for i = 1:threat_num
        threat = threats(i,:);
        threat_x = threat(1);
        threat_y = threat(2);
        threat_z = threat(3);       % 这里默认 z 是圆柱底部的海拔
        threat_radius = threat(4);
        threat_h = threat(5);       % 【新增】读取第5列：该障碍物的高度
        
        [xc,yc,zc]=cylinder(threat_radius); % create a unit cylinder
        
        % set the center and height 
        xc=xc+threat_x;  
        yc=yc+threat_y;
        
        % 【修改】使用当前障碍物的高度 threat_h 进行拉伸
        % 原理：cylinder 生成的 zc 范围是 0~1
        % zc * threat_h 变成 0~h
        % 再加上 threat_z，范围变成 [bottom, bottom+h]
        zc=zc * threat_h + threat_z; 
        
        c = mesh(xc,yc,zc); % plot the cylinder 
        set(c,'Edgecolor','none','Facecolor','red','FaceAlpha',.3); 
    end
end