function model=CreateModel()
    H = imread('ChrismasTerrain2.tif');
    H (H < 0) = 0;
    MAPSIZE_X = size(H,2);
    MAPSIZE_Y = size(H,1);
    [X,Y] = meshgrid(1:MAPSIZE_X,1:MAPSIZE_Y);

    % --- 1. 原有的障碍物 ---
    % 格式: [x, y, z_bottom, Radius, Height]
    threats_existing = [
        200, 230, 200, 70, 250;
        600, 250, 200, 70, 300;
        450, 550, 200, 70, 250;
        200, 500, 200, 60, 250;
        620, 530, 200, 60, 300;
        425, 350, 420,130, 100 
    ];

    % --- 2. 新增的障碍物组 ---
    % 你的原始数据格式: [x, y, Height, Radius]
    raw_new_data = [
        900, 560, 400, 50;
        840, 600, 120, 40;
        780, 640, 400, 50;
        720, 680, 120, 40;
        660, 720, 400, 50;
        600, 760, 120, 40;
        540, 800, 400, 50;
        480, 840, 120, 40;
        420, 880, 400, 50;
        360, 920, 120, 40;
        320, 960, 400, 50
    ];

    % 构造 z 列 (全部设为 200)
    num_new = size(raw_new_data, 1);
    z_fixed = 100 * ones(num_new, 1);

    % 提取并重组列，目标格式: [x, y, z, R, h]
    % raw(:,1) -> x
    % raw(:,2) -> y
    % z_fixed  -> z
    % raw(:,4) -> R (注意：这是你输入的第4列)
    % raw(:,3) -> h (注意：这是你输入的第3列)
    
    threats_new = [raw_new_data(:,1), raw_new_data(:,2), z_fixed, raw_new_data(:,4), raw_new_data(:,3)];

    % --- 3. 合并所有障碍物 ---
    model.threats = [threats_existing; threats_new];

    % ... (其余参数设置保持不变) ...
    xmin= 1; xmax= MAPSIZE_X;
    ymin= 1; ymax= MAPSIZE_Y;
    zmin = 100; zmax = 200;  
    
    start_location = [50;50;150];
    end_location = [800;800;50];
    n=10;
    
    model.start=start_location;
    model.end=end_location;
    model.n=n;
    model.xmin=xmin; model.xmax=xmax;
    model.ymin=ymin; model.ymax=ymax;
    model.zmin=zmin; model.zmax=zmax;
    model.MAPSIZE_X = MAPSIZE_X;
    model.MAPSIZE_Y = MAPSIZE_Y;
    model.X = X; model.Y = Y; model.H = H;
    
    PlotModel(model);
end