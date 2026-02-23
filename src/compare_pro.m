clc; close all; clear all;

%% 1. 加载地图背景
disp('正在加载地图模型...');
model = CreateModel(); % 必须加载同一个地图，否则坐标对不上

%% 2. 定义文件名
File_NMOPSO = 'NMOPSO_Best_Result.txt';
File_MOGWO    = 'MOGWO_Best_Result.txt';
File_GWO    = 'Standard_GWO_Result.txt';    %可以改这里换文件

%% 3. 读取数据
disp('正在读取路径数据...');

[Path_PSO, Fitness_PSO, Exists_PSO] = ReadPathFromTxt(File_NMOPSO);
[Path_MOGWO, Fitness_MOGWO, Exists_MOGWO] = ReadPathFromTxt(File_MOGWO);
[Path_GWO, Fitness_GWO, Exists_GWO] = ReadPathFromTxt(File_GWO);

%% 4. 绘图对比
figure(100);
set(gcf, 'Name', '历史最优路径对比', 'Color', 'w');

% 4.1 画地形
PlotModel(model);
hold on;

% 4.2 画 NMOPSO 路径 (蓝色)
if Exists_PSO
    DrawPathFromData(Path_PSO, model, 'b', '-', 2.5, 'NMOPSO Best');
    fprintf('NMOPSO 记录分数: %.4f\n', Fitness_PSO);
else
    warning(['未找到文件: ' File_NMOPSO]);
end

% 4.3 画 MOGWO 路径 (红色)
if Exists_MOGWO
    DrawPathFromData(Path_MOGWO, model, 'r', '-', 2.5, 'MOGWO Best');
    fprintf('MOGWO    记录分数: %.4f\n', Fitness_MOGWO);
else
    warning(['未找到文件: ' File_MOGWO]);
end

if Exists_GWO
    DrawPathFromData(Path_GWO, model, 'g', '-', 2.5, 'GWO Best');
    fprintf('GWO    记录分数: %.4f\n', Fitness_GWO);
else
    warning(['未找到文件: ' File_GWO]);
end

% 4.4 标注与美化
title('Comparison of Saved Best Paths');
legend_str = {};
if Exists_PSO, legend_str{end+1} = 'NMOPSO Path'; end
if Exists_MOGWO, legend_str{end+1} = 'MOGWO Path'; end
if Exists_GWO, legend_str{end+1} = 'GWO Path'; end
% 注意：Legend 可能会因为 PlotModel 里的对象较多而需要手动调整，这里只添加路径的图例
% 实际上 PlotModel 已经画了 Terrain 和 Obstacles，这里我们只关注新增的线
legend([findobj(gca, 'DisplayName', 'NMOPSO Best'); findobj(gca, 'DisplayName', 'MOGWO Best'); findobj(gca, 'DisplayName', 'GWO Best')], ...
       'Location', 'Best');

view(-30, 50); % 设置一个好看的视角
grid on;

%% ==========================================================
%% 辅助函数：读取 TXT
%% ==========================================================
function [sol, fitness, file_exists] = ReadPathFromTxt(filename)
    sol.x = []; sol.y = []; sol.z = [];
    fitness = inf;
    file_exists = false;
    
    if ~isfile(filename)
        return;
    end
    
    file_exists = true;
    fid = fopen(filename, 'r');
    
    % 1. 读取第一行 Fitness
    line = fgetl(fid);
    fitness = str2double(line);
    
    % 2. 逐行寻找 X, Y, Z
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        
        if startsWith(line, 'X:')
            % 去掉 'X:' (前2个字符) 读取后面的数字
            % 如果你的txt里是 "X: 1 2 3"，则从第3个字符开始截取
            str_nums = line(3:end);
            sol.x = str2num(str_nums); %#ok<ST2NM>
            
        elseif startsWith(line, 'Y:')
            str_nums = line(3:end);
            sol.y = str2num(str_nums); %#ok<ST2NM>
            
        elseif startsWith(line, 'Z:')
            str_nums = line(3:end);
            sol.z = str2num(str_nums); %#ok<ST2NM>
        end
    end
    fclose(fid);
end

%% ==========================================================
%% 辅助函数：绘图
%% ==========================================================
function DrawPathFromData(sol, model, color, style, width, name)
    % 检查数据是否为空
    if isempty(sol.x) || isempty(sol.y) || isempty(sol.z)
        warning(['文件 ' name ' 数据格式不正确或为空']);
        return;
    end

    % 1. 拼接完整路径 (起点 -> 优化点 -> 终点)
    xs = model.start(1); ys = model.start(2); zs = model.start(3);
    xf = model.end(1);   yf = model.end(2);   zf = model.end(3);
    
    x_all = [xs, sol.x, xf];
    y_all = [ys, sol.y, yf];
    z_rel_all = [zs, sol.z, zf]; % 文件里存的是相对高度
    
    % 2. 计算绝对高度 (用于绘图)
    % Z_plot = 地形高度(H) + 无人机相对高度(z)
    z_abs = zeros(1, length(x_all));
    
    for k = 1:length(x_all)
        ix = round(x_all(k));
        iy = round(y_all(k));
        
        % 边界保护
        ix = max(1, min(ix, model.MAPSIZE_X));
        iy = max(1, min(iy, model.MAPSIZE_Y));
        
        z_abs(k) = z_rel_all(k) + model.H(iy, ix);
    end
    
    % 3. 绘制线条
    plot3(x_all, y_all, z_abs, 'Color', color, 'LineStyle', style, ...
        'LineWidth', width, 'DisplayName', name);
    
    % 4. 绘制节点点
    plot3(x_all, y_all, z_abs, 'o', 'Color', color, 'MarkerFaceColor', color, ...
        'MarkerSize', 4, 'HandleVisibility', 'off');
end