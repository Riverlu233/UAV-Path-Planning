function [BestSol, BestCost, model] = MOGWO_PathPlanner(model, MaxIt, nPop)
% MOGWO_PATHPLANNER 基于多目标灰狼优化算法的三维路径规划器
% (带美化绘图与平滑收敛曲线版)

    %% 1. 初始化
    if nargin < 1 || isempty(model), model = CreateModel(); end
    if nargin < 2 || isempty(MaxIt), MaxIt = 3000; end
    if nargin < 3 || isempty(nPop), nPop = 60; end
    
    % 环境设置
    nVar = model.n; VarSize = [1 nVar];
    VarMin.x=model.xmin; VarMax.x=model.xmax;
    VarMin.y=model.ymin; VarMax.y=model.ymax;
    VarMin.z=model.zmin; VarMax.z=model.zmax;
    VarMax.r = 3 * norm(model.start - model.end) / nVar; VarMin.r = VarMax.r / 9;
    AngleRange = pi/4;
    VarMin.psi = -AngleRange; VarMax.psi = AngleRange;
    VarMin.phi = -AngleRange; VarMax.phi = AngleRange;
    
    % 使用 SafeCostFunction 处理 Inf
    CostFunction = @(x) SafeCostFunction(x, model, VarMin);
    
    %% 2. MOGWO 参数
    nArchive = 100;       % 档案库大小
    nGrid = 7;            % 网格划分数
    alpha_grid = 0.1;     % Grid Inflation
    beta = 2;             % Leader Selection Pressure
    gamma = 2;            % Deletion Pressure
    
    %% 3. 初始化狼群
    empty_wolf.Position = [];
    empty_wolf.Cost = [];
    empty_wolf.IsDominated = [];
    empty_wolf.GridIndex = [];
    empty_wolf.GridSubIndex = [];
    
    wolves = repmat(empty_wolf, nPop, 1);
    
    disp('MOGWO Initializing...');
    for i = 1:nPop
        wolves(i).Position = CreateRandomSolution(VarSize, VarMin, VarMax);
        wolves(i).Cost = CostFunction(SphericalToCart2(wolves(i).Position, model));
    end
    
    % --- 初始化档案库 (Archive) ---
    wolves = DetermineDomination(wolves);
    Archive = wolves(~[wolves.IsDominated]);
    
    % 建立网格
    if isempty(Archive), Archive = wolves(1); end 
    Grid = CreateGrid(Archive, nGrid, alpha_grid);
    for i = 1:numel(Archive)
        Archive(i) = FindGridIndex(Archive(i), Grid);
    end
    
    %% 4. MOGWO 主循环
    % 用于记录每代 "加权最优解" 的 Cost，使曲线更平滑
    BestCostCurve = zeros(MaxIt, 4); 
    
    for it = 1:MaxIt
        a = 2 - it * ((2) / MaxIt); % 收敛因子线性递减
        
        % 从档案库中选择三只头狼 (基于轮盘赌)
        if numel(Archive) < 2
            Alpha = Archive(1); Beta = Archive(1); Delta = Archive(1);
        elseif numel(Archive) < 3
            Alpha = SelectLeader(Archive, beta);
            Beta  = SelectLeader(Archive, beta); 
            Delta = Beta; 
        else
            Alpha = SelectLeader(Archive, beta);
            Beta  = SelectLeader(Archive, beta);
            Delta = SelectLeader(Archive, beta);
        end
        
        % [关键修改] 记录当前的"加权最优解"而非 Alpha，避免曲线震荡
        CurrentBest = SelectBestWeighted(Archive);
        BestCostCurve(it, :) = CurrentBest.Cost';
        
        % --- 更新狼群位置 ---
        for i = 1:nPop
            NewPos = wolves(i).Position;
            
            % MOGWO 更新公式
            NewPos.r = MOGWO_Update(wolves(i).Position.r, Alpha.Position.r, Beta.Position.r, Delta.Position.r, a, VarSize);
            NewPos.r = max(min(NewPos.r, VarMax.r), VarMin.r);
            
            NewPos.psi = MOGWO_Update(wolves(i).Position.psi, Alpha.Position.psi, Beta.Position.psi, Delta.Position.psi, a, VarSize);
            NewPos.psi = max(min(NewPos.psi, VarMax.psi), VarMin.psi);
            
            NewPos.phi = MOGWO_Update(wolves(i).Position.phi, Alpha.Position.phi, Beta.Position.phi, Delta.Position.phi, a, VarSize);
            NewPos.phi = max(min(NewPos.phi, VarMax.phi), VarMin.phi);
            
            NewCost = CostFunction(SphericalToCart2(NewPos, model));
            wolves(i).Position = NewPos;
            wolves(i).Cost = NewCost;
        end
        
        % --- 维护档案库 ---
        Archive = [Archive; wolves];
        Archive = DetermineDomination(Archive);
        Archive = Archive(~[Archive.IsDominated]);
        
        if isempty(Archive), Archive = wolves(1); end 
        
        Grid = CreateGrid(Archive, nGrid, alpha_grid);
        for i = 1:numel(Archive)
            Archive(i) = FindGridIndex(Archive(i), Grid);
        end
        
        if numel(Archive) > nArchive
            Extra = numel(Archive) - nArchive;
            for e = 1:Extra
                Archive = DeleteOneRepMember(Archive, gamma);
            end
        end
        
        % 显示进度
        if mod(it, 100) == 0 || it == 1
            fprintf('Iter %d | Archive: %d | J1:%.2f J2:%.4f\n', it, numel(Archive), CurrentBest.Cost(1), CurrentBest.Cost(2));
        end
    end
    
    %% 5. 结果处理
    disp('MOGWO Finished.');
    
    % 从档案库中选出加权最优解
    FinalLeader = SelectBestWeighted(Archive);
    BestSol = SphericalToCart2(FinalLeader.Position, model);
    BestCost = FinalLeader.Cost;
    
    %% [新增] 绘制 J1-J4 变化趋势图 (美化版)
    figure('Name', 'MOGWO Convergence Analysis', 'Color', 'w', 'Position', [100 100 800 600]);
    
    % J1: Path Length (Blue)
    subplot(2,2,1); 
    plot(BestCostCurve(:,1), 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); 
    title('Objective $J_1$: Path Length', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Cost');
    
    % J2: Collision (Orange/Red)
    subplot(2,2,2); 
    plot(BestCostCurve(:,2), 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); 
    title('Objective $J_2$: Collision Risk', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Value');
    ylim([-0.1 1.1]); % 锁定范围
    
    % J3: Altitude (Yellow/Gold)
    subplot(2,2,3); 
    plot(BestCostCurve(:,3), 'LineWidth', 2, 'Color', [0.9290 0.6940 0.1250]); 
    title('Objective $J_3$: Altitude Stability', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Cost');
    
    % J4: Smoothness (Purple)
    subplot(2,2,4); 
    plot(BestCostCurve(:,4), 'LineWidth', 2, 'Color', [0.4940 0.1840 0.5560]); 
    title('Objective $J_4$: Smoothness', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Cost');
    
    % 绘制路径图
    smooth = 1; 
    PlotSolution(BestSol, model, smooth);
    
    %% 6. 结果持久化
    TxtFileName = 'MOGWO_Best_Result.txt';
    if BestCost(2) > 0.0001
        CurrentFit = 1e5 + BestCost(2) * 1000;
    else
        CurrentFit = 1.0 * BestCost(1) + 0.1 * BestCost(4) + 0.05 * BestCost(3);
    end
    
    HistoryFit = inf; 
    if isfile(TxtFileName)
        try, fid = fopen(TxtFileName, 'r'); data = textscan(fid, '%f', 1); fclose(fid);
        if ~isempty(data{1}), HistoryFit = data{1}; end, catch, end
    end
    
    fprintf('Current Fit: %.4f | History Fit: %.4f\n', CurrentFit, HistoryFit);
    
    if CurrentFit < HistoryFit
        disp('>>> 更新 MOGWO_Best_Result.txt ...');
        fid = fopen(TxtFileName, 'w');
        fprintf(fid, '%.6f\n', CurrentFit);
        fprintf(fid, '%% Best Solution (MOGWO)\n');
        fprintf(fid, 'J1: %.4f\nJ2: %.4f\nJ3: %.4f\nJ4: %.4f\n', BestCost(1), BestCost(2), BestCost(3), BestCost(4));
        fprintf(fid, 'X: '); fprintf(fid, '%.2f ', BestSol.x); fprintf(fid, '\n');
        fprintf(fid, 'Y: '); fprintf(fid, '%.2f ', BestSol.y); fprintf(fid, '\n');
        fprintf(fid, 'Z: '); fprintf(fid, '%.2f ', BestSol.z); fprintf(fid, '\n');
        fclose(fid);
    end
end

%% 辅助函数
function new_val = MOGWO_Update(current, alpha, beta, delta, a, dim)
    r1=rand(dim); A1=2*a*r1-a; C1=2*rand(dim); X1=alpha-A1.*abs(C1.*alpha-current);
    r1=rand(dim); A2=2*a*r1-a; C2=2*rand(dim); X2=beta-A2.*abs(C2.*beta-current);
    r1=rand(dim); A3=2*a*r1-a; C3=2*rand(dim); X3=delta-A3.*abs(C3.*delta-current);
    new_val = (X1 + X2 + X3) / 3;
end

function best = SelectBestWeighted(rep)
    min_score = inf;
    best_idx = 1;
    for k = 1:numel(rep)
        c = rep(k).Cost;
        if c(2) > 0.0001
            score = 1e9 + c(2)*1000;
        else
            score = 1.0*c(1) + 0.1*c(4) + 0.05*c(3);
        end
        if score < min_score
            min_score = score;
            best_idx = k;
        end
    end
    best = rep(best_idx);
end

function sol = CreateRandomSolution(VarSize, VarMin, VarMax)
    sol.r = VarMin.r + rand(VarSize) .* (VarMax.r - VarMin.r);
    sol.psi = VarMin.psi + rand(VarSize) .* (VarMax.psi - VarMin.psi);
    sol.phi = VarMin.phi + rand(VarSize) .* (VarMax.phi - VarMin.phi);
end

function c = SafeCostFunction(x, model, VarMin)
    c = MyCost(x, model, VarMin);
    c(isinf(c)) = 100000; 
end