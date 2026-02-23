function [BestSol, BestCost, model] = NMOPSO_PathPlanner(model, MaxIt, nPop)
% NMOPSO_PATHPLANNER (带平滑收敛曲线 + 美化绘图版)
    %% 1. 参数检查与初始化
    if nargin < 1 || isempty(model), model = CreateModel(); end
    if nargin < 2 || isempty(MaxIt), MaxIt = 3000; end
    if nargin < 3 || isempty(nPop), nPop = 60; end
    %% 2. 问题定义
    nVar = model.n;       
    VarSize = [1 nVar];   
    VarMin.x=model.xmin; VarMax.x=model.xmax;           
    VarMin.y=model.ymin; VarMax.y=model.ymax;           
    VarMin.z=model.zmin; VarMax.z=model.zmax;                 
    VarMax.r = 3 * norm(model.start - model.end) / nVar; 
    VarMin.r = VarMax.r / 9;
    AngleRange = pi/4; 
    VarMin.psi = -AngleRange; VarMax.psi = AngleRange;          
    VarMin.phi = -AngleRange; VarMax.phi = AngleRange;          
    
    CostFunction = @(x) SafeCostFunction(x, model, VarMin);    
    %% 3. PSO 参数
    dummy_output = CostFunction(struct('x', ones(1, model.n), 'y', ones(1, model.n), 'z', ones(1, model.n)));
    nObj = numel(dummy_output);                   
    nRep = 100; w = 1; wdamp = 0.98; c1 = 1.5; c2 = 1.5;
    nGrid = 7; alpha_grid = 0.1; beta = 2; gamma = 2; 
    %% 4. 初始化
    empty_particle.Position=[]; empty_particle.Velocity=[]; empty_particle.Cost=[];
    empty_particle.Best.Position=[]; empty_particle.Best.Cost=[];
    empty_particle.IsDominated = []; empty_particle.GridIndex = []; empty_particle.GridSubIndex = [];
    
    GlobalBest.Cost = Inf(nObj, 1);
    particle = repmat(empty_particle, nPop, 1);
    isInit = false;
    
    disp(['NMOPSO Started...']);
    
    while (~isInit)
        for i=1:nPop
            particle(i).Position = CreateRandomSolution(VarSize, VarMin, VarMax);
            particle(i).Velocity.r=zeros(VarSize); particle(i).Velocity.psi=zeros(VarSize); particle(i).Velocity.phi=zeros(VarSize);
            particle(i).Cost = CostFunction(SphericalToCart2(particle(i).Position, model));
            particle(i).Best = particle(i);
            
            if CheckDominance(particle(i).Best.Cost, GlobalBest.Cost)
                GlobalBest = particle(i).Best;
                isInit = true;
            end
        end
    end
    
    particle = DetermineDomination(particle);
    rep = particle(~[particle.IsDominated]); 
    if isempty(rep), rep = particle; end
    Grid = CreateGrid(rep, nGrid, alpha_grid);
    for i = 1:numel(rep), rep(i) = FindGridIndex(rep(i), Grid); end
    %% 5. 主循环
    BestCost_Iter = zeros(MaxIt, nObj); 
    
    for it = 1:MaxIt
        % ... (Repo 维护逻辑) ...
        if isempty(rep)
             particle = DetermineDomination(particle);
             rep = particle(~[particle.IsDominated]);
             if isempty(rep), rep = particle; end 
             Grid = CreateGrid(rep, nGrid, alpha_grid);
             for i = 1:numel(rep), rep(i) = FindGridIndex(rep(i), Grid); end
        end
        
        for i = 1:nPop   
            Leader = SelectLeader(rep, beta);
            
            % --- 更新速度与位置 ---
            % r
            particle(i).Velocity.r = w*particle(i).Velocity.r + c1*rand(VarSize).*(particle(i).Best.Position.r-particle(i).Position.r) + c2*rand(VarSize).*(Leader.Position.r-particle(i).Position.r);
            particle(i).Position.r = particle(i).Position.r + particle(i).Velocity.r;
            particle(i).Position.r = max(min(particle(i).Position.r, VarMax.r), VarMin.r);
            % psi
            particle(i).Velocity.psi = w*particle(i).Velocity.psi + c1*rand(VarSize).*(particle(i).Best.Position.psi-particle(i).Position.psi) + c2*rand(VarSize).*(Leader.Position.psi-particle(i).Position.psi);
            particle(i).Position.psi = particle(i).Position.psi + particle(i).Velocity.psi;
            particle(i).Position.psi = max(min(particle(i).Position.psi, VarMax.psi), VarMin.psi);
            % phi
            particle(i).Velocity.phi = w*particle(i).Velocity.phi + c1*rand(VarSize).*(particle(i).Best.Position.phi-particle(i).Position.phi) + c2*rand(VarSize).*(Leader.Position.phi-particle(i).Position.phi);
            particle(i).Position.phi = particle(i).Position.phi + particle(i).Velocity.phi;
            particle(i).Position.phi = max(min(particle(i).Position.phi, VarMax.phi), VarMin.phi);
            
            particle(i).Cost = CostFunction(SphericalToCart2(particle(i).Position, model));
            
            % PBest 更新
            if CheckDominance(particle(i).Cost, particle(i).Best.Cost)
                particle(i).Best = particle(i);
            elseif ~CheckDominance(particle(i).Best.Cost, particle(i).Cost)
                if rand < 0.5
                     particle(i).Best = particle(i);
                end
            end
        end
        
        % Repo 更新
        particle = DetermineDomination(particle);
        rep = [rep; particle(~[particle.IsDominated])]; 
        rep = DetermineDomination(rep);
        rep = rep(~[rep.IsDominated]);
        Grid = CreateGrid(rep, nGrid, alpha_grid);
        for i = 1:numel(rep), rep(i) = FindGridIndex(rep(i), Grid); end
        if numel(rep) > nRep, for e = 1:numel(rep)-nRep, rep = DeleteOneRepMember(rep, gamma); end, end
        w = w * wdamp;
        
        % [关键修改] 为了画出平滑的收敛曲线，我们不记录随机的 Leader
        % 而是记录当前 Repo 中"加权得分"最高的那个解
        CurrentBestWeighted = SelectBestWeighted(rep);
        BestCost_Iter(it, :) = CurrentBestWeighted.Cost';
        
        if mod(it, 100) == 0 || it == 1
            fprintf('Iter %d | Rep: %d | J1:%.2f J2:%.4f\n', it, numel(rep), CurrentBestWeighted.Cost(1), CurrentBestWeighted.Cost(2));
        end
    end
    
    disp('NMOPSO Finished.');
    % 最终结果也取加权最好的，方便和其他算法公平对比
    GlobalBest = SelectBestWeighted(rep); 
    BestSol = SphericalToCart2(GlobalBest.Position, model);
    BestCost = GlobalBest.Cost;
    
    %% [新增] 绘制 J1-J4 变化图 (美化版 - 保持风格统一)
    figure('Name', 'NMOPSO Convergence Analysis', 'Color', 'w', 'Position', [100 100 800 600]);
    
    % J1: Path Length (Blue)
    subplot(2,2,1); 
    plot(BestCost_Iter(:,1), 'LineWidth', 2, 'Color', [0 0.4470 0.7410]); 
    title('Objective $J_1$: Path Length', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Cost');
    
    % J2: Collision (Orange/Red)
    subplot(2,2,2); 
    plot(BestCost_Iter(:,2), 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980]); 
    title('Objective $J_2$: Collision Risk', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Value');
    ylim([-0.1 1.1]); % 锁定范围
    
    % J3: Altitude (Yellow/Gold)
    subplot(2,2,3); 
    plot(BestCost_Iter(:,3), 'LineWidth', 2, 'Color', [0.9290 0.6940 0.1250]); 
    title('Objective $J_3$: Altitude Stability', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Cost');
    
    % J4: Smoothness (Purple)
    subplot(2,2,4); 
    plot(BestCost_Iter(:,4), 'LineWidth', 2, 'Color', [0.4940 0.1840 0.5560]); 
    title('Objective $J_4$: Smoothness', 'Interpreter', 'latex'); 
    grid on; xlabel('Iteration'); ylabel('Cost');
    
    % 绘制路径图
    smooth = 1; PlotSolution(BestSol, model, smooth);
    
    %% 6. 结果持久化
    TxtFileName = 'NMOPSO_Best_Result.txt';
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
    
    if CurrentFit < HistoryFit
        disp('>>> 更新 NMOPSO_Best_Result.txt ...');
        fid = fopen(TxtFileName, 'w');
        fprintf(fid, '%.6f\n', CurrentFit);
        fprintf(fid, '%% Best Solution (NMOPSO)\n');
        fprintf(fid, 'J1: %.4f\nJ2: %.4f\nJ3: %.4f\nJ4: %.4f\n', BestCost(1), BestCost(2), BestCost(3), BestCost(4));
        fprintf(fid, 'X: '); fprintf(fid, '%.2f ', BestSol.x); fprintf(fid, '\n');
        fprintf(fid, 'Y: '); fprintf(fid, '%.2f ', BestSol.y); fprintf(fid, '\n');
        fprintf(fid, 'Z: '); fprintf(fid, '%.2f ', BestSol.z); fprintf(fid, '\n');
        fclose(fid);
    end
end

% --- 辅助函数 ---

% [新增] 从 Rep 中选出加权分最好的，用于画收敛图
function best = SelectBestWeighted(rep)
    min_score = inf;
    best_idx = 1;
    for k = 1:numel(rep)
        c = rep(k).Cost;
        if c(2) > 0.0001
            score = 1e9 + c(2)*1000;
        else
            score = 1.0*c(1) + 0.1*c(4) + 0.05*c(3); % 保持和 MOGWO 评判标准一致
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
function b = CheckDominance(cx, cy)
    b = all(cx <= cy) && any(cx < cy);
end