function [BestSol, BestCost, model] = NMOPSO_PathPlanner(model, MaxIt, nPop)
% NMOPSO_PATHPLANNER (带 J1-J4 绘图版)

    %% 1. 参数检查与初始化
    if nargin < 1 || isempty(model), model = CreateModel(); end
    if nargin < 2 || isempty(MaxIt), MaxIt = 1000; end
    if nargin < 3 || isempty(nPop), nPop = 200; end

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
    alpha = 0.5;
    VelMax.r = alpha*(VarMax.r - VarMin.r);    VelMin.r = -VelMax.r;                    
    VelMax.psi = alpha*(VarMax.psi - VarMin.psi); VelMin.psi = -VelMax.psi;                    
    VelMax.phi = alpha*(VarMax.phi - VarMin.phi); VelMin.phi = -VelMax.phi;   
    
    CostFunction = @(x) SafeCostFunction(x, model, VarMin);    

    %% 3. PSO 参数
    dummy_output = CostFunction(struct('x', ones(1, model.n), 'y', ones(1, model.n), 'z', ones(1, model.n)));
    nObj = numel(dummy_output);                   

    nRep = 100; w = 1; wdamp = 0.98; c1 = 1.5; c2 = 1.5;
    nGrid = 7; alpha_grid = 0.1; beta = 2; gamma = 2; mu = 0.5; delta = 20;

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
    % 记录历程: J1, J2, J3, J4
    BestCost_Iter = zeros(MaxIt, nObj); 

    for it = 1:MaxIt
        if isempty(rep)
             particle = DetermineDomination(particle);
             rep = particle(~[particle.IsDominated]);
             if isempty(rep), rep = particle; end 
             Grid = CreateGrid(rep, nGrid, alpha_grid);
             for i = 1:numel(rep), rep(i) = FindGridIndex(rep(i), Grid); end
        end

        for i = 1:nPop   
            Leader = SelectLeader(rep, beta);
            
            % 更新 r, psi, phi ... (省略重复代码以节省篇幅，逻辑与之前完全一致)
            % ... (Update Velocity & Position) ...
            % ... (Boundary Checking) ...
            % ... (Mirroring) ...
            
            % 简写的更新逻辑占位 (请保持原有的完整更新代码)
            % ----------------------------------------------------------------
            particle(i).Velocity.r = w*particle(i).Velocity.r + c1*rand(VarSize).*(particle(i).Best.Position.r-particle(i).Position.r) + c2*rand(VarSize).*(Leader.Position.r-particle(i).Position.r);
            particle(i).Position.r = particle(i).Position.r + particle(i).Velocity.r;
            particle(i).Position.r = max(min(particle(i).Position.r, VarMax.r), VarMin.r);
            
            particle(i).Velocity.psi = w*particle(i).Velocity.psi + c1*rand(VarSize).*(particle(i).Best.Position.psi-particle(i).Position.psi) + c2*rand(VarSize).*(Leader.Position.psi-particle(i).Position.psi);
            particle(i).Position.psi = particle(i).Position.psi + particle(i).Velocity.psi;
            particle(i).Position.psi = max(min(particle(i).Position.psi, VarMax.psi), VarMin.psi);
            
            particle(i).Velocity.phi = w*particle(i).Velocity.phi + c1*rand(VarSize).*(particle(i).Best.Position.phi-particle(i).Position.phi) + c2*rand(VarSize).*(Leader.Position.phi-particle(i).Position.phi);
            particle(i).Position.phi = particle(i).Position.phi + particle(i).Velocity.phi;
            particle(i).Position.phi = max(min(particle(i).Position.phi, VarMax.phi), VarMin.phi);
            % ----------------------------------------------------------------

            particle(i).Cost = CostFunction(SphericalToCart2(particle(i).Position, model));
            
            % 变异 ... (省略重复代码)
            % 更新 PBest ... (省略重复代码)
            if CheckDominance(particle(i).Cost, particle(i).Best.Cost)
                particle(i).Best = particle(i);
            end
        end
        
        particle = DetermineDomination(particle);
        rep = [rep; particle(~[particle.IsDominated])]; 
        rep = DetermineDomination(rep);
        rep = rep(~[rep.IsDominated]);
        Grid = CreateGrid(rep, nGrid, alpha_grid);
        for i = 1:numel(rep), rep(i) = FindGridIndex(rep(i), Grid); end
        if numel(rep) > nRep, for e = 1:numel(rep)-nRep, rep = DeleteOneRepMember(rep, gamma); end, end
        w = w * wdamp;
        
        % 记录当前 GlobalBest (从 Rep 中选出的 Leader)
        CurrentLeader = SelectLeader(rep, beta);
        BestCost_Iter(it, :) = CurrentLeader.Cost';
        
        if mod(it, 100) == 0 || it == 1
            fprintf('Iter %d | L:%.2f C:%.4f\n', it, CurrentLeader.Cost(1), CurrentLeader.Cost(2));
        end
    end
    
    disp('NMOPSO Finished.');
    GlobalBest = SelectLeader(rep, beta);
    BestSol = SphericalToCart2(GlobalBest.Position, model);
    BestCost = GlobalBest.Cost;
    
    %% [新增] 绘制 J1-J4 变化图
    figure('Name', 'NMOPSO Convergence (J1-J4)', 'Color', 'w');
    subplot(2,2,1); plot(BestCost_Iter(:,1), 'LineWidth', 1.5); 
    title('J1: Path Length Cost'); grid on; xlabel('Iter'); ylabel('Cost');
    
    subplot(2,2,2); plot(BestCost_Iter(:,2), 'r', 'LineWidth', 1.5); 
    title('J2: Collision Cost'); grid on; xlabel('Iter'); ylabel('Cost');
    
    subplot(2,2,3); plot(BestCost_Iter(:,3), 'g', 'LineWidth', 1.5); 
    title('J3: Altitude Cost'); grid on; xlabel('Iter'); ylabel('Cost');
    
    subplot(2,2,4); plot(BestCost_Iter(:,4), 'm', 'LineWidth', 1.5); 
    title('J4: Smoothness Cost'); grid on; xlabel('Iter'); ylabel('Cost');
    
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

% 辅助函数
function sol = CreateRandomSolution(VarSize, VarMin, VarMax)
    sol.r = VarMin.r + rand(VarSize) .* (VarMax.r - VarMin.r);
    sol.psi = VarMin.psi + rand(VarSize) .* (VarMax.psi - VarMin.psi);
    sol.phi = VarMin.phi + rand(VarSize) .* (VarMax.phi - VarMin.phi);
end

function c = SafeCostFunction(x, model, VarMin)
    c = MyCost(x, model, VarMin);
    c(isinf(c)) = 100000; % 处理 inf
end

function b = CheckDominance(cx, cy)
    b = all(cx <= cy) && any(cx < cy);
end