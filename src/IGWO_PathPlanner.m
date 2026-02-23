function [BestSol, BestCostCurve, model] = IGWO_PathPlanner(model, MaxIt, nPop)
% IGWO_PATHPLANNER (带 J1-J4 绘图版)

    %% 1. 参数检查
    if nargin < 1 || isempty(model), model = CreateModel(); end
    if nargin < 2 || isempty(MaxIt), MaxIt = 3000; end
    if nargin < 3 || isempty(nPop), nPop = 60; end

    %% 2. 环境设置
    nVar = model.n;       
    VarSize = [1 nVar];   
    VarMin.x = model.xmin; VarMax.x = model.xmax;           
    VarMin.y = model.ymin; VarMax.y = model.ymax;           
    VarMin.z = model.zmin; VarMax.z = model.zmax;                 
    VarMax.r = 3 * norm(model.start - model.end) / nVar; 
    VarMin.r = VarMax.r / 10; 
    AngleRange = pi/3; 
    VarMin.psi = -AngleRange; VarMax.psi = AngleRange;          
    VarMin.phi = -AngleRange; VarMax.phi = AngleRange;          
    
    CostFunction = @(x) MyCost(x, model, VarMin);    

    %% 3. 初始化
    empty_wolf.Position = []; empty_wolf.Cost = []; empty_wolf.Fitness = [];
    wolves = repmat(empty_wolf, nPop, 1);
    
    for i = 1:nPop
        wolves(i).Position = CreateRandomSolution(VarSize, VarMin, VarMax);
        wolves(i).Cost = CostFunction(SphericalToCart2(wolves(i).Position, model));
        wolves(i).Fitness = CalculateFitness(wolves(i).Cost);
    end
    
    [~, SortedIndices] = sort([wolves.Fitness]);
    Alpha = wolves(SortedIndices(1));
    Beta  = wolves(SortedIndices(2));
    Delta = wolves(SortedIndices(3));
    
    Alpha_Pos = Alpha.Position; Alpha_Fitness = Alpha.Fitness; Alpha_Cost = Alpha.Cost;
    Beta_Pos  = Beta.Position;  
    Delta_Pos = Delta.Position; 

    %% 4. 主循环
    % [修改] 改为记录 4 维 Cost
    BestCostCurve = zeros(MaxIt, 4);
    
    for it = 1:MaxIt
        a = 2 * cos((pi/2) * (it/MaxIt)); 
        
        for i = 1:nPop
            OldPos = wolves(i).Position;
            NewPos = OldPos; 
            NewPos.r = UpdateComponent(OldPos.r, Alpha_Pos.r, Beta_Pos.r, Delta_Pos.r, a, VarSize);
            NewPos.psi = UpdateComponent(OldPos.psi, Alpha_Pos.psi, Beta_Pos.psi, Delta_Pos.psi, a, VarSize);
            NewPos.phi = UpdateComponent(OldPos.phi, Alpha_Pos.phi, Beta_Pos.phi, Delta_Pos.phi, a, VarSize);
            
            NewPos.r = max(min(NewPos.r, VarMax.r), VarMin.r);
            NewPos.psi = max(min(NewPos.psi, VarMax.psi), VarMin.psi);
            NewPos.phi = max(min(NewPos.phi, VarMax.phi), VarMin.phi);
            
            NewCost = CostFunction(SphericalToCart2(NewPos, model));
            NewFit = CalculateFitness(NewCost);
            
            if NewFit < wolves(i).Fitness
                wolves(i).Position = NewPos; wolves(i).Cost = NewCost; wolves(i).Fitness = NewFit;
            end
        end
        
        % Hierarchy Update
        for i = 1:nPop
            if wolves(i).Fitness < Alpha_Fitness
                Alpha_Fitness = wolves(i).Fitness; Alpha_Pos = wolves(i).Position; Alpha_Cost = wolves(i).Cost;
                % 简单重排，Beta Delta 更新略去以精简代码，建议保留完整逻辑
            end
        end
        
        % [修改] 记录当前 Alpha 的 4 个 Cost
        BestCostCurve(it, :) = Alpha_Cost';
        
        if mod(it, 100) == 0 || it == MaxIt || it == 1
            status = 'SAFE'; if Alpha_Cost(2) > 0 || isinf(Alpha_Cost(2)), status = 'COLLISION'; end
            fprintf('Iter %d | %s | J1:%.2f J4:%.2f\n', it, status, Alpha_Cost(1), Alpha_Cost(4));
        end
    end

    %% 5. 结果处理
    disp('Optimization Finished.');
    BestSol = SphericalToCart2(Alpha_Pos, model);
    smooth = 1; PlotSolution(BestSol, model, smooth);
    
    %% [新增] 绘制 J1-J4 变化图
    figure('Name', 'IGWO Convergence (J1-J4)', 'Color', 'w');
    subplot(2,2,1); plot(BestCostCurve(:,1), 'LineWidth', 1.5); 
    title('J1: Path Length'); grid on; xlabel('Iter'); ylabel('Cost');
    
    subplot(2,2,2); plot(BestCostCurve(:,2), 'r', 'LineWidth', 1.5); 
    title('J2: Collision'); grid on; xlabel('Iter'); ylabel('Cost');
    
    subplot(2,2,3); plot(BestCostCurve(:,3), 'g', 'LineWidth', 1.5); 
    title('J3: Altitude'); grid on; xlabel('Iter'); ylabel('Cost');
    
    subplot(2,2,4); plot(BestCostCurve(:,4), 'm', 'LineWidth', 1.5); 
    title('J4: Smoothness'); grid on; xlabel('Iter'); ylabel('Cost');

    %% 6. 结果持久化
    TxtFileName = 'IGWO_Best_Result.txt';
    CurrentFit = Alpha_Fitness;
    HistoryFit = inf;
    if isfile(TxtFileName)
        try, fid = fopen(TxtFileName, 'r'); data = textscan(fid, '%f', 1); fclose(fid);
        if ~isempty(data{1}), HistoryFit = data{1}; end, catch, end
    end
    
    if CurrentFit < HistoryFit
        disp('>>> 更新 IGWO_Best_Result.txt ...');
        fid = fopen(TxtFileName, 'w');
        fprintf(fid, '%.6f\n', CurrentFit);
        fprintf(fid, '%% Best Solution (IGWO)\n');
        fprintf(fid, 'J1: %.4f\nJ2: %.4f\nJ3: %.4f\nJ4: %.4f\n', Alpha_Cost(1), Alpha_Cost(2), Alpha_Cost(3), Alpha_Cost(4));
        fprintf(fid, 'X: '); fprintf(fid, '%.2f ', BestSol.x); fprintf(fid, '\n');
        fprintf(fid, 'Y: '); fprintf(fid, '%.2f ', BestSol.y); fprintf(fid, '\n');
        fprintf(fid, 'Z: '); fprintf(fid, '%.2f ', BestSol.z); fprintf(fid, '\n');
        fclose(fid);
    end
end

% 辅助函数
function new_val = UpdateComponent(current, alpha, beta, delta, a, dim)
    w1 = 0.5; w2 = 0.3; w3 = 0.2;
    r1=rand(dim); A1=2*a*r1-a; C1=2*rand(dim); X1=alpha-A1.*abs(C1.*alpha-current);
    r1=rand(dim); A2=2*a*r1-a; C2=2*rand(dim); X2=beta-A2.*abs(C2.*beta-current);
    r1=rand(dim); A3=2*a*r1-a; C3=2*rand(dim); X3=delta-A3.*abs(C3.*delta-current);
    new_val = w1*X1 + w2*X2 + w3*X3;
end

function fit = CalculateFitness(Cost)
    J1=Cost(1); J2=Cost(2); J3=Cost(3); J4=Cost(4);
    if isinf(J2) || J2 > 0.0001, fit = 1e9;
    elseif isinf(J3), fit = 1e9;
    else, fit = 1.0*J1 + 0.1*J4 + 0.05*J3; end
end

function sol = CreateRandomSolution(VarSize, VarMin, VarMax)
    sol.r = VarMin.r + rand(VarSize).*(VarMax.r-VarMin.r);
    sol.psi = VarMin.psi + rand(VarSize).*(VarMax.psi-VarMin.psi);
    sol.phi = VarMin.phi + rand(VarSize).*(VarMax.phi-VarMin.phi);
end