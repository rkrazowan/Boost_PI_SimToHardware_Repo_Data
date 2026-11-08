%% Main_Run_All.m  -  robust PSO + GWO + GA + NSGA-II vs baseline PI gains
clc; clear; close all;
cfg  = Boost_Config();
cfgS = cfg;  cfgS.fast = false;            % switching model (validation)
cfgF = cfg;  cfgF.fast = cfg.useFast;      % used INSIDE the optimisers
fobj = @(x) Boost_Cost(x, cfgF, 'scalar');
fvec = @(x) Boost_Cost(x, cfgF, 'vector');

% Base-workspace copies (only needed if you open/run the model by hand)
assignin('base','L',cfg.L); assignin('base','C',cfg.C); assignin('base','R',cfg.R);
assignin('base','Vin',cfg.Vin); assignin('base','Vout',cfg.Vout); assignin('base','fs',cfg.fs);

% Seeds = valid baseline gains (swarm starts with them)
seeds = []; bn = fieldnames(cfg.base);
for i = 1:numel(bn)
    b = cfg.base.(bn{i});  if all(isfinite(b)), seeds(end+1,:) = b; end %#ok<SAGROW>
end

%% 1) Single-objective optimizers, repeated runs
opts = {'PSO','GWO','GA'};   nR = cfg.nRuns;   R = struct();   best = struct();
for o = 1:numel(opts)
    X = zeros(nR,2); J = zeros(nR,1); H = nan(nR,cfg.maxIter);
    for r = 1:nR
        rng(r);                                   % reproducible
        switch opts{o}
            case 'PSO'
                [X(r,:),J(r),h] = PSO_Custom(fobj,cfg.lb,cfg.ub,cfg.nP,cfg.maxIter, ...
                                             cfg.c1,cfg.c2,seeds,cfg.stall);
            case 'GWO'
                [X(r,:),J(r),h] = GWO_Custom(fobj,cfg.lb,cfg.ub,cfg.nP,cfg.maxIter);
            case 'GA'
                go = optimoptions('ga','PopulationSize',cfg.nP, ...
                     'MaxGenerations',cfg.maxIter,'InitialPopulationMatrix',seeds,'Display','off');
                [X(r,:),J(r)] = ga(fobj,2,[],[],[],[],cfg.lb,cfg.ub,[],go);
                h = nan(1,cfg.maxIter);
        end
        H(r,:) = h(:)';
        fprintf('%s run %d/%d  Kp=%.5f Ki=%.5f J=%.4f\n',opts{o},r,nR,X(r,1),X(r,2),J(r));
    end
    R.(opts{o}) = struct('X',X,'J',J,'H',H);
    % re-rank the top-3 runs on the SWITCHING model, keep the best
    [~,ord] = sort(J);  top = X(ord(1:min(3,nR)),:);
    cs = zeros(size(top,1),1);
    for q = 1:size(top,1), cs(q) = Boost_Cost(top(q,:),cfgS,'scalar'); end
    [~,ib] = min(cs);  best.(opts{o}) = top(ib,:);
    fprintf('>> %s: best %.4f | mean %.4f | std %.4f | worst %.4f\n\n', ...
            opts{o}, min(J), mean(J), std(J), max(J));
end

%% 2) NSGA-II (multi-objective) -> Pareto front + knee solution
mo = optimoptions('gamultiobj','PopulationSize',40,'MaxGenerations',30,'Display','off');
rng(1);
[Xp,Fp] = gamultiobj(fvec,2,[],[],[],[],cfg.lb,cfg.ub,mo);
Fn = (Fp-min(Fp))./(max(Fp)-min(Fp)+eps);
[~,ik] = min(vecnorm(Fn,2,2));   best.NSGA2_knee = Xp(ik,:);

figure('Name','Pareto front');
scatter3(Fp(:,1),Fp(:,2),Fp(:,3),40,Fp(:,3),'filled'); grid on;
xlabel('Speed (ITAE+Ts+margin)'); ylabel('Worst overshoot (pu)'); zlabel('Duty effort');
title('NSGA-II Pareto front'); 

%% 3) Convergence plot (mean over runs)
figure('Name','Convergence'); hold on;
plot(mean(R.PSO.H,1,'omitnan'),'-o','LineWidth',1.5,'DisplayName','PSO');
plot(mean(R.GWO.H,1,'omitnan'),'-s','LineWidth',1.5,'DisplayName','GWO');
xlabel('Iteration'); ylabel('Best cost'); legend; grid on; title('Convergence');

%% 4) Final evaluation of all controllers on ALL scenarios
ctrl = struct();
for i = 1:numel(bn)
    b = cfg.base.(bn{i});  if all(isfinite(b)), ctrl.(bn{i}) = b; end
end
ctrl.PSO = best.PSO;  ctrl.GWO = best.GWO;  ctrl.GA = best.GA;  ctrl.NSGA2_knee = best.NSGA2_knee;
cn = fieldnames(ctrl);  nS = size(cfg.S,1);
OS = zeros(nS,numel(cn)); TS = OS; Sum = zeros(numel(cn),7);
for c = 1:numel(cn)
    x = ctrl.(cn{c});  [M,MG] = Boost_Sim_Metrics(x,cfgS);
    OS(:,c) = 100*M(:,2);  TS(:,c) = 1000*M(:,3)*cfg.Tstop;
    Sum(c,:) = [x, max(M(:,1)), max(OS(:,c)), max(TS(:,c)), min(MG(:,1)), min(MG(:,2))];
end
disp('--- Worst-case summary over all scenarios ---');
disp(array2table(Sum,'RowNames',cn,'VariableNames', ...
    {'Kp','Ki','ITAEn_max','OS_max_pct','Ts_max_ms','PM_min_deg','GM_min_dB'}));
disp('--- Overshoot [%] per scenario ---');
disp(array2table(OS,'RowNames',cfg.scenNames,'VariableNames',cn));
disp('--- Settling time [ms] per scenario ---');
disp(array2table(TS,'RowNames',cfg.scenNames,'VariableNames',cn));

figure('Name','Nominal step response'); hold on;
for c = 1:numel(cn)
    ts = Get_Vout(ctrl.(cn{c}),cfg,1);
    plot(ts.Time*1e3, ts.Data, 'LineWidth',1.4,'DisplayName',cn{c});
end
yline(cfg.Vout,':k','HandleVisibility','off'); grid on; legend('Location','southeast');
xlabel('Time (ms)'); ylabel('Vout (V)'); title('Nominal scenario');

% Warn if optimum sits on a bound (widen lb/ub and rerun)
for c = 1:numel(cn)
    x = ctrl.(cn{c});
    if any(abs(x-cfg.lb)<1e-9) || any(abs(x-cfg.ub)<1e-9)
        warning('%s hit a search bound - widen bounds.',cn{c});
    end
end
save('results_all.mat','R','best','Xp','Fp','ctrl','cfg');

%% ================= local functions =================
function ts = Get_Vout(x,cfg,k)
    s = cfg.S(k,:);  in = Simulink.SimulationInput(cfg.model);
    in = in.setVariable('Kp',x(1)).setVariable('Ki',x(2)) ...
            .setVariable('Vin',cfg.Vin*s(1)).setVariable('R',cfg.R*s(2)) ...
            .setVariable('L',cfg.L*s(3)).setVariable('C',cfg.C*s(4)) ...
            .setVariable('Vout',cfg.Vout).setVariable('Vref',cfg.Vout).setVariable('fs',cfg.fs);
    in = in.setModelParameter('StopTime', num2str(cfg.Tstop));
    out = sim(in);  ts = out.get('Vout_sim');
end

function [gB,gC,hist] = PSO_Custom(f,lb,ub,nP,maxIter,c1,c2,seeds,stallMax)
    nV = numel(lb);  vmax = 0.1*(ub-lb);
    X = lb + rand(nP,nV).*(ub-lb);
    if ~isempty(seeds), X(1:size(seeds,1),:) = seeds; end
    V = -vmax + 2*vmax.*rand(nP,nV);
    pB = X;  pC = inf(nP,1);  gC = inf;  gB = X(1,:);
    hist = nan(maxIter,1);  stall = 0;
    for it = 1:maxIter
        w = 0.9 - 0.5*(it/maxIter);   prev = gC;
        for i = 1:nP
            c = f(X(i,:));
            if c < pC(i), pC(i) = c; pB(i,:) = X(i,:); end
            if c < gC,    gC = c;    gB = X(i,:);      end
        end
        hist(it) = gC;
        if prev - gC < 1e-6*max(1,abs(prev)), stall = stall+1; else, stall = 0; end
        if stall >= stallMax, hist(it+1:end) = gC; break; end
        V = w*V + c1*rand(nP,nV).*(pB-X) + c2*rand(nP,nV).*(gB-X);
        V = min(max(V,-vmax),vmax);
        X = min(max(X+V,lb),ub);
    end
end

function [best,bestJ,hist] = GWO_Custom(f,lb,ub,nA,maxIter)
    nV = numel(lb);  X = lb + rand(nA,nV).*(ub-lb);
    bestJ = inf;  best = X(1,:);  hist = zeros(maxIter,1);
    for it = 1:maxIter
        fit = zeros(nA,1);
        for i = 1:nA, X(i,:) = min(max(X(i,:),lb),ub); fit(i) = f(X(i,:)); end
        [fs,idx] = sort(fit);
        Xa = X(idx(1),:);  Xb = X(idx(2),:);  Xd = X(idx(3),:);
        if fs(1) < bestJ, bestJ = fs(1); best = Xa; end
        hist(it) = bestJ;
        a = 2 - 2*(it-1)/maxIter;
        for i = 1:nA
            A = 2*a*rand(1,nV)-a;  C = 2*rand(1,nV);  X1 = Xa - A.*abs(C.*Xa - X(i,:));
            A = 2*a*rand(1,nV)-a;  C = 2*rand(1,nV);  X2 = Xb - A.*abs(C.*Xb - X(i,:));
            A = 2*a*rand(1,nV)-a;  C = 2*rand(1,nV);  X3 = Xd - A.*abs(C.*Xd - X(i,:));
            X(i,:) = (X1+X2+X3)/3;
        end
    end
end