%% Run_PI_Optimization_Paper_FINAL.m
% Robust (worst-case) PI tuning of a boost converter: PSO | GA | GWO | NSGA-II knee
% FINAL RUN for the research paper (cfg.quickTest = false, ~170 min).
%
% HOW TO USE
%   1) Same folder as PSO_PI_Controlled_Booster.slx. Keep the PC awake and plugged in.
%   2) Run this file. Nothing else to change.
%   3) Results:
%        Paper_Figures/  Fig1..Fig12 as PDF + PNG (600 dpi) + FIG, plus Figure_List.csv/.txt
%        Paper_Tables/   6 tables as CSV + LaTeX
%        PI_Opt_Stage1.mat, PI_Opt_Stage2.mat, PI_Opt_Results.mat, Run_Log.txt
%        Final_Results.zip  (everything above in one file)
% Requires: Control System Toolbox + Simulink. No other toolbox.
clear; clc; close all;
warning('off','MATLAB:print:ContentTypeImageSuggested');
if exist('Run_Log.txt','file'), delete('Run_Log.txt'); end
diary('Run_Log.txt'); tAll = tic;

%% ============================ 0. USER SETTINGS ============================
cfg.mdl       = 'PSO_PI_Controlled_Booster';
cfg.quickTest = false;         % <<< FINAL RUN (set true only for a 10 min check)
cfg.budgetMin = 170;           % total wall-clock budget [min]
cfg.fracOpt   = 0.55;          % share of the budget for the optimisation
cfg.fracSim   = 0.30;          % share of the budget for Simulink validation
cfg.popTarget = 30; cfg.popMin = 10;        % population size (auto-reduced if needed)
cfg.runsTarget = 15; cfg.runsMin = 8;       % independent runs per algorithm
cfg.iterMin   = 30;                         % minimum iterations
cfg.nMC       = 500;           % Monte Carlo samples (analytic, held-out)
cfg.nValMin   = 3;             % minimum Simulink validation scenarios per design
cfg.doSimValidation = true;
cfg.seed      = 2026;
if cfg.quickTest
    cfg.budgetMin = 10; cfg.fracOpt = 0.30; cfg.fracSim = 0.35;
    cfg.popTarget = 10; cfg.popMin = 6; cfg.runsTarget = 3; cfg.runsMin = 2;
    cfg.iterMin = 6; cfg.nMC = 60; cfg.nValMin = 2;
end

% nominal circuit
nom.Vin = 10; nom.D = 0.45; nom.fs = 62.5e3;
nom.L = 22.5e-6; nom.C = 470e-6; nom.R = 50; nom.RL = 0.1;
cfg.nom = nom;  cfg.Vref = nom.Vin/(1-nom.D);        % 18.18 V
cfg.Tstop = 0.12;                                    % Simulink validation stop time [s]

% uncertainty set (multiplying factors) -> justify these from datasheets/measurements
cfg.var.Vin = [0.8 1.2];  cfg.var.L = [0.8 1.2];
cfg.var.C   = [0.8 1.2];  cfg.var.R = [0.5 1.5];
cfg.scenarioSet = 'corners';      % 'corners' (17 scenarios) | 'full' (81)

% cost:  J = max_s(Js) + 0.2*mean_s(Js) + phase-margin penalty
cfg.w     = [0.3 0.3 0.4];        % weights: IAE, settling time, overshoot
cfg.ref   = [0.01 0.03 5];        % normalisation: IAE [s], Ts [s], OS [%]
cfg.PMmin = 45;                   % phase margin limit in every scenario [deg]
cfg.big   = 1e3;                  % penalty for unstable / failed designs
cfg.Tend  = 0.1;  cfg.N = 500;    % analytic step grid
cfg.spec  = [10 0.020 45];        % Monte Carlo acceptance: OS [%], Ts [s], PM [deg]
cfg.dist.dVin = -0.2;             % line step: -20 % of Vin
cfg.dist.dIo  = 0.2;              % load step: +20 % of nominal output current

% reference designs (from your pidtune / PID Tuner app results)
refNames = {'pidtune wc=100','pidtune wc=150','pidtune wc=300','Auto (app)','Slider (app)'};
Kp_ref   = [0 0 0 0 0.00243];
Ki_ref   = [3.0443 4.5652 9.1165 48.057 15.314];

% PI search range
lim.Kp = [0 0.03];   lim.Ki = [1.5 150];

set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman', ...
    'defaultAxesFontSize',9,'defaultLineLineWidth',1.4,'defaultAxesBox','on', ...
    'defaultAxesXGrid','on','defaultAxesYGrid','on');

%% ============================ 1. SETUP, TIMING, SIZING ====================
rng(cfg.seed);
scen = buildScenarios(nom, cfg);  nS = numel(scen);  idxAll = 1:nS;
idxNom = find(arrayfun(@(s) all(s.fac==1), scen), 1);
ctx = buildCtx(scen, cfg, lim);
algos = {'PSO','GA','GWO','NSGA-II knee'};  nA = 4;
names = [refNames, algos];  nRef = numel(refNames);  nD = numel(names);
funJ  = @(z) objSO(z, ctx, idxAll, 'analytic');
funMO = @(z) objMO(z, ctx, idxAll, 'analytic');

nT = 20; tic; for i = 1:nT, funJ(rand(1,2)); end; tEval = toc/nT;

tSim = NaN;
if cfg.doSimValidation
    try
        load_system(cfg.mdl);
        assignin('base','fs',nom.fs); assignin('base','Fsw',nom.fs);
        assignin('base','D',nom.D);   assignin('base','Tstop',cfg.Tstop); assignin('base','RL',nom.RL);
        runSim(Kp_ref(5), Ki_ref(5), scen(idxNom), ctx, 0.005);        % warm-up / compile
        tic; [~,~,~,okS] = runSim(Kp_ref(5), Ki_ref(5), scen(idxNom), ctx, cfg.Tstop); tSim = toc;
        if ~okS, error('Simulink test run failed'); end
    catch ME
        warning('Simulink check failed (%s) -> Simulink validation disabled.', ME.message);
        cfg.doSimValidation = false;
    end
end

Btot = cfg.budgetMin*60;
perAlgo = floor(floor(cfg.fracOpt*Btot/tEval)/4);
runs = cfg.runsTarget;
while true
    perRun = floor(perAlgo/runs);
    pop  = min(cfg.popTarget, max(cfg.popMin, round(perRun/40)));
    iter = floor(perRun/pop) - 1;
    if iter >= cfg.iterMin || runs <= cfg.runsMin, break; end
    runs = runs - 1;
end
iter = max(iter, 4);  cfg.pop = pop;  cfg.iter = iter;  cfg.nRuns = runs;
nVal = 0;
if cfg.doSimValidation
    nVal = floor(cfg.fracSim*Btot/tSim/nD);  nVal = max(cfg.nValMin, min(nVal, 25));
end
fprintf('Budget %d min | 1 analytic cost eval = %.3f s | 1 Simulink run = %.1f s\n', cfg.budgetMin, tEval, tSim);
fprintf('PLAN: pop = %d, iterations = %d, runs = %d per algorithm (equal evaluations for all)\n', pop, iter, runs);
fprintf('      expected optimisation time ~ %.1f min | Simulink validation: %d scenarios x %d designs ~ %.1f min\n', ...
    4*runs*pop*(iter+1)*tEval/60, nVal, nD, nVal*nD*tSim/60);
fprintf('Search range: Kp = [%g %g], Ki = [%g %g] (Ki on log scale) | %d scenarios\n', lim.Kp, lim.Ki, nS);

%% ============================ 2. OPTIMISATION =============================
Res(nA).J = [];
for a = 1:nA
    Res(a).z = zeros(runs,2); Res(a).x = zeros(runs,2); Res(a).J = zeros(1,runs);
    Res(a).time = zeros(1,runs); Res(a).hist = zeros(runs, iter+1);
    for r = 1:runs
        rng(cfg.seed + 100*a + r);  tr = tic;
        switch a
            case 1, [z,J,h] = runPSO(funJ, pop, iter, 2);
            case 2, [z,J,h] = runGA (funJ, pop, iter, 2);
            case 3, [z,J,h] = runGWO(funJ, pop, iter, 2);
            case 4
                [Zf,Ff,Jf,h] = runNSGA2(funMO, pop, iter, 2, 3, cfg.big);
                ok = all(Ff < cfg.big, 2);
                if any(ok), Zf = Zf(ok,:); Ff = Ff(ok,:); Jf = Jf(ok); end
                [~,iu] = unique(round(Zf,6), 'rows');
                Zf = Zf(iu,:); Ff = Ff(iu,:); Jf = Jf(iu);
                kk = kneeIndex(Ff);  z = Zf(kk,:);  J = Jf(kk);
                Res(a).front{r} = struct('Z',Zf,'F',Ff,'k',kk);
        end
        x = z2x(z, lim);
        Res(a).z(r,:) = z; Res(a).x(r,:) = x; Res(a).J(r) = J;
        Res(a).hist(r,:) = h; Res(a).time(r) = toc(tr);
        fprintf('%-13s run %2d/%d | J = %.4f | Kp = %.5f | Ki = %.3f | %.0f s | elapsed %.1f min\n', ...
            algos{a}, r, runs, J, x(1), x(2), Res(a).time(r), toc(tAll)/60);
    end
end

%% ============================ 3. STATISTICS ===============================
bestRun = zeros(1,nA); bKp = zeros(1,nA); bKi = zeros(1,nA);
for a = 1:nA
    [~,bestRun(a)] = min(Res(a).J);
    bKp(a) = Res(a).x(bestRun(a),1);  bKi(a) = Res(a).x(bestRun(a),2);
end
% hypervolume of the NSGA-II fronts (common normalisation, Monte Carlo estimate)
Fp = []; for r = 1:runs, Fp = [Fp; Res(4).front{r}.F]; end %#ok<AGROW>
fmin = min(Fp,[],1); span = max(Fp,[],1) - fmin; span(span==0) = 1;
rng(1); U = rand(30000,3); hv = zeros(1,runs);
for r = 1:runs
    Fn = (Res(4).front{r}.F - fmin)./(1.1*span);  dm = false(size(U,1),1);
    for i = 1:size(Fn,1), dm = dm | all(U >= Fn(i,:), 2); end
    hv(r) = mean(dm);
end
pMat = nan(nA);
for i = 1:nA, for j = 1:nA, if i ~= j, pMat(i,j) = rankSumP(Res(i).J, Res(j).J); end, end, end
St = table(string(algos(:)), arrayfun(@(a) min(Res(a).J),1:nA)', arrayfun(@(a) median(Res(a).J),1:nA)', ...
    arrayfun(@(a) mean(Res(a).J),1:nA)', arrayfun(@(a) std(Res(a).J),1:nA)', ...
    arrayfun(@(a) max(Res(a).J),1:nA)', arrayfun(@(a) mean(Res(a).time),1:nA)', bKp(:), bKi(:), ...
    'VariableNames', {'Algorithm','J_best','J_median','J_mean','J_std','J_worst','Time_s_per_run','Kp_best','Ki_best'});
fprintf('\n--- Optimiser statistics (%d runs, final worst-case cost J) ---\n', runs); disp(St)
fprintf('NSGA-II hypervolume: mean %.4f, std %.4f (NSGA-II J is the knee point, not a minimiser of J)\n', mean(hv), std(hv));
fprintf('Rank-sum p-values (rows vs columns):\n'); disp(array2table(pMat,'VariableNames',strrep(algos,'-','_'),'RowNames',strrep(algos,'-','_')))
save('PI_Opt_Stage1.mat','Res','St','cfg','lim','hv','pMat');

%% ============================ 4. ALL DESIGNS: ANALYTIC (all corners) ======
Kp_all = [Kp_ref, bKp];  Ki_all = [Ki_ref, bKi];
MA = cell(nD,1); Jan = zeros(nD,1); wOS = zeros(nD,1); wTs = wOS; wTr = wOS; wIAE = wOS; mPM = wOS;
for d = 1:nD
    Mx = evalMx(Kp_all(d), Ki_all(d), ctx, idxAll, 'analytic');  MA{d} = Mx;
    Jan(d) = aggregate(Mx, cfg);
    wIAE(d) = max(Mx(:,1))*1e3; wOS(d) = max(Mx(:,2)); wTs(d) = max(Mx(:,3))*1e3;
    wTr(d) = max(Mx(:,4))*1e3;  mPM(d) = min(Mx(:,5));
end
T1 = table(string(names(:)), Kp_all(:), Ki_all(:), wOS, wTs, wTr, wIAE, mPM, Jan, 'VariableNames', ...
    {'Design','Kp','Ki','WorstOS_pct','WorstTs_ms','WorstTr_ms','WorstIAE_ms','MinPM_deg','J_worstcase'});
fprintf('\n--- Worst case over all %d corner scenarios (analytic) ---\n', nS); disp(T1)
JS = zeros(nD,nS); for d = 1:nD, JS(d,:) = scenJs(MA{d}, cfg)'; end
[~,iw] = max(mean(JS,1));

%% ============================ 4b. MONTE CARLO (held-out, analytic) ========
rng(cfg.seed + 777);
scenMC = sampleScen(cfg.nMC, cfg);  ctxMC = buildCtx(scenMC, cfg, lim);
MM = cell(nD,1); Unst = zeros(nD,1); Succ = zeros(nD,1);
osM = zeros(nD,1); os95 = osM; osMax = osM; ts95 = osM; pmMin = osM;
for d = 1:nD
    M = evalMx(Kp_all(d), Ki_all(d), ctxMC, 1:cfg.nMC, 'analytic');  MM{d} = M;
    un = any(isnan(M),2);  okk = ~un;
    Unst(d) = 100*mean(un);
    Succ(d) = 100*mean(okk & M(:,2) <= cfg.spec(1) & M(:,3) <= cfg.spec(2) & M(:,5) >= cfg.spec(3));
    osM(d) = mean(M(okk,2));  os95(d) = pctl(M(okk,2),95);  osMax(d) = max(M(okk,2));
    ts95(d) = pctl(M(okk,3),95)*1e3;  pmMin(d) = min(M(okk,5));
end
TMC = table(string(names(:)), Unst, Succ, osM, os95, osMax, ts95, pmMin, 'VariableNames', ...
    {'Design','Unstable_pct','SpecMet_pct','OS_mean_pct','OS_p95_pct','OS_max_pct','Ts_p95_ms','PM_min_deg'});
fprintf('\n--- Monte Carlo (%d random held-out scenarios). Spec: OS<=%g%%, Ts<=%g ms, PM>=%g deg ---\n', ...
    cfg.nMC, cfg.spec(1), cfg.spec(2)*1e3, cfg.spec(3)); disp(TMC)

%% ============================ 4c. DISTURBANCE REJECTION (analytic) ========
DM = cell(nD,1);
for d = 1:nD
    DM{d} = zeros(nS,4);
    for k = 1:nS, DM{d}(k,:) = distMetrics(Kp_all(d), Ki_all(d), ctx, k); end
end
dn = @(c) cellfun(@(m) m(idxNom,c), DM);   dw = @(c) cellfun(@(m) max(m(:,c)), DM);
TD = table(string(names(:)), dn(1), dw(1), dw(2), dn(3), dw(3), dw(4), 'VariableNames', ...
    {'Design','LinePeak_nom_pct','LinePeak_worst_pct','LineRecov_worst_ms', ...
     'LoadPeak_nom_pct','LoadPeak_worst_pct','LoadRecov_worst_ms'});
fprintf('\n--- Disturbance rejection: Vin %+g%% step and +%g%% load-current step (peak in %% of Vref) ---\n', ...
    100*cfg.dist.dVin, 100*cfg.dist.dIo); disp(TD)
save('PI_Opt_Stage2.mat');

%% ============================ 5. SIMULINK VALIDATION ======================
simDone = false;
if cfg.doSimValidation
    [~,ordC] = sort(mean(JS,1),'descend');  ordC(ordC==idxNom) = [];
    nCorner = min(numel(ordC), ceil(0.6*(nVal-1)));  nRand = max(0, nVal-1-nCorner);
    rng(cfg.seed + 4242);
    scenRand = sampleScen(max(nRand,1), cfg);  scenRand = scenRand(1:nRand);
    scenV = [scen(idxNom), scen(ordC(1:nCorner)), scenRand];     % nominal, worst corners, random held-out
    nV = numel(scenV);  ctxV = buildCtx(scenV, cfg, lim);
    tU = (0:20e-6:cfg.Tstop)';
    simV = cell(nD,1); simD = cell(nD,1); MS = cell(nD,1); MAv = cell(nD,1);
    cnt = 0;
    for d = 1:nD
        MAv{d} = evalMx(Kp_all(d), Ki_all(d), ctxV, 1:nV, 'analytic');
        simV{d} = nan(numel(tU),nV); simD{d} = simV{d}; Mx = nan(nV,5);
        for q = 1:nV
            cnt = cnt + 1;
            fprintf('Simulink %3d/%d | %-14s | %s | elapsed %.1f min\n', cnt, nD*nV, names{d}, scenV(q).name, toc(tAll)/60);
            [~,v,du,ok] = runSim(Kp_all(d), Ki_all(d), scenV(q), ctxV, cfg.Tstop);
            if ~ok, continue; end
            simV{d}(:,q) = v;  simD{d}(:,q) = du;
            Mx(q,:) = simRow(tU, v, cfg.Vref, MAv{d}(q,5));
        end
        MS{d} = Mx;
    end
    Jsim = zeros(nD,1); sOS = Jsim; sTs = Jsim; sTr = Jsim; sIAE = Jsim; sSSE = Jsim; sDuty = Jsim;
    for d = 1:nD
        Mx = MS{d};  Jsim(d) = aggregate(Mx, cfg);
        sOS(d) = max(Mx(:,2)); sTs(d) = max(Mx(:,3))*1e3; sTr(d) = max(Mx(:,4))*1e3;
        sIAE(d) = max(Mx(:,1))*1e3;
        sSSE(d) = max(abs(cfg.Vref - mean(simV{d}(tU > 0.9*cfg.Tstop,:),1)));
        sDuty(d) = max(simD{d}(:));
    end
    T2 = table(string(names(:)), Kp_all(:), Ki_all(:), sOS, sTs, sTr, sIAE, sSSE, sDuty, Jsim, 'VariableNames', ...
        {'Design','Kp','Ki','WorstOS_pct','WorstTs_ms','WorstTr_ms','WorstIAE_ms','WorstSteadyErr_V','MaxDuty','J_sim'});
    fprintf('\n--- Simulink validation (%d scenarios: nominal + %d worst corners + %d random held-out) ---\n', nV, nCorner, nRand);
    disp(T2);  simDone = true;
end

%% ============================ 6. RANKING + TABLES =========================
if simDone, Jrank = Jsim; src = 'Simulink'; else, Jrank = Jan; src = 'analytic'; end
[~,ord] = sort(Jrank);
Rk = table((1:nD)', string(names(ord)'), Kp_all(ord)', Ki_all(ord)', Jrank(ord), Succ(ord), 'VariableNames', ...
    {'Rank','Design','Kp','Ki','J','MC_SpecMet_pct'});
fprintf('\n--- Final ranking (%s worst-case cost) ---\n', src); disp(Rk)
Kp = Kp_all(ord(1));  Ki = Ki_all(ord(1));
fprintf('>>> RECOMMENDED: %s  ->  Kp = %.6f,  Ki = %.4f\n', names{ord(1)}, Kp, Ki);

if ~exist('Paper_Tables','dir'), mkdir('Paper_Tables'); end
TT = {T1,'Table_Analytic_WorstCase','Worst-case performance over the corner scenarios (averaged model).','tab:analytic'; ...
      St,'Table_Optimiser_Statistics','Optimiser statistics over independent runs.','tab:stats'; ...
      TMC,'Table_MonteCarlo','Monte Carlo robustness on held-out random scenarios.','tab:mc'; ...
      TD,'Table_Disturbance','Disturbance rejection (peak deviation in percent of Vref).','tab:dist'; ...
      Rk,'Table_Ranking','Final ranking of all designs.','tab:rank'};
if simDone, TT = [TT; {T2,'Table_Simulink','Switching-model (Simulink) validation.','tab:sim'}]; end
for i = 1:size(TT,1)
    writetable(TT{i,1}, fullfile('Paper_Tables',[TT{i,2} '.csv']));
    table2tex(TT{i,1}, fullfile('Paper_Tables',[TT{i,2} '.tex']), TT{i,3}, TT{i,4});
end
save('PI_Opt_Results.mat');

%% ============================ 7. PUBLICATION FIGURES ======================
colR = [0.60 0.60 0.60; 0.40 0.40 0.40; 0.15 0.15 0.15; 0.85 0.33 0.10; 0.47 0.67 0.19];
colA = [0.00 0.45 0.74; 0.93 0.69 0.13; 0.49 0.18 0.56; 0.64 0.08 0.18];
cols = [colR; colA];  lsty = [repmat({'--'},1,nRef) repmat({'-'},1,nA)];
tms = ctx.t*1e3;  short = {'PSO','GA','GWO','NSGA-II'};

% Fig 1 convergence
f = newFig('Fig1',8.8,6.5); hold on; x = 0:iter;
for a = 1:nA
    m = mean(Res(a).hist,1); s = std(Res(a).hist,0,1);
    fill([x fliplr(x)], [m+s fliplr(m-s)], colA(a,:), 'FaceAlpha',0.15, 'EdgeColor','none', 'HandleVisibility','off');
    plot(x, m, 'Color', colA(a,:));
end
xlabel('Iteration'); ylabel('Worst-case cost J'); legend(algos,'Location','northeast'); saveFig(f,'Fig1_Convergence');

% Fig 2 final-cost distribution + rank-sum p-values
f = newFig('Fig2',17.5,6.5);
ax = subplot(1,2,1); drawBox(ax, arrayfun(@(a) Res(a).J, 1:nA, 'UniformOutput', false), short, colA);
ylabel('Final worst-case cost J'); title('(a) Distribution over runs','FontWeight','normal');
ax = subplot(1,2,2); imagesc(ax, pMat, [0 0.2]); colormap(ax, flipud(gray)); colorbar(ax);
set(ax,'XTick',1:nA,'XTickLabel',short,'YTick',1:nA,'YTickLabel',short,'XGrid','off','YGrid','off');
for i = 1:nA, for j = 1:nA, if i ~= j, text(j,i,sprintf('%.3f',pMat(i,j)),'HorizontalAlignment','center'); end, end, end
title('(b) Rank-sum p-values','FontWeight','normal'); saveFig(f,'Fig2_Statistics');

% Fig 3 Kp-Ki plane
f = newFig('Fig3',8.8,7); hold on;
for a = 1:nA, plot(Res(a).x(:,1), Res(a).x(:,2), 'o', 'Color', colA(a,:), 'MarkerFaceColor', colA(a,:), 'MarkerSize', 4); end
plot(Kp_ref, Ki_ref, 's', 'MarkerEdgeColor','k', 'MarkerFaceColor','y', 'MarkerSize', 7);
text(Kp_ref + 0.0006, Ki_ref, refNames, 'FontSize', 7);
set(gca,'YScale','log'); xlim(lim.Kp); ylim(lim.Ki); xlabel('K_p'); ylabel('K_i');
legend([short {'Reference'}],'Location','southeast'); saveFig(f,'Fig3_KpKi_Plane');

% Fig 4 NSGA-II Pareto front
[~,rb] = max(hv);  fr = Res(4).front{rb};  Fk = fr.F(fr.k,:);
f = newFig('Fig4',17.5,6);
subplot(1,3,1); scatter3(fr.F(:,1), fr.F(:,2), fr.F(:,3), 18, 'filled'); hold on
scatter3(Fk(1),Fk(2),Fk(3),90,'p','filled','MarkerFaceColor',colA(4,:)); view(135,25);
xlabel('IAE (norm.)'); ylabel('T_s (norm.)'); zlabel('OS (norm.)'); title('(a) Pareto front','FontWeight','normal');
subplot(1,3,2); plot(fr.F(:,2), fr.F(:,3), 'o','MarkerSize',4); hold on
plot(Fk(2),Fk(3),'p','MarkerSize',11,'MarkerFaceColor',colA(4,:),'MarkerEdgeColor','k');
xlabel('T_s (norm.)'); ylabel('OS (norm.)'); title('(b) Projection, knee marked','FontWeight','normal');
ax = subplot(1,3,3); drawBox(ax, {hv}, {'NSGA-II'}, colA(4,:)); ylabel('Hypervolume'); title('(c) Over runs','FontWeight','normal');
saveFig(f,'Fig4_Pareto_Knee');

% Fig 5 step responses (nominal and most severe corner)
f = newFig('Fig5',17.5,6.5);
for p = 1:2
    subplot(1,2,p); hold on; kk = idxNom; if p == 2, kk = iw; end
    for d = 1:nD, plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,kk), 'Color',cols(d,:), 'LineStyle',lsty{d}); end
    ylim([0 1.5]); xlabel('Time (ms)'); ylabel('V_{out}/V_{ref}');
    title(sprintf('(%c) %s', 'a'+p-1, strrep(scen(kk).name,'%','\%')),'FontWeight','normal','Interpreter','none');
end
legend(names,'Location','southeast','FontSize',6); saveFig(f,'Fig5_Step_Analytic');

% Fig 6 robustness family of the optimised designs
f = newFig('Fig6',17.5,11);
for a = 1:nA
    d = nRef + a; subplot(2,2,a); hold on
    for k = 1:nS, plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,k), 'Color',[0.75 0.75 0.75], 'LineWidth',0.6); end
    plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,idxNom), 'Color',colA(a,:), 'LineWidth',2);
    plot(tms([1 end]),[1 1],'k:'); ylim([0 1.6]); xlabel('Time (ms)'); ylabel('V_{out}/V_{ref}');
    title(sprintf('(%c) %s: K_p=%.4f, K_i=%.1f', 'a'+a-1, short{a}, Kp_all(d), Ki_all(d)),'FontWeight','normal');
end
saveFig(f,'Fig6_Robustness_Family');

% Fig 7 Monte Carlo robustness   (FIXED: legend now on the CDF curves, not the bar chart)
f = newFig('Fig7',17.5,6);
subplot(1,3,1); hold on
for d = 1:nD, v = sort(MM{d}(~isnan(MM{d}(:,2)),2)); plot(v,(1:numel(v))/cfg.nMC,'Color',cols(d,:),'LineStyle',lsty{d}); end
xlabel('Overshoot (%)'); ylabel('Cumulative probability'); title('(a) Overshoot CDF','FontWeight','normal'); xlim([0 40]);
legend(names,'Location','southeast','FontSize',5);
subplot(1,3,2); hold on
for d = 1:nD, v = sort(MM{d}(~isnan(MM{d}(:,3)),3))*1e3; plot(v,(1:numel(v))/cfg.nMC,'Color',cols(d,:),'LineStyle',lsty{d}); end
xlabel('Settling time (ms)'); title('(b) Settling-time CDF','FontWeight','normal'); xlim([0 60]);
subplot(1,3,3); b = bar(Succ); b.FaceColor = 'flat'; b.CData = cols;
set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',60,'FontSize',6); ylabel('Specification met (%)');
title('(c) Success rate','FontWeight','normal');
saveFig(f,'Fig7_MonteCarlo');

% Fig 8 disturbance rejection (nominal)
f = newFig('Fig8',17.5,6);
for p = 1:2
    subplot(1,2,p); hold on
    for d = 1:nD
        y = distResp(Kp_all(d), Ki_all(d), ctx, idxNom, p);
        plot(tms, y/cfg.Vref*100, 'Color', cols(d,:), 'LineStyle', lsty{d});
    end
    xlabel('Time (ms)'); ylabel('\DeltaV_{out} (% of V_{ref})');
end
subplot(1,2,1); title(sprintf('(a) Line step %+g%% V_{in}',100*cfg.dist.dVin),'FontWeight','normal');
subplot(1,2,2); title(sprintf('(b) Load-current step +%g%%',100*cfg.dist.dIo),'FontWeight','normal');
legend(names,'Location','southeast','FontSize',6); saveFig(f,'Fig8_Disturbance');

% Fig 9 worst-case metric summary
f = newFig('Fig9',17.5,9);
Mset = {wOS,'Worst overshoot (%)'; wTs,'Worst settling time (ms)'; mPM,'Min. phase margin (deg)'; Succ,'MC spec met (%)'};
for i = 1:4
    subplot(2,2,i); b = bar(Mset{i,1}); b.FaceColor = 'flat'; b.CData = cols;
    set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',35,'FontSize',7); ylabel(Mset{i,2});
    title(sprintf('(%c)', 'a'+i-1),'FontWeight','normal');
end
saveFig(f,'Fig9_Metric_Summary');

if simDone
    qW = min(2, nV);
    % Fig 10 Simulink validation
    f = newFig('Fig10',17.5,10);
    sigName = {'V_{out}','V_{out}','Duty','Duty'};
    scnName = {'nominal','worst corner','nominal','worst corner'};
    for p = 1:4
        subplot(2,2,p); hold on
        for d = 1:nD
            switch p
                case 1, y = simV{d}(:,1);  case 2, y = simV{d}(:,qW);
                case 3, y = simD{d}(:,1);  case 4, y = simD{d}(:,qW);
            end
            plot(tU*1e3, y, 'Color', cols(d,:), 'LineStyle', lsty{d});
        end
        xlabel('Time (ms)');
        if p <= 2, plot(tU([1 end])*1e3,[cfg.Vref cfg.Vref],'k:'); ylabel('V_{out} (V)'); else, ylabel('Duty'); end
        title(sprintf('(%c) %s: %s', 'a'+p-1, sigName{p}, scnName{p}),'FontWeight','normal');
    end
    subplot(2,2,1); legend(names,'Location','southeast','FontSize',6); saveFig(f,'Fig10_Simulink_Validation');

    % Fig 11 Simulink worst-case metrics
    f = newFig('Fig11',17.5,6);
    Ms = {sOS,'Worst overshoot (%)'; sTs,'Worst settling time (ms)'; Jsim,'Worst-case cost J (Simulink)'};
    for i = 1:3
        subplot(1,3,i); b = bar(Ms{i,1}); b.FaceColor = 'flat'; b.CData = cols;
        set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',60,'FontSize',6); ylabel(Ms{i,2});
        title(sprintf('(%c)', 'a'+i-1),'FontWeight','normal');
    end
    saveFig(f,'Fig11_Simulink_Metrics');

    % Fig 12 averaged model vs switching model (nominal)
    f = newFig('Fig12',17.5,13);
    for d = 1:nD
        subplot(3,3,d); hold on
        plot(tU*1e3, simV{d}(:,1)/cfg.Vref, 'Color', cols(d,:), 'LineWidth', 1.6);
        plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,idxNom), 'k--');
        xlim([0 cfg.Tstop*1e3]); ylim([0 1.5]); title(names{d},'FontWeight','normal','FontSize',8);
        if d == 1, legend('Simulink','Averaged model','Location','southeast','FontSize',6); end
        if d >= 7, xlabel('Time (ms)'); end
    end
    saveFig(f,'Fig12_Model_Mismatch');
end

%% ============================ 8. FIGURE LIST (names + captions) ===========
FL = {'Fig1_Convergence','Convergence of the mean worst-case cost J (shaded: +/-1 std over independent runs) for PSO, GA, GWO and NSGA-II knee.';
      'Fig2_Statistics','(a) Distribution of the final cost J over independent runs. (b) Wilcoxon rank-sum p-values between algorithms.';
      'Fig3_KpKi_Plane','Optimised (Kp, Ki) of every run in the parameter plane, with the reference designs (pidtune and PID Tuner app).';
      'Fig4_Pareto_Knee','NSGA-II: (a) Pareto front of IAE, settling time and overshoot, (b) projection with the knee point, (c) hypervolume over runs.';
      'Fig5_Step_Analytic','Closed-loop step responses of all designs (averaged model): (a) nominal case, (b) most severe corner scenario.';
      'Fig6_Robustness_Family','Step responses of the four optimised designs over all corner scenarios (grey) and the nominal case (colour).';
      'Fig7_MonteCarlo','Monte Carlo robustness on held-out random scenarios: (a) overshoot CDF, (b) settling-time CDF, (c) success rate against the specification.';
      'Fig8_Disturbance','Disturbance rejection (nominal): (a) -20% input-voltage step, (b) +20% load-current step, peak deviation in percent of Vref.';
      'Fig9_Metric_Summary','Worst-case overshoot, worst-case settling time, minimum phase margin and Monte Carlo success rate for all designs.'};
if simDone
    FL = [FL; {'Fig10_Simulink_Validation','Switching-model (Simulink) validation: output voltage and duty cycle for the nominal case and the worst corner.'; ...
               'Fig11_Simulink_Metrics','Simulink worst-case overshoot, worst-case settling time and worst-case cost J for all designs.'; ...
               'Fig12_Model_Mismatch','Averaged (analytic) model versus the Simulink switching model, nominal case, for every design.'}];
end
FigList = cell2table(FL, 'VariableNames', {'File','Caption'});
writetable(FigList, fullfile('Paper_Figures','Figure_List.csv'));
fid = fopen(fullfile('Paper_Figures','Figure_List.txt'),'w');
for i = 1:size(FL,1), fprintf(fid,'%s  (.pdf / .png / .fig)\n    %s\n\n', FL{i,1}, FL{i,2}); end
fclose(fid);
fprintf('\n--- Figure files (Paper_Figures/, each as PDF + PNG + FIG) ---\n');
for i = 1:size(FL,1), fprintf('  %s\n', FL{i,1}); end

save('PI_Opt_Results.mat');
fprintf('\nTotal time: %.1f min (budget %d min). Figures: Paper_Figures/, tables: Paper_Tables/\n', toc(tAll)/60, cfg.budgetMin);
diary off;

% pack everything into one zip
try
    zip('Final_Results.zip', {'Paper_Figures','Paper_Tables','PI_Opt_Stage1.mat','PI_Opt_Stage2.mat','PI_Opt_Results.mat','Run_Log.txt'});
    disp('Saved: Final_Results.zip');
catch ME
    warning('Zip failed: %s', ME.message);
end

%% ====================================================================
%%                          LOCAL FUNCTIONS
%% ====================================================================
function s = makeScen(f, nom)
s.name = sprintf('Vin%.0f%% L%.0f%% C%.0f%% R%.0f%%', 100*f);
s.Vin = nom.Vin*f(1); s.L = nom.L*f(2); s.C = nom.C*f(3); s.R = nom.R*f(4); s.fac = f;
end

function scen = buildScenarios(nom, cfg)
v = cfg.var;  lv = {v.Vin, v.L, v.C, v.R};
if strcmp(cfg.scenarioSet,'full'), lv = cellfun(@(x) [x(1) 1 x(2)], lv, 'UniformOutput', false); end
[g1,g2,g3,g4] = ndgrid(lv{:});  fac = [g1(:) g2(:) g3(:) g4(:)];
if ~strcmp(cfg.scenarioSet,'full'), fac = [ones(1,4); fac]; end
scen = struct('name',{},'Vin',{},'L',{},'C',{},'R',{},'fac',{});
for k = 1:size(fac,1), scen(k) = makeScen(fac(k,:), nom); end
end

function sc = sampleScen(n, cfg)
v = cfg.var;  sc = struct('name',{},'Vin',{},'L',{},'C',{},'R',{},'fac',{});
for k = 1:n
    f = [v.Vin(1)+diff(v.Vin)*rand, v.L(1)+diff(v.L)*rand, v.C(1)+diff(v.C)*rand, v.R(1)+diff(v.R)*rand];
    sc(k) = makeScen(f, cfg.nom);
end
end

function ctx = buildCtx(scen, cfg, lim)
n = numel(scen);  ctx.Gn = cell(1,n); ctx.Gd = cell(1,n); ctx.Nv = cell(1,n); ctx.Nz = cell(1,n);
for k = 1:n
    s = scen(k);  Ds = 1 - s.Vin/cfg.Vref;                % duty needed to reach Vref
    a  = s.Vin/(1-Ds)^2;           b  = s.L/(s.R*(1-Ds)^2);
    c  = s.L*s.C/(1-Ds)^2;         bL = (s.L/s.R + cfg.nom.RL*s.C)/(1-Ds)^2;
    dc = 1 + cfg.nom.RL/(s.R*(1-Ds)^2);
    ctx.Gn{k} = [-a*b a];  ctx.Gd{k} = [c bL dc];          % duty -> Vout
    ctx.Nv{k} = 1/(1-Ds);  ctx.Nz{k} = [s.L/(1-Ds)^2 0];   % line (Vin->Vout) and output impedance numerators
end
ctx.cfg = cfg; ctx.scen = scen; ctx.lim = lim; ctx.t = linspace(0, cfg.Tend, cfg.N)';
end

function x = z2x(z, lim)      % z in [0,1]^2 -> [Kp Ki]  (Ki on log scale)
x(1) = lim.Kp(1) + z(1)*(lim.Kp(2)-lim.Kp(1));
x(2) = exp(log(lim.Ki(1)) + z(2)*(log(lim.Ki(2))-log(lim.Ki(1))));
end

function J = objSO(z, ctx, idx, model)
x = z2x(z, ctx.lim);  J = aggregate(evalMx(x(1), x(2), ctx, idx, model), ctx.cfg);
end

function o = objMO(z, ctx, idx, model)
x = z2x(z, ctx.lim);  [J,F] = aggregate(evalMx(x(1), x(2), ctx, idx, model), ctx.cfg);  o = [F J];
end

function Mx = evalMx(Kp, Ki, ctx, idx, model)
% rows = scenarios, columns = [IAE OS% Ts Tr PM]
n = numel(idx);  Mx = nan(n,5);
for q = 1:n
    k = idx(q);
    numL = conv([Kp Ki], ctx.Gn{k});  denL = conv([1 0], ctx.Gd{k});
    denT = denL + [zeros(1, numel(denL)-numel(numL)), numL];
    if any(real(roots(denT)) >= 0), continue; end
    pm = loopPM(numL, denL);
    if strcmp(model,'simulink')
        [tU,v,~,ok] = runSim(Kp, Ki, ctx.scen(k), ctx, ctx.cfg.Tstop);
        if ~ok, continue; end
        Mx(q,:) = simRow(tU, v, ctx.cfg.Vref, pm);
    else
        y = step(tf(numL, denT), ctx.t);
        [IAE,OS,Ts,Tr] = respMetrics(ctx.t, y);
        Mx(q,:) = [IAE OS Ts Tr pm];
    end
end
end

function pm = loopPM(numL, denL)
try, [~,pm] = margin(tf(numL, denL)); catch, pm = NaN; end
if ~isfinite(pm), pm = 90; end
end

function y = stepAna(Kp, Ki, ctx, k)
numL = conv([Kp Ki], ctx.Gn{k});  denL = conv([1 0], ctx.Gd{k});
denT = denL + [zeros(1, numel(denL)-numel(numL)), numL];
y = step(tf(numL, denT), ctx.t);
end

function y = distResp(Kp, Ki, ctx, k, kind)
% closed-loop output deviation [V]: kind 1 = line step, kind 2 = load-current step
cfg = ctx.cfg;  s = ctx.scen(k);
numL = conv([Kp Ki], ctx.Gn{k});  denL = conv([1 0], ctx.Gd{k});
denT = denL + [zeros(1, numel(denL)-numel(numL)), numL];
if kind == 1
    y = step(tf(ctx.Nv{k}*[1 0], denT), ctx.t)*(cfg.dist.dVin*s.Vin);
else
    y = -step(tf(conv(ctx.Nz{k},[1 0]), denT), ctx.t)*(cfg.dist.dIo*cfg.Vref/s.R);
end
end

function m = distMetrics(Kp, Ki, ctx, k)
numL = conv([Kp Ki], ctx.Gn{k});  denL = conv([1 0], ctx.Gd{k});
denT = denL + [zeros(1, numel(denL)-numel(numL)), numL];
if any(real(roots(denT)) >= 0), m = nan(1,4); return; end
Vref = ctx.cfg.Vref;  m = zeros(1,4);
for kind = 1:2
    y = distResp(Kp, Ki, ctx, k, kind);
    m(2*kind-1) = max(abs(y))/Vref*100;
    o = find(abs(y) > 0.02*Vref, 1, 'last');
    if isempty(o), m(2*kind) = 0; else, m(2*kind) = ctx.t(min(o+1,numel(ctx.t)))*1e3; end
end
end

function [IAE,OS,Ts,Tr] = respMetrics(t, y)    % y = normalised unit-step response
e = abs(1 - y);  IAE = trapz(t, e);
OS = max(0, max(y) - 1)*100;
o = find(e > 0.02, 1, 'last');
if isempty(o), Ts = 0; elseif o == numel(t), Ts = t(end); else, Ts = t(o+1); end
i1 = find(y >= 0.1, 1, 'first');  i2 = find(y >= 0.9, 1, 'first');
if isempty(i2), Tr = t(end); elseif isempty(i1), Tr = t(i2); else, Tr = t(i2) - t(i1); end
end

function Js = scenJs(Mx, cfg)
Js = cfg.w(1)*Mx(:,1)/cfg.ref(1) + cfg.w(2)*Mx(:,3)/cfg.ref(2) + cfg.w(3)*Mx(:,2)/cfg.ref(3);
Js(any(isnan(Mx),2)) = cfg.big;
end

function [J,F] = aggregate(Mx, cfg)
bad = any(isnan(Mx),2);
if any(bad), J = cfg.big*(1 + mean(bad)); F = cfg.big*ones(1,3); return; end
pen = 10*max(0, cfg.PMmin - min(Mx(:,5)))/cfg.PMmin;
Js  = scenJs(Mx, cfg);
J   = max(Js) + 0.2*mean(Js) + pen;
F   = [max(Mx(:,1))/cfg.ref(1), max(Mx(:,3))/cfg.ref(2), max(Mx(:,2))/cfg.ref(3)] + pen;
end

function [tU,v,du,ok] = runSim(Kp, Ki, sc, ctx, Tstop)
cfg = ctx.cfg;  ok = true;  tU = (0:20e-6:Tstop)';  v = [];  du = [];
try
    in = Simulink.SimulationInput(cfg.mdl);
    in = in.setModelParameter('StopTime', num2str(Tstop));
    in = in.setVariable('Kp', Kp);       in = in.setVariable('Ki', Ki);
    in = in.setVariable('Vin', sc.Vin);  in = in.setVariable('L', sc.L);
    in = in.setVariable('RL', cfg.nom.RL); in = in.setVariable('C', sc.C);
    in = in.setVariable('R', sc.R);
    in = in.setVariable('Vref', cfg.Vref); in = in.setVariable('Vout', cfg.Vref);
    in = in.setVariable('fs',  cfg.nom.fs);  in = in.setVariable('Fsw', cfg.nom.fs);
    in = in.setVariable('fsw', cfg.nom.fs);  in = in.setVariable('D',   cfg.nom.D);
    in = in.setVariable('Tstop', Tstop);
    out = sim(in);
    [t,vv] = getSig(out, 'Vout_sim');  [tD,dd] = getSig(out, 'Duty_sim');
    [t,iu] = unique(t);   vv = vv(iu);
    [tD,iu] = unique(tD); dd = dd(iu);
    v  = movmean(interp1(t,  vv, tU, 'linear', 'extrap'), 5);
    du = movmean(interp1(tD, dd, tU, 'linear', 'extrap'), 5);
catch ME
    warning('Simulink run failed: %s', ME.message);  ok = false;
end
end

function [t,v] = getSig(out, nameSig)
s = out.get(nameSig);
if isa(s,'timeseries'),  t = s.Time;   v = squeeze(s.Data);
elseif isstruct(s),      t = s.time;   v = squeeze(s.signals.values);
else,                    t = out.tout; v = squeeze(s);
end
t = t(:);  v = v(:);
end

function row = simRow(tU, v, Vref, pm)
v0 = v(1); if v0 > 0.5*Vref, v0 = 0; end
y = (v - v0)/(Vref - v0);
[IAE,OS,Ts,Tr] = respMetrics(tU, y);
row = [IAE OS Ts Tr pm];
end

%% ---------------------------- optimisers (z in [0,1]^D) -----------------
function f = evalPop(fun, Z)
f = zeros(size(Z,1),1);
for i = 1:size(Z,1), f(i) = fun(Z(i,:)); end
end

function [F,J] = evalPopMO(fun, Z, M)
N = size(Z,1); F = zeros(N,M); J = zeros(N,1);
for i = 1:N, o = fun(Z(i,:)); F(i,:) = o(1:M); J(i) = o(M+1); end
end

function [zb,fb,hist] = runPSO(fun, N, T, D)
Z = rand(N,D); Vl = zeros(N,D); f = evalPop(fun,Z);
P = Z; pf = f; [fb,i] = min(f); zb = Z(i,:);
hist = zeros(1,T+1); hist(1) = fb;
for it = 1:T
    w = 0.9 - 0.5*it/T;
    Vl = w*Vl + 1.6*rand(N,D).*(P-Z) + 1.6*rand(N,D).*(zb-Z);
    Vl = max(min(Vl,0.25),-0.25);
    Z  = min(max(Z+Vl,0),1);
    f  = evalPop(fun,Z);
    m = f < pf; P(m,:) = Z(m,:); pf(m) = f(m);
    [fm,i] = min(pf); if fm < fb, fb = fm; zb = P(i,:); end
    hist(it+1) = fb;
end
end

function [zb,fb,hist] = runGA(fun, N, T, D)
Z = rand(N,D); f = evalPop(fun,Z); nE = 2;
[fb,i] = min(f); zb = Z(i,:); hist = zeros(1,T+1); hist(1) = fb;
for it = 1:T
    [fs,o] = sort(f); E = Z(o(1:nE),:); fE = fs(1:nE);
    P = zeros(N,D);
    for i = 1:N
        a = randi(N,1,2);
        if f(a(1)) <= f(a(2)), P(i,:) = Z(a(1),:); else, P(i,:) = Z(a(2),:); end
    end
    Cc = P;
    for i = 1:2:N-1
        if rand < 0.9, [Cc(i,:),Cc(i+1,:)] = sbx(P(i,:),P(i+1,:),15); end
    end
    Cc = polyMut(Cc, 0.3, 20);
    fc = evalPop(fun,Cc);
    [~,w] = sort(fc,'descend'); Cc(w(1:nE),:) = E; fc(w(1:nE)) = fE;
    Z = Cc; f = fc;
    [fm,i] = min(f); if fm < fb, fb = fm; zb = Z(i,:); end
    hist(it+1) = fb;
end
end

function [zb,fb,hist] = runGWO(fun, N, T, D)
Z = rand(N,D); f = evalPop(fun,Z);
[f,o] = sort(f); Z = Z(o,:);
Ld = Z(1:3,:); fL = f(1:3);
hist = zeros(1,T+1); hist(1) = fL(1);
for it = 1:T
    a = 2*(1 - it/T);  Zn = zeros(N,D);
    for k = 1:3
        A = 2*a*rand(N,D) - a;  Cc = 2*rand(N,D);
        Zn = Zn + (Ld(k,:) - A.*abs(Cc.*Ld(k,:) - Z))/3;
    end
    Zn = min(max(Zn,0),1);
    fn = evalPop(fun,Zn);
    pool = [Ld; Zn]; fp = [fL(:); fn];
    [fp,o] = sort(fp); pool = pool(o,:);
    Ld = pool(1:3,:); fL = fp(1:3); Z = Zn;
    hist(it+1) = fL(1);
end
zb = Ld(1,:); fb = fL(1);
end

function [Zf,Ff,Jf,hist] = runNSGA2(funMO, N, T, D, M, big)
% hist = scalar cost J of the current knee point at every generation
Z = rand(N,D); [F,Jv] = evalPopMO(funMO,Z,M);
[rk,cd] = rankCrowd(F);
hist = zeros(1,T+1); hist(1) = kneeJ(F,Jv,rk,big);
for it = 1:T
    P = zeros(N,D);
    for i = 1:N
        a = randi(N,1,2);
        if rk(a(1)) < rk(a(2)) || (rk(a(1)) == rk(a(2)) && cd(a(1)) >= cd(a(2)))
            P(i,:) = Z(a(1),:);
        else
            P(i,:) = Z(a(2),:);
        end
    end
    Cc = P;
    for i = 1:2:N-1
        if rand < 0.9, [Cc(i,:),Cc(i+1,:)] = sbx(P(i,:),P(i+1,:),15); end
    end
    Cc = polyMut(Cc, 0.3, 20);
    [Fc,Jc] = evalPopMO(funMO,Cc,M);
    R = [Z;Cc]; FR = [F;Fc]; JR = [Jv;Jc];
    [rr,cc] = rankCrowd(FR);
    [~,o] = sortrows([rr, -cc]);  s = o(1:N);
    Z = R(s,:); F = FR(s,:); Jv = JR(s); rk = rr(s); cd = cc(s);
    hist(it+1) = kneeJ(F,Jv,rk,big);
end
f1 = rk == 1;  Zf = Z(f1,:);  Ff = F(f1,:);  Jf = Jv(f1);
end

function j = kneeJ(F, Jv, rk, big)
f1 = rk == 1;  Ff = F(f1,:);  Jf = Jv(f1);  ok = all(Ff < big, 2);
if any(ok), Ff = Ff(ok,:); Jf = Jf(ok); end
j = Jf(kneeIndex(Ff));
end

function [rank,crowd] = rankCrowd(F)
N = size(F,1); rank = zeros(N,1); crowd = zeros(N,1);
dom = false(N,N);
for i = 1:N, dom(i,:) = (all(F(i,:) <= F, 2) & any(F(i,:) < F, 2))'; end
rem = true(N,1); r = 0;
while any(rem)
    r = r + 1;
    cnt = sum(dom(rem,:),1)';
    front = rem & (cnt == 0);
    rank(front) = r;
    idx = find(front);
    crowd(idx) = crowdingDist(F(idx,:));
    rem(front) = false;
end
end

function d = crowdingDist(F)
[n,M] = size(F); d = zeros(n,1);
if n <= 2, d(:) = Inf; return; end
for m = 1:M
    [fs,o] = sort(F(:,m)); span = fs(end) - fs(1); if span == 0, span = 1; end
    d(o(1)) = Inf; d(o(end)) = Inf;
    d(o(2:end-1)) = d(o(2:end-1)) + (fs(3:end) - fs(1:end-2))/span;
end
end

function k = kneeIndex(F)
% knee = point with the largest distance below the hyperplane through the anchor points
N = size(F,1);
if N <= 2, [~,k] = min(sum(F,2)); return; end
M = size(F,2);
fmin = min(F,[],1); fmax = max(F,[],1);
Fn = (F - fmin)./max(fmax - fmin, eps);
A = zeros(M);
for i = 1:M, [~,j] = min(Fn(:,i)); A(i,:) = Fn(j,:); end
if rcond(A) > 1e-8
    n = A\ones(M,1);  d = (1 - Fn*n)/norm(n);
else
    d = -sqrt(sum(Fn.^2,2));
end
[~,k] = max(d);
end

function [c1,c2] = sbx(p1, p2, eta)
u = rand(size(p1)); beta = zeros(size(u)); m = u <= 0.5;
beta(m)  = (2*u(m)).^(1/(eta+1));
beta(~m) = (1./(2*(1-u(~m)))).^(1/(eta+1));
c1 = 0.5*((1+beta).*p1 + (1-beta).*p2);
c2 = 0.5*((1-beta).*p1 + (1+beta).*p2);
c1 = min(max(c1,0),1);  c2 = min(max(c2,0),1);
end

function X = polyMut(X, pm, eta)
[N,D] = size(X); u = rand(N,D); dl = zeros(N,D); a = u < 0.5;
dl(a)  = (2*u(a)).^(1/(eta+1)) - 1;
dl(~a) = 1 - (2*(1-u(~a))).^(1/(eta+1));
X = min(max(X + (rand(N,D) < pm).*dl, 0), 1);
end

%% ---------------------------- statistics + output helpers ---------------
function q = pctl(v, p)
v = sort(v(~isnan(v))); n = numel(v);
if n == 0, q = NaN; return; end
pos = 1 + (n-1)*p/100; lo = floor(pos); hi = ceil(pos);
q = v(lo) + (pos-lo)*(v(hi)-v(lo));
end

function p = rankSumP(x, y)
% two-sided Wilcoxon rank-sum (Mann-Whitney U), normal approximation with tie correction
x = x(:); y = y(:); n1 = numel(x); n2 = numel(y);
if n1 < 2 || n2 < 2, p = NaN; return; end
allv = [x; y]; N = numel(allv); [sv,ord] = sort(allv); r = zeros(N,1); i = 1;
while i <= N
    j = i; while j < N && sv(j+1) == sv(i), j = j + 1; end
    r(i:j) = (i+j)/2; i = j + 1;
end
rk = zeros(N,1); rk(ord) = r;
U = sum(rk(1:n1)) - n1*(n1+1)/2;
[~,~,ic] = unique(allv); t = accumarray(ic,1);
sg = sqrt(n1*n2/12*((N+1) - sum(t.^3 - t)/(N*(N-1))));
if sg == 0, p = 1; return; end
p = erfc(abs((U - n1*n2/2)/sg)/sqrt(2));
end

function drawBox(ax, data, labels, cols)
hold(ax,'on'); if size(cols,1) < numel(data), cols = repmat(cols(1,:), numel(data), 1); end
for i = 1:numel(data)
    v = sort(data{i}(:)); v = v(~isnan(v)); if isempty(v), continue; end
    q1 = pctl(v,25); q2 = pctl(v,50); q3 = pctl(v,75);
    patch(ax, [i-.25 i+.25 i+.25 i-.25], [q1 q1 q3 q3], cols(i,:), 'FaceAlpha',0.35, 'EdgeColor','k');
    plot(ax, [i-.25 i+.25], [q2 q2], 'k', 'LineWidth', 1.8);
    plot(ax, [i i], [min(v) q1], 'k-'); plot(ax, [i i], [q3 max(v)], 'k-');
    plot(ax, i + (rand(size(v))-.5)*0.25, v, 'k.', 'MarkerSize', 6);
end
set(ax,'XTick',1:numel(data),'XTickLabel',labels,'XLim',[0.4 numel(data)+0.6]);
end

function f = newFig(name, W, H)
f = figure('Name',name,'Units','centimeters','Position',[2 2 W H],'Color','w','PaperPositionMode','auto');
end

function saveFig(f, name)
if ~exist('Paper_Figures','dir'), mkdir('Paper_Figures'); end
p = fullfile('Paper_Figures', name);
try
    exportgraphics(f, [p '.pdf'], 'ContentType', 'vector');
    exportgraphics(f, [p '.png'], 'Resolution', 600);
catch
    set(f,'PaperPositionMode','auto'); print(f, p, '-dpdf'); print(f, p, '-dpng', '-r600');
end
try, savefig(f, [p '.fig']); catch, end
end

function table2tex(T, fname, cap, lab)
nc = width(T);  vn = T.Properties.VariableNames;
fid = fopen(fname,'w');
fprintf(fid,'\\begin{table}[t]\n\\centering\\caption{%s}\\label{%s}\n\\begin{tabular}{l%s}\n\\toprule\n', cap, lab, repmat('r',1,nc-1));
fprintf(fid,'%s \\\\\n\\midrule\n', strjoin(strrep(vn,'_','\_'), ' & '));
for i = 1:height(T)
    c = cell(1,nc);
    for j = 1:nc
        col = T.(vn{j});
        if isstring(col) || iscellstr(col), c{j} = strrep(char(col(i)),'_','\_');
        else, c{j} = sprintf('%.4g', col(i)); end
    end
    fprintf(fid,'%s \\\\\n', strjoin(c,' & '));
end
fprintf(fid,'\\bottomrule\n\\end{tabular}\n\\end{table}\n');
fclose(fid);
end