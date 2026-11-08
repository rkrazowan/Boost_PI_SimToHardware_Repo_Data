%% Run_HW_PI_Optimization.m
% PI tuning of the REAL Arduino boost converter: PSO | GA | GWO | NSGA-II knee
% Talks to the Arduino directly over serial (no Simulink -> fast and no pacing problems).
%
% HOW TO USE
%   1) Keep this file in any folder. Arduino sketch = your current sketch (no change needed).
%   2) Close the Arduino Serial Monitor. Set cfg.PORT below.
%   3) OPTIONAL 15-min check: cfg.quickTest = true.
%   4) FULL RUN : cfg.quickTest = false, cfg.budgetMin = 175 (set your own minutes).
%      If the run is interrupted: set cfg.resume = true and run again (continues).
%   5) Results:
%        HW_Figures/  Fig1..Fig10 as PDF + PNG (600 dpi) + FIG, Figure_List.csv/.txt
%        HW_Tables/   tables as CSV + LaTeX
%        HW_Checkpoint.mat, HW_Opt_Results.mat, HW_Run_Log.txt, HW_Final_Results.zip
% EMERGENCY STOP: press Ctrl+C, then type   clear cleanupObj   (sends VREF=5 and RESET).
% Requires: MATLAB R2020a+ (serialport). No toolbox needed.
clear; clc; close all;
warning('off','MATLAB:print:ContentTypeImageSuggested');
global HWLOG HWN %#ok<GVMIS>
diary off; if exist('HW_Run_Log.txt','file'), try, delete('HW_Run_Log.txt'); catch, end, end
diary('HW_Run_Log.txt'); tAll = tic;

%% ============================ 0. USER SETTINGS ============================
cfg.PORT      = "COM10";        % <<< your Arduino port (type serialportlist)
cfg.BAUD      = 115200;
cfg.quickTest = true;          % <<< true = 15 min check, false = full run
cfg.resume    = false;          % <<< true = continue an interrupted run (same settings!)
cfg.budgetMin = 175;            % total wall-clock budget [min]; the run ends close to this
cfg.popTarget = 10; cfg.popMin = 8;        % population size
cfg.runsTarget = 5; cfg.runsMin = 4;       % independent runs per algorithm
cfg.iterMin   = 15;                        % minimum iterations
cfg.nRe       = 3;              % hardware re-evaluations of every run's best design
cfg.nRepVal   = 5;              % final validation repeats per design
cfg.seed      = 2026;

% hardware test: output-voltage levels [V]. Optimisation uses up-steps, validation also down-steps.
cfg.optLevels = [8 12 15];          % 8->12 and 12->15
cfg.valLevels = [8 12 15 12 8];     % 8->12, 12->15, 15->12, 12->8
cfg.tSettle = 0.7;   % s, wait at first level before measuring
cfg.tPre    = 0.25;  % s, pre-step window
cfg.tRec    = 1.0;   % s, recorded time after every step
cfg.dtGrid  = 0.005; % s, common time grid for averaged validation traces
cfg.Vabort  = 17.5;  % V, abort test and go to safe gains above this (check your capacitor rating!)
cfg.safeKp  = 0.1;   cfg.safeKi = 20;      % known-good gains used for recovery
cfg.restSec = 4;     % s, cool-down after every optimiser run (load resistors!)

% cost  J = max_s(Js) + 0.2*mean_s(Js)  (+ penalties for steady error > 2 %, ripple > 5 %)
cfg.w   = [0.3 0.3 0.4];     % weights: IAE, settling time, overshoot
cfg.ref = [0.05 0.2 5];      % normalisation: IAE [s], Ts [s], OS [%]
cfg.big = 1e3;               % penalty for failed / aborted tests
cfg.tolPct = 2;              % settling band [%] (widened automatically by the measured noise)

% unit conversion simulation -> hardware.  Simulink PI: duty[0..1] = Kp*e[V] + Ki*int(e dt)
% Arduino PI: duty[counts/255] with e in ADC counts (1 V of Vout = 1023/(5*5) = 40.92 counts)
% => Kp_sim = Kp_hw*40.92/255 and Ki_sim = Ki_hw*40.92/255
cfg.simToHw = (1023/(5*5))/255;          % = 0.16047

% PI search range  (Kp linear, Ki log).  Units = duty counts per ADC count, as in the Arduino code
lim.Kp = [0 0.5];   lim.Ki = [2 400];    % (Arduino accepts Kp<=5, Ki<=500)

% designs found by the SIMULATION (Final_Results.zip: Table_Ranking / Table_Simulink), simulation units
simNames = {'Sim-GWO','Sim-NSGA-II','Sim pidtune wc300','Sim Slider'};
Kp_sim   = [0.0161399640 0.0196344953 0       0.00243];
Ki_sim   = [44.9641732   46.3597243   9.1165  15.314];
simOS    = [14.2159 17.1862 15.5956 14.2533];   % worst overshoot over the Simulink scenarios [%]
simTs    = [38.80   30.16   90.32   70.54];      % worst settling time over the Simulink scenarios [ms]

% reference designs: two hardware references + the simulation designs converted to hardware units
refNames = [{'Baseline (0.1, 20)','Ref (0.05, 10)'}, simNames];
Kp_ref   = [0.1 0.05, Kp_sim/cfg.simToHw];
Ki_ref   = [20  10,   Ki_sim/cfg.simToHw];

if cfg.quickTest
    cfg.budgetMin = 15; cfg.popTarget = 6; cfg.popMin = 4; cfg.runsTarget = 2; cfg.runsMin = 2;
    cfg.iterMin = 4; cfg.nRe = 2; cfg.nRepVal = 2; cfg.restSec = 2;
end
assert(all([cfg.optLevels cfg.valLevels] >= 5 & [cfg.optLevels cfg.valLevels] <= 18), 'Levels must be 5..18 V');
assert(all(Ki_ref <= 500) && all(Kp_ref <= 5), 'Reference gains exceed the Arduino limits');

set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman', ...
    'defaultAxesFontSize',9,'defaultLineLineWidth',1.4,'defaultAxesBox','on', ...
    'defaultAxesXGrid','on','defaultAxesYGrid','on');

%% ============================ 1. CONNECT + SANITY TEST ====================
rng(cfg.seed);
HWLOG = nan(80000,9); HWN = 0;
s = serialport(cfg.PORT, cfg.BAUD); configureTerminator(s,"LF");
tw = tic; while s.NumBytesAvailable == 0 && toc(tw) < 8, pause(0.1); end   % wait for the Arduino to start talking
pause(0.5); raw = ''; if s.NumBytesAvailable > 0, raw = read(s, s.NumBytesAvailable, "char"); end
if ~contains(string(raw), "DATA,") && ~contains(string(raw), "READY")
    fprintf('Received %d bytes, no DATA/READY lines. First bytes: [%s]\n', numel(raw), strtrim(raw(1:min(end,120))));
    error(['Arduino is not sending DATA lines on ' char(cfg.PORT) ' at ' num2str(cfg.BAUD) ' baud. Run check_arduino.m to find the cause.']);
end
flush(s);
cleanupObj = onCleanup(@() hwShutdown(s)); %#ok<NASGU>
hw.s = s; hw.cfg = cfg; hw.lim = lim;

fprintf('--- Sanity test with the baseline gains (Kp=%g, Ki=%g) ---\n', cfg.safeKp, cfg.safeKi);
tE = zeros(1,3);
for i = 1:3
    t1 = tic; [J,~,M,trs] = hwEval(hw, cfg.safeKp, cfg.safeKi, cfg.optLevels, [0 0]); tE(i) = toc(t1);
    fprintf('  test %d: J = %.3f | OS = %s %% | Ts = %s ms | ess = %s %% | %.2f s\n', i, J, ...
        mat2str(round(M(:,2)',2)), mat2str(round(M(:,3)'*1e3)), mat2str(round(M(:,5)',2)), tE(i));
end
if J >= cfg.big, error('Sanity test failed: no valid step response. Check port, wiring, load, levels.'); end
tEval = mean(tE);
figure('Name','Sanity','Color','w'); hold on
for k = 1:numel(trs), if ~isempty(trs{k}), plot(trs{k}.t, trs{k}.y); end, end
xlabel('Time after step (s)'); ylabel('V_{out} (V)'); title('Sanity test (last run)'); drawnow;

%% ============================ 2. PLAN =====================================
nA = 4; algos = {'PSO','GA','GWO','NSGA-II knee'}; short = {'PSO','GA','GWO','NSGA-II'};
names = [refNames, algos]; nRef = numel(refNames); nSimD = numel(simNames); nD = numel(names);
nVS = numel(cfg.valLevels) - 1;
Btot = cfg.budgetMin*60; ckFile = 'HW_Checkpoint.mat';
tRunOH = 2.0 + cfg.restSec;                                   % recovery + rest after every run
tVal   = nD*cfg.nRepVal*(cfg.tSettle + cfg.tPre + nVS*cfg.tRec + 0.4);
tFixed = 240;                                                 % sanity test, figures, zip
resumed = false;
if cfg.resume && exist(ckFile,'file')
    ck = load(ckFile);
    Res = ck.Res; done = ck.done; pop = ck.plan.pop; iter = ck.plan.iter; runs = ck.plan.runs; nRe = ck.plan.nRe;
    HWN = size(ck.hwlog,1); HWLOG(1:HWN,:) = ck.hwlog; resumed = true;
    fprintf('RESUMED: %d of %d runs already done.\n', nnz(done), numel(done));
end
if ~resumed
    nRe = cfg.nRe;  runs = cfg.runsTarget;
    while true
        tRe    = nA*runs*nRe*tEval;
        tOpt   = Btot - tFixed - tVal - tRe;
        perRun = tOpt/(nA*runs) - tRunOH;
        nEv    = floor(perRun/tEval);                         % evaluations available per run
        pop    = min(cfg.popTarget, max(cfg.popMin, round(nEv/20)));
        iter   = floor(nEv/pop) - 1;
        if iter >= cfg.iterMin || runs <= cfg.runsMin, break; end
        runs = runs - 1;
    end
    if iter < 3, error('Budget too small for this setup. Increase cfg.budgetMin.'); end
    for a = 1:nA
        Res(a).z = zeros(runs,2); Res(a).x = zeros(runs,2); Res(a).J = nan(1,runs);
        Res(a).Jre = nan(1,runs); Res(a).Jre_sd = nan(1,runs); Res(a).time = zeros(1,runs);
        Res(a).hist = zeros(runs, iter+1); Res(a).front = cell(1,runs);
    end
    done = false(nA,runs);
end
plan = struct('pop',pop,'iter',iter,'runs',runs,'nRe',nRe);
tOptEst = nA*runs*(pop*(iter+1)*tEval + tRunOH);  tReEst = nA*runs*nRe*tEval;
fprintf('1 hardware evaluation = %.2f s | PLAN: pop = %d, iterations = %d, runs = %d per algorithm (%d evaluations/run)\n', ...
    tEval, pop, iter, runs, pop*(iter+1));
fprintf('Expected: optimisation %.1f min + re-evaluation %.1f min + validation %.1f min + fixed ~4 min = %.0f min (budget %d)\n', ...
    tOptEst/60, tReEst/60, tVal/60, (tOptEst+tReEst+tVal+tFixed)/60, cfg.budgetMin);
fprintf('Search range: Kp = [%g %g], Ki = [%g %g] (Ki log scale) | sim->hw gain factor = %.5f\n', lim.Kp, lim.Ki, cfg.simToHw);

%% ============================ 3. OPTIMISATION (HARDWARE) ==================
for a = 1:nA
    for r = 1:runs
        if done(a,r), continue; end
        rng(cfg.seed + 100*a + r); trun = tic;
        funJ  = @(z) objSO(z, hw, cfg.optLevels, [a r]);
        funMO = @(z) objMO(z, hw, cfg.optLevels, [a r]);
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
        Res(a).hist(r,:) = h; Res(a).time(r) = toc(trun); done(a,r) = true;
        hwSave(ckFile, Res, done, plan, cfg, lim);
        fprintf('%-13s run %2d/%d | J = %.4f | Kp = %.5f | Ki = %.3f | %.1f min | elapsed %.1f min\n', ...
            algos{a}, r, runs, J, x(1), x(2), Res(a).time(r)/60, toc(tAll)/60);
        hwRecover(s, cfg, cfg.optLevels(1)); pause(cfg.restSec);
    end
end

%% ============================ 4. RE-EVALUATE EVERY RUN'S BEST ON HARDWARE =
fprintf('\n--- Re-evaluating each run''s best design (%d repeats each) ---\n', nRe);
for a = 1:nA
    for r = 1:runs
        if ~isnan(Res(a).Jre(r)), continue; end
        Jr = zeros(1,nRe);
        for q = 1:nRe, Jr(q) = hwEval(hw, Res(a).x(r,1), Res(a).x(r,2), cfg.optLevels, [0 0]); end
        Res(a).Jre(r) = mean(Jr); Res(a).Jre_sd(r) = std(Jr);
    end
    hwSave(ckFile, Res, done, plan, cfg, lim);
    fprintf('%-13s re-evaluated | best Jre = %.4f | elapsed %.1f min\n', algos{a}, min(Res(a).Jre), toc(tAll)/60);
end

%% ============================ 5. STATISTICS ===============================
bestRun = zeros(1,nA); bKp = zeros(1,nA); bKi = zeros(1,nA);
for a = 1:nA
    [~,bestRun(a)] = min(Res(a).Jre);
    bKp(a) = Res(a).x(bestRun(a),1);  bKi(a) = Res(a).x(bestRun(a),2);
end
Fp = []; for r = 1:runs, Fp = [Fp; Res(4).front{r}.F]; end %#ok<AGROW>
fmin = min(Fp,[],1); span = max(Fp,[],1) - fmin; span(span==0) = 1;
rng(1); U = rand(30000,3); hv = zeros(1,runs);
for r = 1:runs
    Fn = (Res(4).front{r}.F - fmin)./(1.1*span);  dm = false(size(U,1),1);
    for i = 1:size(Fn,1), dm = dm | all(U >= Fn(i,:), 2); end
    hv(r) = mean(dm);
end
pMat = nan(nA);
for i = 1:nA, for j = 1:nA, if i ~= j, pMat(i,j) = rankSumP(Res(i).Jre, Res(j).Jre); end, end, end
St = table(string(algos(:)), arrayfun(@(a) min(Res(a).Jre),1:nA)', arrayfun(@(a) median(Res(a).Jre),1:nA)', ...
    arrayfun(@(a) mean(Res(a).Jre),1:nA)', arrayfun(@(a) std(Res(a).Jre),1:nA)', ...
    arrayfun(@(a) max(Res(a).Jre),1:nA)', arrayfun(@(a) mean(Res(a).time)/60,1:nA)', bKp(:), bKi(:), ...
    'VariableNames', {'Algorithm','J_best','J_median','J_mean','J_std','J_worst','Time_min_per_run','Kp_best','Ki_best'});
fprintf('\n--- Optimiser statistics (%d runs, hardware re-evaluated cost J) ---\n', runs); disp(St)
fprintf('NSGA-II hypervolume: mean %.4f, std %.4f\n', mean(hv), std(hv));
disp(array2table(pMat,'VariableNames',strrep(algos,'-','_'),'RowNames',strrep(algos,'-','_')))

%% ============================ 6. FINAL HARDWARE VALIDATION ================
% designs are measured in random order, repeated nRepVal times (cancels temperature drift)
Kp_all = [Kp_ref, bKp];  Ki_all = [Ki_ref, bKi];
tG = (round(-cfg.tPre/cfg.dtGrid):round(cfg.tRec/cfg.dtGrid))'*cfg.dtGrid;  nRep = cfg.nRepVal;
MV = cell(nD,1); VG = cell(nD,1); DG = cell(nD,1);
for d = 1:nD
    MV{d} = nan(nVS,8,nRep); VG{d} = nan(numel(tG),nVS,nRep); DG{d} = VG{d};
end
fprintf('\n--- Final validation: %d designs x %d repeats, steps %s ---\n', nD, nRep, mat2str(cfg.valLevels));
for rep = 1:nRep
    for d = randperm(nD)
        [~,~,M,trs] = hwEval(hw, Kp_all(d), Ki_all(d), cfg.valLevels, [0 0]);
        MV{d}(:,:,rep) = M;
        for k = 1:nVS
            if isempty(trs{k}), continue; end
            [tu,iu] = unique(trs{k}.t);
            VG{d}(:,k,rep) = interp1(tu, trs{k}.y(iu), tG, 'linear', NaN);
            DG{d}(:,k,rep) = interp1(tu, trs{k}.d(iu), tG, 'linear', NaN);
        end
    end
    fprintf('  validation repeat %d/%d done | elapsed %.1f min\n', rep, nRep, toc(tAll)/60);
end
save('HW_Validation.mat','MV','VG','DG','tG','Kp_all','Ki_all','names','cfg');

Jv = zeros(nD,nRep); wOS = Jv; wTs = Jv; wEss = Jv; wRip = Jv; wDut = Jv; wTr = Jv;
for d = 1:nD
    for rep = 1:nRep
        Mx = MV{d}(:,:,rep);  Jv(d,rep) = aggregateHW(Mx, cfg);
        wOS(d,rep) = max(Mx(:,2)); wTs(d,rep) = max(Mx(:,3))*1e3; wTr(d,rep) = max(Mx(:,4))*1e3;
        wEss(d,rep) = max(Mx(:,5)); wRip(d,rep) = max(Mx(:,6)); wDut(d,rep) = max(Mx(:,7));
    end
end
ms = @(X) mean(X,2,'omitnan');  sd = @(X) std(X,0,2,'omitnan');
T2 = table(string(names(:)), Kp_all(:), Ki_all(:), ms(wOS), sd(wOS), ms(wTs), sd(wTs), ms(wTr), ms(wEss), ms(wRip), ms(wDut), ms(Jv), sd(Jv), ...
    'VariableNames', {'Design','Kp','Ki','WorstOS_pct','OS_sd','WorstTs_ms','Ts_sd','WorstTr_ms','WorstSSerr_pct','Ripple_pct','DutyStd_pct','J_mean','J_sd'});
fprintf('\n--- Hardware validation (worst over the %d steps, mean of %d single repeats) ---\n', nVS, nRep); disp(T2)

% ensemble-averaged traces (noise reduced by sqrt(nRep)) -> cleaner OS / Ts / Tr
MEn = cell(nD,1); eOS = zeros(nD,1); eTs = eOS; eTr = eOS; eEss = eOS;
for d = 1:nD
    MEn{d} = ensMetrics(VG{d}, DG{d}, tG, cfg.valLevels, cfg, squeeze(MV{d}(:,8,:)), nRep);
    eOS(d) = max(MEn{d}(:,2)); eTs(d) = max(MEn{d}(:,3))*1e3; eTr(d) = max(MEn{d}(:,4))*1e3; eEss(d) = max(MEn{d}(:,5));
end
T2e = table(string(names(:)), Kp_all(:), Ki_all(:), eOS, eTs, eTr, eEss, 'VariableNames', ...
    {'Design','Kp','Ki','OS_pct_ens','Ts_ms_ens','Tr_ms_ens','SSerr_pct_ens'});
fprintf('\n--- Ensemble-averaged metrics (worst over the steps) ---\n'); disp(T2e)

sIdx = 2 + (1:nSimD);                       % simulation-designed controllers (converted)
Tsim = table(string(names(sIdx)'), Kp_sim(:), Ki_sim(:), Kp_all(sIdx)', Ki_all(sIdx)', simOS(:), simTs(:), ...
    eOS(sIdx), eTs(sIdx), ms(Jv(sIdx,:)), 'VariableNames', ...
    {'Design','Kp_sim','Ki_sim','Kp_hw','Ki_hw','SimOS_pct','SimTs_ms','HwOS_pct','HwTs_ms','HwJ'});
fprintf('\n--- Simulation-optimised gains (converted by %.5f) tested on the hardware ---\n', cfg.simToHw); disp(Tsim)

[~,ord] = sort(ms(Jv));
Rk = table((1:nD)', string(names(ord)'), Kp_all(ord)', Ki_all(ord)', ms(Jv(ord,:)), sd(Jv(ord,:)), ...
    'VariableNames', {'Rank','Design','Kp','Ki','J_mean','J_sd'});
fprintf('\n--- Final ranking (hardware validation cost) ---\n'); disp(Rk)
dBest = ord(1);  Kp = Kp_all(dBest);  Ki = Ki_all(dBest);
fprintf('>>> RECOMMENDED: %s  ->  Kp = %.5f,  Ki = %.4f\n', names{dBest}, Kp, Ki);
stepName = arrayfun(@(k) sprintf('%g->%g V',cfg.valLevels(k),cfg.valLevels(k+1)), 1:nVS, 'UniformOutput', false)';
Mb = MEn{dBest};
T3 = table(string(stepName), Mb(:,2), Mb(:,3)*1e3, Mb(:,4)*1e3, Mb(:,5), Mb(:,6), Mb(:,7), 'VariableNames', ...
    {'Step','OS_pct','Ts_ms','Tr_ms','SSerr_pct','Ripple_pct','DutyStd_pct'});
fprintf('\n--- Recommended design, step by step (ensemble average) ---\n'); disp(T3)

%% ============================ 7. TABLES ===================================
if ~exist('HW_Tables','dir'), mkdir('HW_Tables'); end
Tcfg = table(["Controller";"PI search range Kp";"PI search range Ki";"Optimisation steps (V)";"Validation steps (V)"; ...
    "Population / iterations / runs";"Re-evaluations / validation repeats";"Simulation-to-hardware gain factor";"Cost function"], ...
    [ "discrete PI, 1 kHz, Arduino"; sprintf('[%g, %g]',lim.Kp); sprintf('[%g, %g] (log)',lim.Ki); ...
      string(mat2str(cfg.optLevels)); string(mat2str(cfg.valLevels)); sprintf('%d / %d / %d',pop,iter,runs); ...
      sprintf('%d / %d',nRe,nRep); sprintf('%.5f',cfg.simToHw); "max+0.2 mean of weighted IAE, Ts, OS (+penalties)"], ...
    'VariableNames', {'Item','Value'});
TT = {Tcfg,'Table_Setup','Hardware optimisation setup.','tab:hwsetup'; ...
      St,'Table_Optimiser_Statistics','Optimiser statistics over independent hardware runs.','tab:hwstats'; ...
      T2,'Table_Hardware_Validation','Hardware validation of all designs (worst over steps, mean of single repeats).','tab:hwval'; ...
      T2e,'Table_Hardware_Ensemble','Ensemble-averaged hardware metrics (worst over steps).','tab:hwens'; ...
      Tsim,'Table_Sim_vs_Hardware','Simulation-optimised controllers: predicted versus measured performance.','tab:simhw'; ...
      T3,'Table_Recommended_Design','Step-by-step performance of the recommended design.','tab:hwbest'; ...
      Rk,'Table_Ranking','Final ranking of all designs on hardware.','tab:hwrank'};
for i = 1:size(TT,1)
    writetable(TT{i,1}, fullfile('HW_Tables',[TT{i,2} '.csv']));
    table2tex(TT{i,1}, fullfile('HW_Tables',[TT{i,2} '.tex']), TT{i,3}, TT{i,4});
end
hwlog = HWLOG(1:HWN,:);
saveVars = {'Res','St','T2','T2e','T3','Tsim','Rk','Tcfg','cfg','lim','hv','pMat','MV','VG','DG','tG','Kp_all','Ki_all', ...
    'names','hwlog','Jv','done','plan','bKp','bKi','stepName','Kp_sim','Ki_sim','simOS','simTs'};
save('HW_Opt_Results.mat', saveVars{:});

%% ============================ 8. PUBLICATION FIGURES ======================
baseC = [0.35 0.35 0.35; 0.70 0.70 0.70; 0.13 0.55 0.13; 0.45 0.75 0.30; 0.20 0.35 0.20; 0.62 0.82 0.62];
colR = baseC(mod(0:nRef-1, size(baseC,1))+1,:);
colA = [0.00 0.45 0.74; 0.93 0.69 0.13; 0.49 0.18 0.56; 0.64 0.08 0.18];
cols = [colR; colA];  lsty = [repmat({'--'},1,nRef) repmat({'-'},1,nA)];
tms = tG*1e3;

% Fig 1 convergence
try
f = newFig('Fig1',8.8,6.5); hold on; x = 0:iter;
for a = 1:nA
    m = mean(Res(a).hist,1); s_ = std(Res(a).hist,0,1);
    fill([x fliplr(x)], [m+s_ fliplr(m-s_)], colA(a,:), 'FaceAlpha',0.15, 'EdgeColor','none', 'HandleVisibility','off');
    plot(x, m, 'Color', colA(a,:));
end
xlabel('Iteration'); ylabel('Worst-case cost J (measured)'); legend(algos,'Location','northeast'); saveFig(f,'Fig1_Convergence');
catch ME, warning('Fig1 failed: %s', ME.message); end

% Fig 2 final-cost distribution + p-values
try
f = newFig('Fig2',17.5,6.5);
ax = subplot(1,2,1); drawBox(ax, arrayfun(@(a) Res(a).Jre, 1:nA, 'UniformOutput', false), short, colA);
ylabel('Re-evaluated cost J'); title('(a) Distribution over runs','FontWeight','normal');
ax = subplot(1,2,2); imagesc(ax, pMat, [0 0.2]); colormap(ax, flipud(gray)); colorbar(ax);
set(ax,'XTick',1:nA,'XTickLabel',short,'YTick',1:nA,'YTickLabel',short,'XGrid','off','YGrid','off');
for i = 1:nA, for j = 1:nA, if i ~= j, text(j,i,sprintf('%.3f',pMat(i,j)),'HorizontalAlignment','center'); end, end, end
title('(b) Rank-sum p-values','FontWeight','normal'); saveFig(f,'Fig2_Statistics');
catch ME, warning('Fig2 failed: %s', ME.message); end

% Fig 3 Kp-Ki plane (hardware units): optimisers, references, simulation optima, recommended
try
f = newFig('Fig3',9.5,7.5); hold on;
for a = 1:nA, plot(Res(a).x(:,1), Res(a).x(:,2), 'o', 'Color', colA(a,:), 'MarkerFaceColor', colA(a,:), 'MarkerSize', 4); end
plot(Kp_all(1:2), Ki_all(1:2), 's', 'MarkerEdgeColor','k', 'MarkerFaceColor','y', 'MarkerSize', 7);
plot(Kp_all(sIdx), Ki_all(sIdx), '^', 'MarkerEdgeColor','k', 'MarkerFaceColor',[0.13 0.55 0.13], 'MarkerSize', 7);
plot(Kp, Ki, 'p', 'MarkerEdgeColor','k', 'MarkerFaceColor','r', 'MarkerSize', 12);
set(gca,'YScale','log'); xlim(lim.Kp); ylim(lim.Ki); xlabel('K_p'); ylabel('K_i');
legend([short {'Hardware reference','Simulation optimum (converted)','Recommended'}],'Location','southeast','FontSize',6);
saveFig(f,'Fig3_KpKi_Plane');
catch ME, warning('Fig3 failed: %s', ME.message); end

% Fig 4 NSGA-II Pareto front
try
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
catch ME, warning('Fig4 failed: %s', ME.message); end

% Fig 5 measured cost landscape (every hardware evaluation of the optimisation)
try
f = newFig('Fig5',11,8.5);
Lg = hwlog(~isnan(hwlog(:,5)),:);
scatter(Lg(:,3), Lg(:,4), 10, log10(min(Lg(:,5),cfg.big)), 'filled'); hold on
plot(Kp, Ki, 'p', 'MarkerEdgeColor','k', 'MarkerFaceColor','r', 'MarkerSize', 12);
set(gca,'YScale','log'); xlim(lim.Kp); ylim(lim.Ki); xlabel('K_p'); ylabel('K_i');
cb = colorbar; cb.Label.String = 'log_{10} J (measured)'; colormap(parula);
saveFig(f,'Fig5_Cost_Landscape');
catch ME, warning('Fig5 failed: %s', ME.message); end

% Fig 6 hardware step responses (mean of repeats), all designs
try
f = newFig('Fig6',17.5,11);
for k = 1:nVS
    subplot(2,2,k); hold on
    for d = 1:nD
        plot(tms, mean(VG{d}(:,k,:),3,'omitnan'), 'Color',cols(d,:), 'LineStyle',lsty{d});
    end
    plot(tms([1 end]), cfg.valLevels(k+1)*[1 1], 'k:'); xlabel('Time (ms)'); ylabel('V_{out} (V)');
    title(sprintf('(%c) %s', 'a'+k-1, stepName{k}),'FontWeight','normal'); xlim([tms(1) tms(end)]);
end
subplot(2,2,1); legend(names,'Location','southeast','FontSize',5); saveFig(f,'Fig6_Hardware_Steps');
catch ME, warning('Fig6 failed: %s', ME.message); end

% Fig 7 repeatability of the four optimised designs (first step)
try
f = newFig('Fig7',17.5,11);
for a = 1:nA
    d = nRef + a; subplot(2,2,a); hold on
    for rep = 1:nRep, plot(tms, VG{d}(:,1,rep), 'Color',[0.75 0.75 0.75], 'LineWidth',0.6); end
    plot(tms, mean(VG{d}(:,1,:),3,'omitnan'), 'Color',colA(a,:), 'LineWidth',2);
    plot(tms([1 end]), cfg.valLevels(2)*[1 1], 'k:'); xlabel('Time (ms)'); ylabel('V_{out} (V)');
    title(sprintf('(%c) %s: K_p=%.4f, K_i=%.1f', 'a'+a-1, short{a}, Kp_all(d), Ki_all(d)),'FontWeight','normal');
end
saveFig(f,'Fig7_Repeatability');
catch ME, warning('Fig7 failed: %s', ME.message); end

% Fig 8 duty cycle (control effort)
try
f = newFig('Fig8',17.5,6);
for p = 1:2
    subplot(1,2,p); hold on
    for d = 1:nD, plot(tms, mean(DG{d}(:,p,:),3,'omitnan'), 'Color',cols(d,:), 'LineStyle',lsty{d}); end
    xlabel('Time (ms)'); ylabel('Duty (%)'); title(sprintf('(%c) %s', 'a'+p-1, stepName{p}),'FontWeight','normal');
end
subplot(1,2,1); legend(names,'Location','southeast','FontSize',5); saveFig(f,'Fig8_Duty');
catch ME, warning('Fig8 failed: %s', ME.message); end

% Fig 9 metric summary with error bars
try
f = newFig('Fig9',17.5,9);
Mset = {wOS,'Worst overshoot (%)'; wTs,'Worst settling time (ms)'; wEss,'Worst steady-state error (%)'; wDut,'Duty std in steady state (%)'};
for i = 1:4
    subplot(2,2,i); m = ms(Mset{i,1}); e_ = sd(Mset{i,1});
    b = bar(m); b.FaceColor = 'flat'; b.CData = cols; hold on; errorbar(1:nD, m, e_, 'k.', 'LineWidth', 1);
    set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',45,'FontSize',6); ylabel(Mset{i,2});
    title(sprintf('(%c)', 'a'+i-1),'FontWeight','normal');
end
saveFig(f,'Fig9_Metric_Summary');
catch ME, warning('Fig9 failed: %s', ME.message); end

% Fig 10 validation cost over repeats
try
f = newFig('Fig10',13,7.5); ax = axes;
drawBox(ax, arrayfun(@(d) Jv(d,:), 1:nD, 'UniformOutput', false), names, cols);
set(ax,'XTickLabelRotation',45,'FontSize',6); ylabel('Validation cost J (hardware)');
saveFig(f,'Fig10_Validation_Cost');
catch ME, warning('Fig10 failed: %s', ME.message); end

% Fig 11 simulation prediction versus hardware measurement (simulation-designed controllers)
try
f = newFig('Fig11',17.5,6.5);
subplot(1,2,1); bar([simOS(:) eOS(sIdx)]);
set(gca,'XTick',1:nSimD,'XTickLabel',names(sIdx),'XTickLabelRotation',20,'FontSize',7);
ylabel('Worst overshoot (%)'); legend('Simulation (Simulink)','Hardware','Location','northwest'); title('(a)','FontWeight','normal');
subplot(1,2,2); bar([simTs(:) eTs(sIdx)]);
set(gca,'XTick',1:nSimD,'XTickLabel',names(sIdx),'XTickLabelRotation',20,'FontSize',7);
ylabel('Worst settling time (ms)'); title('(b)','FontWeight','normal');
saveFig(f,'Fig11_Sim_vs_Hardware');
catch ME, warning('Fig11 failed: %s', ME.message); end

%% ============================ 9. FIGURE LIST ==============================
FL = {'Fig1_Convergence','Convergence of the mean measured worst-case cost J (shaded: +/-1 std over independent hardware runs).';
      'Fig2_Statistics','(a) Distribution of the re-evaluated cost over runs. (b) Wilcoxon rank-sum p-values between algorithms.';
      'Fig3_KpKi_Plane','Best (Kp, Ki) of every run, hardware references, simulation optima converted to hardware units, and the recommended design.';
      'Fig4_Pareto_Knee','NSGA-II: (a) Pareto front of IAE, settling time and overshoot, (b) projection with knee, (c) hypervolume over runs.';
      'Fig5_Cost_Landscape','Measured cost of every hardware evaluation in the (Kp, Ki) plane.';
      'Fig6_Hardware_Steps','Measured output-voltage step responses of all designs (mean of repeats) for the up- and down-steps.';
      'Fig7_Repeatability','Repeatability of the four optimised designs: individual repeats (grey) and mean (colour).';
      'Fig8_Duty','Measured duty cycle during the first two steps (control effort).';
      'Fig9_Metric_Summary','Worst-case overshoot, settling time, steady-state error and duty noise (mean +/- std over repeats).';
      'Fig10_Validation_Cost','Hardware validation cost J of every design over repeated measurements.';
      'Fig11_Sim_vs_Hardware','Simulation-predicted (Simulink) versus hardware-measured overshoot and settling time of the simulation-designed controllers.'};
FigList = cell2table(FL, 'VariableNames', {'File','Caption'});
writetable(FigList, fullfile('HW_Figures','Figure_List.csv'));
fid = fopen(fullfile('HW_Figures','Figure_List.txt'),'w');
for i = 1:size(FL,1), fprintf(fid,'%s  (.pdf / .png / .fig)\n    %s\n\n', FL{i,1}, FL{i,2}); end
fclose(fid);

save('HW_Opt_Results.mat', saveVars{:});
fprintf('\nTotal time: %.1f min. Figures: HW_Figures/, tables: HW_Tables/\n', toc(tAll)/60);
fprintf('Recommended: Kp = %.5f, Ki = %.4f  (%s)\n', Kp, Ki, names{dBest});
hwShutdown(s); clear cleanupObj; diary off;
try
    zip('HW_Final_Results.zip', {'HW_Figures','HW_Tables','HW_Opt_Results.mat','HW_Validation.mat','HW_Checkpoint.mat','HW_Run_Log.txt'});
    disp('Saved: HW_Final_Results.zip');
catch ME
    warning('Zip failed: %s', ME.message);
end

%% ====================================================================
%%                          LOCAL FUNCTIONS
%% ====================================================================
%% ---------------------------- hardware layer ----------------------------
function hwShutdown(s)
try, writeline(s,"VREF=5"); writeline(s,"RESET"); catch, end
end

function hwRecover(s, cfg, V0)
try
    writeline(s, sprintf('KP=%.5f', cfg.safeKp)); writeline(s, sprintf('KI=%.5f', cfg.safeKi));
    writeline(s, sprintf('VREF=%.2f', V0)); pause(1.5); flush(s);
catch
end
end

function hwSave(ckFile, Res, done, plan, cfg, lim) %#ok<INUSD>
global HWLOG HWN
hwlog = HWLOG(1:HWN,:); %#ok<NASGU>
save(ckFile, 'Res','done','plan','cfg','lim','hwlog');
end

function [D,ab] = collect(s, dur, Vabort)
% read telemetry for dur seconds -> rows [t_ms target vout duty pwm]
D = zeros(0,5); buf = ''; ab = false; t0 = tic;
while toc(t0) < dur
    n = s.NumBytesAvailable;
    if n == 0, pause(0.002); continue; end
    buf = [buf, read(s, n, "char")]; %#ok<AGROW>
    k = find(buf == newline, 1, 'last');
    if isempty(k), continue; end
    L = splitlines(string(buf(1:k)));  buf = buf(k+1:end);
    for i = 1:numel(L)
        q = split(strtrim(L(i)), ",");
        if numel(q) == 6 && q(1) == "DATA"
            v = str2double(q(2:6));
            if ~any(isnan(v)), D(end+1,:) = v.'; end %#ok<AGROW>
        end
    end
    if ~isempty(D) && any(D(:,3) > Vabort), ab = true; return; end
end
if isempty(D), error('No telemetry from the Arduino (check port / cable / sketch).'); end
end

function [J,F,M,tr] = hwEval(hw, Kp, Ki, levels, tag)
% one hardware test: chain of voltage steps. M rows = steps, cols = [IAE OS Ts Tr ess% rip% dutyStd noise]
global HWLOG HWN
s = hw.s; cfg = hw.cfg; n = numel(levels) - 1;
M = nan(n,8); tr = cell(n,1); t0 = tic;
flush(s);
writeline(s, sprintf('KP=%.5f', Kp)); writeline(s, sprintf('KI=%.5f', Ki));
writeline(s, sprintf('VREF=%.2f', levels(1)));
[~,ab] = collect(s, cfg.tSettle, cfg.Vabort);
if ~ab
    [prev,ab] = collect(s, cfg.tPre, cfg.Vabort);
    for k = 1:n
        if ab, break; end
        writeline(s, sprintf('VREF=%.2f', levels(k+1)));
        [post,ab] = collect(s, cfg.tRec, cfg.Vabort);
        if ab, break; end
        pre = prev(prev(:,1) > prev(end,1) - 1000*cfg.tPre, :);
        [M(k,:), tr{k}] = stepMetrics(pre, post, levels(k+1), cfg);
        prev = post;
    end
end
if ab, hwRecover(s, cfg, levels(1)); end
[J,F] = aggregateHW(M, cfg);
if tag(1) > 0 && HWN < size(HWLOG,1)
    HWN = HWN + 1;  HWLOG(HWN,:) = [tag(1) tag(2) Kp Ki J F toc(t0)];
end
end

function [m,trk] = stepMetrics(pre, post, Vto, cfg, sigOv)
% m = [IAE OS Ts Tr ess% ripple% dutyStd noise_V]  (response normalised 0 -> 1)
m = nan(1,8); trk = [];
i0 = find(abs(post(:,2) - Vto) < 0.01, 1);
if isempty(i0) || size(post,1) < 20 || size(pre,1) < 5, return; end
y0  = mean(pre(:,3));  stp = Vto - y0;
if abs(stp) < 0.5, return; end
t  = (post(:,1) - post(i0,1))/1000;  k = t >= 0;  tt = t(k);
yr = post(k,3);  dd = post(k,4);  last = tt > tt(end) - 0.3;
if nargin < 5 || isempty(sigOv)
    sigN = std(diff(yr(last)))/sqrt(2);             % white-noise level at the FINAL voltage [V]
else
    sigN = sigOv;                                    % given (ensemble-averaged traces)
end
yn  = (movmean(yr,3) - y0)/stp;                     % lightly smoothed, normalised response
e   = abs(1 - yn);
IAE = trapz(tt, e);
yss = mean(yn(last));
OS  = max(0, max(yn) - yss)*100;                    % overshoot relative to the final value
band = max(cfg.tolPct/100, 2*sigN/abs(stp));        % noise-aware settling band
o = find(e > band, 1, 'last');
if isempty(o), Ts = 0; elseif o == numel(tt), Ts = tt(end); else, Ts = tt(o+1); end
Tr = firstUp(tt,yn,0.9) - firstUp(tt,yn,0.1);  if isnan(Tr), Tr = tt(end); end
m = [IAE OS Ts Tr abs(1 - yss)*100 std(yn(last))*100 std(dd(last)) sigN];
trk = struct('t', [(pre(:,1) - post(i0,1))/1000; t], 'y', [pre(:,3); post(:,3)], 'd', [pre(:,4); post(:,4)]);
end

function Me = ensMetrics(VGd, DGd, tG, levels, cfg, sigMat, nRep)
% metrics of the ensemble-averaged response of one design (noise reduced by sqrt(nRep))
nS = numel(levels) - 1;  Me = nan(nS,8);
for k = 1:nS
    y = mean(VGd(:,k,:),3,'omitnan');  d = mean(DGd(:,k,:),3,'omitnan');
    ok = ~isnan(y);  if nnz(ok) < 30, continue; end
    pr = ok & tG < 0;  po = ok & tG >= 0;
    pre  = [tG(pr)*1000, levels(k)*ones(nnz(pr),1),   y(pr), d(pr), zeros(nnz(pr),1)];
    post = [tG(po)*1000, levels(k+1)*ones(nnz(po),1), y(po), d(po), zeros(nnz(po),1)];
    sg = mean(sigMat(k,:),'omitnan')/sqrt(nRep);
    Me(k,:) = stepMetrics(pre, post, levels(k+1), cfg, sg);
end
end

function tc = firstUp(t, y, lvl)
idx = find(y >= lvl, 1);
if isempty(idx), tc = NaN; return; end
if idx == 1, tc = t(1); return; end
tc = t(idx-1) + (lvl - y(idx-1))*(t(idx) - t(idx-1))/(y(idx) - y(idx-1));
end

function [J,F] = aggregateHW(M, cfg)
bad = any(isnan(M),2);
if any(bad), J = cfg.big*(1 + mean(bad)); F = cfg.big*ones(1,3); return; end
pen = max(0, M(:,5) - 2) + max(0, M(:,6) - 5);            % steady error > 2 %, ripple > 5 %
Js  = cfg.w(1)*M(:,1)/cfg.ref(1) + cfg.w(2)*M(:,3)/cfg.ref(2) + cfg.w(3)*M(:,2)/cfg.ref(3) + pen;
J   = max(Js) + 0.2*mean(Js);
F   = [max(M(:,1))/cfg.ref(1), max(M(:,3))/cfg.ref(2), max(M(:,2))/cfg.ref(3)] + max(pen);
end

function x = z2x(z, lim)      % z in [0,1]^2 -> [Kp Ki]  (Ki on log scale)
x(1) = lim.Kp(1) + z(1)*(lim.Kp(2)-lim.Kp(1));
x(2) = exp(log(lim.Ki(1)) + z(2)*(log(lim.Ki(2))-log(lim.Ki(1))));
end

function J = objSO(z, hw, levels, tag)
x = z2x(z, hw.lim);  J = hwEval(hw, x(1), x(2), levels, tag);
end

function o = objMO(z, hw, levels, tag)
x = z2x(z, hw.lim);  [J,F] = hwEval(hw, x(1), x(2), levels, tag);  o = [F J];
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
if ~exist('HW_Figures','dir'), mkdir('HW_Figures'); end
p = fullfile('HW_Figures', name);
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