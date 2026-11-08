%% Run_PI_Hardware_Optimization.m
% PSO | GA | GWO | NSGA-II (knee) tuning of the PI controller DIRECTLY on the physical
% Arduino boost converter (same serial protocol as boostIO.m / boost_simulink.slx).
%
% STEPS
%   1) cfg.mock = true,  cfg.quickTest = true   -> test the whole script WITHOUT hardware (~5 min)
%   2) cfg.mock = false, cfg.quickTest = true   -> short hardware check (~25 min)
%   3) cfg.mock = false, cfg.quickTest = false  -> full study (~2.5 h, sized to cfg.budgetMin)
% BEFORE hardware runs: upload boost_controller_hil.ino, close Simulink / Serial Monitor
% (only one program can use the COM port), connect the load, set cfg.port.
% Kp and Ki here are in the units of the Arduino sketch (ADC counts, 1 ms loop).
% Needs MATLAB R2019b or newer (serialport / writeline). No extra toolbox required.
clear; clc; close all; clear functions
if exist('HW_Log.txt','file'), delete('HW_Log.txt'); end
diary('HW_Log.txt'); tAll = tic;

%% ============================ 0. USER SETTINGS ============================
cfg.mock      = true;          % <<< true = emulated circuit (test), false = real hardware
cfg.port      = "COM10";       % <<< your Arduino port (see list printed below)
cfg.baud      = 115200;
cfg.quickTest = true;          % <<< true = short run
cfg.budgetMin = 150;           % total hardware time budget [min]
cfg.fracOpt   = 0.75;          % share for optimisation; the rest = final repeated confirmation
cfg.popTarget = 10; cfg.popMin = 6; cfg.runsTarget = 3; cfg.runsMin = 2; cfg.iterMin = 5;
cfg.nRep      = 5;             % repeated measurements of every final design
cfg.seed      = 2026;
cfg.resume    = false;         % true = reuse HW_Checkpoint.mat after an interruption
cfg.testSimDesigns = true;     % also measure the gains designed in simulation (sim-to-real gap)
cfg.testAuto  = true;          % also measure the PID-Tuner "Auto" design (aggressive!)
cfg.simFile   = 'PI_Opt_Results.mat';   % result file of the simulation study (optional)
if cfg.quickTest
    cfg.budgetMin = 25; cfg.popTarget = 6; cfg.popMin = 4; cfg.runsTarget = 1; cfg.runsMin = 1;
    cfg.iterMin = 3; cfg.nRep = 2;
end
if cfg.mock, cfg.budgetMin = min(cfg.budgetMin, 5); end

% search range in the sketch's units (ADC counts).  Your working design: Kp=0.1, Ki=20
lim.Kp = [0 0.3];   lim.Ki = [5 300];

% test protocol per candidate: [from to] voltage steps, each tested UP and DOWN
cfg.vBase = 12;  cfg.steps = [12 14; 12 16; 11 17];   % small, medium, large (worst-case) steps
cfg.tSettle = 1.0;  cfg.tHold = 0.6;                  % seconds
cfg.vTrip = 19.5;  cfg.tripRest = 2.0;  cfg.maxTrips = 60;
cfg.midRow = 3;                                       % row of the 12->16 UP step in the metrics
cfg.osDead = 0.05;                                    % [V] overshoot dead-band = 2 ADC counts (ignores quantisation noise)

% cost per step: Js = w1*IAEn/ref1 + w2*Ts/ref2 + w3*OS/ref3 + w4*chatter/ref4
% J = max_steps(Js) + 0.2*mean_steps(Js)
cfg.w   = [0.2 0.3 0.3 0.2];
cfg.ref = [0.03 0.08 5 1.0];   % IAE/step [s], Ts [s], OS [%], duty chatter [% std]
cfg.big = 100;

% emulated plant (mock mode only)
cfg.mockPlant = struct('Vin',10,'L',22.5e-6,'C',470e-6,'R',50,'RL',0.1);

[cfg.ev, cfg.jumps, cfg.Tend] = buildSched(cfg);
sim2hw = 255/(1023/25);        % sim gains (duty per volt) -> sketch units: x 6.2317

warning('off','MATLAB:print:ContentTypeImageSuggested');
set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman', ...
    'defaultAxesFontSize',9,'defaultLineLineWidth',1.4,'defaultAxesBox','on', ...
    'defaultAxesXGrid','on','defaultAxesYGrid','on');
try, fprintf('Serial ports found: %s\n', strjoin(string(serialportlist("available")), ', ')); catch, end

%% ============================ 1. DESIGNS TO COMPARE =======================
algos = {'PSO','GA','GWO','NSGA-II knee'};  nA = 4;
% sim-designed gains (converted to sketch units)
simNames = {'PSO','GA','GWO','NSGA-II knee'};
simKp = [0.0088312 0.0063608 0.0071811 0.0065301];       % fallback: analytic quick run
simKi = [39.741 36.503 37.999 37.152];
if exist(cfg.simFile,'file')
    try
        S = load(cfg.simFile, 'names', 'Kp_all', 'Ki_all');
        simKp = S.Kp_all(end-3:end);  simKi = S.Ki_all(end-3:end);
        fprintf('Loaded simulation-designed gains from %s\n', cfg.simFile);
    catch, fprintf('Could not read %s, using built-in simulation gains.\n', cfg.simFile); end
end
dN = {'Baseline (Kp=0.1, Ki=20)','pidtune wc=100','pidtune wc=150','pidtune wc=300','Slider (app)'};
dKp = [0.1 0 0 0 0.00243*sim2hw];   dKi = [20 3.0443 4.5652 9.1165 15.314]*1;
dKi(2:5) = dKi(2:5)*sim2hw;  dGrp = ones(1,5);
if cfg.testAuto, dN{end+1} = 'Auto (app)'; dKp(end+1) = 0; dKi(end+1) = 48.057*sim2hw; dGrp(end+1) = 1; end
if cfg.testSimDesigns
    for a = 1:nA
        dN{end+1} = [simNames{a} ' (sim)']; dKp(end+1) = simKp(a)*sim2hw; dKi(end+1) = simKi(a)*sim2hw; dGrp(end+1) = 2; %#ok<SAGROW>
    end
end
dKp = round(dKp,4); dKi = round(dKi,3);
nPre = numel(dN);

%% ============================ 2. CONNECT + SANITY CHECK ===================
H = hwOpen(cfg);
CACHE = containers.Map('KeyType','char','ValueType','any');
ST = containers.Map();  ST('nEval') = 0;  ST('nTrip') = 0;  ST('evals') = {};
plan = [];
if cfg.resume && exist('HW_Checkpoint.mat','file')
    L = load('HW_Checkpoint.mat'); CACHE = L.CACHE; plan = L.plan; fprintf('Resumed: %d cached evaluations.\n', CACHE.Count);
end
try
    rec0 = hwEval(0.1, 20, H, cfg, CACHE, ST, false);
    tEval = rec0.wall;
    fprintf('Baseline Kp=0.1, Ki=20: J = %.3f | trip = %d | one trial = %.1f s\n', rec0.J, rec0.trip, tEval);
    if rec0.trip || ~rec0.ok
        error('Baseline design gave no valid response. Check wiring/load/port before optimising.');
    end
    fprintf('   per-step [IAE_n OS%% Ts Tr chatter]:\n'); disp(rec0.M(:,1:5))

    % ---------------- sizing from the time budget ----------------
    if isempty(plan)
        Btot = cfg.budgetMin*60;
        perAlgo = floor(floor(cfg.fracOpt*Btot/tEval)/4);  runs = cfg.runsTarget;
        while true
            perRun = floor(perAlgo/runs);
            pop  = min(cfg.popTarget, max(cfg.popMin, round(perRun/8)));
            iter = floor(perRun/pop) - 1;
            if iter >= cfg.iterMin || runs <= cfg.runsMin, break; end
            runs = runs - 1;
        end
        plan = struct('pop',pop,'iter',max(iter,3),'runs',runs);
    end
    pop = plan.pop; iter = plan.iter; runs = plan.runs;
    fprintf('PLAN: pop = %d, iterations = %d, runs = %d per algorithm | expected ~ %.0f min optimisation + %.0f min confirmation\n', ...
        pop, iter, runs, 4*runs*pop*(iter+1)*tEval/60, nPre*cfg.nRep*tEval/60 + 4*cfg.nRep*tEval/60);

    %% ============================ 3. OPTIMISATION ON THE HARDWARE =============
    warm = [0.1 20];  if cfg.testSimDesigns, warm = [warm; dKp(nPre-3) dKi(nPre-3)]; end   % known-good + sim-PSO
    funJ  = @(z) hwObj(z, H, cfg, lim, CACHE, ST, 1);
    funMO = @(z) hwObj(z, H, cfg, lim, CACHE, ST, 2);
    Res(nA).J = [];
    for a = 1:nA
        Res(a).z = zeros(runs,2); Res(a).x = zeros(runs,2); Res(a).J = zeros(1,runs);
        Res(a).time = zeros(1,runs); Res(a).hist = zeros(runs, plan.iter+1);
        for r = 1:runs
            rng(cfg.seed + r);  Z0 = rand(pop,2);                 % SAME start population for all 4 algorithms
            for i = 1:min(size(warm,1),pop), Z0(i,:) = x2z(warm(i,:), lim); end
            rng(cfg.seed + 100*a + r);  tr = tic;
            switch a
                case 1, [z,J,h] = runPSO(funJ, pop, plan.iter, 2, Z0);
                case 2, [z,J,h] = runGA (funJ, pop, plan.iter, 2, Z0);
                case 3, [z,J,h] = runGWO(funJ, pop, plan.iter, 2, Z0);
                case 4
                    [Zf,Ff,Jf,h] = runNSGA2(funMO, pop, plan.iter, 2, 3, cfg.big, Z0);
                    ok = all(Ff < cfg.big, 2);
                    if any(ok), Zf = Zf(ok,:); Ff = Ff(ok,:); Jf = Jf(ok); end
                    [~,iu] = unique(round(Zf,6), 'rows');  Zf = Zf(iu,:); Ff = Ff(iu,:); Jf = Jf(iu);
                    kk = kneeIndex(Ff);  z = Zf(kk,:);  J = Jf(kk);
                    Res(a).front{r} = struct('Z',Zf,'F',Ff,'k',kk);
            end
            x = z2x(z, lim);
            Res(a).z(r,:) = z; Res(a).x(r,:) = x; Res(a).J(r) = J; Res(a).hist(r,:) = h; Res(a).time(r) = toc(tr);
            fprintf('%-13s run %d/%d | J = %.3f | Kp = %.4f | Ki = %.2f | trials %d, trips %d | elapsed %.1f min\n', ...
                algos{a}, r, runs, J, x(1), x(2), ST('nEval'), ST('nTrip'), toc(tAll)/60);
            save('HW_Checkpoint.mat','CACHE','plan');
            if ST('nTrip') > cfg.maxTrips, error('Too many trips (%d). Reduce lim.Ki / lim.Kp.', ST('nTrip')); end
        end
    end
    EV = ST('evals');

    %% ============================ 4. FINAL DESIGNS + REPEATED CONFIRMATION ====
    bestRun = zeros(1,nA); bKp = zeros(1,nA); bKi = zeros(1,nA);
    for a = 1:nA
        [~,bestRun(a)] = min(Res(a).J);  bKp(a) = Res(a).x(bestRun(a),1);  bKi(a) = Res(a).x(bestRun(a),2);
    end
    names = [dN, cellfun(@(s) [s ' (HW)'], algos, 'UniformOutput', false)];
    Kp_all = [dKp bKp];  Ki_all = [dKi bKi];  grp = [dGrp 3*ones(1,nA)];  nD = numel(names);
    CONF = cell(nD, cfg.nRep);
    rng(cfg.seed + 999);
    for rep = 1:cfg.nRep
        for d = randperm(nD)                                   % randomised order -> no drift bias
            fprintf('Confirm rep %d/%d | %-26s Kp=%.4f Ki=%.2f | elapsed %.1f min\n', rep, cfg.nRep, names{d}, Kp_all(d), Ki_all(d), toc(tAll)/60);
            CONF{d,rep} = hwEval(Kp_all(d), Ki_all(d), H, cfg, CACHE, ST, true);
        end
    end
catch ME
    hwClose(H); diary off; rethrow(ME);
end
hwClose(H);
save('HW_Raw.mat','Res','CONF','EV','cfg','lim','names','Kp_all','Ki_all','grp','plan');

%% ============================ 5. STATISTICS ===============================
Fp = []; for r = 1:runs, Fp = [Fp; Res(4).front{r}.F]; end %#ok<AGROW>
fmin = min(Fp,[],1); span = max(Fp,[],1) - fmin; span(span==0) = 1;
rng(1); U = rand(30000,3); hv = zeros(1,runs);
for r = 1:runs
    Fn = (Res(4).front{r}.F - fmin)./(1.1*span);  dm = false(size(U,1),1);
    for i = 1:size(Fn,1), dm = dm | all(U >= Fn(i,:), 2); end
    hv(r) = mean(dm);
end
St = table(string(algos(:)), arrayfun(@(a) min(Res(a).J),1:nA)', arrayfun(@(a) median(Res(a).J),1:nA)', ...
    arrayfun(@(a) mean(Res(a).J),1:nA)', arrayfun(@(a) std(Res(a).J),1:nA)', arrayfun(@(a) max(Res(a).J),1:nA)', ...
    bKp(:), bKi(:), 'VariableNames', {'Algorithm','J_best','J_median','J_mean','J_std','J_worst','Kp_best','Ki_best'});
fprintf('\n--- Optimiser statistics on hardware (%d runs) ---\n', runs); disp(St)

% repeated-measurement statistics per design
Jrep = nan(nD,cfg.nRep); wOS = Jrep; wTs = Jrep; wTr = Jrep; wIAE = Jrep; mCh = Jrep; trips = zeros(nD,1);
mid = zeros(nD,6);
for d = 1:nD
    for rep = 1:cfg.nRep
        c = CONF{d,rep};  Jrep(d,rep) = c.J;
        if c.trip || ~c.ok, trips(d) = trips(d) + 1; continue; end
        M = c.M;  wIAE(d,rep) = max(M(:,1))*1e3; wOS(d,rep) = max(M(:,2)); wTs(d,rep) = max(M(:,3))*1e3;
        wTr(d,rep) = max(M(:,4))*1e3; mCh(d,rep) = max(M(:,5));
    end
end
for d = 1:nD                       % up/down breakdown of the medium (12<->16 V) step
    acc = nan(cfg.nRep,6);
    for rep = 1:cfg.nRep
        c = CONF{d,rep};
        if ~c.trip && c.ok
            u = c.M(cfg.midRow,:);  w = c.M(cfg.midRow+1,:);
            acc(rep,:) = [u(2) u(4)*1e3 u(3)*1e3 w(2) w(4)*1e3 w(3)*1e3];
        end
    end
    mid(d,:) = nanmean2(acc,1);
end
Kp_se = Kp_all/sim2hw;  Ki_se = Ki_all/sim2hw;
Tconf = table(string(names(:)), grp(:), Kp_all(:), Ki_all(:), Kp_se(:), Ki_se(:), ...
    nanmean2(wOS,2), nanstd2(wOS,2), nanmean2(wTs,2), nanstd2(wTs,2), nanmean2(wTr,2), nanstd2(wTr,2), ...
    nanmean2(mCh,2), nanmean2(Jrep,2), nanstd2(Jrep,2), trips, 'VariableNames', ...
    {'Design','Group','Kp_hw','Ki_hw','Kp_simEq','Ki_simEq','OS_mean_pct','OS_std','Ts_mean_ms','Ts_std', ...
     'Tr_mean_ms','Tr_std','Chatter_pct','J_mean','J_std','Trips'});
fprintf('\n--- Hardware confirmation (%d repeated measurements per design; worst of 6 steps) ---\n', cfg.nRep); disp(Tconf)
Tstep = table(string(names(:)), mid(:,1), mid(:,2), mid(:,3), mid(:,4), mid(:,5), mid(:,6), 'VariableNames', ...
    {'Design','OS_up_pct','Rise_ms','Ts_up_ms','Undershoot_down_pct','Fall_ms','Ts_down_ms'});
fprintf('--- Medium step 12 V <-> 16 V (up = rise, down = fall) ---\n'); disp(Tstep)

[~,ord] = sort(nanmean2(Jrep,2));
Pb = nan(nD,1); Pbest = nan(nD,1);
for d = 1:nD, Pb(d) = rankSumP(Jrep(d,:), Jrep(1,:)); Pbest(d) = rankSumP(Jrep(d,:), Jrep(ord(1),:)); end
Rk = table((1:nD)', string(names(ord)'), Kp_all(ord)', Ki_all(ord)', nanmean2(Jrep(ord,:),2), nanstd2(Jrep(ord,:),2), ...
    Pb(ord), Pbest(ord), 'VariableNames', {'Rank','Design','Kp_hw','Ki_hw','J_mean','J_std','p_vs_baseline','p_vs_best'});
fprintf('\n--- Final ranking on hardware (rank-sum p-values from repeated measurements) ---\n'); disp(Rk)
Kp = Kp_all(ord(1)); Ki = Ki_all(ord(1));
fprintf('>>> BEST ON HARDWARE: %s  ->  KP = %.4f,  KI = %.3f  (sketch units; sim-equivalent Kp = %.5f, Ki = %.3f)\n', ...
    names{ord(1)}, Kp, Ki, Kp/sim2hw, Ki/sim2hw);

dirT = 'Paper_Tables_HW';  if ~exist(dirT,'dir'), mkdir(dirT); end
TT = {Tconf,'Table_HW_Confirmation','Hardware confirmation, mean over repeated measurements.','tab:hwconf'; ...
      Tstep,'Table_HW_MediumStep','Rise and fall response for the 12 V to 16 V step.','tab:hwstep'; ...
      St,'Table_HW_Optimisers','Optimiser statistics on the physical converter.','tab:hwopt'; ...
      Rk,'Table_HW_Ranking','Ranking on hardware.','tab:hwrank'};
for i = 1:size(TT,1)
    writetable(TT{i,1}, fullfile(dirT,[TT{i,2} '.csv']));  table2tex(TT{i,1}, fullfile(dirT,[TT{i,2} '.tex']), TT{i,3}, TT{i,4});
end
save('HW_Results.mat');

%% ============================ 6. PUBLICATION FIGURES ======================
dirF = 'Paper_Figures_HW';
colA = [0.00 0.45 0.74; 0.93 0.69 0.13; 0.49 0.18 0.56; 0.64 0.08 0.18];
nR = nPre - (cfg.testSimDesigns*4);                          % number of reference designs
cols = zeros(nD,3);  cols(1:nR,:) = repmat([0.45 0.45 0.45], nR, 1);  cols(1,:) = [0.1 0.1 0.1];
if cfg.testSimDesigns, cols(nR+1:nR+4,:) = 0.55*colA + 0.45; end
cols(end-3:end,:) = colA;
lsty = repmat({'-'},1,nD);  lsty(grp==1) = {'--'};  lsty(grp==2) = {':'};
short = {'PSO','GA','GWO','NSGA-II'};
iHW = nD-3:nD;  iSim = nR+1:nR+4;

% HW1 convergence
f = newFig('HW1',8.8,6.5); hold on; x = 0:plan.iter;
for a = 1:nA
    m = mean(Res(a).hist,1); s = std(Res(a).hist,0,1);
    fill([x fliplr(x)], [m+s fliplr(m-s)], colA(a,:), 'FaceAlpha',0.15, 'EdgeColor','none', 'HandleVisibility','off');
    plot(x, m, 'Color', colA(a,:));
end
xlabel('Iteration'); ylabel('Hardware worst-case cost J_{hw}'); legend(algos); saveFig(f,'HW1_Convergence',dirF);

% HW2 measured cost landscape + final designs
Ke = cellfun(@(r) r.Kp, EV); Ie = cellfun(@(r) r.Ki, EV); Je = cellfun(@(r) r.J, EV);
f = newFig('HW2',10,7.5); hold on
scatter(Ke, Ie, 16, log10(min(Je,cfg.big)), 'filled'); colormap(parula); cb = colorbar; cb.Label.String = 'log_{10} J_{hw}';
for a = 1:nA, plot(bKp(a), bKi(a), 'p', 'MarkerSize', 12, 'MarkerFaceColor', colA(a,:), 'MarkerEdgeColor','k'); end
plot(dKp(grp(1:nPre)==1), dKi(grp(1:nPre)==1), 's', 'MarkerFaceColor','w','MarkerEdgeColor','k','MarkerSize',6);
if cfg.testSimDesigns, plot(dKp(nR+1:nR+4), dKi(nR+1:nR+4), 'd', 'MarkerFaceColor','w','MarkerEdgeColor','k','MarkerSize',6); end
set(gca,'YScale','log'); xlim(lim.Kp); ylim(lim.Ki); xlabel('K_p (sketch units)'); ylabel('K_i (sketch units)');
legend([{'measured trials'} short {'references'} {'simulation designs'}], 'Location','southeast','FontSize',7);
saveFig(f,'HW2_Measured_Landscape',dirF);

% HW3 Pareto front measured on hardware
[~,rb] = max(hv); fr = Res(4).front{rb}; Fk = fr.F(fr.k,:);
f = newFig('HW3',17.5,6);
subplot(1,3,1); scatter3(fr.F(:,1), fr.F(:,2), fr.F(:,3), 18, 'filled'); hold on
scatter3(Fk(1),Fk(2),Fk(3),90,'p','filled','MarkerFaceColor',colA(4,:)); view(135,25);
xlabel('T_s (norm.)'); ylabel('OS (norm.)'); zlabel('Chatter (norm.)'); title('(a) Pareto front','FontWeight','normal');
subplot(1,3,2); plot(fr.F(:,1), fr.F(:,2), 'o','MarkerSize',4); hold on
plot(Fk(1),Fk(2),'p','MarkerSize',11,'MarkerFaceColor',colA(4,:),'MarkerEdgeColor','k');
xlabel('T_s (norm.)'); ylabel('OS (norm.)'); title('(b) Projection, knee marked','FontWeight','normal');
ax = subplot(1,3,3); drawBox(ax, {hv}, {'NSGA-II'}, colA(4,:)); ylabel('Hypervolume'); title('(c) Over runs','FontWeight','normal');
saveFig(f,'HW3_Pareto_Knee',dirF);

% HW4 measured step responses (medium step, up and down) and duty
sel = [1, iSim(1)*cfg.testSimDesigns + (~cfg.testSimDesigns)*1, iHW];  sel = unique(sel,'stable');
f = newFig('HW4',17.5,10);
for p = 1:4
    subplot(2,2,p); hold on; segIdx = cfg.midRow + (p==2 || p==4);
    for d = sel
        c = CONF{d,1}; if c.trip || ~c.ok, continue; end
        sg = c.segs(segIdx);
        if p <= 2, plot(sg.tt*1e3, sg.y, 'Color', cols(d,:), 'LineStyle', lsty{d}); else, plot(sg.tt*1e3, sg.du, 'Color', cols(d,:), 'LineStyle', lsty{d}); end
    end
    xlabel('Time after step (ms)'); xlim([0 400]);
    if p <= 2, ylabel('V_{out} (V)'); else, ylabel('Duty (%)'); end
    tl4 = {'Rise 12\rightarrow16 V','Fall 16\rightarrow12 V','Duty during rise','Duty during fall'};
    title(sprintf('(%c) %s', 'a'+p-1, tl4{p}),'FontWeight','normal');
end
subplot(2,2,1); legend(names(sel),'Location','southeast','FontSize',6); saveFig(f,'HW4_Measured_Steps',dirF);

% HW5 full measured trace of the best design
cb1 = CONF{ord(1),1};
if ~cb1.trip
    f = newFig('HW5',17.5,8); Dt = cb1.D; tt = (Dt(:,1)-Dt(1,1))/1000;
    subplot(2,1,1); plot(tt, Dt(:,3), 'Color', colA(1,:)); hold on; plot(tt, Dt(:,2), 'k--');
    ylabel('V_{out} (V)'); legend('Measured','Target','Orientation','horizontal','Location','north'); title('(a) Best design on the physical converter','FontWeight','normal');
    subplot(2,1,2); plot(tt, Dt(:,4), 'Color', colA(2,:)); ylabel('Duty (%)'); xlabel('Time (s)'); title('(b) Duty cycle','FontWeight','normal');
    saveFig(f,'HW5_Best_Trace',dirF);
end

% HW6 metrics with error bars (mean +- std over repeated measurements)
f = newFig('HW6',17.5,9);
Mset = {wOS,'Worst overshoot (%)'; wTs,'Worst settling time (ms)'; wTr,'Worst rise/fall time (ms)'; mCh,'Duty chatter (% std)'; Jrep,'Hardware cost J_{hw}'};
for i = 1:5
    subplot(2,3,i); mu = nanmean2(Mset{i,1},2); sd = nanstd2(Mset{i,1},2);
    b = bar(mu); b.FaceColor = 'flat'; b.CData = cols; hold on; errorbar(1:nD, mu, sd, 'k.');
    set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',55,'FontSize',6); ylabel(Mset{i,2}); title(sprintf('(%c)','a'+i-1),'FontWeight','normal');
end
subplot(2,3,6); b = bar(trips); b.FaceColor = 'flat'; b.CData = cols;
set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',55,'FontSize',6); ylabel(sprintf('Trips in %d repeats', cfg.nRep)); title('(f)','FontWeight','normal');
saveFig(f,'HW6_Metrics',dirF);

% HW7 simulation-designed vs hardware-optimised (sim-to-real gap)
if cfg.testSimDesigns
    f = newFig('HW7',8.8,6.5);
    mS = nanmean2(Jrep(iSim,:),2); sS = nanstd2(Jrep(iSim,:),2);  mH = nanmean2(Jrep(iHW,:),2); sH = nanstd2(Jrep(iHW,:),2);
    bar(1:4, [mS mH]); hold on; errorbar((1:4)-0.14, mS, sS, 'k.'); errorbar((1:4)+0.14, mH, sH, 'k.');
    set(gca,'XTick',1:4,'XTickLabel',short); ylabel('Hardware cost J_{hw}'); legend('Designed in simulation','Optimised on hardware','Location','northwest');
    saveFig(f,'HW7_SimToReal_Gap',dirF);
end

% HW8 repeatability
f = newFig('HW8',17.5,6); ax = axes(f); drawBox(ax, arrayfun(@(d) Jrep(d,:), 1:nD, 'UniformOutput', false), names, cols);
set(ax,'XTickLabelRotation',45,'FontSize',7); ylabel('Hardware cost J_{hw} (repeats)'); saveFig(f,'HW8_Repeatability',dirF);

fprintf('\nTotal time: %.1f min. Trips: %d. Figures: %s/, tables: %s/\n', toc(tAll)/60, ST('nTrip'), dirF, dirT);
diary off;

%% ====================================================================
%%                          LOCAL FUNCTIONS
%% ====================================================================
function [ev, jumps, Tend] = buildSched(cfg)
ev = [0 cfg.vBase];  t = cfg.tSettle;  jumps = zeros(0,3);       % jumps = [from to evaluated]
for s = 1:size(cfg.steps,1)
    f = cfg.steps(s,1); to = cfg.steps(s,2);
    if abs(ev(end,2)-f) > 0.01
        ev(end+1,:) = [t f]; jumps(end+1,:) = [ev(end-1,2) f 0]; t = t + cfg.tHold; %#ok<AGROW>
    end
    ev(end+1,:) = [t to]; jumps(end+1,:) = [f to 1]; t = t + cfg.tHold; %#ok<AGROW>
    ev(end+1,:) = [t f];  jumps(end+1,:) = [to f 1]; t = t + cfg.tHold; %#ok<AGROW>
end
Tend = t;
end

function x = z2x(z, lim)
kp = lim.Kp(1) + z(1)*(lim.Kp(2)-lim.Kp(1));
ki = exp(log(lim.Ki(1)) + z(2)*(log(lim.Ki(2))-log(lim.Ki(1))));
x = [round(kp,4) round(ki,3)];
end

function z = x2z(x, lim)
z = [(x(1)-lim.Kp(1))/(lim.Kp(2)-lim.Kp(1)), (log(x(2))-log(lim.Ki(1)))/(log(lim.Ki(2))-log(lim.Ki(1)))];
z = min(max(z,0),1);
end

%% ---------------------------- hardware layer ----------------------------
function H = hwOpen(cfg)
H.sp = [];  H.mock = [];
if cfg.mock
    s = struct('iL',0.4,'vC',9.5,'y',0,'perr',0,'tgt',cfg.vBase,'clk',0);
    H.mock = containers.Map();  H.mock('s') = s;  fprintf('MOCK MODE: emulated circuit, no hardware used.\n');  return
end
H.sp = serialport(cfg.port, cfg.baud);  configureTerminator(H.sp, "LF");
pause(2.5);  flush(H.sp);                                   % Arduino restarts when the port opens
writeline(H.sp,"RESET"); writeline(H.sp,"WDT=1"); writeline(H.sp,"KP=0.1000"); writeline(H.sp,"KI=20.0000"); writeline(H.sp,"VREF=12");
pause(1.5);  buf = ''; got = false; t0 = tic;
while toc(t0) < 3
    [Dn,buf] = readData(H.sp, buf);  if ~isempty(Dn), got = true; break; end;  pause(0.05);
end
if ~got, error('No DATA lines from the Arduino. Check port, baud rate and that the sketch is uploaded.'); end
end

function hwClose(H)
if ~isempty(H.sp)
    try, writeline(H.sp,"WDT=0"); writeline(H.sp,"VREF=5"); writeline(H.sp,"RESET"); catch, end
    try, delete(H.sp); catch, end
end
end

function [Dn, buf] = readData(sp, buf)
Dn = zeros(0,5);  nb = sp.NumBytesAvailable;
if nb > 0
    buf = [buf, read(sp, nb, "char")];
    parts = split(string(buf), newline);  buf = char(parts(end));
    if numel(buf) > 2000, buf = ''; end
    for k = 1:numel(parts)-1
        q = split(strtrim(parts(k)), ",");
        if numel(q) == 6 && q(1) == "DATA"
            v = str2double(q(2:6));  v = v(:)';
            if ~any(isnan(v)), Dn(end+1,:) = v; end %#ok<AGROW>
        end
    end
end
end

function tr = hwTrial(H, cfg, Kp, Ki)
if cfg.mock, [D, trip] = mockTrial(H, cfg, Kp, Ki); tr.D = D; tr.trip = trip; return; end
sp = H.sp;  ev = cfg.ev;  flush(sp);
writeline(sp, sprintf('KP=%.4f', Kp));  writeline(sp, sprintf('KI=%.4f', Ki));
t0 = tic; tData = tic; k = 1; buf = ''; D = zeros(0,5); trip = false;
while true
    te = toc(t0);
    while k <= size(ev,1) && te >= ev(k,1), writeline(sp, sprintf('VREF=%.2f', ev(k,2))); k = k + 1; end
    [Dn, buf] = readData(sp, buf);
    if ~isempty(Dn)
        D = [D; Dn]; tData = tic; %#ok<AGROW>
        if any(Dn(:,3) > cfg.vTrip), trip = true; break; end
    elseif toc(tData) > 2
        error('No telemetry from the Arduino for 2 s (cable / sketch / port).');
    end
    if te >= cfg.Tend, break; end
    pause(0.002);
end
if trip
    writeline(sp,'VREF=5'); writeline(sp,'RESET'); pause(cfg.tripRest);
    fprintf('   !! TRIP (Vout > %.1f V) with Kp=%.4f Ki=%.3f\n', cfg.vTrip, Kp, Ki);
end
tr.D = D;  tr.trip = trip;
end

function [D, trip] = mockTrial(H, cfg, Kp, Ki)
% emulation of the sketch (1 ms incremental PI in ADC counts) on an averaged boost model
s = H.mock('s');  P = cfg.mockPlant;  ev = cfg.ev;  dt = 20e-6;  nsub = 50;
nCtl = round(cfg.Tend/0.001);  D = zeros(0,5);  trip = false;  k = 1;
for n = 1:nCtl
    te = n*0.001;
    while k <= size(ev,1) && te >= ev(k,1), s.tgt = ev(k,2); k = k + 1; end
    adc = min(max(round(s.vC/25*1023 + 0.8*randn), 0), 1023);
    err = s.tgt/25*1023 - adc;
    y = min(max(s.y + Kp*(err - s.perr) + Ki*0.001*err, 0), 200);
    s.y = y; s.perr = err;  oc = floor(y);  d = oc/255;
    for i = 1:nsub
        s.iL = s.iL + dt*(P.Vin - (1-d)*s.vC - P.RL*s.iL)/P.L;  if s.iL < 0, s.iL = 0; end
        s.vC = s.vC + dt*((1-d)*s.iL - s.vC/P.R)/P.C;
    end
    s.clk = s.clk + 1;
    if mod(n,5) == 0
        vout = adc/1023*25;  D(end+1,:) = [s.clk s.tgt vout d*100 oc]; %#ok<AGROW>
        if vout > cfg.vTrip, trip = true; break; end
    end
end
if trip, s.y = 0; s.perr = 0; s.tgt = 5; end
H.mock('s') = s;
end

function rec = hwEval(Kp, Ki, H, cfg, CACHE, ST, forceNew)
key = sprintf('%.4f_%.3f', Kp, Ki);
if ~forceNew && isKey(CACHE, key), rec = CACHE(key); return; end
tw = tic;  tr = hwTrial(H, cfg, Kp, Ki);
rec = struct('Kp',Kp,'Ki',Ki,'J',cfg.big,'F',cfg.big*ones(1,3),'M',[],'segs',[],'trip',tr.trip,'ok',false,'D',tr.D,'wall',0);
if tr.trip
    ST('nTrip') = ST('nTrip') + 1;
else
    [M, segs, ok] = hwMetrics(tr.D, cfg);  rec.M = M; rec.segs = segs; rec.ok = ok;
    if ok, [rec.J, rec.F] = hwAggregate(M, cfg); end
end
rec.wall = toc(tw);
if ~forceNew, ST('nEval') = ST('nEval') + 1; ST('evals') = [ST('evals'), {rec}]; CACHE(key) = rec; end
end

function out = hwObj(z, H, cfg, lim, CACHE, ST, mode)
x = z2x(z, lim);  rec = hwEval(x(1), x(2), H, cfg, CACHE, ST, false);
if mode == 1, out = rec.J; else, out = [rec.F rec.J]; end
end

function [M, segs, ok] = hwMetrics(D, cfg)
% metrics follow analyze_steps.m (10-90 % rise/fall, 2 % settling band) + IAE and duty chatter
M = []; segs = []; ok = false;
if size(D,1) < 50, return; end
t = (D(:,1)-D(1,1))/1000;  tgt = D(:,2);  v = D(:,3);  du = D(:,4);
chg = find(abs(diff(tgt)) > 0.05) + 1;  jumps = cfg.jumps;  nJ = size(jumps,1);
if numel(chg) < nJ, return; end
chg = chg(end-nJ+1:end);
segs = struct('tt',{},'y',{},'du',{},'from',{},'to',{});
for k = 1:nJ
    if abs(tgt(chg(k)) - jumps(k,2)) > 0.06, M = []; segs = []; return; end
    if jumps(k,3) == 0, continue; end
    i1 = chg(k);  if k < nJ, i2 = chg(k+1) - 1; else, i2 = numel(t); end
    if i2 - i1 < 10, M = []; segs = []; return; end
    y = v(i1:i2);  tt = t(i1:i2) - t(i1);  dd = du(i1:i2);
    from = jumps(k,1);  to = jumps(k,2);  ss = abs(to - from);  d = sign(to - from);
    ys = movmedian(y, 3);
    if d > 0, pk = max(ys); else, pk = min(ys); end
    os = max(0, d*(pk - to) - cfg.osDead)/ss*100;      % noise-aware overshoot
    z = d*(ys - from)/ss;  t10 = crossT(tt, z, 0.1);  t90 = crossT(tt, z, 0.9);
    if isnan(t10) || isnan(t90), trr = tt(end); else, trr = t90 - t10; end
    o = find(abs(ys - to) > 0.02*to, 1, 'last');
    if isempty(o), ts = 0; elseif o >= numel(tt), ts = tt(end); else, ts = tt(o+1); end
    iae = trapz(tt, abs(ys - to))/ss;
    n = numel(y);  ch = std(dd(max(1,round(0.7*n)):end));
    nT = max(3, round(0.2*n));  sse = to - mean(y(end-nT+1:end));
    M(end+1,:) = [iae os ts trr ch sse pk]; %#ok<AGROW>
    segs(end+1) = struct('tt',tt,'y',y,'du',dd,'from',from,'to',to); %#ok<AGROW>
end
ok = size(M,1) == sum(jumps(:,3) == 1);
end

function tc = crossT(tt, z, lv)
i = find(z >= lv, 1);
if isempty(i), tc = NaN; elseif i == 1, tc = tt(1);
else, tc = tt(i-1) + (lv - z(i-1))/(z(i) - z(i-1))*(tt(i) - tt(i-1)); end
end

function [J, F] = hwAggregate(M, cfg)
if isempty(M) || any(isnan(M(:))), J = cfg.big; F = cfg.big*ones(1,3); return; end
Js = cfg.w(1)*M(:,1)/cfg.ref(1) + cfg.w(2)*M(:,3)/cfg.ref(2) + cfg.w(3)*M(:,2)/cfg.ref(3) + cfg.w(4)*M(:,5)/cfg.ref(4);
J = max(Js) + 0.2*mean(Js);
F = [max(M(:,3))/cfg.ref(2), max(M(:,2))/cfg.ref(3), max(M(:,5))/cfg.ref(4)];
end

%% ---------------------------- optimisers (z in [0,1]^D, start population Z0) ----
function f = evalPop(fun, Z)
f = zeros(size(Z,1),1);
for i = 1:size(Z,1), f(i) = fun(Z(i,:)); end
end

function [F,J] = evalPopMO(fun, Z, M)
N = size(Z,1); F = zeros(N,M); J = zeros(N,1);
for i = 1:N, o = fun(Z(i,:)); F(i,:) = o(1:M); J(i) = o(M+1); end
end

function [zb,fb,hist] = runPSO(fun, N, T, D, Z)
Vl = zeros(N,D); f = evalPop(fun,Z);
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

function [zb,fb,hist] = runGA(fun, N, T, D, Z)
f = evalPop(fun,Z); nE = 2;
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

function [zb,fb,hist] = runGWO(fun, N, T, D, Z)
f = evalPop(fun,Z);
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

function [Zf,Ff,Jf,hist] = runNSGA2(funMO, N, T, D, M, big, Z)
[F,Jv] = evalPopMO(funMO,Z,M);
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
function m = nanmean2(X, dim)
ok = ~isnan(X);  X(~ok) = 0;  n = sum(ok, dim);  m = sum(X, dim)./n;  m(n == 0) = NaN;
end

function s = nanstd2(X, dim)
m = nanmean2(X, dim);  ok = ~isnan(X);  n = sum(ok, dim);
if dim == 1, Xc = X - m; else, Xc = X - m; end
Xc(~ok) = 0;  s = sqrt(sum(Xc.^2, dim)./max(n-1, 1));  s(n < 2) = 0;
end

function q = pctl(v, p)
v = sort(v(~isnan(v))); n = numel(v);
if n == 0, q = NaN; return; end
pos = 1 + (n-1)*p/100; lo = floor(pos); hi = ceil(pos);
q = v(lo) + (pos-lo)*(v(hi)-v(lo));
end

function p = rankSumP(x, y)
x = x(:); y = y(:); x = x(~isnan(x)); y = y(~isnan(y)); n1 = numel(x); n2 = numel(y);
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

function saveFig(f, name, dirF)
if ~exist(dirF,'dir'), mkdir(dirF); end
p = fullfile(dirF, name);
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