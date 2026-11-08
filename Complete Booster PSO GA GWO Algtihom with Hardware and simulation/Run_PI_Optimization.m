    %% Run_PI_Optimization.m
    % Worst-case robust PI tuning of the boost converter with
    %   PSO | GA | GWO | NSGA-II (knee point)
    % and comparison with pidtune / PID-Tuner-app / slider designs.
    %
    % HOW TO USE
    %   1) Put this file in the same folder as PSO_PI_Controlled_Booster.slx
    %   2) Leave cfg.preset = 'quick' and press Run (first test, few iterations)
    %   3) If all is fine -> 'medium', then 'full' (multiple runs)
    %   4) Optional: cfg.optModel = 'simulink' (switching model inside the loop)
    % Requires: Control System Toolbox (+ Simulink for validation). No other toolbox.
    clear; clc; close all;
    
    %% ============================ 0. USER SETTINGS ============================
    cfg.mdl      = 'PSO_PI_Controlled_Booster';
    cfg.preset   = 'quick';       % 'quick' | 'medium' | 'full'
    cfg.optModel = 'simulink';    % 'analytic' (fast) | 'simulink' (slow)
    cfg.doSimValidation = true;   % validate all final designs in Simulink
    cfg.valScenarios    = 'reduced';  % 'reduced' = nominal + worst corners | 'all'
    cfg.nWorst   = 2;             % worst corners used for 'reduced' / 'simulink' mode
    cfg.seed     = 1;
    
    % nominal circuit (same as your analytic script)
    nom.Vin = 10; nom.D = 0.45; nom.fs = 62.5e3;
    nom.L = 22.5e-6; nom.C = 470e-6; nom.R = 50; nom.RL = 0.1;
    cfg.nom  = nom;
    cfg.Vref = nom.Vin/(1-nom.D);     % 18.18 V
    cfg.Tstop    = 0.2;               % Simulink stop time (validation)
    cfg.TstopOpt = 0.06;               % Simulink stop time inside optimisation loop
    
    % worst-case variation (multiplying factors of the nominal value)
    cfg.var.Vin = [0.8 1.2];          % +-20 % input voltage
    cfg.var.L   = [0.8 1.2];          % +-20 % inductor
    cfg.var.C   = [0.8 1.2];          % +-20 % capacitor
    cfg.var.R   = [0.5 1.5];          % 25 ohm ... 75 ohm load
    cfg.scenarioSet = 'corners';      % 'corners' (2^4+nominal = 17) | 'full' (3^4 = 81)
    
    % cost function:  J = max_s(Js) + 0.2*mean_s(Js) + PM penalty
    cfg.w     = [0.4 0.4 0.2];        % weights of IAE, settling time, overshoot
    cfg.ref   = [0.01 0.03 5];        % normalisation: IAE [s], Ts [s], OS [%]
    cfg.PMmin = 45;                   % minimum phase margin in every scenario [deg]
    cfg.big   = 1e3;                  % penalty for unstable / failed designs
    cfg.Tend  = 0.1;  cfg.N = 500;  % analytic step-response grid
    
    % reference designs (taken from your last output)
    refNames = {'pidtune wc=100','pidtune wc=150','pidtune wc=300','Auto (app)','Slider (app)'};
    Kp_ref   = [0 0 0 0 0.00243];
    Ki_ref   = [3.0443 4.5652 9.1165 48.057 15.314];
    
    % PI search range, derived from the reference designs
    lim.Kp = [0, round(4*max(Kp_ref),1,'significant')];                       % 0 ... 0.01
    lim.Ki = [round(0.5*min(Ki_ref),2,'significant'), ...
              round(2.0*max(Ki_ref),2,'significant')];                        % 1.5 ... 96
    
    % presets: [population  iterations  runs]
    P.quick = [10 8 1];  P.medium = [24 40 3];  P.full = [40 100 10];
    pp = P.(cfg.preset);  cfg.pop = pp(1);  cfg.iter = pp(2);  cfg.nRuns = pp(3);
    fprintf('Preset %s: pop=%d, iter=%d, runs=%d | optModel=%s\n', ...
            cfg.preset, cfg.pop, cfg.iter, cfg.nRuns, cfg.optModel);
    fprintf('Search range: Kp = [%g %g], Ki = [%g %g] (Ki searched on log scale)\n', ...
            lim.Kp, lim.Ki);
    
    %% ============================ 1. SCENARIOS + PLANTS =======================
    scen = buildScenarios(nom, cfg);
    nS = numel(scen);  idxAll = 1:nS;
    idxNom = find(arrayfun(@(s) all(s.fac==1), scen), 1);
    Gn = cell(1,nS); Gd = cell(1,nS);
    for k = 1:nS
        s = scen(k);  Ds = 1 - s.Vin/cfg.Vref;       % duty needed to reach Vref
        a  = s.Vin/(1-Ds)^2;
        b  = s.L/(s.R*(1-Ds)^2);
        c  = s.L*s.C/(1-Ds)^2;
        bL = (s.L/s.R + nom.RL*s.C)/(1-Ds)^2;
        dc = 1 + nom.RL/(s.R*(1-Ds)^2);
        Gn{k} = [-a*b a];   Gd{k} = [c bL dc];       % same model as before
    end
    ctx.cfg = cfg; ctx.scen = scen; ctx.Gn = Gn; ctx.Gd = Gd; ctx.lim = lim;
    ctx.t = linspace(0, cfg.Tend, cfg.N)';
    fprintf('%d scenarios (%s set), nominal index = %d\n', nS, cfg.scenarioSet, idxNom);
    
    if cfg.doSimValidation || strcmp(cfg.optModel,'simulink')
        load_system(cfg.mdl);
        % same names as in your analytic script, in case a block (e.g. PWM 'Fsw') uses them
        assignin('base','fs',nom.fs);   assignin('base','Fsw',nom.fs);
        assignin('base','D',nom.D);     assignin('base','Tstop',cfg.Tstop);
        assignin('base','RL',nom.RL);
    end
    
    % worst corners (judged with the slider design) -> used for 'reduced' modes
    MxRef = evalMx(Kp_ref(5), Ki_ref(5), ctx, idxAll, 'analytic');
    [~,o] = sort(scenJs(MxRef,cfg), 'descend');
    idxWorst = o(1:min(cfg.nWorst,nS));  idxWorst = idxWorst(:)';   % force row vector
    idxWorst(idxWorst==idxNom) = [];
    idxSim = unique([idxNom idxWorst]);
    if strcmp(cfg.optModel,'simulink'), idxOpt = idxSim; else, idxOpt = idxAll; end
    
    funJ  = @(z) objSO(z, ctx, idxOpt, cfg.optModel);
    funMO = @(z) objMO(z, ctx, idxOpt, cfg.optModel);
    
    % timing estimate
    nTest = 10; if strcmp(cfg.optModel,'simulink'), nTest = 2; end
    tic; for i = 1:nTest, funJ(rand(1,2)); end; tEval = toc/nTest;
    fprintf('One cost evaluation = %.3f s -> this run ~ %.1f min, full preset ~ %.1f min\n', tEval, ...
        4*cfg.nRuns*cfg.pop*(cfg.iter+1)*tEval/60, 4*P.full(3)*P.full(1)*(P.full(2)+1)*tEval/60);
    
    %% ============================ 2. OPTIMISATION =============================
    algos = {'PSO','GA','GWO','NSGA-II knee'};  nA = numel(algos);
    Res(nA).J = [];
    for a = 1:nA
        Res(a).z = zeros(cfg.nRuns,2); Res(a).x = zeros(cfg.nRuns,2);
        Res(a).J = zeros(1,cfg.nRuns); Res(a).time = zeros(1,cfg.nRuns);
        Res(a).hist = zeros(cfg.nRuns, cfg.iter+1);
        for r = 1:cfg.nRuns
            rng(cfg.seed + 100*a + r);  tic;
            switch a
                case 1, [z,J,h] = runPSO(funJ, cfg.pop, cfg.iter, 2);
                case 2, [z,J,h] = runGA (funJ, cfg.pop, cfg.iter, 2);
                case 3, [z,J,h] = runGWO(funJ, cfg.pop, cfg.iter, 2);
                case 4
                    [Zf,Ff,Jf,h] = runNSGA2(funMO, cfg.pop, cfg.iter, 2, 3);
                    ok = all(Ff < cfg.big, 2);
                    if any(ok), Zf = Zf(ok,:); Ff = Ff(ok,:); Jf = Jf(ok); end
                    [~,iu] = unique(round(Zf,6), 'rows');
                    Zf = Zf(iu,:); Ff = Ff(iu,:); Jf = Jf(iu);
                    kk = kneeIndex(Ff);  z = Zf(kk,:);  J = Jf(kk);
                    Res(a).front{r} = struct('Z',Zf,'F',Ff,'k',kk);
            end
            x = z2x(z, lim);
            Res(a).z(r,:) = z; Res(a).x(r,:) = x; Res(a).J(r) = J;
            Res(a).hist(r,:) = h; Res(a).time(r) = toc;
            fprintf('%-13s run %2d/%d | J = %.4f | Kp = %.5f | Ki = %.3f | %.1f s\n', ...
                    algos{a}, r, cfg.nRuns, J, x(1), x(2), Res(a).time(r));
        end
    end
    
    %% ============================ 3. STATISTICS ===============================
    bestRun = zeros(1,nA); bKp = zeros(1,nA); bKi = zeros(1,nA);
    for a = 1:nA
        [~,bestRun(a)] = min(Res(a).J);
        bKp(a) = Res(a).x(bestRun(a),1);  bKi(a) = Res(a).x(bestRun(a),2);
    end
    St = table(string(algos(:)), arrayfun(@(a) min(Res(a).J),1:nA)', ...
        arrayfun(@(a) mean(Res(a).J),1:nA)', arrayfun(@(a) std(Res(a).J),1:nA)', ...
        arrayfun(@(a) max(Res(a).J),1:nA)', arrayfun(@(a) mean(Res(a).time),1:nA)', ...
        bKp(:), bKi(:), 'VariableNames', ...
        {'Algorithm','J_best','J_mean','J_std','J_worst','Time_s_per_run','Kp_best','Ki_best'});
    fprintf('\n--- Optimiser statistics over %d run(s) ---\n', cfg.nRuns); disp(St)
    
    %% ============================ 4. ALL DESIGNS, ANALYTIC (all scenarios) ====
    names = [refNames, algos];  nRef = numel(refNames);  nD = numel(names);
    Kp_all = [Kp_ref, bKp];  Ki_all = [Ki_ref, bKi];
    MA = cell(nD,1);  Jan = zeros(nD,1);
    wOS = zeros(nD,1); wTs = wOS; wTr = wOS; wIAE = wOS; mPM = wOS;
    for d = 1:nD
        Mx = evalMx(Kp_all(d), Ki_all(d), ctx, idxAll, 'analytic');  MA{d} = Mx;
        Jan(d) = aggregate(Mx, cfg);
        wIAE(d) = max(Mx(:,1))*1e3;  wOS(d) = max(Mx(:,2));
        wTs(d)  = max(Mx(:,3))*1e3;  wTr(d) = max(Mx(:,4))*1e3;  mPM(d) = min(Mx(:,5));
    end
    T1 = table(string(names(:)), Kp_all(:), Ki_all(:), wOS, wTs, wTr, wIAE, mPM, Jan, ...
        'VariableNames', {'Design','Kp','Ki','WorstOS_pct','WorstTs_ms','WorstTr_ms', ...
        'WorstIAE_ms','MinPM_deg','J_worstcase'});
    fprintf('\n--- Worst case over ALL %d scenarios (analytic model) ---\n', nS); disp(T1)
    
    JS = zeros(nD,nS);
    for d = 1:nD, JS(d,:) = scenJs(MA{d}, cfg)'; end
    [~,iw] = max(mean(JS,1));
    fprintf('Most severe scenario overall: #%d (%s)\n', iw, scen(iw).name);
    
    %% ============================ 5. SIMULINK VALIDATION ======================
    simDone = false;
    if cfg.doSimValidation
        idxVal = idxAll; if strcmp(cfg.valScenarios,'reduced'), idxVal = idxSim; end
        nV = numel(idxVal);  tU = (0:20e-6:cfg.Tstop)';
        simV = cell(nD,1); simD = cell(nD,1); MS = cell(nD,1);
        for d = 1:nD
            simV{d} = nan(numel(tU),nV); simD{d} = simV{d}; Mx = nan(nV,5);
            for q = 1:nV
                k = idxVal(q);
                fprintf('Simulink: %-14s | scenario %2d/%d (%s)\n', names{d}, q, nV, scen(k).name);
                [~,v,du,ok] = runSim(Kp_all(d), Ki_all(d), scen(k), ctx, cfg.Tstop);
                if ~ok, continue; end
                simV{d}(:,q) = v;  simD{d}(:,q) = du;
                Mx(q,:) = simRow(tU, v, cfg.Vref, MA{d}(k,5));
            end
            MS{d} = Mx;
        end
        Jsim = zeros(nD,1); sOS = Jsim; sTs = Jsim; sTr = Jsim; sIAE = Jsim; sSSE = Jsim; sDuty = Jsim;
        for d = 1:nD
            Mx = MS{d};  Jsim(d) = aggregate(Mx, cfg);
            sOS(d) = max(Mx(:,2)); sTs(d) = max(Mx(:,3))*1e3;
            sTr(d) = max(Mx(:,4))*1e3; sIAE(d) = max(Mx(:,1))*1e3;
            sSSE(d) = max(abs(cfg.Vref - mean(simV{d}(tU>0.9*cfg.Tstop,:),1)));
            sDuty(d) = max(simD{d}(:));
        end
        T2 = table(string(names(:)), Kp_all(:), Ki_all(:), sOS, sTs, sTr, sIAE, sSSE, sDuty, Jsim, ...
            'VariableNames', {'Design','Kp','Ki','WorstOS_pct','WorstTs_ms','WorstTr_ms', ...
            'WorstIAE_ms','WorstSteadyErr_V','MaxDuty','J_sim'});
        fprintf('\n--- Simulink validation (%d scenarios) ---\n', nV); disp(T2)
        simDone = true;
    end
    
    %% ============================ 6. RANKING + RECOMMENDATION =================
    if simDone, Jrank = Jsim; src = 'Simulink'; else, Jrank = Jan; src = 'analytic'; end
    [~,ord] = sort(Jrank);
    Rk = table((1:nD)', string(names(ord)'), Kp_all(ord)', Ki_all(ord)', Jrank(ord), ...
        'VariableNames', {'Rank','Design','Kp','Ki','J'});
    fprintf('\n--- Final ranking (%s worst-case cost) ---\n', src); disp(Rk)
    Kp = Kp_all(ord(1));  Ki = Ki_all(ord(1));
    fprintf('\n>>> RECOMMENDED: %s  ->  Kp = %.6f,  Ki = %.4f\n', names{ord(1)}, Kp, Ki);
    
    writetable(T1, 'PI_Opt_Analytic.csv');  writetable(St, 'PI_Opt_Stats.csv');
    writetable(Rk, 'PI_Opt_Ranking.csv');
    if simDone, writetable(T2, 'PI_Opt_Simulink.csv'); end
    save('PI_Opt_Results.mat');
    
    %% ============================ 7. GRAPHS ===================================
    cols = lines(nD);  tms = ctx.t*1e3;
    
    figure('Name','Convergence'); hold on
    for a = 1:nA, plot(0:cfg.iter, mean(Res(a).hist,1), 'LineWidth', 1.8); end
    set(gca,'YScale','log'); grid on; legend(algos); xlabel('Iteration');
    ylabel('Best worst-case cost J'); title('Convergence (mean of runs)')
    
    figure('Name','Run statistics');
    mu = arrayfun(@(a) mean(Res(a).J),1:nA); sd = arrayfun(@(a) std(Res(a).J),1:nA);
    bar(mu); hold on; errorbar(1:nA, mu, sd, 'k.', 'LineWidth', 1.5);
    set(gca,'XTick',1:nA,'XTickLabel',algos); grid on; ylabel('J (mean \pm std)');
    title(sprintf('Final cost over %d run(s)', cfg.nRuns))
    
    figure('Name','Kp-Ki plane'); hold on
    for a = 1:nA, plot(Res(a).x(:,1), Res(a).x(:,2), 'o', 'MarkerSize', 8, 'LineWidth', 1.8); end
    plot(Kp_ref, Ki_ref, 'ks', 'MarkerFaceColor', 'y', 'MarkerSize', 9);
    text(Kp_ref + 0.02*lim.Kp(2), Ki_ref, refNames, 'FontSize', 8);
    set(gca,'YScale','log'); xlim(lim.Kp); ylim(lim.Ki); grid on
    xlabel('Kp'); ylabel('Ki'); legend([algos {'reference designs'}]); title('Solutions in the search range')
    
    figure('Name','NSGA-II Pareto front'); ib = bestRun(4);
    fr = Res(4).front{ib}; Fk = fr.F(fr.k,:);
    subplot(1,2,1); scatter3(fr.F(:,1), fr.F(:,2), fr.F(:,3), 30, 'filled'); hold on
    scatter3(Fk(1), Fk(2), Fk(3), 140, 'rp', 'filled'); grid on
    xlabel('worst IAE (norm)'); ylabel('worst Ts (norm)'); zlabel('worst OS (norm)'); title('Pareto front + knee')
    subplot(1,2,2); plot(fr.F(:,2), fr.F(:,3), 'o'); hold on; plot(Fk(2), Fk(3), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    grid on; xlabel('worst Ts (norm)'); ylabel('worst OS (norm)'); title('Projection'); legend('front','knee')
    
    figure('Name','Analytic step: nominal & worst');
    subplot(1,2,1); hold on
    for d = 1:nD, plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,idxNom), 'Color', cols(d,:), 'LineWidth', 1.4); end
    grid on; ylim([0 1.5]); xlabel('Time (ms)'); ylabel('Vout / Vref'); title('Nominal'); legend(names,'Location','southeast')
    subplot(1,2,2); hold on
    for d = 1:nD, plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,iw), 'Color', cols(d,:), 'LineWidth', 1.4); end
    grid on; ylim([0 1.5]); xlabel('Time (ms)'); title(['Worst: ' scen(iw).name])
    
    figure('Name','Robustness family (analytic)');
    for a = 1:nA
        d = nRef + a; subplot(2,2,a); hold on
        for k = 1:nS, plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,k), 'Color', [0.75 0.75 0.75]); end
        plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,idxNom), 'Color', cols(d,:), 'LineWidth', 2);
        plot(tms([1 end]), [1 1], 'k:'); grid on; ylim([0 1.6]);
        title(sprintf('%s: Kp=%.4f Ki=%.2f', algos{a}, Kp_all(d), Ki_all(d)));
        xlabel('Time (ms)');
    end
    
    figure('Name','Worst-case metrics (analytic)');
    M = {wOS,'Overshoot (%)'; wTs,'Settling time (ms)'; wTr,'Rise time (ms)'; ...
         wIAE,'IAE (ms)'; mPM,'Min phase margin (deg)'; Jan,'J'};
    for i = 1:6
        subplot(2,3,i); bar(M{i,1}); title(M{i,2}); grid on
        set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',40);
    end
    
    figure('Name','Loop Bode (nominal)'); hold on
    for d = 1:nD
        bode(tf(conv([Kp_all(d) Ki_all(d)],Gn{idxNom}), conv([1 0],Gd{idxNom})));
    end
    grid on; legend(names); title('Open loop C*Gp, nominal')
    
    figure('Name','Kp / Ki comparison');
    subplot(1,2,1); bar(Kp_all); title('Kp'); grid on
    set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',40);
    subplot(1,2,2); bar(Ki_all); title('Ki'); grid on
    set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',40);
    
    if simDone
        qNom = find(idxVal == idxNom, 1);
        qW = find(idxVal == iw, 1); if isempty(qW), qW = min(2, nV); end
        figure('Name','Simulink validation');
        subplot(2,2,1); hold on
        for d = 1:nD, plot(tU*1e3, simV{d}(:,qNom), 'Color', cols(d,:), 'LineWidth', 1.3); end
        plot(tU([1 end])*1e3, [cfg.Vref cfg.Vref], 'k--'); grid on
        xlabel('Time (ms)'); ylabel('Vout (V)'); title('Simulink: nominal'); legend([names {'Vref'}],'Location','southeast')
        subplot(2,2,2); hold on
        for d = 1:nD, plot(tU*1e3, simV{d}(:,qW), 'Color', cols(d,:), 'LineWidth', 1.3); end
        plot(tU([1 end])*1e3, [cfg.Vref cfg.Vref], 'k--'); grid on
        xlabel('Time (ms)'); title(['Simulink: ' scen(idxVal(qW)).name])
        subplot(2,2,3); hold on
        for d = 1:nD, plot(tU*1e3, simD{d}(:,qNom), 'Color', cols(d,:), 'LineWidth', 1.3); end
        plot(tU([1 end])*1e3, [0.9 0.9], 'k--'); grid on; xlabel('Time (ms)'); ylabel('Duty'); title('Duty, nominal (limit 0.9)')
        subplot(2,2,4); bar([sOS sTs/10 sTr]); grid on
        set(gca,'XTick',1:nD,'XTickLabel',names,'XTickLabelRotation',40);
        legend('Overshoot (%)','Ts (ms/10)','Tr (ms)'); title('Simulink worst case')
    
        figure('Name','Analytic vs Simulink (nominal)');
        for d = 1:nD
            subplot(3,3,d); hold on
            plot(tU*1e3, simV{d}(:,qNom)/cfg.Vref, 'Color', cols(d,:), 'LineWidth', 1.4);
            plot(tms, stepAna(Kp_all(d),Ki_all(d),ctx,idxNom), 'k--');
            grid on; title(names{d}); xlim([0 100]); ylim([0 1.5]);
            if d == 1, legend('Simulink','Analytic','Location','southeast'); end
        end
    end
    fprintf('\nDone. Results saved in PI_Opt_Results.mat and CSV files.\n');
    
    %% ====================================================================
    %%                          LOCAL FUNCTIONS
    %% ====================================================================
    function scen = buildScenarios(nom, cfg)
    v  = cfg.var;  lv = {v.Vin, v.L, v.C, v.R};
    if strcmp(cfg.scenarioSet,'full')
        lv = cellfun(@(x) [x(1) 1 x(2)], lv, 'UniformOutput', false);
    end
    [g1,g2,g3,g4] = ndgrid(lv{:});
    fac = [g1(:) g2(:) g3(:) g4(:)];
    if ~strcmp(cfg.scenarioSet,'full'), fac = [ones(1,4); fac]; end
    scen = struct('name',{},'Vin',{},'L',{},'C',{},'R',{},'fac',{});
    for k = 1:size(fac,1)
        f = fac(k,:);
        scen(k).Vin = nom.Vin*f(1);  scen(k).L = nom.L*f(2);
        scen(k).C   = nom.C*f(3);    scen(k).R = nom.R*f(4);
        scen(k).fac = f;
        scen(k).name = sprintf('Vin%.0f%% L%.0f%% C%.0f%% R%.0f%%', 100*f);
    end
    end
    
    function x = z2x(z, lim)      % z in [0,1]^2 -> [Kp Ki]  (Ki on log scale)
    x(1) = lim.Kp(1) + z(1)*(lim.Kp(2)-lim.Kp(1));
    x(2) = exp(log(lim.Ki(1)) + z(2)*(log(lim.Ki(2))-log(lim.Ki(1))));
    end
    
    function J = objSO(z, ctx, idx, model)
    x = z2x(z, ctx.lim);
    J = aggregate(evalMx(x(1), x(2), ctx, idx, model), ctx.cfg);
    end
    
    function o = objMO(z, ctx, idx, model)
    x = z2x(z, ctx.lim);
    [J,F] = aggregate(evalMx(x(1), x(2), ctx, idx, model), ctx.cfg);
    o = [F J];
    end
    
    function Mx = evalMx(Kp, Ki, ctx, idx, model)
    % rows = scenarios, columns = [IAE OS% Ts Tr PM]
    n = numel(idx);  Mx = nan(n,5);
    for q = 1:n
        k = idx(q);
        numL = conv([Kp Ki], ctx.Gn{k});  denL = conv([1 0], ctx.Gd{k});
        denT = denL + [zeros(1, numel(denL)-numel(numL)), numL];
        if any(real(roots(denT)) >= 0), continue; end        % unstable -> NaN row
        pm = loopPM(numL, denL);
        if strcmp(model,'simulink')
            [tU,v,~,ok] = runSim(Kp, Ki, ctx.scen(k), ctx, ctx.cfg.TstopOpt);
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
        % variables the model may read from the base workspace (the script starts with 'clear')
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
        [~,w] = sort(fc,'descend'); Cc(w(1:nE),:) = E; fc(w(1:nE)) = fE;   % elitism
        Z = Cc; f = fc;
        [fm,i] = min(f); if fm < fb, fb = fm; zb = Z(i,:); end
        hist(it+1) = fb;
    end
    end
    
    function [zb,fb,hist] = runGWO(fun, N, T, D)
    Z = rand(N,D); f = evalPop(fun,Z);
    [f,o] = sort(f); Z = Z(o,:);
    Ld = Z(1:3,:); fL = f(1:3);                      % alpha, beta, delta
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
    
    function [Zf,Ff,Jf,hist] = runNSGA2(funMO, N, T, D, M)
    Z = rand(N,D); [F,Jv] = evalPopMO(funMO,Z,M);
    hist = zeros(1,T+1); Jbest = min(Jv); hist(1) = Jbest;
    [rk,cd] = rankCrowd(F);
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
        Jbest = min(Jbest, min(Jc)); hist(it+1) = Jbest;
    end
    f1 = rk == 1;  Zf = Z(f1,:);  Ff = F(f1,:);  Jf = Jv(f1);
    end
    
    function [rank,crowd] = rankCrowd(F)
    N = size(F,1); rank = zeros(N,1); crowd = zeros(N,1);
    dom = false(N,N);                                 % dom(i,j): i dominates j
    for i = 1:N
        dom(i,:) = (all(F(i,:) <= F, 2) & any(F(i,:) < F, 2))';
    end
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
        n = A\ones(M,1);
        d = (1 - Fn*n)/norm(n);
    else
        d = -sqrt(sum(Fn.^2,2));                      % fallback: closest to the ideal point
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