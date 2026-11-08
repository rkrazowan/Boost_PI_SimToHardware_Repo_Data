%% Run_HW_Retest.m
% Quick hardware re-test (about 5 minutes): the scaled SIMULATION gains + derated versions,
% measured with EXACTLY the same functions as your Run_PI_Hardware_Optimization_1.m
% (the functions below were copied from that file without change).
% HOW TO USE
%   1) Arduino sketch unchanged. Close the Serial Monitor. Set cfg.PORT below (serialportlist).
%   2) Run (F5). It first does the 3 baseline sanity tests, then measures every design
%      cfg.nRep times, in random order (so slow drift of the supply or load does not bias one design).
%   3) Copy the table printed at the end and paste it to me. Also send HW_Retest.mat and HW_Retest.csv.
%   4) EMERGENCY STOP: Ctrl+C, then type   clear cleanupObj
% SAFETY: the test aborts above cfg.Vabort (17.5 V). Check your capacitor rating before the run.
clear; clc; close all;
global HWLOG HWN %#ok<GVMIS>
diary off; if exist('HW_Retest_Log.txt','file'), try, delete('HW_Retest_Log.txt'); catch, end, end
diary('HW_Retest_Log.txt');
%% ============================ 0. USER SETTINGS ============================
cfg.PORT      = "COM10";        % <<< your Arduino port (type serialportlist)
cfg.BAUD      = 115200;
cfg.quickTest = false;          % <<< true = 15 min check, false = full run
cfg.resume    = false;          % <<< true = continue an interrupted run (same settings!)
cfg.budgetMin = 180;             % <<< TOTAL wall-clock budget [min]  (SET YOUR OWN)

% settings that give the MAXIMUM number of iterations for the budget
cfg.popTarget = 6; cfg.popMin = 5;         % population size
cfg.runsTarget = 3; cfg.runsMin = 3;       % independent runs per algorithm (3 = enough for mean/std)
cfg.iterMin   = 10;                        % minimum iterations
cfg.nRe       = 2;              % hardware re-evaluations of every run's best design
cfg.nRepVal   = 3;              % final validation repeats per design
cfg.seed      = 2026;

% hardware test: output-voltage levels [V]. Optimisation uses up-steps, validation also down-steps.
cfg.optLevels = [8 12 15];          % 8->12 and 12->15
cfg.valLevels = [8 12 15 12 8];     % 8->12, 12->15, 15->12, 12->8
cfg.tSettle = 0.5;   % s, wait at first level before measuring (was 0.7, shorter = faster)
cfg.tPre    = 0.25;  % s, pre-step window
cfg.tRec    = 0.8;   % s, recorded time after every step (was 1.0). Ts is capped at this value.
cfg.dtGrid  = 0.005; % s, common time grid for averaged validation traces
cfg.Vabort  = 17.5;  % V, abort test and go to safe gains above this (check your capacitor rating!)
cfg.safeKp  = 0.1;   cfg.safeKi = 20;      % known-good gains used for recovery
cfg.restSec = 4;     % s, cool-down after every optimiser run (load resistors!)

% cost  J = max_s(Js) + 0.2*mean_s(Js)  (+ penalties for steady error > 2 %, ripple > 5 %)
cfg.w   = [0.3 0.3 0.4];     % weights: IAE, settling time, overshoot
cfg.ref = [0.05 0.2 5];      % normalisation: IAE [s], Ts [s], OS [%]
cfg.big = 1e3;               % penalty for failed / aborted tests
cfg.tolPct = 2;              % settling band [%] (widened automatically by the measured noise)

% PI search range  (Kp linear, Ki log).  Units = duty counts per ADC count, as in the Arduino code
lim.Kp = [0 0.5];   lim.Ki = [2 200];    % (Arduino accepts Kp<=5, Ki<=500)

% reference designs measured on the hardware
refNames = {'Baseline (0.1, 20)','Ref (0.05, 10)'};
Kp_ref   = [0.1 0.05];
Ki_ref   = [20  10];

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
tEval = mean(tE); J0 = J;
figure('Name','Sanity','Color','w'); hold on
for k = 1:numel(trs), if ~isempty(trs{k}), plot(trs{k}.t, trs{k}.y); end, end
xlabel('Time after step (s)'); ylabel('V_{out} (V)'); title('Sanity test (last run)'); drawnow;


%% ============================ 2. DESIGNS TO TEST (Arduino units) ==========
Sc = 255*25/1023;                         % simulation gain -> Arduino gain
cfg.nRep = 5;                             % repeats per design
Des = {
 'Sim-GWO scaled',        0.100578954545455, 280.201959090909     % best simulation design, scaled
 'Sim NSGA-II scaled',    0.122355725843108, 288.898575183284
 'Sim slider scaled',     0.0151429618768328, 95.4318181818182
 'Sim pidtune300 scaled', 0.0,               56.8110337243402
 'Sim-GWO, Ki/3',         0.100578954545455, 280.201959090909/3   % derated: same Kp, integral gain divided by 3
 'Sim-GWO, Ki/6',         0.100578954545455, 280.201959090909/6   % derated: integral gain divided by 6
 'HW GA (control)',       0.0601286462,      48.2391711951        % best hardware design (control)
 'Baseline (control)',    0.1,               20.0                 % baseline gains (control)
 };
nD = size(Des,1);  levels = cfg.valLevels;  nS = numel(levels)-1;
assert(all([Des{:,3}] <= 500) && all([Des{:,2}] <= 5), 'Gains exceed Arduino limits');
R = nan(nD, cfg.nRep, 6);   % [J maxOS maxTr(ms) maxTs(ms) maxEss maxRipple]
TRC = cell(nD, cfg.nRep);
fprintf('\n--- Measuring %d designs x %d repeats (about %.1f min) ---\n', nD, cfg.nRep, nD*cfg.nRep*(cfg.tSettle+cfg.tPre+nS*cfg.tRec+0.6)/60);
for rep = 1:cfg.nRep
    for d = randperm(nD)
        [J,~,M,trs] = hwEval(hw, Des{d,2}, Des{d,3}, levels, [0 0]);
        R(d,rep,:) = [J max(M(:,2)) max(M(:,4))*1e3 max(M(:,3))*1e3 max(M(:,5)) max(M(:,6))];
        TRC{d,rep} = trs;
        fprintf('rep %d  %-22s J = %6.3f | OS %5.1f %% | Tr %6.1f ms | ess %4.2f %%\n', rep, Des{d,1}, J, R(d,rep,2), R(d,rep,3), R(d,rep,5));
    end
end

%% ============================ 3. SUMMARY ==================================
Jm = mean(R(:,:,1),2,'omitnan'); Js = std(R(:,:,1),0,2,'omitnan');
T = table(Des(:,1), cell2mat(Des(:,2)), cell2mat(Des(:,3)), Jm, Js, ...
    mean(R(:,:,2),2,'omitnan'), std(R(:,:,2),0,2,'omitnan'), mean(R(:,:,3),2,'omitnan'), ...
    mean(R(:,:,4),2,'omitnan'), mean(R(:,:,5),2,'omitnan'), mean(R(:,:,6),2,'omitnan'), sum(R(:,:,1) >= cfg.big,2), ...
    'VariableNames', {'Design','Kp','Ki','J_mean','J_sd','OS_mean','OS_sd','Tr_ms','Ts_ms','ess_pct','ripple_pct','n_failed'});
fprintf('\n===== RESULT TABLE (copy everything from here) =====\n'); disp(T);
fprintf('Sanity J of baseline before the test: %.3f\n', J0);
writetable(T,'HW_Retest.csv'); save('HW_Retest.mat','T','R','Des','cfg','TRC');
f = figure('Color','w','Position',[100 100 950 420]);
subplot(1,2,1); bar(Jm); hold on; errorbar(1:nD, Jm, Js, 'k.'); set(gca,'XTick',1:nD,'XTickLabel',Des(:,1),'XTickLabelRotation',45); ylabel('Cost J'); grid on
subplot(1,2,2); hold on
for d = 1:nD, if ~isempty(TRC{d,1}) && ~isempty(TRC{d,1}{1}), plot(TRC{d,1}{1}.t, TRC{d,1}{1}.y); end, end
xlabel('Time after step (s)'); ylabel('V_{out} (V), first step'); legend(Des(:,1),'Location','southeast'); grid on
saveas(f,'HW_Retest.png');
hwShutdown(s); disp('DONE. Send me the table above, HW_Retest.csv, HW_Retest.mat and HW_Retest.png');

%% ====================================================================
%%                          LOCAL FUNCTIONS (copied from your script)
%% ====================================================================
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

