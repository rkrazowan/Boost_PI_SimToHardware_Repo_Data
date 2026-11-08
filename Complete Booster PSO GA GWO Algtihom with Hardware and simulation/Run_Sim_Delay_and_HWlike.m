%% Run_Sim_Delay_and_HWlike.m
% TWO quick Simulink checks for the paper (no optimisation, only a few runs).
%   PART A  Does a 1 ms sampled PI (like the Arduino) make the scaled simulation gains worse?
%           (start-up to 18.18 V, Vin = 10 V, continuous PI versus discrete PI, Ts = 1 ms)
%   PART B  Hardware-like test: Vin = 7.2 V, reference 8 -> 12 -> 15 -> 12 -> 8 V, discrete PI 1 ms.
%           Gains are the SAME designs that you measured on the bench.
% HOW TO USE
%   1) Put this file in the folder that contains PSO_PI_Controlled_Booster.slx.  Run it (F5).
%   2) Your original model is NOT changed: a copy "Boost_HWlike_copy.slx" is created and used.
%   3) Copy the text printed in the Command Window (the two tables) and paste it to me.
%      Also send Sim_Check_Results.csv and Sim_Check_PartB.png
%   4) If it is too slow, set cfg.quick = true (fewer designs, shorter holds).
clear; clc; close all;
cfg.mdl   = 'PSO_PI_Controlled_Booster';
cfg.copy  = 'Boost_HWlike_copy';
cfg.skipA = true;             % true = skip PART A (you already have its result)
cfg.quick = false;            % true = 3 designs, hold 0.15 s (about 3x faster)
cfg.hold  = 0.25;             % s, time spent at every reference level in PART B
cfg.Ts    = 1e-3;             % s, Arduino control period
Sc = 255*25/1023;              % simulation gain -> Arduino gain (same factor as in the paper)
nom.Vin = 10; nom.D = 0.45; nom.fs = 62.5e3; nom.L = 22.5e-6; nom.C = 470e-6; nom.R = 50; nom.RL = 0.1;
nom.Vref = nom.Vin/(1-nom.D);                 % 18.18 V

% ---- designs in SIMULATION units [Kp Ki]  (hardware gains / Sc for the bench designs)
Des = {
 'Sim-GWO scaled'     [0.016139964  44.9641732]
 'Sim NSGA-II scaled' [0.0196344953 46.3597243]
 'Sim slider scaled'  [0.00243      15.314]
 'Sim pidtune300'     [0.0          9.1165]
 'Sim-GWO, Ki/3'      [0.016139964  44.9641732/3]
 'Sim-GWO, Ki/6'      [0.016139964  44.9641732/6]
 'HW GA (bench best)' [0.0601286462 48.2391711951]/Sc
 'HW baseline'        [0.1 20]/Sc
 };
if cfg.quick, Des = Des([1 5 7],:); cfg.hold = 0.15; end
nD = size(Des,1);

%% ---- prepare model copy (original untouched)
% clean up anything left open by an earlier (failed) run
if bdIsLoaded(cfg.copy), close_system(cfg.copy,0); end
if bdIsLoaded(cfg.mdl),  close_system(cfg.mdl,0);  end
if exist([cfg.copy '.slx'],'file'), delete([cfg.copy '.slx']); end
load_system(cfg.mdl);
save_system(cfg.mdl, cfg.copy);                % now the loaded model is the copy
mdl = cfg.copy;
cleanMdl = onCleanup(@() cleanupModel(cfg.copy));   %#ok<NASGU>
pid = find_system(mdl,'LookUnderMasks','none','MaskType','PID 1dof');
if isempty(pid), pid = find_system(mdl,'LookUnderMasks','none','BlockType','PID'); end
if isempty(pid)      % last try: any block whose name contains PID or PI
    all_b = find_system(mdl,'LookUnderMasks','none','Type','Block');
    pid = all_b(contains(all_b,'PID') | contains(all_b,'PI('));
end
if isempty(pid)
    disp('Blocks found in the model:'); disp(find_system(mdl,'LookUnderMasks','none','Type','Block'));
    error('PI block not found. Send me the list printed above.');
end
pid = pid{1};
fprintf('PI block used: %s\n', pid);

%% ================= PART A : delay check (start-up, Vin = 10 V) ==============
fprintf('\n=== PART A: continuous PI versus discrete PI (Ts = 1 ms), start-up to %.2f V ===\n', nom.Vref);
A = nan(nD,4);
for d = 1:nD*(~cfg.skipA)
    for m = 1:2
        if m == 1, set_param(pid,'TimeDomain','Continuous-time');
        else,      set_param(pid,'TimeDomain','Discrete-time','SampleTime',num2str(cfg.Ts)); end
        [t,v,du] = runSim(mdl, Des{d,2}(1), Des{d,2}(2), nom.Vin, nom.Vref, nom, 0.12);
        [os,ts] = startMetrics(t, v, nom.Vref);
        A(d,2*m-1:2*m) = [os ts*1e3];
    end
    fprintf('%-20s  continuous: OS %6.2f %%  Ts %6.1f ms | discrete 1 ms: OS %6.2f %%  Ts %6.1f ms\n', Des{d,1}, A(d,:));
end

%% ================= PART B : hardware-like steps (Vin = 7.2 V) ===============
levels = [8 12 15 12 8];
set_param(pid,'TimeDomain','Discrete-time','SampleTime',num2str(cfg.Ts));
% replace the constant reference by a staircase read from the workspace (only in the copy)
refTS = [(0:numel(levels)-1)'*cfg.hold, levels(:)];          % [time, value] pairs, zero-order hold
assignin('base','refTS',refTS);
load_system('simulink');
blk = [mdl '/Constant'];
try
    replace_block(mdl,'Name','Constant','FromWorkspace','noprompt');
catch
    % fallback: delete the Constant block and add a From Workspace block on the same wire
    pos = get_param(blk,'Position');  ph = get_param(blk,'PortHandles');
    lh  = get_param(ph.Outport(1),'Line');  dst = get_param(lh,'DstPortHandle');
    delete_line(lh);  delete_block(blk);
    add_block('simulink/Sources/From Workspace', blk, 'Position', pos);
    nph = get_param(blk,'PortHandles');
    for q = 1:numel(dst), add_line(mdl, nph.Outport(1), dst(q)); end
end
set_param([mdl '/Constant'],'VariableName','refTS','Interpolate','off', ...
    'OutputAfterFinalValue','Holding final value','SampleTime','0');
Tstop = numel(levels)*cfg.hold;
fprintf('\n=== PART B: Vin = 7.2 V, levels %s, hold %.2f s, discrete PI 1 ms ===\n', mat2str(levels), cfg.hold);
B = nan(nD,6); TR = cell(nD,1);
for d = 1:nD
    t1 = tic;
    [t,v,du] = runSim(mdl, Des{d,2}(1), Des{d,2}(2), 7.2, levels(1), nom, Tstop);
    TR{d} = {t,v};
    [os,tr,ts,ess,J] = stepMetricsB(t, v, levels, cfg.hold);
    B(d,:) = [max(os) max(tr)*1e3 max(ts)*1e3 max(ess) J Des{d,2}(2)*Sc];
    fprintf('%-20s Kp_hw %.4f Ki_hw %7.1f | OS %5.2f %%  Tr %6.1f ms  Ts %6.1f ms  ess %4.2f %%  J %.3f   (%.0f s)\n', ...
        Des{d,1}, Des{d,2}(1)*Sc, Des{d,2}(2)*Sc, B(d,1:5), toc(t1));
end

%% ---- save
T = table(Des(:,1), A(:,1), A(:,2), A(:,3), A(:,4), B(:,1), B(:,2), B(:,3), B(:,4), B(:,5), ...
  'VariableNames', {'Design','A_OS_cont','A_Ts_cont_ms','A_OS_disc','A_Ts_disc_ms','B_OS','B_Tr_ms','B_Ts_ms','B_ess','B_J'});
writetable(T,'Sim_Check_Results.csv'); disp(T);
f = figure('Color','w','Position',[100 100 900 450]); hold on
for d = 1:nD, plot(TR{d}{1}, TR{d}{2}, 'LineWidth',1.3); end
yline(levels(2:end),':'); xlabel('Time (s)'); ylabel('V_{out} (V)'); legend(Des(:,1),'Location','best'); grid on
title('PART B: hardware-like simulation (Vin = 7.2 V, discrete PI 1 ms)');
exportgraphics(f,'Sim_Check_PartB.png','Resolution',200);
close_system(mdl,0);
disp('DONE. Send me the two tables printed above, Sim_Check_Results.csv and Sim_Check_PartB.png');

%% ======================= local functions ===================================
function [t,v,du] = runSim(mdl, Kp, Ki, Vin, Vref, nom, Tstop)
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(Tstop));
in = in.setVariable('Kp',Kp);  in = in.setVariable('Ki',Ki);
in = in.setVariable('Vin',Vin); in = in.setVariable('L',nom.L); in = in.setVariable('RL',nom.RL);
in = in.setVariable('C',nom.C); in = in.setVariable('R',nom.R);
in = in.setVariable('Vref',Vref); in = in.setVariable('Vout',Vref);
in = in.setVariable('fs',nom.fs); in = in.setVariable('Fsw',nom.fs); in = in.setVariable('fsw',nom.fs);
in = in.setVariable('D',nom.D); in = in.setVariable('Tstop',Tstop);
out = sim(in);
[t,v]  = getSig(out,'Vout_sim');  [tD,du] = getSig(out,'Duty_sim');
[t,iu] = unique(t); v = v(iu);
tU = (0:20e-6:Tstop)';  v = movmean(interp1(t,v,tU,'linear','extrap'),5);  t = tU;
du = interp1(tD,du,tU,'linear','extrap');
end

function [t,v] = getSig(out,nameSig)
s = out.get(nameSig);
if isa(s,'timeseries'), t = s.Time; v = squeeze(s.Data);
elseif isstruct(s),     t = s.time; v = squeeze(s.signals.values);
else,                   t = out.tout; v = squeeze(s); end
t = t(:); v = v(:);
end

function [os,ts] = startMetrics(t,v,Vref)      % start-up metrics, same idea as the paper
v0 = v(1); if v0 > 0.5*Vref, v0 = 0; end
y = (v - v0)/(Vref - v0);
os = max(0,(max(y)-1)*100);
o = find(abs(1-y) > 0.02, 1, 'last');
if isempty(o), ts = 0; elseif o == numel(t), ts = t(end); else, ts = t(o+1); end
end

function [os,tr,ts,ess,J] = stepMetricsB(t,v,levels,hold)
n = numel(levels)-1; os = nan(n,1); tr = os; ts = os; ess = os; Js = os;
for k = 1:n
    i0 = find(t >= k*hold,1);  i1 = find(t >= (k+1)*hold,1); if isempty(i1), i1 = numel(t); end
    ip = find(t >= k*hold - 0.2*hold & t < k*hold);
    y0 = mean(v(ip));  stp = levels(k+1) - y0;
    tt = t(i0:i1) - t(i0);  y = (v(i0:i1) - y0)/stp;
    last = tt > tt(end) - 0.2*hold;
    os(k)  = max(0, max(y) - mean(y(last)))*100;
    a = find(y >= 0.1,1); b = find(y >= 0.9,1);
    if isempty(a)||isempty(b), tr(k) = tt(end); else, tr(k) = tt(b)-tt(a); end
    o = find(abs(1-y) > 0.02,1,'last');
    if isempty(o), ts(k) = 0; elseif o == numel(tt), ts(k) = tt(end); else, ts(k) = tt(o+1); end
    ess(k) = abs(1-mean(y(last)))*100;
    Js(k)  = 0.3*trapz(tt,abs(1-y))/0.05 + 0.3*ts(k)/0.2 + 0.4*os(k)/5;   % same cost as the hardware (no noise penalties)
end
J = max(Js) + 0.2*mean(Js);
end

function cleanupModel(name)
if bdIsLoaded(name), close_system(name,0); end
end