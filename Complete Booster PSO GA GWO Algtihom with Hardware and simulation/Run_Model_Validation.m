%% Run_Model_Validation.m
% Saves the TIME TRACES of the switching-level Simulink model for small reference steps, so that the
% CCM averaged model, a DCM-aware model and the switching model can be compared (model-validation figure).
%   CONFIG 1: Vin = 10 V, continuous PI (the simulation of the paper), reference 16 V -> 18.18 V
%             (a small step from a settled operating point, not a start-up from zero)
%   CONFIG 2: Vin = 7.2 V, sampled PI Ts = 1 ms, duty limit 200/255 (as the firmware), reference 8 -> 12 -> 15 -> 12 -> 8 V
% HOW TO USE: same as before. Put it next to PSO_PI_Controlled_Booster.slx, type  bdclose all , run (F5), about 8 min.
% Send me  Model_Validation.mat  (and the text printed at the end).
clear; clc; close all;
cfg.mdl = 'PSO_PI_Controlled_Booster'; cfg.copy = 'Boost_HWlike_copy'; cfg.Ts = 1e-3; cfg.hold1 = 0.12; cfg.hold2 = 0.25;
Sc = 255*25/1023;
nom.Vin = 10; nom.D = 0.45; nom.fs = 62.5e3; nom.L = 22.5e-6; nom.C = 470e-6; nom.R = 50; nom.RL = 0.1; nom.Vref = 18.18;
Des = {
 'Sim-GWO scaled'     [0.016139964  44.9641732]
 'Sim slider scaled'  [0.00243      15.314]
 'Sim pidtune300'     [0.0          9.1165]
 'Sim pidtune100'     [0.0          3.0443]
 'HW GA (bench best)' [0.0601286462 48.2391711951]/Sc
 'HW baseline'        [0.1 20]/Sc
 };
nD = size(Des,1);
if bdIsLoaded(cfg.copy), close_system(cfg.copy,0); end
if bdIsLoaded(cfg.mdl),  close_system(cfg.mdl,0);  end
if exist([cfg.copy '.slx'],'file'), delete([cfg.copy '.slx']); end
load_system(cfg.mdl); save_system(cfg.mdl, cfg.copy); mdl = cfg.copy;
cleanMdl = onCleanup(@() cleanupModel(cfg.copy));   %#ok<NASGU>
pid = find_system(mdl,'LookUnderMasks','none','MaskType','PID 1dof');
if isempty(pid), pid = find_system(mdl,'LookUnderMasks','none','BlockType','PID'); end
pid = pid{1}; fprintf('PI block used: %s\n', pid);
% reference block -> From Workspace (only in the copy)
load_system('simulink'); blk = [mdl '/Constant'];
try
    replace_block(mdl,'Name','Constant','FromWorkspace','noprompt');
catch
    pos = get_param(blk,'Position');  ph = get_param(blk,'PortHandles');
    lh  = get_param(ph.Outport(1),'Line');  dst = get_param(lh,'DstPortHandle');
    delete_line(lh);  delete_block(blk);
    add_block('simulink/Sources/From Workspace', blk, 'Position', pos);
    nph = get_param(blk,'PortHandles');
    for q = 1:numel(dst), add_line(mdl, nph.Outport(1), dst(q)); end
end
set_param(blk,'VariableName','refTS','Interpolate','off','OutputAfterFinalValue','Holding final value','SampleTime','0');

%% CONFIG 1 : continuous PI, Vin = 10 V, small step 16 -> 18.18 V
lev1 = [16 18.18]; assignin('base','refTS',[(0:1)'*cfg.hold1, lev1(:)]);
set_param(pid,'TimeDomain','Continuous-time','UpperSaturationLimit','0.9');
V1 = cell(nD,1); T1 = [];
for d = 1:nD
    [t,v,du] = runSim(mdl, Des{d,2}(1), Des{d,2}(2), 10, 18.18, nom, 2*cfg.hold1);
    V1{d} = v(1:5:end); T1 = t(1:5:end);
    fprintf('CONFIG 1  %-20s done\n', Des{d,1});
end
%% CONFIG 2 : sampled PI (1 ms), Vin = 7.2 V, duty limit as the firmware, bench reference sequence
lev2 = [8 12 15 12 8]; assignin('base','refTS',[(0:4)'*cfg.hold2, lev2(:)]);
set_param(pid,'TimeDomain','Discrete-time','SampleTime',num2str(cfg.Ts),'UpperSaturationLimit',num2str(200/255));
V2 = cell(nD,1); T2 = [];
for d = 1:nD
    [t,v,du] = runSim(mdl, Des{d,2}(1), Des{d,2}(2), 7.2, 8, nom, 5*cfg.hold2);
    V2{d} = v(1:5:end); T2 = t(1:5:end);
    fprintf('CONFIG 2  %-20s done\n', Des{d,1});
end
names = Des(:,1); gains_sim = cell2mat(Des(:,2));
save('Model_Validation.mat','T1','V1','T2','V2','names','gains_sim','lev1','lev2','cfg','nom');
disp('DONE. Send me Model_Validation.mat');

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
