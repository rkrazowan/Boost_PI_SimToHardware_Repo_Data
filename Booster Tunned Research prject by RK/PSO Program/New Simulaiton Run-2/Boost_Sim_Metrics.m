function [M, MG] = Boost_Sim_Metrics(x, cfg, idx)
% x=[Kp Ki]; idx = scenario indices (default: all).
% cfg.fast=true  -> averaged small-signal model (milliseconds, no Simulink)
% cfg.fast=false -> switching Simulink model
% M  (n x 4): [ITAE_norm, Overshoot_pu, Ts_norm, DutyEffort] (NaN = failed)
% MG (n x 2): [PhaseMargin_deg, GainMargin_dB]
if nargin < 3, idx = 1:size(cfg.S,1); end
n = numel(idx);  M = NaN(n,4);  MG = NaN(n,2);

for k = 1:n
    s = cfg.S(idx(k),:);
    MG(k,:) = margins_one(x, cfg, s);
    if cfg.fast
        M(k,:) = fast_one(x, cfg, s);
    else
        in = Simulink.SimulationInput(cfg.model);
        in = in.setVariable('Kp',  x(1));
        in = in.setVariable('Ki',  x(2));
        in = in.setVariable('Vin', cfg.Vin*s(1));
        in = in.setVariable('R',   cfg.R*s(2));
        in = in.setVariable('L',   cfg.L*s(3));
        in = in.setVariable('C',   cfg.C*s(4));
        in = in.setVariable('Vout',cfg.Vout);
        in = in.setVariable('Vref',cfg.Vout);
        in = in.setVariable('fs',  cfg.fs);
        in = in.setModelParameter('StopTime', num2str(cfg.Tstop));
        in = in.setModelParameter('SimulationMode', cfg.simMode);
        try
            out = sim(in);
            ts  = out.get('Vout_sim');
            d = [];  td = [];
            try, dd = out.get('Duty_sim'); if ~isempty(dd), d = dd.Data(:); td = dd.Time(:); end, catch, end
            M(k,:) = metrics_core(ts.Time, ts.Data(:), td, d, cfg);
        catch
        end
    end
    if k == 1 && any(isnan(M(1,:))), break; end   % nominal failed -> skip the rest
end
end

function m = fast_one(x, cfg, s)
[G, Cc] = plant_tf(x, cfg, s);
T = feedback(Cc*G, 1);
if ~isstable(T), m = NaN(1,4); return; end
t = linspace(0, cfg.Tstop, 2000)';
v = cfg.Vout*step(T, t);                     % full reference step from 0
d = cfg.Vout*step(feedback(Cc, G), t);       % duty response
m = metrics_core(t, v, t, d, cfg);
end

function m = metrics_core(t, v, td, d, cfg)
Vref = cfg.Vout;  Tend = t(end);
if any(~isfinite(v)) || max(v) > 3*Vref, m = NaN(1,4); return; end
itae = trapz(t, t.*abs(Vref-v)) / (Vref*Tend^2/2);   % 1 = "never left zero"
os   = max(0, max(v)-Vref) / Vref;
idx  = find(abs(v-Vref) > 0.02*Vref, 1, 'last');
if isempty(idx),        Ts = 0;
elseif idx >= numel(t), Ts = Tend;
else,                   Ts = t(idx+1);
end
eff = 0;
if ~isempty(d)
    [tu,iu] = unique(td);                    % sample duty once per switching period:
    tk = (0:1/cfg.fs:tu(end))';              % removes PWM ripple, keeps the trend
    du = interp1(tu, d(iu), tk, 'linear');
    eff = sum(abs(diff(du)));
end
m = [itae, os, Ts/Tend, eff];
end

function mg = margins_one(x, cfg, s)
[G, Cc] = plant_tf(x, cfg, s);
Lo = Cc*G;
if ~isstable(feedback(Lo,1))
    mg = [0, -10];      % unstable in the averaged model -> heavy margin penalty
    return
end
[Gm, Pm] = margin(Lo);
mg = [Pm, 20*log10(Gm)];
end

function [G, Cc] = plant_tf(x, cfg, s)
% Averaged CCM model with RHP zero (unity PWM gain, ideal parts)
Vin = cfg.Vin*s(1); R = cfg.R*s(2); L = cfg.L*s(3); C = cfg.C*s(4);
D  = 1 - Vin/cfg.Vout;  Dp = 1 - D;
G  = tf(Vin/Dp^2*[-L/(R*Dp^2) 1], [L*C/Dp^2, L/(R*Dp^2), 1]);
Cc = tf([x(1) x(2)], [1 0]);
end