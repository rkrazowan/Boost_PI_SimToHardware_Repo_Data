function cfg = Boost_Config(caseID)
% All settings in ONE place (report these in the paper for reproducibility)
% caseID = 1 -> Case 1: Kp in [0, 0.03]      (integral-only law allowed)
% caseID = 2 -> Case 2: Kp in [0.001, 0.03]  (genuine PI controller)
% Called without an argument it behaves as before (Case 2).
if nargin < 1, caseID = 2; end
cfg.caseID = caseID;

%% Converter (same equations as your original code)
P = 500; Vin = 12; Vout = 24; fs = 10e3;
Iout = P/Vout; dIL = 0.01*Iout*(Vout/Vin); dV = 0.01*Vout;
cfg.Vin = Vin;  cfg.Vout = Vout;  cfg.fs = fs;
cfg.L = (Vin*(Vout-Vin))/(dIL*fs*Vout);
cfg.C = (Iout*(1-Vin/Vout))/(fs*dV);
cfg.R = Vout/Iout;
%% Simulink
cfg.model   = 'PSO_PI_Controlled_Booster';
cfg.Tstop   = 0.4;        % [s] set by code; keep >= 1.5x baseline settling time
cfg.simMode = 'normal';   % try 'accelerator' after everything works
%% Operating scenarios: [Vin_mult  R_mult  L_mult  C_mult]
% R_mult = 2 -> 50 % load, 5 -> 20 % load
cfg.scenNames = {'Nominal','Vin-20%','Vin+20%','Load50%','Load20%','L-20%','C-20%'};
cfg.S = [1 1 1 1;
         0.8 1 1 1;
         1.2 1 1 1;
         1 2 1 1;
         1 5 1 1;
         1 1 0.8 1;
         1 1 1 0.8];
cfg.ws = ones(size(cfg.S,1),1);  cfg.ws(1) = 2;   % nominal weighted double
cfg.Sopt = [1 2 5];   % scenarios used INSIDE the optimiser (nominal first!)
cfg.fast    = false;  % internal flag (Main sets it)
cfg.useFast = true;   % true: optimise on averaged model, validate on Simulink
%% Cost weights: [ITAE_norm  Overshoot(pu)  Ts_norm  DutyEffort]
% CALIBRATE: print metrics for your baseline gains, then scale so that
% each term contributes a similar amount.
cfg.w  = [10 5 0.5 0.2];
cfg.lambda = 0.5;   % J = (1-lambda)*weighted mean + lambda*worst case
cfg.PMmin  = 45;    % deg  (phase-margin penalty threshold)
cfg.GMmin  = 6;     % dB   (gain-margin penalty threshold)
cfg.wm = 0.5;   % margin penalty weight
%% Optimizer settings
KpLowerBound = [0 0.001];                    % Case 1 / Case 2
cfg.lb = [KpLowerBound(caseID) 0.1];         % [Kp Ki] lower bounds
cfg.ub = [0.03 2];                           % [Kp Ki] upper bounds
cfg.nP = 12;  cfg.maxIter = 20;
cfg.c1 = 1.5; cfg.c2 = 1.5;                  % PSO (w decays 0.9 -> 0.4)
cfg.stall = 8;                               % PSO stopping: no gain for 8 iters
cfg.nRuns = 30;   % paper setting. useFast=true: 30 is cheap. useFast=false: use 10 or fewer
%% Baseline gains (fill in your three-way comparison; NaN = skip)
cfg.base = struct('Manual',     [0.0132 0.6867], ...
                  'Calculated', [0.0040 0.4045], ...
                  'PIDTuner',   [0      0.8837]);
%% Best PSO design reported in the paper (Tables 7 and 11), [Kp Ki]
cfg.psoBest = struct('Case1', [0      0.4086], ...
                     'Case2', [0.0010 0.4147]);
end
