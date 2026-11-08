% Make_Fig_Startup_Load20.m
% Start-up response at 20 % load (scenario 5) for BOTH search spaces in one run,
% saved as a single side-by-side figure: (a) Case 1 (Kp >= 0), (b) Case 2 (Kp >= 0.001).
% Requires: Boost_Config.m (updated version) and the Simulink model
% 'PSO_PI_Controlled_Booster' on the MATLAB path. Needs 8 switching simulations (~1 min).
clear; clc; close all;

designs    = {'Manual', 'Calculated', 'PID Tuner', 'PSO'};
caseTitles = {'(a) Load 20 % (K_p \geq 0)', '(b) Load 20 % (K_p \geq 0.001)'};
scen       = 5;                                  % Load 20 %  -> cfg.S(5,:) = [1 5 1 1]
matFiles   = {'results_A_Kp0.mat', 'results_B_Kp001.mat'};   % saved by Main_Run_All (Case 1, Case 2)

% same colours as the nominal start-up figures
colors = [0.0000 0.4470 0.7410;    % Manual
          0.8500 0.3250 0.0980;    % Calculated
          0.9290 0.6940 0.1250;    % PID Tuner
          0.4940 0.1840 0.5560];   % PSO

includeAll = false;   % true: also plot GWO, GA and NSGA-II knee (needs the results .mat files)
if includeAll
    designs = [designs, {'GWO', 'GA', 'NSGA-II knee'}];
    colors  = [colors; 0.4660 0.6740 0.1880;     % GWO
                       0.3010 0.7450 0.9330;     % GA
                       0.6350 0.0780 0.1840];    % NSGA-II knee
end

fig = figure('Units', 'inches', 'Position', [0.5 0.5 11 4.4], 'Color', 'w');
tl  = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); %#ok<NASGU>

results = struct();
fprintf('\nLoad 20 %% start-up metrics (compare with Tables 8 and 12)\n');
fprintf('%-8s %-12s %12s %16s\n', 'Case', 'Design', 'Overshoot[%]', 'Settling[ms]');

for c = 1:2
    cfg = Boost_Config(c);
    if isfile(matFiles{c})
        % exact gains saved by Main_Run_All (variable 'ctrl')
        S = load(matFiles{c}, 'ctrl');
        gains = [S.ctrl.Manual; S.ctrl.Calculated; S.ctrl.PIDTuner; S.ctrl.PSO];
        if includeAll
            gains = [gains; S.ctrl.GWO; S.ctrl.GA; S.ctrl.NSGA2_knee];
        end
        fprintf('Case %d: gains loaded from %s\n', c, matFiles{c});
    else
        % fallback: rounded values reported in Tables 7 and 11
        if includeAll
            error('includeAll = true needs %s in the current folder.', matFiles{c});
        end
        gains = [cfg.base.Manual;
                 cfg.base.Calculated;
                 cfg.base.PIDTuner;
                 cfg.psoBest.(sprintf('Case%d', c))];
        fprintf('Case %d: %s not found, using the gains from Boost_Config\n', c, matFiles{c});
    end

    ax = nexttile; hold(ax, 'on');
    for k = 1:numel(designs)
        [t, v] = Boost_Waveform(gains(k,:), cfg, scen);
        plot(ax, t*1e3, v, 'LineWidth', 1.8, 'Color', colors(k,:));

        % overshoot and settling time, same definitions as Boost_Sim_Metrics
        Vref = cfg.Vout;
        os   = max(0, max(v) - Vref) / Vref * 100;
        idx  = find(abs(v - Vref) > 0.02*Vref, 1, 'last');
        if isempty(idx),         Ts = 0;
        elseif idx >= numel(t),  Ts = NaN;               % not settled
        else,                    Ts = t(idx+1)*1e3;
        end
        if isnan(Ts), tsStr = 'NS'; else, tsStr = sprintf('%.1f', Ts); end
        fprintf('%-8d %-12s %12.2f %16s\n', c, designs{k}, os, tsStr);

        results(c,k).case   = c;       %#ok<SAGROW>
        results(c,k).design = designs{k};
        results(c,k).gains  = gains(k,:);
        results(c,k).t      = t;
        results(c,k).v      = v;
    end
    yline(ax, cfg.Vout, ':k', 'HandleVisibility', 'off');

    grid(ax, 'on'); box(ax, 'on');
    xlim(ax, [0 cfg.Tstop*1e3]);  ylim(ax, [0 30]);
    xlabel(ax, 'Time (ms)');      ylabel(ax, 'Vout (V)');
    title(ax, caseTitles{c}, 'FontWeight', 'bold');
    legend(ax, designs, 'Location', 'southeast');
    set(ax, 'FontSize', 11);
end

exportgraphics(fig, 'fig_startup_load20.png', 'Resolution', 300);
save('startup_load20_data.mat', 'results');
fprintf('\nSaved fig_startup_load20.png and startup_load20_data.mat\n');


function [t, v] = Boost_Waveform(x, cfg, scenIdx)
% One switching-model run: x = [Kp Ki], scenIdx = row of cfg.S.
% Uses exactly the same settings as Boost_Sim_Metrics (non-fast branch).
s  = cfg.S(scenIdx, :);
in = Simulink.SimulationInput(cfg.model);
in = in.setVariable('Kp',   x(1));
in = in.setVariable('Ki',   x(2));
in = in.setVariable('Vin',  cfg.Vin*s(1));
in = in.setVariable('R',    cfg.R*s(2));
in = in.setVariable('L',    cfg.L*s(3));
in = in.setVariable('C',    cfg.C*s(4));
in = in.setVariable('Vout', cfg.Vout);
in = in.setVariable('Vref', cfg.Vout);
in = in.setVariable('fs',   cfg.fs);
in = in.setModelParameter('StopTime',       num2str(cfg.Tstop));
in = in.setModelParameter('SimulationMode', cfg.simMode);
out = sim(in);
ts  = out.get('Vout_sim');
t   = ts.Time(:);
v   = ts.Data(:);
end