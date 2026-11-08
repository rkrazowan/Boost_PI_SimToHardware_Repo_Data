clc;
clear;
close all;

%% 1. Nominal System & Component Calculation
P = 500;              % Nominal Power (W)
Vin_nom = 12;         % Nominal Input Voltage (V)
Vout = 24;            % Target Output Voltage (V)
fs = 10e3;            % Switching Frequency (Hz)

Ioutmax = P / Vout;
delIL = 0.01 * Ioutmax * (Vout / Vin_nom);
delvout = 0.01 * Vout;

L_nom = (Vin_nom * (Vout - Vin_nom)) / (delIL * fs * Vout);
C_nom = (Ioutmax * (1 - (Vin_nom / Vout))) / (fs * delvout);
R_nom = Vout / Ioutmax;

% Store nominal values in Workspace
assignin('base', 'L_nom', L_nom);
assignin('base', 'C_nom', C_nom);
assignin('base', 'R_nom', R_nom);
assignin('base', 'Vin_nom', Vin_nom);

assignin('base', 'L', L_nom);
assignin('base', 'C', C_nom);
assignin('base', 'R', R_nom);
assignin('base', 'Vin', Vin_nom);
assignin('base', 'Vout', Vout);
assignin('base', 'fs', fs);

model_name = 'PSO_PI_Controlled_Booster';

if ~exist([model_name, '.slx'], 'file')
    error('Simulink file %s.slx not found!', model_name);
end

%% 2. PSO Setup for Robust Search
num_particles = 12;
max_iter = 10;

c1 = 1.5;
c2 = 1.5;

lb = [0.0010, 0.1000];   % [Kp_min, Ki_min]
ub = [0.0300, 2.0000];   % [Kp_max, Ki_max]
nVar = 2;

vmax = 0.1 * (ub - lb);
vmin = -vmax;

% Initialize swarm
particles = zeros(num_particles, nVar);
velocity  = zeros(num_particles, nVar);
pBest     = zeros(num_particles, nVar);
pBest_cost = inf(num_particles, 1);

gBest = zeros(1, nVar);
gBest_cost = inf;

% Seed particle 1 with nominal PSO / manual starting point
particles(1, :) = [0.0132, 0.6867];

for i = 2:num_particles
    particles(i, :) = lb + rand(1, nVar) .* (ub - lb);
end

for i = 1:num_particles
    velocity(i, :) = vmin + rand(1, nVar) .* (vmax - vmin);
end

disp('====================================================');
disp('     Starting Robustness-Aware PSO Tuning Process   ');
disp('====================================================');

cost_history = zeros(max_iter, 1);

for iter = 1:max_iter
    w = 0.9 - (0.9 - 0.4) * (iter / max_iter);
    fprintf('\n--- Iteration %d / %d (Inertia w = %.3f) ---\n', iter, max_iter, w);
    
    for i = 1:num_particles
        current_cost = Robust_Boost_PI_Objective(particles(i, :), model_name, Vout);
        
        if current_cost < pBest_cost(i)
            pBest(i, :) = particles(i, :);
            pBest_cost(i) = current_cost;
        end
        
        if current_cost < gBest_cost
            gBest = particles(i, :);
            gBest_cost = current_cost;
        end
        
        fprintf('Particle %2d | Kp: %.6f | Ki: %.6f | Worst-Case Cost: %.4f\n', ...
            i, particles(i,1), particles(i,2), current_cost);
    end
    
    for i = 1:num_particles
        r1 = rand(1, nVar);
        r2 = rand(1, nVar);
        
        velocity(i, :) = w * velocity(i, :) ...
            + c1 * r1 .* (pBest(i, :) - particles(i, :)) ...
            + c2 * r2 .* (gBest - particles(i, :));
        
        velocity(i, :) = max(velocity(i, :), vmin);
        velocity(i, :) = min(velocity(i, :), vmax);
        
        particles(i, :) = particles(i, :) + velocity(i, :);
        particles(i, :) = max(particles(i, :), lb);
        particles(i, :) = min(particles(i, :), ub);
    end
    
    cost_history(iter) = gBest_cost;
    fprintf('>> Iteration %d Best Worst-Case Cost: %.4f (Kp = %.6f, Ki = %.6f)\n', ...
        iter, gBest_cost, gBest(1), gBest(2));
end

disp('====================================================');
disp('        Robust PSO Optimization Completed           ');
disp('====================================================');
fprintf('Robust Optimal Kp = %.6f\n', gBest(1));
fprintf('Robust Optimal Ki = %.6f\n', gBest(2));

%% 4. Detailed 5-Corner Evaluation & Comparison
% Evaluate Nominal PSO (or manual) gains vs. Robust PSO gains across all 5 corners
Kp_nom = 0.0132; Ki_nom = 0.6867;
Kp_rob = gBest(1); Ki_rob = gBest(2);

scenarios = [
    1.00,  1.0,  1.00,  1.00;  % S1: Nominal
    0.80,  1.0,  0.90,  0.80;  % S2: Low Line, Full Load, Min L/C
    1.20,  5.0,  1.10,  1.20;  % S3: High Line, Light Load, Max L/C
    0.80,  5.0,  0.90,  0.80;  % S4: Low Line, Light Load, Min L/C
    1.20,  1.0,  1.10,  1.20   % S5: High Line, Full Load, Max L/C
];

scenario_names = {'Nominal'; 'Low Vin / Heavy Load / Min L,C'; 'High Vin / Light Load / Max L,C'; ...
                  'Low Vin / Light Load / Min L,C'; 'High Vin / Heavy Load / Max L,C'};

num_scen = size(scenarios, 1);

% Storage for metric tables
m_nom_list = cell(num_scen, 1);
m_rob_list = cell(num_scen, 1);

figure('Name', 'Multi-Scenario Response Comparison', 'NumberTitle', 'off');

for k = 1:num_scen
    % Set scenario params
    assignin('base', 'Vin', Vin_nom * scenarios(k, 1));
    assignin('base', 'R', R_nom * scenarios(k, 2));
    assignin('base', 'L', L_nom * scenarios(k, 3));
    assignin('base', 'C', C_nom * scenarios(k, 4));
    
    % Simulate Nominal/Standard Tuning
    assignin('base', 'Kp', Kp_nom); assignin('base', 'Ki', Ki_nom);
    simOut_n = sim(model_name);
    ts_n = extract_ts(simOut_n);
    
    % Simulate Robust Tuning
    assignin('base', 'Kp', Kp_rob); assignin('base', 'Ki', Ki_rob);
    simOut_r = sim(model_name);
    ts_r = extract_ts(simOut_r);
    
    % Calculate Metrics
    m_nom_list{k} = calculate_detailed_metrics(ts_n.Time, ts_n.Data, Vout);
    m_rob_list{k} = calculate_detailed_metrics(ts_r.Time, ts_r.Data, Vout);
    
    % Plot response per scenario
    subplot(3, 2, k);
    plot(ts_n.Time * 1000, ts_n.Data, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Standard PSO');
    hold on;
    plot(ts_r.Time * 1000, ts_r.Data, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Robust PSO');
    yline(Vout, ':k', 'Target Vout', 'HandleVisibility', 'off');
    xlabel('Time (ms)'); ylabel('Voltage (V)');
    title(sprintf('Scenario %d: %s', k, scenario_names{k}));
    grid on;
    if k == 1, legend('Location', 'southeast'); end
end

% Restore base parameters
assignin('base', 'Vin', Vin_nom); assignin('base', 'L', L_nom);
assignin('base', 'C', C_nom); assignin('base', 'R', R_nom);

%% 5. Print Complete Research Data Tables
disp(' ');
disp('========================================================================================');
disp('                          RESEARCH DATA: STANDARD PSO VS ROBUST PSO                    ');
disp('========================================================================================');

scen_col = (1:num_scen)';
tr_nom = cellfun(@(x) x.RiseTime*1000, m_nom_list);
ts_nom = cellfun(@(x) x.SettlingTime*1000, m_nom_list);
ov_nom = cellfun(@(x) x.PeakOvershoot, m_nom_list);
vr_nom = cellfun(@(x) x.VoltageRipple, m_nom_list);

tr_rob = cellfun(@(x) x.RiseTime*1000, m_rob_list);
ts_rob = cellfun(@(x) x.SettlingTime*1000, m_rob_list);
ov_rob = cellfun(@(x) x.PeakOvershoot, m_rob_list);
vr_rob = cellfun(@(x) x.VoltageRipple, m_rob_list);

T_results = table(scen_col, scenario_names, tr_nom, tr_rob, ts_nom, ts_rob, ov_nom, ov_rob, vr_nom, vr_rob, ...
    'VariableNames', {'Scenario', 'Description', 'RiseTime_Std_ms', 'RiseTime_Rob_ms', ...
                      'SettlingTime_Std_ms', 'SettlingTime_Rob_ms', ...
                      'Overshoot_Std_V', 'Overshoot_Rob_V', ...
                      'Ripple_Std_V', 'Ripple_Rob_V'});
disp(T_results);

%% Helper Functions
function ts = extract_ts(simOut)
    if isa(simOut, 'Simulink.SimulationOutput')
        ts = simOut.get('Vout_sim');
    elseif isstruct(simOut) && isfield(simOut, 'Vout_sim')
        ts = simOut.Vout_sim;
    else
        ts = evalin('base', 'Vout_sim');
    end
end

function m = calculate_detailed_metrics(t, v, Vref)
    idx10 = find(v >= 0.10 * Vref, 1, 'first');
    idx90 = find(v >= 0.90 * Vref, 1, 'first');
    if ~isempty(idx10) && ~isempty(idx90)
        m.RiseTime = t(idx90) - t(idx10);
    else
        m.RiseTime = NaN;
    end
    
    band = 0.02 * Vref;
    outside_band = find(abs(v - Vref) > band, 1, 'last');
    if ~isempty(outside_band) && outside_band < length(t)
        m.SettlingTime = t(outside_band + 1);
    else
        m.SettlingTime = t(end);
    end
    
    m.PeakOvershoot = max(0, max(v) - Vref);
    
    % Steady-state voltage ripple calculation (last 20% of simulation time)
    ss_start = round(0.80 * length(v));
    v_ss = v(ss_start:end);
    m.VoltageRipple = max(v_ss) - min(v_ss);
end