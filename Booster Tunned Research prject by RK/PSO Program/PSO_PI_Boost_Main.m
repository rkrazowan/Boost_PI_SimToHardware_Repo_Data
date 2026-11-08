clc;
clear;
close all;

%% 1. System & Boost Converter Parameters
P = 6.61;            % Power (W)
Vin = 10;           % Input Voltage (V)
Vout = 18.18;          % Target Output Voltage (V)
fs = 62.5e3;          % Switching Frequency (Hz)


% Component calculations
L = 22.5e-6;
C = 470e-6;
R = 50x;

% Store components into Base Workspace
assignin('base', 'L', L);
assignin('base', 'C', C);
assignin('base', 'R', R);
assignin('base', 'Vin', Vin);
assignin('base', 'Vout', Vout);
assignin('base', 'fs', fs);

model_name = 'PSO_PI_Controlled_Booster';

% Verify Simulink model presence
if ~exist([model_name, '.slx'], 'file')
    error('Simulink file %s.slx not found! Make sure it is in your active folder.', model_name);
end

%% 2. Improved PSO Parameter Setup
num_particles = 15;   % Population size for thorough exploration
max_iter = 15;        % Total iterations

c1 = 1.5;            
c2 = 1.5;            

% Corrected Bounds to include manual tuning (0.0132, 0.6867) safely
lb = [0.0010, 0.1000];   % Lower bounds [Kp_min, Ki_min]
ub = [0.0300, 2.0000];   % Upper bounds [Kp_max, Ki_max]
nVar = 2;             

vmax = 0.1 * (ub - lb);
vmin = -vmax;

%% 3. Particle Swarm Initialization
particles = zeros(num_particles, nVar);
velocity  = zeros(num_particles, nVar);
pBest     = zeros(num_particles, nVar);
pBest_cost = inf(num_particles, 1);

gBest = zeros(1, nVar);
gBest_cost = inf;

% Seed particle 1 with your manual values (Kp = 0.0132, Ki = 0.6867)
particles(1, :) = [0.0132, 0.6867];

% Initialize remaining swarm uniformly within bounds
for i = 2:num_particles
    particles(i, :) = lb + rand(1, nVar) .* (ub - lb);
end

% Initial Velocity setup
for i = 1:num_particles
    velocity(i, :) = vmin + rand(1, nVar) .* (vmax - vmin);
end

disp('=======================================');
disp('     Starting PSO Tuning Process       ');
disp('=======================================');

%% 4. Optimization Loop
cost_history = zeros(max_iter, 1);

for iter = 1:max_iter
    % Dynamic Inertia Weight Decay (0.9 down to 0.4)
    w = 0.9 - (0.9 - 0.4) * (iter / max_iter);
    
    fprintf('\n--- Iteration %d / %d (Inertia w = %.3f) ---\n', iter, max_iter, w);
    
    for i = 1:num_particles
        % Evaluate Objective Function
        current_cost = Boost_PI_Objective(particles(i, :), model_name, Vout);
        
        % Update Personal Best (pBest)
        if current_cost < pBest_cost(i)
            pBest(i, :) = particles(i, :);
            pBest_cost(i) = current_cost;
        end
        
        % Update Global Best (gBest)
        if current_cost < gBest_cost
            gBest = particles(i, :);
            gBest_cost = current_cost;
        end
        
        fprintf('Particle %2d | Kp: %.6f | Ki: %.6f | Cost: %.4f\n', ...
            i, particles(i,1), particles(i,2), current_cost);
    end
    
    % Update Velocities and Positions
    for i = 1:num_particles
        r1 = rand(1, nVar);
        r2 = rand(1, nVar);
        
        % PSO Velocity Equation
        velocity(i, :) = w * velocity(i, :) ...
            + c1 * r1 .* (pBest(i, :) - particles(i, :)) ...
            + c2 * r2 .* (gBest - particles(i, :));
        
        % Velocity Clamping
        velocity(i, :) = max(velocity(i, :), vmin);
        velocity(i, :) = min(velocity(i, :), vmax);
        
        % Position Update
        particles(i, :) = particles(i, :) + velocity(i, :);
        
        % Position Bounds Clamping [lb, ub]
        particles(i, :) = max(particles(i, :), lb);
        particles(i, :) = min(particles(i, :), ub);
    end
    
    cost_history(iter) = gBest_cost;
    fprintf('>> Best Cost after Iteration %d: %.4f (Best Kp = %.6f, Best Ki = %.6f)\n', ...
        iter, gBest_cost, gBest(1), gBest(2));
end

%% 5. Results, Performance Table & Comparative Plots
fprintf('\n=======================================\n');
fprintf('       PSO Tuning Complete!\n');
fprintf('=======================================\n');
fprintf('Optimal Kp = %.6f\n', gBest(1));
fprintf('Optimal Ki = %.6f\n', gBest(2));
fprintf('Best Cost  = %.4f\n', gBest_cost);

% Plot 1: Optimization Convergence Curve
figure('Name', 'PSO Convergence', 'NumberTitle', 'off');
plot(1:max_iter, cost_history, '-o', 'LineWidth', 2, 'Color', [0 0.447 0.741]);
xlabel('Iteration');
ylabel('Objective Cost (ITAE + Penalty)');
title('PSO Convergence Curve');
grid on;

% --- Run Simulation 1: Manual Tuning ---
assignin('base', 'Kp', 0.1);
assignin('base', 'Ki', 20);
simOut_manual = sim(model_name);
vout_manual = extract_timeseries(simOut_manual);

% --- Run Simulation 2: Optimized Tuning ---
assignin('base', 'Kp', gBest(1));
assignin('base', 'Ki', gBest(2));
simOut_opt = sim(model_name);
vout_opt = extract_timeseries(simOut_opt);

% Plot 2: Comparative Voltage Response
figure('Name', 'Voltage Response Comparison', 'NumberTitle', 'off');
plot(vout_manual.Time * 1000, vout_manual.Data, 'b--', 'LineWidth', 1.5, ...
    'DisplayName', 'Manual Tuning (Kp=0.0132, Ki=0.6867)');
hold on;
plot(vout_opt.Time * 1000, vout_opt.Data, 'r-', 'LineWidth', 2, ...
    'DisplayName', sprintf('PSO Tuned (Kp=%.6f, Ki=%.6f)', gBest(1), gBest(2)));
yline(Vout, ':k', 'Target Vout (24V)', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xlabel('Time (ms)');
ylabel('Voltage (V)');
title('Boost Converter Output Voltage: Manual vs. PSO-Tuned PI');
legend('Location', 'southeast');
grid on;

% --- Calculate Research Performance Metrics ---
m_manual = calculate_metrics(vout_manual.Time, vout_manual.Data, Vout);
m_opt    = calculate_metrics(vout_opt.Time, vout_opt.Data, Vout);

% Display Comparison Table
fprintf('\n--- Research Performance Metrics Comparison ---\n');
T = table({'Manual PI'; 'PSO-Tuned PI'}, ...
    [0.0132; gBest(1)], [0.6867; gBest(2)], ...
    [m_manual.RiseTime*1000; m_opt.RiseTime*1000], ...
    [m_manual.SettlingTime*1000; m_opt.SettlingTime*1000], ...
    [m_manual.PeakOvershoot; m_opt.PeakOvershoot], ...
    'VariableNames', {'Controller', 'Kp', 'Ki', 'RiseTime_ms', 'SettlingTime_ms', 'Overshoot_V'});
disp(T);

%% Helper Function: Safe Data Extraction
function ts = extract_timeseries(simOut)
    if isa(simOut, 'Simulink.SimulationOutput')
        ts = simOut.get('Vout_sim');
    elseif isstruct(simOut) && isfield(simOut, 'Vout_sim')
        ts = simOut.Vout_sim;
    else
        ts = evalin('base', 'Vout_sim');
    end
end

%% Helper Function: Transient Performance Analysis
function metrics = calculate_metrics(t, v, Vref)
    % Rise Time (10% to 90% of Vref)
    idx10 = find(v >= 0.10 * Vref, 1, 'first');
    idx90 = find(v >= 0.90 * Vref, 1, 'first');
    if ~isempty(idx10) && ~isempty(idx90)
        metrics.RiseTime = t(idx90) - t(idx10);
    else
        metrics.RiseTime = NaN;
    end
    
    % Settling Time (2% band around Vref)
    band = 0.02 * Vref;
    outside_band = find(abs(v - Vref) > band, 1, 'last');
    if ~isempty(outside_band) && outside_band < length(t)
        metrics.SettlingTime = t(outside_band + 1);
    else
        metrics.SettlingTime = t(end);
    end
    
    % Peak Overshoot (V)
    metrics.PeakOvershoot = max(0, max(v) - Vref);
end