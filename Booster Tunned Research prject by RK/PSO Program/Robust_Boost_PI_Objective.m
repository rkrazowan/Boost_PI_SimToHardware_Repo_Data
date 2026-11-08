function worst_cost = Robust_Boost_PI_Objective(x, model_name, Vref)
    % Extract candidate controller gains
    Kp = x(1);
    Ki = x(2);
    
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    
    % --- Scenario Matrix Definition ---
    % Columns: [Vin_factor, Load_R_factor, L_factor, C_factor]
    % 1. Nominal Operating Point (12V, 100% Load, Nominal L & C)
    % 2. Low Line Voltage (-20%), Full Load, Low Tolerance L (-10%), Low Tolerance C (-20%)
    % 3. High Line Voltage (+20%), Light Load (20% Load -> R*5), High L (+10%), High C (+20%)
    % 4. Low Line Voltage (-20%), Light Load (20% Load -> R*5), Low L (-10%), Low C (-20%)
    % 5. High Line Voltage (+20%), Full Load, High L (+10%), High C (+20%)
    scenarios = [
        1.00,  1.0,  1.00,  1.00;  % Scenario 1: Nominal
        0.80,  1.0,  0.90,  0.80;  % Scenario 2: Low Line, Full Load, Min Components
        1.20,  5.0,  1.10,  1.20;  % Scenario 3: High Line, Light Load, Max Components
        0.80,  5.0,  0.90,  0.80;  % Scenario 4: Low Line, Light Load, Min Components
        1.20,  1.0,  1.10,  1.20   % Scenario 5: High Line, Full Load, Max Components
    ];
    
    num_scenarios = size(scenarios, 1);
    scenario_costs = zeros(num_scenarios, 1);
    
    % Fetch nominal values from Base Workspace
    Vin_nom = evalin('base', 'Vin_nom');
    L_nom   = evalin('base', 'L_nom');
    C_nom   = evalin('base', 'C_nom');
    R_nom   = evalin('base', 'R_nom');
    
    for k = 1:num_scenarios
        % Compute scenario parameters
        Vin_k = Vin_nom * scenarios(k, 1);
        R_k   = R_nom   * scenarios(k, 2);
        L_k   = L_nom   * scenarios(k, 3);
        C_k   = C_nom   * scenarios(k, 4);
        
        % Push current scenario values to Base Workspace
        assignin('base', 'Vin', Vin_k);
        assignin('base', 'R', R_k);
        assignin('base', 'L', L_k);
        assignin('base', 'C', C_k);
        
        try
            simOut = sim(model_name, 'SimulationMode', 'normal', 'AbsTol', '1e-5', 'RelTol', '1e-3');
            
            % Safe signal extraction
            if isa(simOut, 'Simulink.SimulationOutput')
                vout_ts = simOut.get('Vout_sim');
                duty_ts = simOut.get('Duty_sim');
            else
                vout_ts = evalin('base', 'Vout_sim');
                duty_ts = evalin('base', 'Duty_sim');
            end
            
            t = vout_ts.Time;
            v = vout_ts.Data;
            d = duty_ts.Data;
            
            % 1. ITAE (Integral of Time-weighted Absolute Error)
            error_val = Vref - v;
            itae = trapz(t, t .* abs(error_val));
            
            % 2. Peak Overshoot Penalty
            max_v = max(v);
            overshoot = max(0, max_v - Vref);
            penalty_overshoot = 20 * (overshoot^2);
            
            % 3. Control Effort Penalty (Duty cycle derivative / chatter)
            if length(t) > 1
                dt = diff(t);
                % Avoid zero dt
                dt(dt == 0) = 1e-6;
                dd = diff(d) ./ dt;
                control_effort = trapz(t(1:end-1), dd.^2);
            else
                control_effort = 0;
            end
            
            % Combined cost for scenario k
            scenario_costs(k) = itae + penalty_overshoot + 1e-4 * control_effort;
            
        catch
            % Assign heavy penalty if model destabilizes or fails to converge
            scenario_costs(k) = 1e6;
        end
    end
    
    % Restore nominal base parameters
    assignin('base', 'Vin', Vin_nom);
    assignin('base', 'L', L_nom);
    assignin('base', 'C', C_nom);
    assignin('base', 'R', R_nom);
    
    % Robust Optimization Objective: Evaluate WORST-CASE performance
    worst_cost = max(scenario_costs);
end