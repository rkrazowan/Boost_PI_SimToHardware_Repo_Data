function cost = Boost_PI_Objective(x, model_name, Vref)
    Kp = x(1);
    Ki = x(2);
    
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    
    try
        simOut = sim(model_name, 'SimulationMode', 'normal', 'AbsTol', '1e-5', 'RelTol', '1e-3');
        
        if isa(simOut, 'Simulink.SimulationOutput')
            vout_ts = simOut.get('Vout_sim');
        elseif isstruct(simOut) && isfield(simOut, 'Vout_sim')
            vout_ts = simOut.Vout_sim;
        else
            vout_ts = evalin('base', 'Vout_sim');
        end
        
        t = vout_ts.Time;
        vout = vout_ts.Data;
        error = Vref - vout;
        
        % 1. Primary Metric: ITAE
        itae = trapz(t, t .* abs(error));
        
        % 2. Quadratic Overshoot Penalty (Prevents high voltage spikes smoothly)
        max_vout = max(vout);
        overshoot = max(0, max_vout - Vref);
        penalty_overshoot = 15 * (overshoot^2); 
        
        % Total Fitness Value
        cost = itae + penalty_overshoot;
        
        if isnan(cost) || isinf(cost)
            cost = 1e6;
        end
        
    catch
        cost = 1e6;
    end
end