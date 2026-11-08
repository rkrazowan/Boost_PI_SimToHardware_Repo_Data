% build_boost_model.m
% Run this ONCE. It creates the Simulink model "boost_simulink" for you.
% Keep boostIO.m in the same folder.

mdl = 'boost_simulink';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

% ---------- inputs you can change ----------
add_block('simulink/Sources/Constant', [mdl '/Vref'], 'Value', '12',  'Position', [30 30 90 60]);
add_block('simulink/Sources/Constant', [mdl '/Kp'],   'Value', '0.1', 'Position', [30 90 90 120]);
add_block('simulink/Sources/Constant', [mdl '/Ki'],   'Value', '20',  'Position', [30 150 90 180]);

add_block('simulink/Signal Routing/Mux', [mdl '/MuxIn'], ...
    'Inputs', '3', 'Position', [140 30 145 180]);

% ---------- the bridge to the Arduino ----------
% The block name changed between MATLAB versions, so try both names.
try
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', [mdl '/BoostIO']);
catch
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Fcn', [mdl '/BoostIO']);
end
set_param([mdl '/BoostIO'], ...
    'MATLABFcn', 'boostIO', 'OutputDimensions', '4', ...
    'OutputSignalType', 'real', 'SampleTime', '0.05', ...
    'Position', [200 85 330 125]);

add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
    'Outputs', '4', 'Position', [380 40 385 180]);

% ---------- outputs ----------
add_block('simulink/Signal Routing/Mux', [mdl '/MuxV'], ...
    'Inputs', '2', 'Position', [450 30 455 80]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_Voltage'], 'Position', [520 30 560 80]);
add_block('simulink/Sinks/Scope', [mdl '/Scope_Duty'],    'Position', [520 100 560 150]);
add_block('simulink/Sinks/Display', [mdl '/Show_Vout'],   'Position', [520 170 600 200]);
add_block('simulink/Sinks/Display', [mdl '/Show_Duty'],   'Position', [520 220 600 250]);
add_block('simulink/Sinks/Terminator', [mdl '/Unused'],   'Position', [450 270 470 290]);
add_block('simulink/Sinks/To Workspace', [mdl '/Log'], ...
    'VariableName', 'logData', 'SaveFormat', 'Array', ...
    'SampleTime', '-1', 'Position', [380 320 450 350]);

% ---------- wires ----------
add_line(mdl, 'Vref/1',  'MuxIn/1');
add_line(mdl, 'Kp/1',    'MuxIn/2');
add_line(mdl, 'Ki/1',    'MuxIn/3');
add_line(mdl, 'MuxIn/1', 'BoostIO/1');
add_line(mdl, 'BoostIO/1', 'Demux/1');
add_line(mdl, 'BoostIO/1', 'Log/1');
add_line(mdl, 'Demux/3', 'MuxV/1');         % Target
add_line(mdl, 'Demux/1', 'MuxV/2');         % Vout
add_line(mdl, 'MuxV/1',  'Scope_Voltage/1');
add_line(mdl, 'Demux/2', 'Scope_Duty/1');
add_line(mdl, 'Demux/1', 'Show_Vout/1');
add_line(mdl, 'Demux/2', 'Show_Duty/1');
add_line(mdl, 'Demux/4', 'Unused/1');

% ---------- model settings ----------
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.05', 'StopTime', '30');
set_param(mdl, 'EnablePacing', 'on', 'PacingRate', '1');     % run in real time
set_param(mdl, 'StopFcn', 'boostIO(''close''); clear boostIO');

save_system(mdl);
disp('Model "boost_simulink" created. Edit PORT inside boostIO.m, then press Run.');