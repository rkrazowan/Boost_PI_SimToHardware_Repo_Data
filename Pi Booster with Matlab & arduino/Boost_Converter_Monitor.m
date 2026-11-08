clear;
clc;
close all;

%% 1. SERIAL COMMUNICATION SETTINGS
port = "COM10";
baudRate = 9600;

%% 2. CONNECT TO ARDUINO
arduinoSerial = serialport(port, baudRate);
configureTerminator(arduinoSerial, "LF");
flush(arduinoSerial);
%% SEND REFERENCE VOLTAGE TO ARDUINO

Vref_command = 12.0;
command = "VREF=" + string(Vref_command);
writeline(arduinoSerial, command);
disp("Sent command: " + command);

disp("Connected to Arduino on COM10.");
disp("Reading boost converter measurements...");
disp("Press Ctrl+C in the Command Window to stop.");

%% 3. INITIALIZE DATA ARRAYS
Time = [];
Vref_data = [];
Vout_data = [];
Duty_data = [];
PWM_data = [];

startTime = tic;

%% 4. CREATE LIVE PLOT
figure("Name", "Physical Boost Converter Monitoring");

h = animatedline("LineWidth", 1.5);

xlabel("Time (s)");
ylabel("Output Voltage (V)");
title("Real-Time Boost Converter Output Voltage");
grid on;

%% 5. READ SERIAL DATA
try
    while true

        % Read one complete line from Arduino
        line = strtrim(readline(arduinoSerial));

        % Display the original Arduino message
        disp(line);

        % Extract the four numerical values
        values = sscanf(line, ...
            'Target VOUT: %f V | Actual VOUT: %f V | Duty: %f%% | OCR0B: %d');

        % Ignore startup messages or malformed lines
        if numel(values) ~= 4
            continue;
        end

        % Store the received measurements
        t = toc(startTime);

        Vref = values(1);
        Vout = values(2);
        Duty = values(3);
        PWM = values(4);

        Time(end+1,1) = t;
        Vref_data(end+1,1) = Vref;
        Vout_data(end+1,1) = Vout;
        Duty_data(end+1,1) = Duty;
        PWM_data(end+1,1) = PWM;

        % Update the live plot
        addpoints(h, t, Vout);
        drawnow limitrate;

        % Update current Workspace variables
        assignin("base", "Vref", Vref);
        assignin("base", "Vout", Vout);
        assignin("base", "Duty", Duty);
        assignin("base", "PWM", PWM);

        % Update complete data arrays in the Workspace
        assignin("base", "Time", Time);
        assignin("base", "Vref_data", Vref_data);
        assignin("base", "Vout_data", Vout_data);
        assignin("base", "Duty_data", Duty_data);
        assignin("base", "PWM_data", PWM_data);

    end

catch ME
    disp("Serial monitoring stopped.");
    disp(ME.message);
end

%% 6. CLOSE THE SERIAL CONNECTION
clear arduinoSerial;

disp("Arduino serial connection closed.");