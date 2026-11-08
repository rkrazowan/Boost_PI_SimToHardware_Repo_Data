clear; clc; close all;

%% 1. CONNECT
s = serialport("COM10", 115200);
configureTerminator(s, "LF");
s.Timeout = 2;
pause(2.5);                 % wait for Arduino restart
flush(s);

%% 2. CHECK A: IS DATA ARRIVING? (converter power can be OFF)
rows = [];
t0 = tic;
while toc(t0) < 3
    row = readDataRow(s);
    if ~isempty(row), rows = [rows; row]; end %#ok<AGROW>
end
rate = size(rows,1) / 3;
showResult("Data lines arriving (expect ~20 per second)", rate > 10, sprintf("%.1f lines/s", rate));
if size(rows,1) > 1
    showResult("Arduino time runs correctly (millis)", ...
        abs((rows(end,1)-rows(1,1))/1000 - 3) < 0.5, "");
end

%% 3. CHECK B: DO COMMANDS GET ANSWERED? (ACK)
showResult("RESET command", sendCmd(s, "RESET"), "");
showResult("KP command",    sendCmd(s, "KP=0.1"), "");
showResult("KI command",    sendCmd(s, "KI=40"), "");
showResult("VREF command",  sendCmd(s, "VREF=8"), "");

%% 4. CHECK C: DOES THE ARDUINO USE THE NEW VREF?
flush(s); pause(0.5);
row = readDataRow(s);
if isempty(row)
    showResult("Target column shows 8 V", false, "no data received");
else
    showResult("Target column shows 8 V", abs(row(2) - 8) < 0.1, ...
           sprintf("Target = %.2f V", row(2)));
end

%% 5. CHECK D: DOES THE CONVERTER FOLLOW? (turn converter power ON now)
% Watch your supply. Output must stay below 20 V.
sendCmd(s, "RESET");
sendCmd(s, "VREF=8");
vals = []; duty = [];
t0 = tic;
while toc(t0) < 5
    row = readDataRow(s);
    if isempty(row), continue; end
    vals(end+1) = row(3); duty(end+1) = row(4); %#ok<SAGROW>
    if row(3) > 20
        sendCmd(s, "VREF=5"); sendCmd(s, "RESET");
        error("Vout above 20 V, stopped for safety.");
    end
end
if numel(vals) > 11
    finalV = mean(vals(end-10:end));
    finalD = mean(duty(end-10:end));
    fprintf("Final Vout = %.2f V (target 8.00 V), Duty = %.1f %%\n", finalV, finalD);
    showResult("Vout reaches target within 0.3 V", abs(finalV - 8) < 0.3, "");
    showResult("Duty is not stuck at 0% or at the max", finalD > 1 && finalD < 78, "");
    figure; plot(vals, "LineWidth", 1.5); yline(8, "--"); grid on;
    xlabel("Sample (50 ms each)"); ylabel("Vout (V)"); title("Check D: response to 8 V target");
else
    showResult("Enough data in Check D", false, "too few samples");
end

%% 6. FINISH
sendCmd(s, "VREF=5");
clear s;

%% ============ FUNCTIONS (keep at the bottom) ============
function showResult(name, ok, extra)
    if ok, tag = "PASS"; else, tag = "FAIL"; end
    fprintf("[%s] %s  %s\n", tag, name, extra);
end

function ok = sendCmd(s, cmd)
    ok = false;
    flush(s);
    writeline(s, cmd);
    t0 = tic;
    while toc(t0) < 1
        line = readline(s);
        if isempty(line) || ismissing(line), continue; end
        line = strtrim(line);
        if startsWith(line, "ACK"), ok = true; return; end
        if startsWith(line, "ERROR"), disp("Arduino said: " + line); return; end
    end
end

function row = readDataRow(s)
    row = [];
    line = readline(s);
    if isempty(line) || ismissing(line), return; end
    p = split(strtrim(line), ",");
    if numel(p) ~= 6 || p(1) ~= "DATA", return; end
    v = str2double(p(2:6))';
    if ~any(isnan(v)), row = v; end
end