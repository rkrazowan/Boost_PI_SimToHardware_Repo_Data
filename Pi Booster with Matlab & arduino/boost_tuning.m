clear; clc; close all;

%% 1. CONNECT
s = serialport("COM10", 115200);
configureTerminator(s, "LF");
s.Timeout = 2;
pause(2.5);                 % wait for Arduino restart
flush(s);

%% 2. SETTINGS (change these)
Vlow   = 8;                 % starting voltage (V)
Vhigh  = 12;                % step target voltage (V)
tLow   = 10;                 % seconds to hold Vlow
tHigh  = 15;                 % seconds to hold Vhigh

%% 3. SINGLE TEST (run this first, on its own)
Kp = 0.1;
Ki = 40;

[T, m] = runStepTest(s, Kp, Ki, Vlow, Vhigh, tLow, tHigh);
if ~isempty(T)
    plotResult(T, m, Kp, Ki);
end
disp(m)                     % Overshoot, RiseTime, SettlingTime, SteadyError, Cost
% T (Time, Vref, Vout, Duty, PWM) and m are now in the Workspace.

%% 4. ROUND 1: WIDE SEARCH (9 tests, about 1.5 minutes)
KpList = [0.05 0.1 0.2];
KiList = [10 20 40];

results = searchGrid(s, KpList, KiList, Vlow, Vhigh, tLow, tHigh);
disp(results(1:min(5,height(results)), :));      % top 5
bestKp = results.Kp(1);
bestKi = results.Ki(1);
fprintf("ROUND 1 BEST: Kp = %.3f, Ki = %.1f\n", bestKp, bestKi);

%% 5. ROUND 2: NARROW SEARCH AROUND THE BEST (runs automatically)
KpList2 = linspace(bestKp*0.5, bestKp*1.5, 5);
KiList2 = linspace(bestKi*0.5, bestKi*1.5, 5);

results2 = searchGrid(s, KpList2, KiList2, Vlow, Vhigh, tLow, tHigh);
disp(results2(1:min(5,height(results2)), :));
bestKp = results2.Kp(1);
bestKi = results2.Ki(1);
fprintf("FINAL BEST: Kp = %.3f, Ki = %.1f\n", bestKp, bestKi);

%% 6. RUN THE FINAL BEST AND PLOT IT
[T, m] = runStepTest(s, bestKp, bestKi, Vlow, Vhigh, tLow, tHigh);
if ~isempty(T)
    plotResult(T, m, bestKp, bestKi);
end
disp(m)

%% 7. FINISH
sendCmd(s, "VREF=5");
clear s;

%% ================= FUNCTIONS (keep at the bottom) =================
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
        if startsWith(line, "ERROR"), warning("Arduino: " + line); return; end
    end
    warning("No reply to " + cmd);
end

function results = searchGrid(s, KpList, KiList, Vlow, Vhigh, tLow, tHigh)
    rows = [];
    total = numel(KpList) * numel(KiList);
    n = 0;
    for kp = KpList
        for ki = KiList
            n = n + 1;
            [~, mm] = runStepTest(s, kp, ki, Vlow, Vhigh, tLow, tHigh);
            rows = [rows; kp, ki, mm.Overshoot, mm.RiseTime, ...
                    mm.SettlingTime, mm.SteadyError, mm.Cost]; %#ok<AGROW>
            fprintf("[%d/%d] Kp=%.3f Ki=%.1f -> cost=%.3f overshoot=%.1f%%\n", ...
                    n, total, kp, ki, mm.Cost, mm.Overshoot);
            pause(1);               % let the converter settle between tests
        end
    end
    results = array2table(rows, "VariableNames", ...
        ["Kp","Ki","Overshoot","RiseTime","SettlingTime","SteadyError","Cost"]);
    results = sortrows(results, "Cost");
end

function [T, m] = runStepTest(s, Kp, Ki, Vlow, Vhigh, tLow, tHigh)
    badM = struct("Overshoot",NaN,"RiseTime",NaN,"SettlingTime",NaN, ...
                  "SteadyError",NaN,"Cost",Inf);
    T = [];

    sendCmd(s, "RESET");
    sendCmd(s, sprintf("KP=%.4f", Kp));
    sendCmd(s, sprintf("KI=%.4f", Ki));
    sendCmd(s, sprintf("VREF=%.2f", Vlow));
    flush(s);

    raw = []; stepSent = false; unsafe = false;
    t0 = tic;
    while toc(t0) < tLow + tHigh
        if ~stepSent && toc(t0) >= tLow
            writeline(s, sprintf("VREF=%.2f", Vhigh));
            stepSent = true;
        end
        line = readline(s);
        if isempty(line) || ismissing(line), continue; end
        p = split(strtrim(line), ",");
        if numel(p) ~= 6 || p(1) ~= "DATA", continue; end
        v = str2double(p(2:6))';
        if any(isnan(v)), continue; end
        raw = [raw; v]; %#ok<AGROW>
        if v(3) > 20                 % safety: output too high
            unsafe = true; break;
        end
    end

    if unsafe
        writeline(s, "VREF=5"); writeline(s, "RESET");
        warning("Output went above 20 V - test aborted.");
    end
    if size(raw,1) < 20
        warning("Not enough data received.");
        m = badM; return;
    end

    Time = (raw(:,1) - raw(1,1)) / 1000;
    T = table(Time, raw(:,2), raw(:,3), raw(:,4), raw(:,5), ...
        'VariableNames', ["Time","Vref","Vout","Duty","PWM"]);

    % ---------- performance numbers ----------
    iStep = find(T.Vref >= Vhigh - 0.01, 1);
    if isempty(iStep) || unsafe
        m = badM; return;
    end
    t = T.Time(iStep:end) - T.Time(iStep);
    y = T.Vout(iStep:end);
    span = Vhigh - Vlow;

    overshoot = max(0, (max(y) - Vhigh) / span * 100);

    i10 = find(y >= Vlow + 0.1*span, 1);
    i90 = find(y >= Vlow + 0.9*span, 1);
    if isempty(i10) || isempty(i90), riseT = NaN; else, riseT = t(i90) - t(i10); end

    outside = find(abs(y - Vhigh) > 0.02*Vhigh, 1, "last");   % 2% band
    if isempty(outside), settleT = 0; else, settleT = t(outside); end

    ssErr = abs(mean(y(round(0.8*end):end)) - Vhigh);
    iae = trapz(t, abs(y - Vhigh));                           % total error area
    cost = iae + 0.05*overshoot + 5*ssErr;                    % smaller = better

    m = struct("Overshoot",overshoot,"RiseTime",riseT,"SettlingTime",settleT, ...
               "SteadyError",ssErr,"Cost",cost);
end

function plotResult(T, m, Kp, Ki)
    figure("Name","Step response");
    subplot(2,1,1);
    plot(T.Time, T.Vref, "--", T.Time, T.Vout, "LineWidth", 1.5); grid on;
    legend("Vref","Vout"); ylabel("Voltage (V)");
    title(sprintf("Kp=%.3f  Ki=%.1f  Overshoot=%.1f%%  Settling=%.2fs  Cost=%.2f", ...
          Kp, Ki, m.Overshoot, m.SettlingTime, m.Cost));
    subplot(2,1,2);
    plot(T.Time, T.Duty, "LineWidth", 1.5); grid on;
    xlabel("Time (s)"); ylabel("Duty (%)");
end