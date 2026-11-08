% analyze_steps.m
% Calculates overshoot, rise/fall time, settling time and steady-state error
% for EVERY target (Vref) step found in your Simulink run.
%
% Uses the data the model saves:
%   out.tout          -> time
%   out.logData(:,1)  -> Vout
%   out.logData(:,2)  -> Duty
%   out.logData(:,3)  -> Target
%
% Run your Simulink model first, then run this script.

%% ===== SETTINGS =====
minHold = 1.0;     % ignore a target value held shorter than this (seconds)
minStep = 0.5;     % ignore steps smaller than this (volts)
band    = 0.02;    % settling band: 2 % of the target

%% ===== GET THE DATA =====
if ~exist("out", "var") || ~isprop(out, "logData")
    error("Run the Simulink model first. 'out.logData' was not found.");
end
t    = out.tout(:);
vout = out.logData(:,1);
duty = out.logData(:,2);
tgt  = out.logData(:,3);

dtMean = mean(diff(t));
fprintf("Data points: %d, sample time: %.1f ms\n\n", numel(t), dtMean*1000);

%% ===== FIND THE TARGET STEPS =====
chg = find(abs(diff(tgt)) > 0.05) + 1;       % samples where the target changed
if isempty(chg)
    error("The target never changed. Change Vref during the run to make steps.");
end
segEnd = [chg(2:end) - 1; numel(t)];          % last sample of each segment

%% ===== MEASURE EACH STEP =====
rows = {};
for k = 1:numel(chg)
    i1 = chg(k);
    i2 = segEnd(k);
    if t(i2) - t(i1) < minHold, continue; end          % too short, skip

    Vt = median(tgt(i1:i2));                           % target for this step
    if t(i1) - t(1) < minHold
        y0 = vout(1);                                  % step at the very start
    else
        y0 = mean(vout(max(1, i1-3):i1-1));            % voltage just before step
    end

    stepSize = abs(Vt - y0);
    if stepSize < minStep, continue; end
    d = sign(Vt - y0);                                 % +1 = up, -1 = down

    y  = vout(i1:i2);
    tt = t(i1:i2) - t(i1);                             % time since the step

    % ---- overshoot (undershoot for a down step) ----
    if d > 0
        [peak, ip] = max(y);
    else
        [peak, ip] = min(y);
    end
    overshoot = max(0, d*(peak - Vt)) / stepSize * 100;
    if overshoot > 0, peakTime = tt(ip); else, peakTime = NaN; end

    % ---- rise time (up) or fall time (down): 10 % to 90 % of the step ----
    i10 = find(d*(y - y0) >= 0.1*stepSize, 1);
    i90 = find(d*(y - y0) >= 0.9*stepSize, 1);
    if isempty(i10) || isempty(i90)
        riseT = NaN;
    else
        riseT = tt(i90) - tt(i10);
    end

    % ---- settling time: last time the output was outside the band ----
    outside = find(abs(y - Vt) > band*Vt, 1, "last");
    if isempty(outside)
        settleT = 0;
    elseif outside == numel(y)
        settleT = NaN;                                 % never settled
    else
        settleT = tt(outside + 1);
    end

    % ---- steady-state error (average of the last 20 % of the segment) ----
    nTail = max(3, round(0.2*numel(y)));
    ssErr = Vt - mean(y(end-nTail+1:end));

    if d > 0, kind = "Up"; else, kind = "Down"; end
    rows(end+1, :) = {y0, Vt, kind, overshoot, peakTime, riseT, settleT, ssErr}; %#ok<SAGROW>
end

if isempty(rows)
    error("No usable steps found. Try a smaller minHold or minStep.");
end

results = cell2table(rows, "VariableNames", ...
    ["From_V","To_V","Type","Overshoot_pct","PeakTime_s","RiseFallTime_s", ...
     "SettlingTime_s","SteadyError_V"]);

%% ===== SHOW THE RESULTS =====
disp(results);
fprintf("\nNotes:\n");
fprintf(" - Rise/Fall time is 10%% to 90%% of the step. For an Up step it is the rise time,\n");
fprintf("   for a Down step it is the fall time.\n");
fprintf(" - Settling time uses a +/-%.0f%% band around the target.\n", band*100);
fprintf(" - Times can only be as fine as the sample time (%.0f ms). A time of 0\n", dtMean*1000);
fprintf("   or about one sample means the converter is faster than the data rate.\n");
fprintf(" - NaN means that value could not be measured (for example no overshoot).\n");

%% ===== PLOT =====
figure("Name", "Step analysis");
subplot(2,1,1);
plot(t, tgt, "--", t, vout, "LineWidth", 1.4); grid on;
legend("Target", "Vout"); ylabel("Voltage (V)");
title("Output voltage and target");
subplot(2,1,2);
plot(t, duty, "LineWidth", 1.4); grid on;
xlabel("Time (s)"); ylabel("Duty (%)");