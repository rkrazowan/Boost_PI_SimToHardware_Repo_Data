%% run_boost_test.m
% Put this file in the SAME folder as boost_simulink.slx and boostIO.m
% Press Run (F5).

%% ===== 1) SETTINGS (change only this part) =====
mdl     = 'boost_simulink';
Kp      = 0.1;        % proportional gain
Ki      = 20;         % integral gain
V0      = 8;          % starting target (V)
V1      = 15;         % final target after the step (V)
t_step  = 3;          % time of the step (s)
Tstop   = 12;         % total run time (s)
tolPct  = 2;          % settling band (% of step size)
runName = 'Kp0p1_Ki20';

%% ===== 2) RUN =====
clear boostIO                          % release old serial connection
load_system(mdl);
set_param(mdl,'StopTime',num2str(Tstop));
set_param(mdl,'EnablePacing','on');    % real-time running
set_param(mdl,'PacingRate',1);

boostIO([V0; Kp; Ki]);                 % open port + send gains before the run
pause(3);                              % let Vout settle at V0
out = sim(mdl);
Y   = out.outLog;

%% ===== 3) CLEAN DATA =====
Y = Y(Y(:,5) > 0, :);                  % drop rows before first Arduino data
[~,iu] = unique(Y(:,5),'stable');      % drop repeated rows
Y = Y(iu,:);
Vout = Y(:,1);  Duty = Y(:,2);  Tgt = Y(:,3);  tms = Y(:,5);

i0 = find(abs(Tgt - V1) < 0.01, 1);    % moment the step reaches the Arduino
if isempty(i0), error('Target never reached V1. Check t_step < Tstop.'); end
t = (tms - tms(i0))/1000;              % seconds, 0 = step instant

pre = (t < 0) & (t > -0.5);
y0  = mean(Vout(pre));
if isnan(y0), y0 = Vout(max(i0-1,1)); end

k = t >= 0;  tt = t(k);  y = Vout(k);  d = Duty(k);
last = tt > (tt(end) - 1);             % last 1 second
yss  = mean(y(last));
stepSize = yss - y0;
up = stepSize > 0;

%% ===== 4) METRICS =====
t10 = firstCross(tt,y,y0+0.1*stepSize,up);
t50 = firstCross(tt,y,y0+0.5*stepSize,up);
t90 = firstCross(tt,y,y0+0.9*stepSize,up);
tr  = t90 - t10;                       % rise time 10-90 %

if up, [ypk,ip] = max(y); else, [ypk,ip] = min(y); end
tp = tt(ip);                           % peak time
OS = max(0, (ypk - yss)*sign(stepSize)/abs(stepSize)*100);   % overshoot %

band = tolPct/100*abs(stepSize);
outside = abs(y - yss) > band;
if ~any(outside), ts = 0;
elseif find(outside,1,'last') == numel(y), ts = NaN;         % never settled
else, ts = tt(find(outside,1,'last') + 1);
end

ess    = V1 - yss;                     % steady-state error
essPct = ess/V1*100;
ripple = max(y(last)) - min(y(last));  % pk-pk ripple in last 1 s
Dss    = mean(d(last));  Dmax = max(d);

e = V1 - y;
IAE  = trapz(tt, abs(e));
ISE  = trapz(tt, e.^2);
ITAE = trapz(tt, tt.*abs(e));

if OS > 0.5                            % 2nd-order estimates
    lnOS = log(OS/100);
    zeta = -lnOS/sqrt(pi^2 + lnOS^2);
    wn   = pi/(tp*sqrt(1 - zeta^2));
else
    zeta = NaN;  wn = NaN;
end

names = {'Kp';'Ki';'Initial voltage y0';'Target V1';'Final value yss'; ...
         'Delay time (50%)';'Rise time (10-90%)';'Peak time';'Peak value'; ...
         'Overshoot';sprintf('Settling time (%g%%)',tolPct); ...
         'Steady-state error';'Steady-state error (%)';'Ripple (pk-pk)'; ...
         'Steady duty';'Max duty';'IAE';'ISE';'ITAE'; ...
         'Damping ratio (est.)';'Natural freq (est.)'};
vals  = [Kp;Ki;y0;V1;yss;t50;tr;tp;ypk;OS;ts;ess;essPct;ripple;Dss;Dmax; ...
         IAE;ISE;ITAE;zeta;wn];
units = {'-';'-';'V';'V';'V';'s';'s';'s';'V';'%';'s';'V';'%';'V';'%';'%'; ...
         'V*s';'V^2*s';'V*s^2';'-';'rad/s'};
T = table(names,vals,units,'VariableNames',{'Parameter','Value','Unit'});
disp(T)

%% ===== 5) SAVE + PLOT =====
writetable(T,[runName '_metrics.csv']);
writetable(table(t,Vout,Duty,Tgt),[runName '_raw.csv']);
save([runName '_data.mat'],'t','Vout','Duty','Tgt','Kp','Ki','V0','V1');

figure('Color','w','Position',[100 100 700 600]);
subplot(2,1,1)
plot(t,Vout,'b','LineWidth',1.5); hold on; grid on
plot(t,Tgt,'r--','LineWidth',1.2)
yline(yss+band,':k'); yline(yss-band,':k');
plot(tp,ypk,'ko','MarkerFaceColor','k')
xlabel('Time (s)'); ylabel('V_{out} (V)')
title(sprintf('Kp=%g, Ki=%g | OS=%.1f%%, tr=%.3fs, ts=%.3fs',Kp,Ki,OS,tr,ts))
legend('V_{out}','Target',sprintf('\\pm%g%% band',tolPct),'Peak','Location','best')
subplot(2,1,2)
plot(t,Duty,'m','LineWidth',1.5); grid on
xlabel('Time (s)'); ylabel('Duty (%)')
print(gcf,[runName '.png'],'-dpng','-r300');

%% ===== local function (keep at the END of the file) =====
function tc = firstCross(t,y,lvl,up)
if up, idx = find(y(1:end-1) <  lvl & y(2:end) >= lvl, 1);
else,  idx = find(y(1:end-1) >  lvl & y(2:end) <= lvl, 1); end
if isempty(idx), tc = NaN; return; end
tc = t(idx) + (lvl - y(idx))*(t(idx+1) - t(idx))/(y(idx+1) - y(idx));
end