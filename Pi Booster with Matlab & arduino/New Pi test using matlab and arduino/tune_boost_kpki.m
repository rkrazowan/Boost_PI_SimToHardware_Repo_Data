%% tune_boost_kpki.m  -- try a few Kp/Ki values and pick the best
% Put this file in the SAME folder as boost_simulink.slx and boostIO.m
% Everything is inside this one file (no other helper files needed).
% Press Run (F5). 9 runs take about 3-4 minutes.

%% ===== 1) SETTINGS (change only this part) =====
mdl     = 'boost_simulink';
KpList  = [0.05 0.1 0.3];     % few Kp values to test
KiList  = [10 20 40];         % few Ki values to test
% (or give your own pairs instead:  pairs = [0.05 5; 0.1 20; 0.2 40];)
V0      = 8;                  % starting target (V)
V1      = 12;                 % final target after the step (V)
t_step  = 2;                  % time of the step (s)
Tstop   = 15;                 % run time of each test (s)
tolPct  = 2;                  % settling band (%)
OSmax   = 10;                 % allowed overshoot (%) for "good" gains
essMax  = 1;                  % allowed steady-state error (%) for "good" gains

%% ===== 2) PREPARE =====
[KP,KI] = meshgrid(KpList,KiList);
pairs   = [KP(:) KI(:)];      % <- delete this line if you gave your own pairs above
N       = size(pairs,1);

clear boostIO
load_system(mdl);
set_param(mdl,'StopTime',num2str(Tstop));
set_param(mdl,'EnablePacing','on');
set_param(mdl,'PacingRate',1);

%% ===== 3) RUN ALL TESTS =====
R = cell(N,1);
for n = 1:N
    fprintf('Run %d/%d : Kp=%g  Ki=%g  V0=%g  V1=%g\n',n,N,pairs(n,1),pairs(n,2),V0,V1);
    clear boostIO
    try
        R{n} = runOne(mdl,pairs(n,1),pairs(n,2),V0,V1,t_step,Tstop,tolPct);
    catch ME
        warning('Run %d failed: %s',n,ME.message);
        clear boostIO
        R{n} = nanRun(pairs(n,1),pairs(n,2));
    end
end
clear boostIO

%% ===== 4) SUMMARY TABLE =====
OS   = cellfun(@(r) r.OS,     R);
tr   = cellfun(@(r) r.tr,     R);
ts   = cellfun(@(r) r.ts,     R);
ess  = cellfun(@(r) r.essPct, R);
rip  = cellfun(@(r) r.ripple, R);
ITAE = cellfun(@(r) r.ITAE,   R);

T = table(pairs(:,1),pairs(:,2),OS,tr,ts,ess,rip,ITAE, 'VariableNames', ...
    {'Kp','Ki','Overshoot_pct','Rise_s','Settle_s','SSerr_pct','Ripple_V','ITAE'});
disp(sortrows(T,'Settle_s'))
writetable(T,'tuning_summary.csv');
save('tuning_results.mat','R','T');

good = OS <= OSmax & abs(ess) <= essMax & ~isnan(ts);
if any(good)
    idx = find(good);
    [~,j] = min(ts(idx));
    best = idx(j);
    fprintf('\nBEST (fastest settling with OS<=%g%% and error<=%g%%): Kp=%g  Ki=%g\n', ...
        OSmax,essMax,pairs(best,1),pairs(best,2));
else
    [~,best] = min(ITAE);
    fprintf('\nNo run met the limits. Lowest ITAE: Kp=%g  Ki=%g\n',pairs(best,1),pairs(best,2));
end
fprintf('   OS=%.2f %%  tr=%.3f s  ts=%.3f s  error=%.3f %%\n', ...
    OS(best),tr(best),ts(best),ess(best));

%% ===== 5) PLOT ALL RUNS =====
figure('Color','w','Position',[100 100 760 640]);
subplot(2,1,1); hold on; grid on
subplot(2,1,2); hold on; grid on
for n = 1:N
    if isempty(R{n}.t), continue; end
    lw = 1.1 + 1.6*(n == best);
    nm = sprintf('Kp=%g Ki=%g',pairs(n,1),pairs(n,2));
    subplot(2,1,1); plot(R{n}.t,R{n}.v,'LineWidth',lw,'DisplayName',nm);
    subplot(2,1,2); plot(R{n}.t,R{n}.d,'LineWidth',lw,'DisplayName',nm);
end
subplot(2,1,1); yline(V1,'k--','Target','HandleVisibility','off');
xlim([-1 Tstop-t_step]); xlabel('Time (s)'); ylabel('V_{out} (V)');
title('Step response for each Kp / Ki (thick = best)'); legend('Location','eastoutside');
subplot(2,1,2); xlim([-1 Tstop-t_step]); xlabel('Time (s)'); ylabel('Duty (%)');
print(gcf,'tuning_plot.png','-dpng','-r300');

%% ===== local functions (keep at the END of the file) =====
function r = runOne(mdl,Kp,Ki,V0,V1,t_step,Tstop,tolPct)
assignin('base','Kp',Kp);   assignin('base','Ki',Ki);
assignin('base','V0',V0);   assignin('base','V1',V1);
assignin('base','t_step',t_step);  assignin('base','Tstop',Tstop);
boostIO([V0; Kp; Ki]);               % open port + send gains
pause(3);                            % let Vout settle at V0
out = sim(mdl);
r = analyzeStep(out.outLog,Kp,Ki,V1,tolPct);
end

function r = analyzeStep(Y,Kp,Ki,V1,tolPct)
Y = Y(Y(:,5) > 0, :);
[~,iu] = unique(Y(:,5),'stable');
Y = Y(iu,:);
Vout = Y(:,1);  Duty = Y(:,2);  Tgt = Y(:,3);  tms = Y(:,5);

i0 = find(abs(Tgt - V1) < 0.01, 1);
if isempty(i0), error('Target never reached V1'); end
t = (tms - tms(i0))/1000;

pre = (t < 0) & (t > -0.5);
y0  = mean(Vout(pre));
if isnan(y0), y0 = Vout(max(i0-1,1)); end

k = t >= 0;  tt = t(k);  y = Vout(k);  d = Duty(k);
last = tt > (tt(end) - 1);
yss  = mean(y(last));
stepSize = yss - y0;
if abs(stepSize) < 0.2, error('No step response detected'); end
up = stepSize > 0;

t10 = firstCross(tt,y,y0+0.1*stepSize,up);
t90 = firstCross(tt,y,y0+0.9*stepSize,up);
tr  = t90 - t10;

if up, [ypk,ip] = max(y); else, [ypk,ip] = min(y); end
tp = tt(ip);
OS = max(0,(ypk - yss)*sign(stepSize)/abs(stepSize)*100);

band = tolPct/100*abs(stepSize);
outside = abs(y - yss) > band;
if ~any(outside), ts = 0;
elseif find(outside,1,'last') == numel(y), ts = NaN;
else, ts = tt(find(outside,1,'last') + 1);
end

ess = V1 - yss;
e   = V1 - y;
r = struct('Kp',Kp,'Ki',Ki,'y0',y0,'yss',yss,'tr',tr,'tp',tp,'OS',OS,'ts',ts, ...
    'ess',ess,'essPct',ess/V1*100,'ripple',max(y(last))-min(y(last)), ...
    'Dss',mean(d(last)),'IAE',trapz(tt,abs(e)),'ISE',trapz(tt,e.^2), ...
    'ITAE',trapz(tt,tt.*abs(e)),'t',t,'v',Vout,'d',Duty,'tgt',Tgt);
end

function r = nanRun(Kp,Ki)
r = struct('Kp',Kp,'Ki',Ki,'y0',NaN,'yss',NaN,'tr',NaN,'tp',NaN,'OS',NaN,'ts',NaN, ...
    'ess',NaN,'essPct',NaN,'ripple',NaN,'Dss',NaN,'IAE',NaN,'ISE',NaN,'ITAE',NaN, ...
    't',[],'v',[],'d',[],'tgt',[]);
end

function tc = firstCross(t,y,lvl,up)
if up, idx = find(y >= lvl,1); else, idx = find(y <= lvl,1); end
if isempty(idx), tc = NaN; return; end
if idx == 1, tc = t(1); return; end
tc = t(idx-1) + (lvl - y(idx-1))*(t(idx) - t(idx-1))/(y(idx) - y(idx-1));
end
