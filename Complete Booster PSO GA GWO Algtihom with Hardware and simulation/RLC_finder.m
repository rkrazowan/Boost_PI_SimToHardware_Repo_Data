clear; clc; close all
%% 1. Hardware parameters
Vin  = 10;            % input voltage (V)
D    = 0.45;          % duty cycle
fs   = 62.5e3;        % Arduino D9 PWM frequency (Hz)
L    = 22.5e-6;       % inductor (H)
C    = 470e-6;        % output capacitor (F)
R    = 50;            % load: five 10 ohm in series (use 2 for parallel)
RL   = 0.1;           % estimated total loss resistance (ohm): inductor + MOSFET + diode
Hdiv = 10e3/(10e3+40e3);   % feedback divider = 0.2
%% 2. Steady state check
Vout = Vin/(1-D);
Iout = Vout/R;
fprintf('Vout = %.2f V | Iout = %.3f A | A5 voltage = %.2f V\n', Vout, Iout, Vout*Hdiv);
fprintf('Current ripple = %.2f A | Voltage ripple = %.1f mV | Lcrit = %.1f uH\n', ...
        Vin*D/(L*fs), Iout*D/(fs*C)*1e3, D*(1-D)^2*R/(2*fs)*1e6);
%% 3. Plants: ideal (lossless) and damped (with RL)
a  = Vin/(1-D)^2;
b  = L/(R*(1-D)^2);
c  = (L*C)/(1-D)^2;
bL = (L/R + RL*C)/(1-D)^2;
dc = 1 + RL/(R*(1-D)^2);
Gp_ideal = tf([-a*b a], [c b  1 ])*Hdiv;   % duty -> A5 voltage, lossless
Gp_loss  = tf([-a*b a], [c bL dc])*Hdiv;   % duty -> A5 voltage, with losses
fprintf('Damping ratio: ideal = %.4f | with RL = %.3f\n', b/(2*sqrt(c)), bL/(2*sqrt(c)));
figure; bode(Gp_ideal, Gp_loss); grid on
legend('ideal (lossless)','with RL'); title('Plant Bode')
%% 4. Check your hardware gains (Kp = 0.1, Ki = 20)
C_hw = pid(0.1, 20);
T_hw = feedback(C_hw*Gp_loss, 1);
[Gm,Pm,~,Wcp] = margin(C_hw*Gp_loss);
fprintf('\nHardware gains on damped model: GM = %.1f dB | PM = %.1f deg | crossover = %.0f rad/s\n', ...
        20*log10(Gm), Pm, Wcp);
if isstable(T_hw)
    S = stepinfo(T_hw);
    fprintf('Rise = %.1f ms | Settling = %.1f ms | Overshoot = %.1f %%\n', ...
            S.RiseTime*1e3, S.SettlingTime*1e3, S.Overshoot);
else
    disp('Closed loop is UNSTABLE with hardware gains on the damped model.');
end
%% 5. Automatic tuning with pidtune (low bandwidth, damped model)
wc_list = [100 150 300];                    % rad/s
fprintf('\n--- pidtune results (damped model) ---\n');
Ctune = cell(1,numel(wc_list));
for k = 1:numel(wc_list)
    Ctune{k} = pidtune(Gp_loss, 'PI', wc_list(k));
    fprintf('wc = %3d rad/s -> Kp = %.4f, Ki = %.3f\n', wc_list(k), Ctune{k}.Kp, Ctune{k}.Ki);
end
%% 6. Open the PID Tuner app (manual sliders) on the damped plant
pidTuner(Gp_loss, 'PI')
%% 7. Compare responses
tEnd = 0.15;
figure; hold on
step(feedback(C_hw*Gp_loss,1), tEnd)
for k = 1:numel(wc_list)
    step(feedback(Ctune{k}*Gp_loss,1), tEnd)
end
step(feedback(pid(0,5.469)*Gp_loss,1), tEnd)
legend(['hardware Kp=0.1 Ki=20', arrayfun(@(w) sprintf('pidtune wc=%d',w), wc_list, 'UniformOutput', false), 'app Ki=5.469'])
grid on; title('Closed-loop step (damped model)')
%% 8. Stability vs unknown loss resistance (robustness of hardware gains)
fprintf('\n--- Phase margin of Kp=0.1, Ki=20 vs RL ---\n');
figure; hold on
for RLs = [0.02 0.05 0.1 0.2 0.5]
    bLs = (L/R + RLs*C)/(1-D)^2;
    dcs = 1 + RLs/(R*(1-D)^2);
    Gs_ = tf([-a*b a], [c bLs dcs])*Hdiv;
    [~,Pm_] = margin(C_hw*Gs_);
    fprintf('RL = %.2f ohm -> PM = %.1f deg\n', RLs, Pm_);
    step(feedback(C_hw*Gs_,1), tEnd)
end
legend('RL=0.02','RL=0.05','RL=0.1','RL=0.2','RL=0.5'); grid on
title('Hardware gains vs RL')
%% 9. Largest safe Kp (Ki = 20) at a low-damping case, RL = 0.05
bLw = (L/R + 0.05*C)/(1-D)^2; dcw = 1 + 0.05/(R*(1-D)^2);
Gw  = tf([-a*b a], [c bLw dcw])*Hdiv;
fprintf('\n--- Phase margin vs Kp (Ki = 20, RL = 0.05) ---\n');
for Kp_ = [0 0.02 0.05 0.1 0.2]
    [~,Pm_] = margin(pid(Kp_,20)*Gw);
    fprintf('Kp = %.2f -> PM = %.1f deg\n', Kp_, Pm_);
end
%% 10. Final gains for the Arduino (divider-volt units, duty 0-1)
Cfinal = Ctune{2};                          % wc = 150 rad/s, change index to choose
Kp_arduino = Cfinal.Kp
Ki_arduino = Cfinal.Ki
% If your PWM duty is 0-255 (or 0-1023), multiply both gains by that range