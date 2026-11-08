function y = boostIO(u)
% boostIO  Bridge between Simulink and the Arduino boost converter.
%   Input  u = [Vref; Kp; Ki]
%   Output y = [Vout; Duty; Target; PWM; Time_ms]
% Keep this file in the SAME folder as your Simulink model.

PORT = "COM10";          % <<< CHANGE to your Arduino port (type serialportlist)
BAUD = 115200;

persistent s last vals buf
if isempty(vals), vals = zeros(5,1); end
if isempty(buf),  buf  = ''; end

% ---------- called by the model's StopFcn: turn off and close ----------
if ischar(u) || isstring(u)
    if ~isempty(s)
        try
            writeline(s, "VREF=5");
            writeline(s, "RESET");
        catch
        end
    end
    s = []; last = []; vals = []; buf = '';
    y = [];
    return
end

% ---------- first call: open the port ----------
if isempty(s)
    s = serialport(PORT, BAUD);
    configureTerminator(s, "LF");
    pause(2.5);                  % Arduino restarts when the port opens
    flush(s);
    last = [NaN; NaN; NaN];      % NaN = nothing sent yet
    writeline(s, "RESET");
end

% ---------- send commands only when a value changed ----------
u    = u(:);
new  = [min(max(u(1), 5), 18);       % Vref  limited to 5..18 V
        min(max(u(2), 0), 5);        % Kp    limited to 0..5
        min(max(u(3), 0), 500)];     % Ki    limited to 0..500
chg  = isnan(last) | abs(new - last) > 1e-9;
if chg(2), writeline(s, sprintf("KP=%.4f",   new(2))); end
if chg(3), writeline(s, sprintf("KI=%.4f",   new(3))); end
if chg(1), writeline(s, sprintf("VREF=%.2f", new(1))); end
last = new;

% ---------- read everything that has arrived ----------
nb = s.NumBytesAvailable;
if nb > 0
    buf = [buf, read(s, nb, "char")];
    parts = split(string(buf), newline);
    buf = char(parts(end));                  % keep the unfinished line
    if numel(buf) > 2000, buf = ''; end
    for k = 1:numel(parts) - 1
        q = split(strtrim(parts(k)), ",");
        if numel(q) == 6 && q(1) == "DATA"
            v = str2double(q(2:6));
            if ~any(isnan(v))
                vals = [v(3); v(4); v(2); v(5); v(1)];   % Vout, Duty, Target, PWM, Arduino time (ms)
            end
        end
    end
end

% ---------- safety ----------
if vals(1) > 20
    writeline(s, "VREF=5");
    writeline(s, "RESET");
    error("Vout went above 20 V. Stopped for safety.");
end

y = vals;
end