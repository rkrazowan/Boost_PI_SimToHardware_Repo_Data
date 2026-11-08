function out = Boost_Cost(x, cfg, mode)
% mode = 'scalar' -> robust weighted cost (PSO / GWO / GA)
% mode = 'vector' -> [speed, overshoot, effort] (NSGA-II)
% Uses only the scenarios in cfg.Sopt (keeps optimisation cheap).
idx = cfg.Sopt;  ws = cfg.ws(idx);
[M, MG] = Boost_Sim_Metrics(x, cfg, idx);
bad = any(~isfinite(M), 2);
M(bad,:) = 0;

pmPen = max(0, cfg.PMmin - MG(:,1)) / cfg.PMmin;   % NaN -> 0
gmPen = max(0, cfg.GMmin - MG(:,2)) / cfg.GMmin;
mPen  = cfg.wm * (pmPen + gmPen);

if strcmp(mode, 'scalar')
    Js = M * cfg.w(:) + mPen;
    Js(bad) = 100;
    out = (1-cfg.lambda) * sum(ws.*Js)/sum(ws) + cfg.lambda * max(Js);
else
    F = [max(M(:,1) + M(:,3)) + max(mPen), max(M(:,2)), max(M(:,4))];
    if any(bad), F = [100 100 100]; end
    out = F;
end
end
