function vals = localInWindow(ang, lo, hi)
    cands = [ang, ang+360, ang-360];
    mask  = (cands >= lo) & (cands <= hi);
    vals  = cands(mask);
end