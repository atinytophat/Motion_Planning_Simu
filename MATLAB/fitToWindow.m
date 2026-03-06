function vals = fitToWindow(ang, lo, hi)
    cands = [ang, ang+360, ang-360];
    vals = cands(cands >= lo & cands <= hi);
end