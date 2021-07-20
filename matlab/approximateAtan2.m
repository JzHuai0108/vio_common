function r = approximateAtan2(y, x)
  absx = abs(x);
  absy = abs(y);
  a = min(absx, absy) / max(absx, absy);
  s = a * a;
  r = ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a;
  if absy > absx
    r = 1.57079637 - r;
  end
  if (x < 0) 
    r = 3.14159274 - r;
  end
  if (y < 0)
    r = -r;
  end
end