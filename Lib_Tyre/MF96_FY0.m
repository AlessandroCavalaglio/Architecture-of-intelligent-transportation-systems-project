% Pure lateral force FY0
function [fy0] = MF96_FY0(kappa, alpha, phi, Fz, tyre_data, road_condition)

 % precode

  [alpha__y,By,Cy,Dy,Ey,SVy] = MF96_FY0_coeffs(kappa, alpha, phi, Fz, tyre_data, road_condition);

 % main code

  fy0 = magic_formula(alpha__y, By, Cy, Dy, Ey, SVy);
  
 end