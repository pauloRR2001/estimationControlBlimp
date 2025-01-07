function [b1] = dynamic_vertical(b1,time)
if isnan(b1.pos)
    return
end
% PID gains initiliazation
Kp = .55;    %0.0212117720895436;
Ki = .015;     %1.61432437511288e-06;
Kd = .2;     %0.171591794101992;

if isfield(b1,"alt_error_prev")
else
    b1.alt_error_prev = 0;
end 

if isfield(b1, "alt_integral")
else
    b1.alt_integral = 0;
end 

% Error
alt_des = 1200;
alt = b1.pos(3);
alt_error = alt_des - alt;


% integral and derivative terms
b1.alt_integral = b1.alt_integral + alt_error*time.dt;
alt_deriv = (alt_error-b1.alt_error_prev)/time.dt;

% Calculate PWMV

PWMV = Kp*alt_error + Ki*b1.alt_integral + Kd*alt_deriv;
b1.PWMV = PWMV;


if (b1.PWMV + b1.PWMR + b1.PWML) > 450
    total = b1.PWMV + b1.PWMR + b1.PWML;
    b1.PWMV = b1.PWMV/total * 450;
    b1.PWMR = b1.PWMR/total * 450;
    b1.PWML = b1.PWML/total * 450;
end 

%b1.PWMV = 100;
b1.PWMV = min(254, b1.PWMV);
b1.PWMV = uint8(max(0, b1.PWMV));
b1.PWML = min(254, b1.PWML);
b1.PWML = uint8(max(0, b1.PWML));
b1.PWMR = min(254, b1.PWMR);
b1.PWMR = uint8(max(0, b1.PWMR));

b1.alt_error_prev = alt_error;
b1.pos_old = b1.pos;

disp(b1.PWMV)

end 
