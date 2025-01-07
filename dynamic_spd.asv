function [b1] = dynamic_spd(b1, time)
    % Gains (designed using Simulink)
    b1.pgain_v = .1;    % 1; %0.0925236108186868;
    b1.igain_v = .5;    %40;  %0.17645031875233;
    b1.dgain_v = 1;    %2;  %0.00501863563263352;
    
    % Initialize Variables
    if isfield(b1, "error_prev")
    else
        b1.error_prev = 0;
    end
    
    if isfield(b1, "integral")
    else
        b1.integral = 0;
    end
   
    
    % PID Controller Loop
    
    % Compute Speed
    v_vec = (b1.pos(1:2) - b1.pos_old(1:2)) ./ time.dt;
    % disp(speed)
    speed = dot(v_vec, b1.heading_vec./ norm(b1.heading_vec));
%     disp(speed)
%     disp(v_vec)
%     fprintf("pos")
%     disp(b1.pos(1:2))
    
    % Calculate error
    error = b1.forward_spd - speed;
    
    % Calculate integral and derivative term
    
    b1.integral = b1.integral + error * time.dt;
    disp(b1.integral)
    derivative = (error-b1.error_prev)/time.dt;
    
    % Calculate control output
    PWML = b1.pgain_v*error + b1.igain_v*b1.integral + b1.dgain_v*derivative;
    PWMR = PWML;
    b1.PWMV = double(b1.PWMV);

    b1.PWML = 0.85*PWML;
    b1.PWMR = PWMR;

    if (b1.PWMV + b1.PWMR + b1.PWML) > 450
        total = b1.PWMV + b1.PWMR + b1.PWML;
        b1.PWMV = b1.PWMV/total * 450;
        b1.PWMR = b1.PWMR/total * 450;
        b1.PWML = b1.PWML/total * 450;
    end

%     b1.PWML = 100;
%     b1.PWMR = 100;

    % % setting the PWM limits, keep the codes here
%     b1.PWMLd = PWML;
%     b1.PWMVd = PWMV;

    b1.PWML = min(255, b1.PWML);
    b1.PWML = uint8(max(0, b1.PWML));
    b1.PWMR = min(255, b1.PWMR);
    b1.PWMR = uint8(max(0, b1.PWMR));
    b1.PWMV = min(254, b1.PWMV);
    b1.PWMV = uint8(max(0, b1.PWMV));

    b1.error_prev = error;
    b1.pos_old = b1.pos;
    disp(error)
    disp(b1.PWML)
end

