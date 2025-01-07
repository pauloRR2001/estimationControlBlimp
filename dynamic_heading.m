function [b1] = dynamic_heading(b1, time)

if isnan(b1.pos(1))
    return
end

    % PID Parameters
    K_P = .25; %1;  % Proportional gain
    K_I = .055; %5;  % Integral gain
    K_D = .055; %2;   % Derivative gain

    %  Initialize Variables
    if isfield(b1, "error_prev_hdg")
    else
        b1.error_prev_hdg = 0;
    end

    if isfield(b1, "integral_hdg")
    else
        b1.integral_hdg = 0;
    end

    % Make vectors 2-D
    b1.heading_dir = b1.heading_dir(1:2);
    b1.heading_vec = b1.heading_vec(1:2);   

    % Normalize vectors
    b1.heading_dir = b1.heading_dir/norm(b1.heading_dir);
    b1.heading_vec = b1.heading_vec/norm(b1.heading_vec);

    % Angle Between
    error = acosd(dot(b1.heading_vec, b1.heading_dir));  % deg

    % Compute signed angle relative
    signed_angle_current = atan2(b1.heading_vec(2), b1.heading_vec(1));
    signed_angle_desired = atan2(b1.heading_dir(2), b1.heading_dir(1));

    % Determine signed difference
    signed_angle_diff = rad2deg(signed_angle_desired - signed_angle_current);

    if abs(error) < b1.fwd_deg
        b1.PWML = 0;
        b1.PWMR = 0;
    else
        % Compute integral and derivative
        b1.integral_hdg = b1.integral_hdg + error*time.dt;
        derivative = (error - b1.error_prev_hdg)/time.dt;

        % Calculate Control Input
        PWM = K_P*error + K_D*derivative + K_I*b1.integral_hdg;

        % Determine direction
        if signed_angle_diff < 0
            % Turn Right
            b1.PWML = PWM; 
            b1.PWMR = 0;
        else
            % Turn Left
            b1.PWML = 0; 
            b1.PWMR = 0.8*PWM;
        end

        % Update for next step
        b1.error_prev_hdg = error;
    end

    % PWM Power Limit Checker
    sum_PWM = b1.PWMV + b1.PWML + b1.PWMR;
    if sum_PWM <= 450
    else
        scale_factor = 450/sum_PWM;
        b1.PWMV = scale_factor*b1.PWMV;
        b1.PWML = scale_factor*b1.PWML;
        b1.PWMR = scale_factor*b1.PWMR;
    end  

    % setting the PWM limits 
    b1.PWMV = min(250, b1.PWMV);
    b1.PWMV = uint8(max(0, b1.PWMV));
    b1.PWML = min(250, b1.PWML);
    b1.PWML = uint8(max(0, b1.PWML));
    b1.PWMR = min(250, b1.PWMR);
    b1.PWMR = uint8(max(0, b1.PWMR));

    disp(b1.PWMR)
    disp(b1.PWML)
end


