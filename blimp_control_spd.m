function [b1] = blimp_control_spd(b1, time)
    if isnan(b1.pos(1)) 
        return
    end

    % Calculate current heading error
    b1.heading_dir = b1.heading_dir(1:2);
    b1.heading_vec = b1.heading_vec(1:2);

    b1.heading_dir = b1.heading_dir/norm(b1.heading_dir);
    b1.heading_vec = b1.heading_vec/norm(b1.heading_vec);

    heading_error_b1 = acosd(dot(b1.heading_vec, b1.heading_dir));

    des_alt = 1200; % desired altitude
    % des_speed = 100; % desired speed
    % des_heading = 0; % desired heading

    alt_margin = 100; % margin for altitude
  
    % Determine necessary modes
%     if abs(heading_error_b1) > b1.fwd_deg
%         % Heading control
%         b1 = dynamic_heading(b1, time);
%     else
%         % Speed control
     b1 = dynamic_spd(b1, time);
%     end
    
    if abs(b1.pos(3) - des_alt) > alt_margin
        % Altitude control
        b1 = dynamic_vertical(b1, time);
    end

    % % setting the PWM limits 
    % b1.PWMV = min(250, b1.PWMV);
    % b1.PWMV = uint8(max(0, b1.PWMV));
    % b1.PWML = min(250, b1.PWML);
    % b1.PWML = uint8(max(0, b1.PWML));
    % b1.PWMR = min(250, b1.PWMR) * .9;
    % b1.PWMR = uint8(max(0, b1.PWMR));

end



