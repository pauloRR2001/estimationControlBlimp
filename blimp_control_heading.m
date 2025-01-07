function [b1] = blimp_control_heading(b1, time)

    % Calculate current speed
    speed = (b1.pos(1:2) - b1.pos_old(1:2))/time.dt;

    des_alt = 1200; % desired altitude
    b1.forward_spd = 0;  % desired speed

    alt_margin = 200; % margin for altitude
    spd_margin = 20;  % margin for speed

    %b1 = dynamic_heading(b1, time);

%     % Determine necessary modes
%     if abs(b1.pos(3) - des_alt) > alt_margin
%         % Altitude control
%         b1 = dynamic_vertical(b1, time);
%     end

%     if abs(speed - b1.forward_spd) > spd_margin
%         % Speed control
%         b1 = dynamic_spd(b1, time);
%     else
%         % Heading control
%         b1 = dynamic_heading(b1, time);
%     end
    b1 = dynamic_heading(b1, time);

    alt_des = 1200;
    alt = b1.pos(3);

if (alt_des-alt)>100
   b1 = dynamic_vertical(b1,time);
end

end


