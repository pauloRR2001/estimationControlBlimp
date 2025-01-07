function [b1] = blimp_control_vertical(b1, time)


% vertical control 
alt_des = 1200;
alt = b1.pos(3);

if (alt_des-alt)>100
   b1 = dynamic_vertical(b1,time);
end

% heading control
b1.heading_dir = b1.heading_dir(1:2);
b1.heading_vec = b1.heading_vec(1:2);

b1.heading_dir = b1.heading_dir/norm(b1.heading_dir);
b1.heading_vec = b1.heading_vec/norm(b1.heading_vec);

heading_error = acosd(dot(b1.heading_vec, b1.heading_dir));

% if abs(heading_error) > b1.fwd_deg
%      b1 = dynamic_heading(b1,time);
% end 



end
