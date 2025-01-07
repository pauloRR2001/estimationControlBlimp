function [b1] = blimp_control_wp(b1, time)

if isnan(b1.pos(1))
    return
end

b1.heading_dir = (b1.WP(b1.curWP,:) - b1.pos) / norm(b1.WP(b1.curWP,:) - b1.pos);
b1.heading_dir = b1.heading_dir(1:2);
b1.heading_vec = b1.heading_vec(1:2);

b1.heading_dir = b1.heading_dir/norm(b1.heading_dir);
b1.heading_vec = b1.heading_vec/norm(b1.heading_vec);

heading_error = acosd(dot(b1.heading_vec, b1.heading_dir));


if abs(norm(b1.pos - b1.WP(b1.curWP,:))) < 900
    b1.forward_spd = 40;
end


if abs(heading_error) < b1.fwd_deg

    % Iterate to next waypoint if within distance margin
    if (abs(norm(b1.pos - b1.WP(b1.curWP,:))) < b1.dist_mar)
        b1.curWP = b1.curWP + 1;
    end

    % Speed Control Mode
    b1 = dynamic_spd(b1, time);

else
    % Heading Control Mode
    b1 = dynamic_heading(b1, time);
end


alt_des = 1200;
alt = b1.pos(3);
if abs(alt_des-alt) > 100
    b1 = dynamic_vertical(b1,time);
end 


end