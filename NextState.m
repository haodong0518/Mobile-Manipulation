% 12 vector (3 variable for chassis , 5 variable for arm configuration, 4 for wheel angle)
% 9 vector (4 vector for speed u, 5 vector for arm joint speed theta_dot)
function [chassis_pose_new,arm_pose_new,wheel_angle_new] = NextState(current_state, controls, dt, u_max)
% min_speed = -max_speed; 
% controls(controls>max_speed) = max_speed;
% controls(controls<min_speed) = min_speed;


le = 0.235;
w = 0.15;
r = 0.0475;
F = (r/4) * [[-1/(le+w),1/(le+w),1/(le+w),-1/(le+w)];[1,1,1,1];[-1,1,-1,1]];
chassis_pose = current_state(1:3,:);
arm_pose= current_state(4:8,:);
wheel_angle = current_state(9:12,:);
wheel_v = controls(1:4,:);
arm_v = controls(5:9,:);
wheel_v = max(min(wheel_v, u_max), -u_max);

Vb = F*wheel_v;
% Twist_b_size6 = zeros(6,1);
% Twist_b_size6(3:5,:)= Twist_b;
% Twist_se3_b = VecTose3(Twist_b_size6);
% T_bk_bk_next = MatrixExp6(Twist_se3_b);
% [r_matrix,p_matrix] = TransToRp(T_bk_bk_next);
% w_so3 =MatrixLog3(r_matrix);
% rotation = so3ToVec(w_so3);
% [omghat, dtheta] = AxisAng3(rotation);
% % disp(p_matrix(1:2,:));
% delta_q= [dtheta;p_matrix(1:2,:)];
% chassis_pose_new = chassis_pose + delta_q;
% disp(chassis_pose_new);

% Chassis coordinate change in body frame
if Vb(1) == 0
    dq_b = [0; Vb(2); Vb(3)];
else
    dq_b = [Vb(1); ...
           (Vb(2)*sin(Vb(1)) + Vb(3)*(cos(Vb(1)) - 1)) / Vb(1); ...
           (Vb(3)*sin(Vb(1)) + Vb(2)*(1 - cos(Vb(1)))) / Vb(1)];
end

% Chassis change in fixed frame
phik = chassis_pose(1);
T = [1, 0, 0; 0, cos(phik), -sin(phik); 0, sin(phik), cos(phik)];
dq = T * dq_b;

    % Update configurations
chassis_pose_new = chassis_pose + dq * dt;
wheel_angle_new = wheel_angle + wheel_v * dt;
arm_pose_new = arm_pose + arm_v * dt;

% Make sure the arm pose is in [-pi, pi]
% while or(arm_pose_new>pi,arm_pose_new < -pi) 
%     arm_pose_new = arm_pose_new - sign(arm_pose_new)*2*pi;
% end 
end 

