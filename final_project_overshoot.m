% Final Step 
%1.calculate the control law using FeedbackControl and generate the wheel and joint controls using pinv(J)
%2.send the controls, configuration, and timestep to NextState to calculate the new configuration;
%3.store every kth configuration for later animation(note that the reference trajectory has k reference configurations per 0.01 second step) 
%4.Store X_err 6 vector for plotting evolution of error 

Tsc_init = [[1,0,0,1];[0,1,0,0];[0,0,1,0.025];[0,0,0,1]];
Tsc_goal = [[0,1,0,0];[-1,0,0,-1];[0,0,1,0.025];[0,0,0,1]];
Tse_init = [[0,0,1,0];[0,1,0,0];[-1,0,0,0.5];[0,0,0,1]];
Tce_standoff = [[1,0,0,0];[0,1,0,0];[0,0,1,0.1];[0,0,0,1]];
T_b0 = [[1,0,0,0.1662];[0,1,0,0];[0,0,1,0.0026];[0,0,0,1]];
M_0e = [[1,0,0,0.033];[0,1,0,0];[0,0,1,0.6546];[0,0,0,1]];
B_list = [[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0],[0;-1;0;-0.3526;0;0],[0;-1;0;-0.2176;0;0],[0;0;1;0;0;0]];

% the reference trajectory is fixed, the initial reference Tse is given
% The only thing change is w_speed and v_speed 
w_max = 0.5;
v_max = 0.5;
k=1;
diary('output_log.txt');
disp('Generating Reference Trajectory');
traj_test = TrajectoryGenerator(Tse_init, Tsc_init,Tsc_goal,Tce_standoff,k,w_max,v_max);
disp('Finished Generating Reference Trajectory ');
gripper_state = traj_test(:,13);

% This parameter is for Next State function
dt = 0.01;
max_speed = 5;

arm_init = [0;pi/2;0.2;-2;1];
Tse = FKinBody(M_0e, B_list,arm_init);
current_state = [0;0;0;   0;pi/2;0.2;-2;1;     0;0;0;0]; 

% Initialize the current state 
configurations = zeros(size(traj_test,1),13);
Xerr_conf = zeros(size(traj_test,1),6);

%--------------------------
Kp=50;
Ki=0;

for i = 1:size(traj_test,1)-1
  
    
Tse_d = traj_test(i,1:12);
reshap_Tse_d= [[Tse_d(1), Tse_d(2), Tse_d(3), Tse_d(10)]; 
               [Tse_d(4), Tse_d(5), Tse_d(6), Tse_d(11)]; 
               [Tse_d(7), Tse_d(8), Tse_d(9), Tse_d(12)];
               [0       , 0       ,        0,        1]];
Tse_dnext =  traj_test(i+1,1:12);
reshap_Tse_dnext= [[Tse_dnext(1), Tse_dnext(2), Tse_dnext(3), Tse_dnext(10)]; 
               [Tse_dnext(4), Tse_dnext(5), Tse_dnext(6), Tse_dnext(11)]; 
               [Tse_dnext(7), Tse_dnext(8), Tse_dnext(9), Tse_dnext(12)];
               [0       ,     0,        0,        1]];
           
Xerr = se3ToVec(MatrixLog6(Tse\reshap_Tse_d));
Xerr_conf(i,:) = transpose(Xerr);
%disp(size(reshap_Tse_d));
% Calculate the Control
Ve = FeedbackControl(Tse, reshap_Tse_d,reshap_Tse_dnext,Kp,Ki,dt);
robot_pose8 = transpose(current_state(1:8,:));
J_total = Find_Je(robot_pose8, B_list,Tse);
controls = pinv(J_total,1e-3)*Ve;
[chassis_pose_new,arm_pose_new,wheel_angle_new] = NextState(current_state, controls, dt, max_speed);

% Update the current state and current e-e position T_se
current_state = [chassis_pose_new;arm_pose_new;wheel_angle_new];
% disp(size(current_state));
theta = chassis_pose_new(1,:);
x = chassis_pose_new(2,:);
y = chassis_pose_new(3,:);
T_sb = [[cos(theta),-sin(theta),0,x];[sin(theta),cos(theta),0,y];[0,0,1,0.0963];[0,0,0,1]];
%disp(size(arm_pose_new));
T_0e = FKinBody(M_0e, B_list, arm_pose_new);
Tse = T_sb*T_b0*T_0e;

% Save the configurations
configurations(i,1:3)=  chassis_pose_new; 
configurations(i,4:8)=  arm_pose_new;
configurations(i,9:12)=  wheel_angle_new;

%configurations(13,i) = 0;
%store Xerr
%store configuration! 

end
configurations(:,13)= gripper_state;
disp('writing Xerr and CoppelisSim into excel files');
csvwrite('final_project_test_overshoot.csv',configurations);
csvwrite('final_project_Xerr_config_overshoot.csv',Xerr_conf);
disp('Done');
diary off
