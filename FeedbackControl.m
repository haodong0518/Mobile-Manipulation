function Ve = FeedbackControl(Tse, Tse_d,Tse_dnext,Kp,Ki,dt)
% Tse is the current actual end effector 
Xsb = Tse;
% Tse_d is e-e current reference configuration
Xsd = Tse_d;
% Tse_dnext is e-e next timestep reference configuration
Xsd_next = Tse_dnext;

Xerr = se3ToVec(MatrixLog6(Xsb\Xsd));
Kp_matrix = Kp * eye(6);
Ki_matrix = Ki* eye(6);
% Transfer the twist from desired frame to end-effector {b} frame
Ad_bracket = Adjoint(Xsb\Xsd);
Vd= (1/dt)* se3ToVec(MatrixLog6(Xsd\Xsd_next));
Ve = Ad_bracket*Vd + Kp_matrix*Xerr + Ki_matrix*Xerr*dt;

end