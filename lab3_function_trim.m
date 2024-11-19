% Estephany Barba Matta
% April 2024
% Lab 3: Aircraft Longitudinal Dynamics Simulation
% Trim function

function xdot = CessnaFuncTrim1 (t_trim,x_trim,C_L,g,rho, S, m, E_max)

% Formulas
C_D = C_L/E_max;
q=0.5*rho*(x_trim(1)^2);


% Functions
xdot(1,1) = -((q*S*C_D)/m)-g*sin(x_trim(2));        %velocity
xdot(2,1) = (((q*S*C_L)/m)-g*cos(x_trim(2)))/x_trim(1);  %angle
xdot(3,1) = x_trim(1)*cos(x_trim(2));                    %x position
xdot(4,1) = x_trim(1)*sin(x_trim(2));                    %y position

end

