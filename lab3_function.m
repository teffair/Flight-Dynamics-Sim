function xdot = lab3_function(t,x,C_L,g,rho, E_max, S, m)


%initial variables
C_D = C_L/E_max;
q=0.5*rho*(x(1)^2);


%functions
xdot(1,1) = -((q*S*C_D)/m)-g*sin(x(2));        %vel
xdot(2,1) = (((q*S*C_L)/m)-g*cos(x(2)))/x(1);  %angle
xdot(3,1) = x(1)*cos(x(2));                    %pos x
xdot(4,1) = x(1)*sin(x(2));                    %pos y

end


