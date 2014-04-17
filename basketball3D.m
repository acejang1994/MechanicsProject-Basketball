
function res = basketball3D(itheta, iphi, iVel)

    v_x = iVel* cos(itheta/180*pi)*cos(iphi/180*pi);  % velocity in x direction
    v_y = iVel* sin(itheta/180*pi)*sin(iphi/180*pi);  % velocity in y direction
    v_z = iVel* sin(itheta/180*pi);   % velocity in z direction


    C_r = .82;   % coeffiecent of restitution
    m = .62;   % kg
    spin = [0, .4, 0];
   
    options = odeset('Events', @events);
    
    end_time = 2;
    step = end_time/100;
    
%     initial position = (.267329907, 2.747389194)
    [T, M] = ode45(@projectile, [0:step:end_time], [0.267329907, 0, 2.747389194, v_x, v_y, v_z], options);  % option will end the ode45 calculation when y position hits 0
    animate_func(T,M);
   M
   axis([0, 5, -2,2, 0,5]);

    %      13 ft 9 in = 4.2
    
%     coefficent of restitution is around .82~.88 W
    hold all   

    plot3(4.2, 0, 3.048, 'b.', 'MarkerSize', 30);
    plot3(M(:,1), M(:,2), M(:,3),'r', 'Linewidth', .1);           %plot position of the ball
    
%     set(gca,'fontsize',14)
%     title('Validation Graph','fontsize', 14)
%     xlabel('x coordinate (m)', 'fontsize', 14);
%     ylabel('y coordinate (m)', 'fontsize',14);
%     
%    x  4.572 , z 3.048 of the hoop
%     ball radius = .24m
%     hoop radius = .27m

    plot3([4.57 4.57 4.57 4.57, 4.57], [.9144 .9144 -.9144 -.9144, .9144], [3.048 4.118 4.118 3.048 3.048])
    line([4.57 4.57], [0 0] ,[0 3.048]);
    
    hoopx = M(end,1);
    hoopy = M(end,2);
    hoopz = M(end,3);
    res = 0;
    if(abs(4.2 - hoopx) < .11)
        if(hoopy < M(end-1,2))
            res = 1;
        end
    end
   
    vx = 0; 
    vy = 0;
    x_new = 0;
    y_new = 0;
    
%     line([4.57, 4.57], [3.048, 4.118], 'Color', 'g');
    for i = 1:length(T)
        if M(i,1) > 4.56 && M(i,1)< 4.58
            if M(i,2) < 4.118 && M(i,2)> 3.048
                vx = M(i,4);
                vy = M(i,5);
                x_new = M(i,1);
                y_new = M(i,2);
            end 
        end 
    end 
    
    vx_new = -vx*C_r;
    
    I_n = m*vx*(1+ C_r);
%     coeffiecent .7 leather 1.8 synthetic
    I_f = .8* I_n;
    I = 2/5*m*.12^2;
    
    vy_new = (I_f* + m*vy)/m;
    w_new = [0, -.3, 0];
    
%     [T1, M1] = ode45(@backboard, [T(end):step:3], [x_new, y_new 0, vx_new , vy_new, 0], options); 
%     if x_new > 0
%         plot(M1(:,1), M1(:,2), 'b', 'Linewidth', .1);
%     end
    
    %returns the range
  function [value,isterminal,direction] = events(t,X)
        value = X(3) - 3.048;       % Extract the current height.
%         value = X(2) - 4.1148;
        isterminal = 1;     % Stop the integration if height crosses zero.
        direction = -1;     % But only if the height is decreasing.
  end

    
    function res = projectile(t, W)  % defining the projectile motion
        P = W(1:3);             % position
        V = W(4:6);             % velocity

        dPdt = V;
        dVdt = acceleration(t, P, V);  

        res = [dPdt; dVdt];
    end 
    
    function res = acceleration(t, P, V)
      
        % F_drag = -C_d* A* p/2 * abs(v)^2  
        A = .041;   % r is in m
        C_d = .25;    % drag coeffiecent
        p = 1.2;     % kg/m
        m = .62;     % mass of the baseball
        v = norm(V);                % normalizes the vector
        F_dragx = -C_d* A* p/2 * V(1)*v;  % separating drag force in x and y
        F_dragz = -C_d* A* p/2 * V(3)*v;
        g = -9.8;
         
%         magnus force
        xhat = [1,0,0];
        zhat = [0,0,1];
        s = .5;
        
        li = s * cross(spin, V);
        liftx = dot(li, xhat);
        liftz = dot(li, zhat);

        
        res = [(F_dragx + liftx)/m; 0; g + (F_dragz + liftz)/m];  % x acceleration doesnt have g but in y it does
    end


    function res = backboard(t, W)  % defining the projectile motion
        P = W(1:3);             % position
        V = W(4:6);             % velocity

        dPdt = V;
        dVdt = acceleration_back(t, P, V);  

        res = [dPdt; dVdt];
    end 
    
    function res = acceleration_back(t, P, V)
      
        % F_drag = -C_d* A* p/2 * abs(v)^2  
        A = .041;   % r is in m
        C_d = .25;    % drag coeffiecent
        p = 1.2;     % kg/m
        m = .62;     % mass of the baseball
        v = norm(V);                % normalizes the vector
        F_dragx = -C_d* A* p/2 * V(1)*v;  % separating drag force in x and y
        F_dragz = -C_d* A* p/2 * V(3)*v;
        g = -9.8;
         
%         magnus force
        xhat = [1,0,0];
        zhat = [0,0,1];
        s = .5;
        
        li = s * cross(w_new, V);
        liftx = dot(li, xhat);
        liftz = dot(li, zhat);

        
        res = [(F_dragx + liftx)/m; 0; g + (F_dragz + liftz)/m];  % x acceleration doesnt have g but in y it does
    end
end