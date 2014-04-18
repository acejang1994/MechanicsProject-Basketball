
function res = iteration1(angle, iVel)

    v_x = iVel* cos(angle/180*pi);  % velocity in x direction
    v_y = iVel* sin(angle/180*pi);  % velocity in y direction
    
    C_r = .82;   % coeffiecent of restitution
    m = .62;   % kg
    
    options = odeset('Events', @events);
    
    end_time = 2;
    step = end_time/100;
    
%     initial position = (.267329907, 2.747389194)
    [T, M] = ode45(@projectile, [0:step:end_time], [0.267329907, 2.747389194, 0, v_x, v_y, 0], options);  % option will end the ode45 calculation when y position hits 0
%     animate_func(T,M);
   M

    %      13 ft 9 in = 4.2
    
%     coefficent of restitution is around .82~.88 
    hold all
    plot(4.2, 3.048, 'b.', 'MarkerSize', 30);
    plot(M(:,1), M(:,2), 'r', 'Linewidth', .1);           %plot position of the ball
    
    set(gca,'fontsize',14)
    title('Validation Graph','fontsize', 14)
    xlabel('x coordinate (m)', 'fontsize', 14);
    ylabel('y coordinate (m)', 'fontsize',14);
    
%     
%      load vali
%     plot(vali(1,:), vali(2,:), 'r-') 
%     4.572 , 3.048
%     ball radius = .24m
%     hoop radius = .27m

    hoopx = M(end,1);
    hoopy = M(end,2);
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
    
    for i = 1:length(T)
        if M(i,1) > 4.54 && M(i,1)< 4.6
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
    w_new = [0, 0, -.4];
    
    [T1, M1] = ode45(@projectile, [T(end):step:3], [x_new, y_new 0, vx_new , vy_new, 0], options); 
    if x_new > 0
        plot(M1(:,1), M1(:,2), 'b', 'Linewidth', .1);
    end
    
    %returns the range
  function [value,isterminal,direction] = events(t,X)
        value = X(2) - 3.048;       % Extract the current height.
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
        F_dragy = -C_d* A* p/2 * V(2)*v;
        g = -9.8;
         
%         magnus force
        xhat = [1,0,0];
        yhat = [0,1,0];
        s = .5;
        li = s * cross([0, 0, .4], V);
        liftx = dot(li, xhat);
        lifty = dot(li, yhat);

        
        res = [(F_dragx + liftx)/m; g + (F_dragy + lifty)/m; 0];  % x acceleration doesnt have g but in y it does
    end
end