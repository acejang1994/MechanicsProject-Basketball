function animate_func(T, M)
    % animate the positions of the planets, assuming that the
    % columns of M are x1, y1, x2, y2.
    X1 = M(:,1);
    Y1 = M(:,2);
    Z1 = M(:,3);
    
    minmax = [min(X1), max(X1)+1, min(Y1), max(Y1),min(Z1), max(Z1)];
    
    for i=1:length(T)
        clf;
        axis([0, 5, -2,2, 0,5]);
        hold on;
        draw_func(X1(i), Y1(i), Z1(i));
        drawnow;
    end
end