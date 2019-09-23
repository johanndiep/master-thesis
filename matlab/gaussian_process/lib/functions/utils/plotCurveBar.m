% Johann Diep (jdiep@student.ethz.ch), Mo Chen (sth4nth@gmail.com) - July 2019
%
% Plot 1d curve and variance.
%
% Input:
%   - X: Testing data in form (1 x n) 
%   - Y: Prediction data in form (1 x n) 
%   - Std: Standard deviation (1 x n)

function plotCurveBar(X,Y,Std)    
    color = [255,228,225]/255;
    [X,idx] = sort(X);
    Y = Y(idx);
    Std = Std(idx);

    fill([X,fliplr(X)],[Y+Std,fliplr(Y-Std)],color,'LineStyle','none');
    hold on;
    plot(X,Y,'r-');
    alpha(0.7)
    hold off
    axis([X(1),X(end),-inf,inf])
end