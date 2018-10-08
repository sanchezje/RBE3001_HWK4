function [ posnPlot ] = createPosnPlot()
% is actually an angle versus time plot
% consumes nothing
% produces a handler to the plot to be modified later

    posnPlot = plot(0,0,0,0,0,0);
    xlabel('Elapsed Time (sec)');
    ylabel('Position (rad)');
    legend('Axis 1', 'Axis 2', 'Axis 3');
    title('Angular Positions vs. Time');

end