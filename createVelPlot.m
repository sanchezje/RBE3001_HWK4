function [ velPlot ] = createVelPlot(measurementType)
% is actually an angular velocity versus time plot
% consumes either "Axis" or "Tip" to switch between angular velocity by
% axis and linear velocity of the tip
% produces a handler to the plot to be modified later

    switch measurementType
        case 'Axis'
            velPlot = plot(0,0,0,0,0,0);
            xlabel('Elapsed Time (sec)');
            ylabel('Velocity (rad/s)');
            legend('Axis 1', 'Axis 2', 'Axis 3');
            title('Angular Velocity vs. Time');

        case 'Tip'
            velPlot = plot(0,0,0,0,0,0);
            xlabel('Elapsed Time (sec)');
            ylabel('Velocity (mm/s)');
            legend('X velocity', 'Y velocity', 'Z velocity');
            title('Tip Velocity vs. Time');
    end

end