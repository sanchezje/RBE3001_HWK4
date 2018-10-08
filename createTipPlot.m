function [ tipPlot ] = createTipPlot()%setpoint_array)
%CREATETIPPLOT Plot x-y-z position of tip
%   initialize the plot to be used to plot the tip posn vs time
%   one plot will hold the logged x-y-z positions of the tip
%   the superimposed plot will show the target setpoints

    % USE THIS INITIALIZER IF WORKING WITH ONLY DATA
    % init a plot, with *labelled* axis
     tipPlot = plot(0,0,0,0,0,0);
     xlabel('Elapsed Time (s)'); % elapsed time in seconds here
     ylabel('Position (mm)');
     legend('X Position', 'Y Position', 'Z Position');
     title('Tip XYZ Position vs Time');
     hold on;
    
    % USE THIS INITIALIZER IF WORKING WITH DATA AND SET POINTS
    % init a plot, with *labelled* axis
%    tipPlot = plot(0,0,0,0,0,0,0,0,0,0,0,0);
%    xlabel('Elapsed Time (s)'); % elapsed time in seconds here
%    ylabel('Encoder Position (ticks)');
%    legend('X Position', 'Y Position', 'Z Position', 'X Setpoint', 'Y Setpoint', 'Z Setpoint');
%    title('Tip XYZ Position vs Time');
%    hold on;
    
    % setpoint_array is the status packets of the 3 locations to go to
    % it contains tip positions and time
%     timeSet = setpoint_array(:, 16);
%     xLengths = setpoint_array(:, 17);
%     zLengths = setpoint_array(:, 19);
    
    %plot 1 is the continuous x position
    set(tipPlot(1), 'XData', 0, 'YData', 0, 'LineWidth',1,'Color',[1 0 0]);
    %plot 2 is the continuous y position
    set(tipPlot(2), 'XData',0, 'YData', 0 ,'LineWidth',1,'Color',[0 1 0]);
    %plot 3 is the continous z position
    set(tipPlot(3), 'XData', 0, 'YData', 0,'LineWidth',1,'Color',[0 0 1]);
    
    % UNCOMMENT THESE LINES IF USING SET POINTS
%     %plot 4 is the X setpoint
%     set(tipPlot(4), 'XData', 0, 'YData', 0, 'Marker','*','LineWidth',1,'Color',[0.64 0.08 0.18]);
%     %plot 2 is the Y setpoint
%     set(tipPlot(5), 'XData',0, 'YData', 0 ,'Marker','*','LineWidth',1,'Color',[0.47 0.67 0.19]);
%     %plot 3 is the Z setpoint
%     set(tipPlot(6), 'XData', 0, 'YData', 0,'Marker','*','LineWidth',1,'Color',[0 0.45 0.75]);    
    
    hold off;

end

