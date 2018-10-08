function [ outboundStatusPacket ] = plotTipPosn( inboundStatusPacket, plotToModify, setpointArray)
%PLOTTIPPOSN Summary of this function goes here
%   Detailed explanation goes here

    persistent setpointData;
    setpointData = [setpointData; setpointArray];
    persistent timeData;
    
    persistent xLengData;
    persistent yLengData;
    persistent zLengData;
    
    timeData = [timeData ; inboundStatusPacket(16)/1000]; % time is stored in ms, we want s
    xLengData = [xLengData; inboundStatusPacket(17)];
    yLengData = [yLengData; inboundStatusPacket(18)];
    zLengData = [zLengData; inboundStatusPacket(19)];

    %plot 1 is the continuous x position
    set(plotToModify(1), 'XData', timeData, 'YData', xLengData);
    %plot 2 is the continuous z position
    set(plotToModify(2), 'XData',timeData, 'YData', yLengData);
    % plot the current setpoints
    set(plotToModify(3), 'XData', timeData, 'YData', zLengData);
    
    % UNCOMMENT THIS TO ITERATIVELY PLOT SET POINTS
    %set(plotToModify(4),'XData', timeData, 'YData', setpointData(:,1));
    %set(plotToModify(4),'XData', timeData, 'YData', setpointData(:,2));
    %set(plotToModify(4),'XData', timeData, 'YData', setpointData(:,3));

    %refreshdata
    %drawnow
    
    outboundStatusPacket = inboundStatusPacket;
end

