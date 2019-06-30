% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Plotting anchor setup.
%
% Input:
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]
%   - NumberOfAnchors: Amount of anchors in the setup

function AnchorSetupPlot(AnchorPositions,NumberOfAnchors)
    figure()
    hold on
    title("Tinamu Labs Flying Machine Arena");
    xlabel("x-Axis [m]");
    ylabel("y-Axis [m]");
    zlabel("z-Axis [m]");
    grid on

    scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),'MarkerEdgeColor','k','MarkerFaceColor',[0,0,0]);

    if NumberOfAnchors == 8
        AnchorCombinations = [1,5;4,8;2,6;3,7];
        for i = 1:size(AnchorCombinations,1)
            line([AnchorPositions(AnchorCombinations(i,1),1),AnchorPositions(AnchorCombinations(i,2),1)],[AnchorPositions(AnchorCombinations(i,1),2),AnchorPositions(AnchorCombinations(i,2),2)],[AnchorPositions(AnchorCombinations(i,1),3),AnchorPositions(AnchorCombinations(i,2),3)],'Color',[.9412,.9412,.9412],'LineWidth',3);
        end
    elseif NumberOfAnchors == 6
        for i = 1:size(AnchorPositions,1)
            if mod(i,2) == 1
                line([AnchorPositions(i,1),AnchorPositions(i+1,1)],[AnchorPositions(i,2),AnchorPositions(i+1,2)],[AnchorPositions(i,3),AnchorPositions(i+1,3)],'Color',[.9412,.9412,.9412],'LineWidth',3);
            end
        end
    end

    for i = 1:size(AnchorPositions,1)
        text(AnchorPositions(i,1)+0.1,AnchorPositions(i,2)+0.1,AnchorPositions(i,3)+0.1,"Anchor "+int2str(i));
    end
end