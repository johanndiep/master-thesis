% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This program implements the method of anchor positions estimation
% similiar as described in the paper "Iterative approach for anchor 
% configuration of positioning systems" by Mathias Pelka, Grigori Goronzy 
% and Horst Hellbrueck.
%
% Placement of the anchors with the following assumptions:
%   - Anchor 1 is set to be the origin of the coordinate system.
%   - Anchor 3 and 5 are fixed on the same height as anchor 1.
%   - Anchor 2, 4 and 6 are fixed at a known constant height.
%   - Anchor 5 is assumed to be on the same axis with anchor 1 
%     without loss of generality.
%   - Top anchors are assumed to have same x/y-coordinates as 
%     bottom anchors.
% 
% The anchors are distributed as follows where p1/p2/p3 are the unknown parameters:
%   - Pole 1: Anchor 1 (0,0,0), Anchor 2 (0,0,h)
%   - Pole 2: Anchor 3 (p1,p2,0), Anchor 4 (p1,p2,h)
%   - Pole 3: Anchor 5 (0,p3,0), Anchor 6 (0,p3,h)
%
% Furthermore, the following points need to be investigated:
%   - How to deal with ranging offset resulting in positioning inaccuracy?
%   - Are the assumptions too limiting?
%   - Test out different placements of the poles.
%   - Relevance of the variable NrIterAv. 
%     [Empirically set to 100, since larger values does not result in an increase 
%     of accuracy but in a higher computational effort.]
%   - Can one be sure, that after re-interrogating in case of a wrong tag number,
%     no wrong tag number will follow?
%   - Any delays introduced due to reading processing?
%     [If any delays are introduced, these will not have large influence since
%      the anchors are not moving.]
%   - Closing procedure right?
%   - Check under which condition outliers are removed.
%
% Step-by-Step:
%   1. Place pole 1 and pole 3 such that the corresponding anchors have 
%      the same orientation.
%   2. Place pole 2 such that its anchors are opposite directed compared to 
%      the anchors from pole 1/3 and have positive x/y-coordinates.
%   3. Power up each anchor, the modules should be in Anchor mode already.
%      This can also be tested in Terminal using the following picocom 
%      command and a computer-attached Sniffer module: sudo picocom /dev/ttyACM*
%   4. Connect the Sniffer node to the computer and run the following
%      command in terminal: sudo chmod 666 /dev/ttyACM*
%   5. Run the following script.

clear; clc;

%% Data acquisition

NrIterAv = 100;
RangeMeasObj = RangeMeasurements();
AnchorRangeMean = RangeMeasObj.AnchorSelfRanging(NrIterAv)/1000;

save('AnchorRangeMean.mat','AnchorRangeMean');

%% Parameters

h = 2.156; % distance between the top and bottom anchor
s = zeros(1,3); % initialization of parameter p1/p2/p3

%% Optimization

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% objective function
ObjNorm = @(p) getObjectiveNorm(AnchorRangeMean,h,p(1),p(2),p(3));

p = fmincon(ObjNorm,s,[],[],[],[],[0,0,0],[6,6,6],[],options);

%% Plotting and Results

AnchorPos = [0,0,0;0,0,h;p(1),p(2),0;p(1),p(2),h;0,p(3),0;0,p(3),h];
save('AnchorPos.mat','AnchorPos');

figure()

title("Bebop Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([0,4]);
ylim([0,4]);
zlim([0,2.5]);
hold on;

scatter3(AnchorPos(:,1),AnchorPos(:,2),AnchorPos(:,3),10,'ks');

for i = 1:size(AnchorPos,1)
    text(AnchorPos(i,1)+0.1,AnchorPos(i,2)+0.1,AnchorPos(i,3)+0.1, ...
        "Anchor "+int2str(i));
end

quiver3(0,0,0,1,0,0,0.5,'r','LineWidth',2);
quiver3(0,0,0,0,1,0,0.5,'g','LineWidth',2);
quiver3(0,0,0,0,0,1,0.5,'b','LineWidth',2);

grid on;
hold off;
