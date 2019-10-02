clear; clc;

load('FlightCommand.mat');

CurState = zeros(1,6);
CurState(1) = ViconPos(1,1); CurState(3) = ViconPos(1,3); CurState(5) = ViconPos(1,5);
dT = T/size(FlightCommand,1);

DynamicObject = DynamicModel();

for i = 1:size(FlightCommand,1)
    CurState = DynamicObject.BebopDiscreteModel(FlightCommand(i,:),CurState,dT);
    NewState(i,:) = CurState;
end

Pos = NewState(:,1:2:end);
Pos = Pos(1:50:end,:);
Vel = NewState(:,2:2:end);
Vel = Vel(1:50:end,:);

scatter3(Pos(:,1),Pos(:,2),Pos(:,3),10,'ro');
hold on;
quiver3(Pos(:,1),Pos(:,2),Pos(:,3),Vel(:,1),Vel(:,2),Vel(:,3),0.25,'k');