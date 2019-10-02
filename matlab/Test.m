clear; clc;

CurState = zeros(1,6);
dT = 0.05;

u(:,1) = [ones(100,1);zeros(100,1);-ones(100,1);zeros(100,1)];
u(:,2) = [zeros(100,1);ones(100,1);zeros(100,1);-ones(100,1)];
u(:,3) = [ones(20,1);zeros(380,1)];

DynamicObject = DynamicModel();

for i = 1:size(u,1)
    CurState = DynamicObject.BebopDiscreteModel(u(i,:),CurState,dT);
    NewState(i,:) = CurState;
end

Pos = NewState(:,1:2:end);
Pos = Pos(1:5:end,:);
Vel = NewState(:,2:2:end);
Vel = Vel(1:5:end,:);


scatter3(Pos(:,1),Pos(:,2),Pos(:,3),10,'ro');
hold on;
quiver3(Pos(:,1),Pos(:,2),Pos(:,3),Vel(:,1),Vel(:,2),Vel(:,3),0.25,'k');