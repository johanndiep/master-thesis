% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the norm of the objective function for 
% anchor positions estimation.
%
% The anchors are distributed as follows:
%   - Pole 1: Anchor 1 (0,0,0), Anchor 2 (0,0,h)
%   - Pole 2: Anchor 3 (p1,p2,0), Anchor 4 (p1,p2,h)
%   - Pole 3: Anchor 5 (0,p3,0), Anchor 6 (0,p3,h)
%
% Input:
%   - AnchorRangeMean: Measured distance between the anchors in
%     symmetric form (6 x 6)
%   - h: Distance between the antenna of the top and bottom anchor 
%   - p1/p2/p3: Unknown position values
%
% Output:
%   - ObjNorm: Norm of the objective function

function ObjNorm = getObjectiveNorm(AnchorRangeMean,h,p1,p2,p3)
    AnchorPos = [0,0,0;0,0,h;p1,p2,0;p1,p2,h;0,p3,0;0,p3,h];
    
    Iterdex = 1;
    
    ObjFunc = zeros(36,1);
    for i = 1:6
        for j = 1:6
            ObjFunc(Iterdex,1) = norm(AnchorPos(i,:)-AnchorPos(j,:))- ...
                AnchorRangeMean(i,j);
            Iterdex = Iterdex+1;
        end
    end
    
    ObjNorm = norm(ObjFunc);
end