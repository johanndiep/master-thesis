% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% This function calculates the objective norm of the triangulated position 
% from six range measurements.
%
% Input:
%   - RangeArray: Range measurements in form (6 x 1)
%   - AnchorPos: The position of the 6 anchors in form (6 x 3) 
%   - Tx: Triangulated position in form (3 x 1)
%
% Output: 
%   - ObjNorm: Objective norm

function ObjNorm = getTriangulationNorm(RangeArray,AnchorPos,Tx)
    ObjFunc = zeros(6,1);
    for i = 1:6
        ObjFunc(i) = norm(Tx-AnchorPos(i,:)')-RangeArray(i);
    end
    ObjNorm = norm(ObjFunc);
end