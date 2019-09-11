function ObjNorm = getTriangulationNorm(RangeArray,AnchorPos,Tx)
    ObjFunc = zeros(6,1);
    for i = 1:6
        ObjFunc(i) = norm(Tx-AnchorPos(i,:)')-RangeArray(i);
    end
    ObjNorm = norm(ObjFunc);
end