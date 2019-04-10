% Johann Diep (jdiep@student.ethz.ch) - April 2018

% This function enables arrays as input to function handles.

function f = convertToAcceptArray(old_f)
    function r = new_f(X)
        X = num2cell(X);
        r = old_f(X{:});
    end 
    f = @new_f;
end