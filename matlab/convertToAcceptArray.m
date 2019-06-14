% Johann Diep (jdiep@student.ethz.ch) - April 2018
%
% This function enables arrays as input to function handles.
%
% Input:
%   - old_f: Old function with conventional function handle
%
% Output:
%   - f: New function with array-input function handle

function f = convertToAcceptArray(old_f)
    function r = new_f(X)
        X = num2cell(X);
        r = old_f(X{:});
    end 
    f = @new_f;
end