function new_val = checkLimits(val, mmin, mmax, name)

new_val = val;

if val > mmax
    if nargin == 4
        warning('Maximum value for %s is %d. Automatically set to %d.', name, mmax, mmax);
    else
       warning('Maximum value is %d. Automatically set to %d.', mmax, mmax);
    end
    new_val = mmax;
elseif val < mmin
    if nargin == 4
        warning('Minimum value for %s is %f. Automatically set to %f.', name, mmin, mmin);
    else
        warning('Minimum value is %f. Automatically set to %f.', mmin, mmin);
    end
    new_val = mmin;
end 