function [x y] = logx_hist(data, bins, maximum)

%x = sort(x);

log_x   = log10(data);
log_max = log10(maximum);
step    = log_max / bins;

for i=1:bins-1
    tmp        = log_x <= step * i;
    log_x(tmp) = inf;
    
    count(i)   = sum(tmp);
    val(i)     = 10^(step * i);
end
count(bins) = sum(log_x < inf);
val  (bins) = 10^(step * bins);

x = val;
y = count;

%semilogx(val, count);

    
