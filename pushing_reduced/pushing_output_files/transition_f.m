close all, clear all,clc
n=10000;
x=linspace(0, 0.01, n);
Te=0.001;

for i=1:n
X=500*(x(i)-Te);
f(i)=sm_f(X)/(sm_f(X) + sm_f(1-(X)));
end

plot(x,f)

function y = sm_f(x)
    if(x>0)
        y=exp(-1/x);
    else
        y=0;
    end
end


