function [s, dot_s, ddot_s] = trapezoidal_vel_profile(t, ti, tf, n)
%Variant with velocity

%s goes from 0 to the length of the segment (n). Before reaching the
%segment, the parameter s is equal to 0. After traveling the segment, the
%parameter is equal to n.

if(t<=ti)
    s=0;
    dot_s=0;
    ddot_s=0;
    return
end
if(t>tf)
    s=n;
    dot_s=0;
    ddot_s=0;
    return
end

%elimination of initial time dependence

t=t-ti;
tf=tf-ti;

%velocity is chosen as 3/2 of the minimal value
dot_sc=1.5*n/tf;
tc=(dot_sc*tf-n)/dot_sc;

%Computation of acceleration
ddot_sc=dot_sc^2/(dot_sc*tf-n);

%Initialization
s=0;
dot_s=0;
ddot_s=0;

%Time law
switch(true)  
        
    case t>=0 && t<=tc
        s=1/2*ddot_sc*t^2;
        dot_s=(dot_sc/tc)*t;
        ddot_s=ddot_sc;
        
    case t>tc && t<=tf-tc
        s=ddot_sc*tc*(t-tc/2);
        dot_s=dot_sc;
        ddot_s=0;
        
    case t>tf-tc && t<=tf
        s=n-1/2*ddot_sc*(tf-t)^2;
        dot_s=-(dot_sc/tc)*(t-tf);
        ddot_s=-ddot_sc;
end

end

