Attatched is the matlab code I used for the experimental part of the paper. It is unfortunately not especially well documented, but I believe it will run. You can examine "ekf.m" and "ukf.m" for the implementations of the extended- and unscented Kalman filter. I haven't looked at the code for several years, so any support might be difficult.
 
I would recommend you to examine the "ReBEL"-library for matlab, which includes several sequential state estimation algorithms (extended Kalman, unscented Kalman, particle filters, etc.). This library is probably much more mature and tested compared to my code.
http://choosh.csee.ogi.edu/rebel/
 
Regards,
Fredrik Orderud
