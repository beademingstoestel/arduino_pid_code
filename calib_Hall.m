clear; clc; close all;

angle=[28, 42, 52, 61,70 77] ;
V_m=[100, 200, 300, 400, 500, 600];

[p,S]=polyfit(angle,V_m,2);

p(end) = p(end)+23.77;

angle_lin=linspace(0,80);
V_m_lin=polyval(p,angle_lin);


figure;hold on
plot(angle,V_m,'ro')
plot(angle_lin,V_m_lin,'b')
S.R
