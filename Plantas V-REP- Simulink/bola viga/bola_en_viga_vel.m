function dxdt=bola_en_viga_vel(t,x,flag,m,Iv,Ib,g,L,R,u)


dxdt(1,1)=x(3);
dxdt(2,1)=u;
dxdt(3,1)=(m*x(1)*u^2-m*g*sin(x(2))/(m+(Ib/R^2)));