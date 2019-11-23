clear all
close all
clc


% Parametros del sistema bola plato 
T=0.06; % tiempo de muestreo




Mb=1; % masa del riel [kg]
L=1; % longitud del riel [m]
m=0.1; % masa de la bola [kg]
R=0.034; % radio de la bola [m]
Iv=(Mb*(L)^2)/12; %momento de inercia de una barra [kg m^2];
Ib=0.4*m*R^2; % momento de inercia de la bola [kg m^2]
g=9.81; % gravedad [m/s^2]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% syms x1 x2 x3 x4 u Mb L m R Iv Ib g real
syms x1 x2 x3 x4 u real
% estados
% x1: distancia de la bola en x
% x2: angulo del motor
% x3: velocidad de la bola en x
% x4: velocidad angular del motor
% u: señal de control [torque] 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sistema bola viga no lineal
f1=x3;
f2=x4;
f3=(m*x1*x4^2-m*g*sin(x2)/(m+(Ib/R^2)));
f4=(u-(m*g*x1*cos(x2))-(2*m*x1*x3*x4))/((Iv+(m*x1^2)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%salidas

h=x1;
hm1=x1;
hm2=x2;
hm3=x3;
hm4=x4;

% puntos de operacion

ref=0;
x1ss=0;
x2ss=0;
x3ss=0;
x4ss=0;
uss=ref;
A2=jacobian([f1;f2;f3;f4],[x1 x2 x3 x4]);
B2=jacobian([f1;f2;f3;f4],u);
C2=jacobian(h,[x1 x2 x3 x4]);
D2=jacobian(h,u);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linealizacion

As=[diff(f1,x1) diff(f1,x2) diff(f1,x3) diff(f1,x4);diff(f2,x1) diff(f2,x2) diff(f2,x3) diff(f2,x4);diff(f3,x1) diff(f3,x2) diff(f3,x3) diff(f3,x4);diff(f4,x1) diff(f4,x2) diff(f4,x3) diff(f4,x4)];
A1=subs(As,[x1 x2 x3 x4 u],[x1ss x2ss x3ss x4ss uss]);
A=double(A1);
Bs=[diff(f1,u);diff(f2,u);diff(f3,u);diff(f4,u)];
B1=subs(Bs,[x1 x2 x3 x4 u],[x1ss x2ss x3ss x4ss uss]);
B=double(B1);
C=[1 0 0 0;0 1 0 0];
CA=[1 0 0 0 0];
D=[0;0];

% descetización del sistema
PI=ss(A,B,C,D);
PIS=tf(PI);
sistemad=c2d(PI,T,'zoh');
[AD,BD,CD,DD]=ssdata(sistemad);
controlabilidad=ctrb(A,B); %matriz de controlabilidad
rangoc=rank(controlabilidad);
polosd=eig(AD);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% polos observador
ts=2;
Mp=0.01;
sigma=4/ts;
P1=-sigma;
P2=8*P1';
P3=10*real(P1);
P4=20*real(P1);
P=[P1;P2];
p3=[P;P3];
polos=[p3;P4]';
% Pz=exp(T*polos);

Pzo=exp(5*T*polos);
C1=[1 0 0 0];

AA=[AD,(0*C1)';(C1*AD),1];
BB=[BD;C1*BD];


observabilidad=[CD;CD*AD;CD*AD*AD;CD*AD*AD*AD]; % matrices de observabiidad 
rangoob=rank(observabilidad);  %rango de matriz de observabilidad
% parametros LQR con integrador y sin integrador
QA=[300 0 0 0 0;0 1 0 0 0;0 0 800 0 0;0 0 0 1 0;0 0 0 0 0.1];
Q=[10 0 0 0 ;0 1 0 0;0 0 10 0;0 0 0 1];
R1=10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K=dlqr(AD,BD,Q,R1); % gannancia lqr sin integrador
[K1,po,ee]=dlqr(AA,BB,QA,R1); % ganancia lqr con integrador discreto

% observador de estados
L=place(AD',CD',Pzo)';
Pzo=0.2*ee(1:4);
L1=place(AD',(CD*AD)',Pzo)';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parametro filtro kalman

%covarianza de la inovacion de posicion
pk=1e0*eye(4);

%covarianza del ruido de prediccion
Sz=0.01*eye(2);
%covarianza del ruido de posicion del objeto
W=zeros(4);
W(1,1)=0.05^2;
W(2,2)=0.05^2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% control bola viga
vrep=remApi('remoteApi');
vrep.simxFinalizar(-1);
clientID=vrep.simxComenzar(true,5000,5);

if (clientID>-1)
        disp('connected');

        %[number returnCode,number handle]=simxGetObjectHandle(number clientID,string objectName,number operationMode); 
        state=true;
        % handle de los objetos relevantes de la siulación
        [res,motor1]=vrep.simxObtenerObjeto(clientID,'abase',vrep.simx_modo_bloqueando); %inicializo motor izquierdo
        [res,sensor1]=vrep.simxObtenerObjeto(clientID,'Proximity_sensor',vrep.simx_modo_bloqueando);
        [res,bola]=vrep.simxObtenerObjeto(clientID,'ball',vrep.simx_modo_bloqueando);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        inicial=0.5; % variable que permite medir la posicion de la bola
        % iniciliazación de las variables
        u=0;
        vant=0;
        %[res,dist1]=vrep.simxGetObjectPosicition(clientID,bola,-1,vrep.simx_opmode_streaming)
        [res]=vrep.simxLeerPosicionMotor(clientID,motor1,sensor1,vrep.simx_modo_retransmision);
        [res,detectionState,detectedPoint,~,~]=vrep.simxLeerSensorProximidad(clientID,sensor1,vrep.simx_modo_retransmision); 
        pause(0.5);
        [res,detectionState,detectedPoint,~,~]=vrep.simxLeerSensorProximidad(clientID,sensor1,vrep.simx_modo_regulacion);
        %[res,posicion(1)]=vrep.simxGetObjectPosicion(clientID,bola,-1,vrep.simx_opmode_buffer);
        u=0;
        xkant=zeros(4,1);
        xk=zeros(4,1);
        vf=zeros(2,1);
        angm=zeros(2,1);
        dist=zeros(2,1);
        yf=zeros(2,1);
        dist(1)=norm(detectedPoint)-inicial;
        stat=true;
        k=1;
        ref=0.2;
        kkp=1; % constante que multiplica los tiempos
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [res,angm(1)]=vrep.simxLeerPosicionMotor(clientID,motor1,vrep.simx_opmode_buffer);
         while stat==true
            tic

            % lectura de los sensores
            [res,angm(1)]=vrep.simxLeerPosicionMotor(clientID,motor1,vrep.simx_modo_regulacion);
            [res,detectionState,detectedPoint,~,~]=vrep.simxLeerSensorProximidad(clientID,sensor1,vrep.simx_modo_regulacion);
            dist(1)=norm(detectedPoint);
            dist(1)=-inicial+dist(1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % variables que tengo
            yf(2)=angm(1);
            yf(1)=dist(1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             yf(2)=angm(1)+1e-3*randn(1,1);
%             yf(1)=dist(1)+1e-2*randn(1,1);

%         observador de estados discreto
            zk=AD*xkant+BD*u;
            xk=zk+(L1*(yf-CD*zk));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filtro de kalman

%             zk=AD*xkant+BD*u;
%             pk=AD*pk*AD'+W;
%             kk=(pk*CD')/(CD*pk*CD'+Sz);
%             xk=zk+(kk*(yf-CD*zk));
%             pk=(eye(4)-kk*CD)*pk;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            e=ref-dist(1);
            v=vant+e*K1(5);
            %u=((K(1)*e)-(K(2)*(xk(2)))-(K(3)*xk(3))-(K(4)*xk(4)));
            u=((-K1(1)*dist(1))-(K1(2)*angm(1))-(K1(3)*xk(3))-(K1(4)*xk(4)))+v; % señal de control
            if u>2.11
                u=2.11;
            elseif u<-2.11
                u=2.11;
            end
            vf(2)=vf(1);
            vf(1)=(((u*T)/Iv)+vf(2)); % transformacion señal de control de torque a velocidad
            vrep.simxFijarPosicionObjetivoMotor(clientID,motor1,vf(1),vrep.simx_modo_bloqueando);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % vectores para graficar
            xkant=xk;
            vant=v;
            error(k)=e*100;
            angulo(k)=angm(1)*180/pi;
            salida(k)=u;
            tiempo(k)=k*T;
            distancia(k)=dist(1)*100;
            velocidad(k)=vf(1);
            estados(k,:)=xk;
            %%%%%%%%%%%%%%%%%%%%%%%%%%
            k=k+1; % contador
            % cambio de referencia
            if k==600*kkp 
            ref=0.4;
            end
            
            if k==1200*kkp 
            ref=-0.4;
            end
            
             if k==1800*kkp 
            ref=-0.1;
             end
             if k==2400*kkp 
            ref=-0.3;
             end
            
            if k==3000*kkp 
                stat=false;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            toc
            pause(T-toc)
            
         end
         
        vrep.simxFinish(-1);
end
vrep.delete();  


%% graficas
    


    figure(1)
    
    subplot(2,1,1);
    plot(tiempo,distancia);
    xlabel('tiempo: [s]');
    ylabel('distancia:[cm]');
    title('distancia de la bola')
    grid on;
    subplot(2,1,2);
    plot(tiempo,error);
    xlabel('tiempo: [s]');
    ylabel('error:[cm]');
    title('error de la posicion de la bola')
    grid on;
    
    figure(2)
    
    plot(tiempo,salida);
    xlabel('tiempo: [s]');
    ylabel('torque:[N.m]');
    title('señal de control')
    grid on;
    
    figure(3)
    subplot(2,2,1);
    plot(tiempo,estados(:,1));
    xlabel('tiempo: [s]');
    ylabel('distancia:[m]');
    title('distancia de la bola')
    grid on;
    subplot(2,2,2);
    plot(tiempo,estados(:,2));
    xlabel('tiempo: [s]');
    ylabel('angulo del motor:[rad]');
    title('angulo del motor')
    grid on;
    subplot(2,2,3);
    plot(tiempo,estados(:,3));
    xlabel('tiempo: [s]');
    ylabel('velocidad lineal:[m/s]');
    title('velocidad de la bola')
    grid on;
    subplot(2,2,4);
    plot(tiempo,estados(:,4));
    xlabel('tiempo: [s]');
    ylabel('velocidad angular:[rad/s]');
    title('velocidad angular del motor')
    grid on;
    