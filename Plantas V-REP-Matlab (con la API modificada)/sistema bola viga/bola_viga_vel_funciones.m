% % Parametros del sistema bola viga 
% T=0.06; % tiempo de muestreo
% 
% 
% 
% 
% Mb=1; % masa del riel [kg]
% L=1; % longitud del riel [m]
% m=0.1; % masa de la bola [kg]
% R=0.034; % radio de la bola [m]
% Iv=(Mb*(L)^2)/12; %momento de inercia de una barra [kg m^2];
% Ib=0.4*m*R^2; % momento de inercia de la bola [kg m^2]
% g=9.81; % gravedad [m/s^2]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms x1 x2 x3 x4 Mb L m R Iv Ib g u ref ref1 real

% estados
% x1: distancia de la bola en x
% x2: angulo del motor
% x3: velocidad de la bola en x
% u: señal de control [velocidad angular] 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sistema bola viga no lineal
f1=x3;
f2=u;
f3=(m*x1*u^2-m*g*sin(x2)/(m+(Ib/R^2)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% puntos de operacion

x1ss=ref;
x2ss=ref1;
x3ss=0;
% uss=;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linealización

% As=[diff(f1,x1) diff(f1,x2) diff(f1,x3);diff(f2,x1) diff(f2,x2) diff(f2,x3);
%     diff(f3,x1) diff(f3,x2) diff(f3,x3)];
% A1=subs(As,[x1 x2 x3 u],[x1ss x2ss x3ss uss]);
% Bs=[diff(f1,u);diff(f2,u);diff(f3,u)];
% B1=subs(Bs,[x1 x2 x3 u],[x1ss x2ss x3ss uss]);

uss1=solve(f3==0,u);
uss1=simplify(uss1);
Uss=subs(uss1,[x1 x2 x3],[x1ss x2ss x3ss]);

% D=[0;0];
%%
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
syms x1 x2 x3 x4 u 

% estados
% x1: distancia de la bola en x
% x2: angulo del motor
% x3: velocidad de la bola en x
% u: señal de control [velocidad angular] 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sistema bola viga no lineal
f1=x3;
f2=u;
f3=(m*x1*u^2-m*g*sin(x2)/(m+(Ib/R^2)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%salidas

h=x1;
hm1=x1;
hm2=x2;
hm3=x3;

% puntos de operacion

ref=0;
x1ss=0;
x2ss=0;
x3ss=0;
uss=ref;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linealización

As=[diff(f1,x1) diff(f1,x2) diff(f1,x3);diff(f2,x1) diff(f2,x2) diff(f2,x3);
    diff(f3,x1) diff(f3,x2) diff(f3,x3)];
A1=subs(As,[x1 x2 x3 u],[x1ss x2ss x3ss uss]);
A=double(A1);
Bs=[diff(f1,u);diff(f2,u);diff(f3,u)];
B1=subs(Bs,[x1 x2 x3 u],[x1ss x2ss x3ss uss]);
B=double(B1);
C=[1 0 0;0 1 0];
D=0;
% D=[0;0];

% descetización del sistema
PI=ss(A,B,C,D);
sistemad=c2d(PI,T,'zoh');
[AD,BD,CD,DD]=ssdata(sistemad);
polosd=eig(AD);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% función de transferencia
PIS=tf(PI);
%%%%%%%%%%%%%%%%%%%%

% polos del observador
ts=5; % tiempo de establecimiento 
% Mp=0.01;
sigma=4/ts;
P1=-sigma;
P2=8*P1';
P3=10*real(P1);
P=[P1;P2];
polos=[P;P3];
% Pz=exp(T*polos);

Pzo=exp(5*T*polos); % transformacion de poolos contrinuos a discretos

% matrices ampliadadas
C1=[1 0 0];

AA=[AD,(0*C1)';(C1*AD),1];
BB=[BD;C1*BD];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% controlabilidad y observabilidad del sistema
controlabilidad=ctrb(A,B); %matriz de controlabilidad
rangoc=rank(controlabilidad);
observabilidad=[CD;CD*AD;CD*AD*AD]; % matrices de observabiidad 
rangoob=rank(observabilidad);  %rango de matriz de observabilidad
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parametros LQR con integrador y sin integrador
QA=[300 0 0 0;0 1 0 0;0 0 800 0;0 0 0 0.1];
R1=10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% K=dlqr(AD,BD,Q,R1); % gannancia lqr sin integrador
[K1,po,ee]=dlqr(AA,BB,QA,R1); % ganancia lqr con integrador discreto
% observador de estados
% L=place(AD',CD',Pzo)';
% Pzo=0.2*ee(1:3);
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
        % handle de los objetos relevantes de la simulación
        [~,motor1]=vrep.simxObtenerObjeto(clientID,'abase',vrep.simx_modo_bloqueando); %inicializo motor izquierdo
        [~,sensor1]=vrep.simxObtenerObjeto(clientID,'Proximity_sensor',vrep.simx_modo_bloqueando);
        [~,bola]=vrep.simxObtenerObjeto(clientID,'ball',vrep.simx_modo_bloqueando);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % inicializacion de los sensores
        [~,detectionState,detectedPoint,~,~]=vrep.simxLeerSensorProximidad(clientID,sensor1,vrep.simx_modo_retransmision); % sensor de proximidad mide posicion de la bola (x1)
        [~,angm]=vrep.simxLeerPosicionMotor(clientID,motor1,vrep.simx_modo_retransmision); % posicion del motor (x2)
        [~,lineal,angular]=vrep.simxLeerVelocidad(clientID,bola,vrep.simx_modo_retransmision);  % velocidad lineal y angular del objeto bola en x,y,z
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        pause(0.5);
        % lectura de los sensores
        [~,detectionState,detectedPoint,~,~]=vrep.simxLeerSensorProximidad(clientID,sensor1,vrep.simx_modo_regulacion); % lectura del sensor de proximidad (posicion bola x1)
        [~,angm]=vrep.simxLeerPosicionMotor(clientID,motor1,vrep.simx_modo_regulacion);  % lectura de la posicion angular del motor(x2)
        [~,lineal,angular]=vrep.simxLeerVelocidad(clientID,bola,vrep.simx_modo_regulacion); % velocidad lineal y angular del objeto bola en x,y,z
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
         % iniciliazación de las variables
        inicial=0.5; % variable que permite medir la posicion de la bola
        u=0;
        vant=0;
        xkant=zeros(3,1);
        xk=zeros(3,1);
        yf=zeros(2,1);
        dist=norm(detectedPoint)-inicial;
        stat=true;
        k=1;
        refv=[0.2 0.4 -0.4 -0.1 -0.3];
        ref=refv(1,1);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
        kkp=0.8; % constante que multiplica los tiempos
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         while stat==true
            tic

             % lectura de los sensores
            [~,detectionState,detectedPoint,~,~]=vrep.simxLeerSensorProximidad(clientID,sensor1,vrep.simx_modo_regulacion); % lectura del sensor de proximidad (posicion bola x1)
            dist=norm(detectedPoint)-inicial;
            [~,angm]=vrep.simxLeerPosicionMotor(clientID,motor1,vrep.simx_modo_regulacion);  % lectura de la posicion angular del motor(x2)
            [~,lineal,angular]=vrep.simxLeerVelocidad(clientID,bola,vrep.simx_modo_regulacion); % velocidad lineal y angular del objeto bola en x,y,z
            velb=lineal(2); % velocidad lineal de la bola en y (x1)
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
            estadosm=[dist(1) angm(1)  velb];
            estadosob=[dist(1) angm(1)  xk(3)];
            e=ref-dist(1)
            % controlador integral 
            v=vant+e;
            %u=-dot(K1(1:3),estadosm)+K1(4)*v; % señal de control (velocidad angular)
            u=-dot(K1(1:3),estadosob)+K1(4)*v; % señal de control (velociad angular)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % saturacion de controlador torque maximo [-2.11,2.11] N.m
            if u>2.11
                u=2.11;
            elseif u<-2.11
                u=-2.11;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Transformación de torque a velocidad
            vrep.simxFijarVelocidadObjetivoMotor(clientID,motor1,u,vrep.simx_modo_bloqueando);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % actualizar variables
            xkant=xk;
            vant=v;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % vectores para graficar
            error(k)=e*100;
            angulo(k)=angm(1)*180/pi;
            salida(k)=u;
            tiempo(k)=k*T;
            distancia(k)=dist(1)*100;
            estados(k,:)=estadosm;
            velocidadbola(k)=velb;
            %%%%%%%%%%%%%%%%%%%%%%%%%%
            k=k+1; % contador
            % cambio de referencia
            if k==600*kkp 
            ref=refv(1,2);
            end
            
            if k==1200*kkp 
            ref=refv(1,3);
            end
            
             if k==1800*kkp 
            ref=refv(1,4);
             end
             if k==2400*kkp 
            ref=refv(1,5);
             end
            
            if k==3000*kkp 
                stat=false;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            tr=toc;
            pause(T-tr)
            
         end
         
        vrep.simxFinalizar(-1);
end
vrep.eliminar();  


%% graficas
    

% 
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
    ylabel('Velocidad angular:[rad/s]');
    title('velocidad angular asignada al motor')
    grid on;
    
    figure(3)
    subplot(3,1,1);
    plot(tiempo,estados(:,1));
    xlabel('tiempo: [s]');
    ylabel('distancia:[m]');
    title('distancia de la bola')
    grid on;
    subplot(3,1,2);
    plot(tiempo,estados(:,2));
    xlabel('tiempo: [s]');
    ylabel('angulo del motor:[rad]');
    title('angulo del motor')
    grid on;
    subplot(3,1,3);
    plot(tiempo,estados(:,3));
    xlabel('tiempo: [s]');
    ylabel('velocidad lineal:[m/s]');
    title('velocidad de la bola')
    grid on;
    
%     figure(1)
%     subplot(2,1,1);
%     plot(tiempo,distancia);
%     xlabel('tiempo: [s]');
%     ylabel('distancia:[cm]');
%     title('distancia de la bola')
%     grid on;
%     subplot(2,1,2);
%     plot(tiempo,error);
%     xlabel('tiempo: [s]');
%     ylabel('error:[cm]');
%     title('error de la posicion de la bola')
%     grid on;
   

    

    
%     figure(2)
%     plot(tiempo,velocidad);
%     hold on 
%     plot(tiempo,datos.signals(4).values(19:length(velocidad)+18));  
%     xlabel('time:[s]');
%     ylabel('Velocity :[rad/s]');
%     title('Control signal motor')
%     legend('LQR V-REP','LQR Simulink')
%     hold off
%     grid on;