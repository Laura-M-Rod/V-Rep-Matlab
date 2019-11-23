clear all               
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PLANEAMIENTO DE TRAYECTORIAS DE UN VEHICULO CON TRACCION DIFERENCIAL%
%EN EL ENTRONO DE SIMULACIÓN V-REP                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xmin=-5;   % limite inferior del escenario      
xmax=5;  % limite inferior del escenario

Nsteps=500;            % Maximum number of steps to produce

%inicialización de las parametros del Planning codigo Passino
Ns=16;        % Numero de puntos en un circulo de sensado
r=1;          % radio de sensado
xs=0*ones(2,Ns); %inicialización posciones siguientes
J(:,1)=0*ones(Ns,1); %inicialización Función de costo total
Jo(:,1)=0*ones(Ns,1);%inicialización Función de costo obstaculos
Jg(:,1)=0*ones(Ns,1);%inicialización Función de costo objetivo
theta(:,1)=0*ones(Ns,1);%inicialización de los angulos alrededor del circulo de sensado
for m=2:Ns %los angulos alrededor del circulo de sensado
    
	theta(m,1)=theta(m-1,1)+(pi/180)*(360/Ns); 
end

w1=1; % peso obstaculo
w2=1e-04; % peso objetivo
% parametos cinemáticos del vehiculo (tracción diferencial)
T=0.1; % tiempo de muestreo
d=0.3; % distancia entre los centros de las ruedas
r1=0.0975; % radio de la rueda
a=0.1; % distancia al punto de control
K=0.2*eye(2,2); % matriz de constantes de calibración del controlador
kang=2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parametros odometría 
posx=0;
posy=0;
Di=0; % distancia recorrida por la rueda izquierda
Dd=0; % distancia recorrida por la rueda derecha
angri=0;
angrd=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% contadores
s=1;
l=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% V-REP
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('connected');
          % handle de los objetos relevantes del vehiculo
         [~,sensorp]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
         [~,motord]=vrep.simxGetObjectHandle(clientID,'motord',vrep.simx_opmode_blocking);
         [~,motori]=vrep.simxGetObjectHandle(clientID,'motori',vrep.simx_opmode_blocking);
         [~,goal]=vrep.simxGetObjectHandle(clientID,'objetivo',vrep.simx_opmode_blocking);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % handle de los obstaculos
         [~,obstaculo(1)]=vrep.simxGetObjectHandle(clientID,'cubo1b',vrep.simx_opmode_blocking);
         [~,obstaculo(2)]=vrep.simxGetObjectHandle(clientID,'cubo2b',vrep.simx_opmode_blocking);
         [~,obstaculo(3)]=vrep.simxGetObjectHandle(clientID,'cubo3b',vrep.simx_opmode_blocking);
         [~,obstaculo(4)]=vrep.simxGetObjectHandle(clientID,'cubo4b',vrep.simx_opmode_blocking);
         [~,obstaculo(5)]=vrep.simxGetObjectHandle(clientID,'cubo5b',vrep.simx_opmode_blocking);
         [~,obstaculo(6)]=vrep.simxGetObjectHandle(clientID,'cubo6b',vrep.simx_opmode_blocking);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         %[~,mass]=vrep.simxGetObjectFloatParameter(clientID,obstaculo1,vrep.sim_shapefloatparam_mass,vrep.simx_opmode_blocking)

         %inicializacion de los sensores
         [~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_streaming);
         [~,posiciongoal]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_streaming);
         [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_streaming);
         [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_streaming);
         for i=1:6
            [~,posicionobs(:,i)]=vrep.simxGetObjectPosition(clientID,obstaculo(i),-1,vrep.simx_opmode_streaming);
         end
   
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         pause(1);
         %toma de datos de los sensores
         [~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_buffer); % posicion x,y, z del vehiculo
         [~,posiciongoal]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_buffer);
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_buffer);%orientacion del vehiculo (orientacion(3)=fi)
         [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_buffer);
         [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_buffer);
         for i=1:6
            [~,posicionobs(:,i)]=vrep.simxGetObjectPosition(clientID,obstaculo(i),-1,vrep.simx_opmode_buffer);%posicion x,y,z de los obstaculos
         end
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Posicion obstaculos         
posicionobs=posicionobs(1:2,:);  
% Posicion inicial del vehiculo
x=posicion(1:2)';
% Posicion Objetivo
xgoal=posiciongoal(1:2)';
A=20;
% Graficar funciones:

xx=-5:31/100:5;   % rango de valores considrados
yy=xx;

for jj=1:length(xx)
	for ii=1:length(yy)
		zz(ii,jj)=funcion_obstaculos([xx(jj);yy(ii)],A,posicionobs,w1);
	end
end
for jj=1:length(xx)
	for ii=1:length(yy)
		zzz(ii,jj)=goalfunction([xx(jj);yy(ii)],xgoal,w2);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
clf
contour(xx,yy,zz+zzz,50)
colormap(jet)
xlabel('x');
ylabel('y');
title('J=w_1J_o + w_2J_g and initial (square) and goal (x) positions');
hold on
grid on;
% Plot initial and final positions
plot(x(1),x(2),'s',xgoal(1),xgoal(2),'x')
for k=1:Nsteps
	[~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_buffer);
    [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_buffer);
    [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_buffer);
    x=posicion(1:2)';
    angri=angi;
    angrd=angd;

	    for m=1:Ns
		xs(:,m)=[x(1)+r*cos(theta(m,1)); x(2)+r*sin(theta(m,1))]; %Puntos en el circulo de sensado
        Jo(m,1)=funcion_obstaculos(xs(:,m),A,posicionobs,w1);% Función de costo obstaculos
		Jg(m,1)=goalfunction(xs(:,m),xgoal,w2); % Función de costo objetivo
 		J(m,1)=Jo(m,1)+Jg(m,1); % Función de costo objetivo
        end
    [val,bestone]=min(J); % angulo optimo
 	x_siguiente=[x(1)+r*cos(theta(bestone,1)); x(2)+r*sin(theta(bestone,1))];% Posición siguiente
    posref=x_siguiente';
    posactual=[posicion(1) posicion(2)];
    posx=posactual(1);
    posy=posactual(2);
    angulo=orientacionp(3);
    eang=theta(bestone,1)-angulo;
    e=posref-posactual;
    normg=norm(e);
    Di=0;
    Dd=0;
    Dc=0;
    while eang>=1e-2 && s<=1000
        tic
        % controlador Proporcional para control del angulo de giro
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_buffer);% medir orientación
         angulo=orientacionp(3); % tomar la orientación en z
         eang=theta(bestone,1)-angulo; % error angulo
         wang=kang*eang; % velocidad angular deseada en el carro
         v_rang=(d*wang);% velocidad lineal deseada en el carro
         velrueda1ang=v_rang/(2*r1); % velocidad rueda derecha
          if velrueda1ang>=12
              velrueda1ang=0;
              break;
         end
         vrep.simxSetJointTargetVelocity(clientID,motord,velrueda1ang,vrep.simx_opmode_blocking);
         vrep.simxSetJointTargetVelocity(clientID,motori,0,vrep.simx_opmode_blocking);
         
         tiempos(s)=s*T;
         angulomed(s)=angulo*180/pi;
         velocidadmed(s)=velrueda1ang;
         s=s+1;
         
        ts=toc;
        pause(T-ts)
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while normg>=5e-2 && l<=2000
    tic     
          % medir variables
         [~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_buffer);
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_buffer);
         [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_buffer);
         [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_buffer);


         angi=angi-angri;
         angd=angd-angrd;
         Di=r1*(angi);
         Dd=r1*(angd);
         Dc=(Di+Dd)/2;
%          orientacion(3)=(Di-Dd)/d;
         posactual(1)=posx+Dc*cos(orientacionp(3));
         posactual(2)=posy+Dc*sin(orientacionp(3));
         posactualc=posicion(1:2);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         e=posref-posactual; % error de la posición del vehiculo

% caculo de señal de control primera forma          
%           J2=[cos(orientacionp(3)) -a*sin(orientacionp(3));
%              sin(orientacionp(3)) a*cos(orientacionp(3))];
          
          %V=inv(J2)*K*e';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
% caculo de señal de control segunda forma          
          J2=[cos(orientacionp(3)) sin(orientacionp(3));
             -(1/a)*sin(orientacionp(3)) (1/a)*cos(orientacionp(3))]; 
          V=J2*K*e';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          u=V(1);
          w=V(2);  
          % transformacion a velocidad de ruedas
          v_r=(2*u+d*w);
          v_l=(2*u-d*w);
          velrueda1=v_r/(2*r1);
          velrueda2=v_l/(2*r1);
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             
         % Aplicar acciones de control al robot
         
         vrep.simxSetJointTargetVelocity(clientID,motord,velrueda1,vrep.simx_opmode_blocking);
         vrep.simxSetJointTargetVelocity(clientID,motori,velrueda2,vrep.simx_opmode_blocking);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         % vectores para graficar
         normg=norm(e)
        if norm(xgoal-posactual')<=5e-1
           break;
        end	
          tiempo(l)=l*T;
          velocidadl(l)=u;
          velocidadang(l)=w;
          velocidadruedas(:,l)=[velrueda1 velrueda2];
          variables(:,l)=posactual;
          error(:,l)=e;
          posicioncarodo(:,l)=posactual(1:2);
          posicioncar(:,l)=posactualc(1:2);
          posest(:,l)=[posx posy];
          l=l+1;
          ts=toc;
          pause(T-ts);
    end	
        x1(:,k)=x;
	    if norm(xgoal-posactual')<=5e-1
           break;
        end	
        vrep.simxSetJointTargetVelocity(clientID,motord,0,vrep.simx_opmode_blocking);
        vrep.simxSetJointTargetVelocity(clientID,motori,0,vrep.simx_opmode_blocking);
end % End main loop...
vrep.simxSetJointTargetVelocity(clientID,motord,0,vrep.simx_opmode_blocking);
vrep.simxSetJointTargetVelocity(clientID,motori,0,vrep.simx_opmode_blocking);
% Distance_x=totaldistance(x1(1,:),x1(2,:)) % distancia ideal
% Distance_real=totaldistance(posicioncar(1,:),posicioncar(2,:)) % distancia real recorrida
% Distance_odometria=totaldistance(posicioncarodo(1,:),posicioncarodo(2,:)) % distancia estimada por odmetria	
figure(1)
plot(x1(1,:),x1(2,:)','Color', 'k', 'LineWidth', 1.5)
plot(posicioncar(1,:),posicioncar(2,:),'Color', 'r', 'LineWidth', 1.5)
plot(posicioncarodo(1,:),posicioncarodo(2,:),'Color', 'b', 'LineWidth', 1.5)
vrep.simxFinish(-1);
end
vrep.delete();  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of program
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
    figure(2)
    
    subplot(1,2,1);
    plot(tiempo,error(1,:));
    hold on
    plot(tiempo,error(2,:));
    hold off
    xlabel('tiempo: [s]');
    ylabel('Posición:[m]');
    title('error posición')
    grid on;
    subplot(1,2,2);
    plot(tiempo,posicioncar(1,:));
    hold on
    plot(tiempo,posicioncar(2,:));
    hold off
    xlabel('tiempo: [s]');
    ylabel('Posición:[m]');
    title('posición robot')
    grid on;
    
    figure(3)
    
    subplot(1,2,1);
    plot(tiempo,velocidadruedas(1,:));
    xlabel('tiempo: [s]');
    ylabel('Velocidad:[rad/s]');
    title('Velocidad motor derecho')
    grid on;
    subplot(1,2,2);
     plot(tiempo,velocidadruedas(2,:));
    xlabel('tiempo: [s]');
    ylabel('Velocidad:[rad/s]');
    title('Velocidad motor izquierdo')
    grid on;
    
    figure(4)
    plot(posicioncar(1,:),posicioncar(2,:));
    xlabel('posicion en x:[m]');
    ylabel('posicion en y:[m]');
    title('trayectoria en el plano(x,y)')
    grid on;

