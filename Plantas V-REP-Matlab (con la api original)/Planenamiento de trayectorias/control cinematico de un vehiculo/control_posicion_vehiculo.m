clear all               
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CONTROL DE UN VEHICULO CON TRACCIÓN DIFEREMCIAL             %
%EN EL ENTORNO DE SIMULACIÓN V-REP                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% parametos cinemáticos del vehiculo (tracción diferencial)
T=0.1; % tiempo de muestreo
d=0.3; % distancia entre los centros de las ruedas
r1=0.0975; % radio de la rueda
a=0.1; % distancia al punto de control
K=0.05*eye(2,2); % matriz de constantes de calibración del controlador 
xmin=0;   % limite inferior del escenario      
xmax=10;  % limite inferior del escenario
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parametros odometría 
posx=0;
posy=0;
Di=0; % distancia recorrida por la rueda izquierda
Dd=0; % distancia recorrida por la rueda derecha
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% contador
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
         
           %inicializacion de los sensores
         [~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_streaming);
         [~,posiciongoal]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_streaming);
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_streaming);
         [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_streaming);
         [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_streaming);
   
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         pause(1);
         %toma de datos de los sensores
         [~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_buffer); % posicion x,y, z del vehiculo
         [~,posiciongoal]=vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_buffer);
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_buffer);%orientacion del vehiculo (orientacion(3)=fi)
         [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_buffer);
         [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_buffer);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
    x=posicion(1:2)';  % Posicion inicial del vehiculo
    xgoal=posiciongoal(1:2)';% Posicion Objetivo
    posref=xgoal';
% calculo de posición del vehiculo mediante odometria
    posx=posicion(1);
    posy=posicion(2);
    posactual=[posicion(1) posicion(2)];
    Di=0;
    Dd=0;
    Dc=0; % desplazamiento total del vehiculo
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    e=posref-posactual; % error de posición en el plano (x,y) del vehiculo
    normg=norm(e);
    % control de posición del veiculo en el plano (x,y)
    while normg>=5e-2 && l<=2000
    tic     
          % medir variables
         [~,posicion]=vrep.simxGetObjectPosition(clientID,sensorp,-1,vrep.simx_opmode_buffer);
         [~,orientacionp]=vrep.simxGetObjectOrientation(clientID,sensorp,-1,vrep.simx_opmode_buffer);
         [~,angi]=vrep.simxGetJointPosition(clientID,motori,vrep.simx_opmode_buffer);
         [~,angd]=vrep.simxGetJointPosition(clientID,motord,vrep.simx_opmode_buffer);

         % calculo de posición por odometria
%          angi=angi-angri;
%          angd=angd-angrd;
         Di=r1*(angi);
         Dd=r1*(angd);
         Dc=(Di+Dd)/2;
%          orientacion(3)=(Di-Dd)/d;
         posactual(1)=posx+Dc*cos(orientacionp(3));
         posactual(2)=posy+Dc*sin(orientacionp(3));
         posactualc=posicion(1:2);
         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         e=posref-posactual % error de la posición del vehiculo
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
           
          normg=norm(e);
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
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vrep.simxSetJointTargetVelocity(clientID,motord,0,vrep.simx_opmode_blocking);
vrep.simxSetJointTargetVelocity(clientID,motori,0,vrep.simx_opmode_blocking);
vrep.simxFinish(-1); % termina simulación
% calculo de las distancia recorridas 
Distance_real=totaldistance(posicioncar(1,:),posicioncar(2,:)) % distancia real recorrida
Distance_odometria=totaldistance(posicioncarodo(1,:),posicioncarodo(2,:)) % distancia estimada por odmetria	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
vrep.delete();  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fin del programa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% otras graficas
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
    xlim([0 10])
    ylim([0 10])
    xlabel('posicion en x:[m]');
    ylabel('posicion en y:[m]');
    title('posición en el plano(x,y)')
    grid on;

