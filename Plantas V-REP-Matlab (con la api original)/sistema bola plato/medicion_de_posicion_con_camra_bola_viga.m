clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   el sistema bola plato puede ser tratado como un sistema MIMO O como 2  
%   sistemas SISO en este codigo el sistema bola plato es tratado como un
%   sistema MIMO por lo tAnto exiten 1 controlador LQR para todas las
%   entradas y salidas
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% parametros del sistema
T=0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tamaño del plato en la zona de trabajo 
% 22.5 cm de largo
% 22.5 cm de ancho
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% programa

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
        disp('connected');
        %[number returnCode,number handle]=simxGetObjectHandle(number clientID,string objectName,number operationMode); 
        % inicialización de los handles de los objetos mas relevantes
        [~,h(1)]=vrep.simxGetObjectHandle(clientID,'motor1',vrep.simx_opmode_blocking);
        [~,h(2)]=vrep.simxGetObjectHandle(clientID,'motor2',vrep.simx_opmode_blocking);
        [~,h(3)]=vrep.simxGetObjectHandle(clientID,'Bola',vrep.simx_opmode_blocking);
        [~,h(4)]=vrep.simxGetObjectHandle(clientID,'camara',vrep.simx_opmode_blocking);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % inicializar sensores
        [~,pos]=vrep.simxGetObjectPosition(clientID,h(3),-1,vrep.simx_opmode_streaming);
        [~,ang1]=vrep.simxGetJointPosition(clientID,h(1),vrep.simx_opmode_streaming);
        [~,ang2]=vrep.simxGetJointPosition(clientID,h(2),vrep.simx_opmode_streaming);
        [~,lineal,angular]=vrep.simxGetObjectVelocity(clientID,h(3),vrep.simx_opmode_streaming);
        [~,res,img]=vrep.simxGetVisionSensorImage2(clientID,h(4),0,vrep.simx_opmode_streaming);
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        pause(1)
        %leer sensores
        [~,pos]=vrep.simxGetObjectPosition(clientID,h(3),-1,vrep.simx_opmode_buffer);
        [~,ang1]=vrep.simxGetJointPosition(clientID,h(1),vrep.simx_opmode_buffer);
        [~,ang2]=vrep.simxGetJointPosition(clientID,h(2),vrep.simx_opmode_buffer);
        [~,lineal,angular]=vrep.simxGetObjectVelocity(clientID,h(3),vrep.simx_opmode_buffer);
        [~,res,img]=vrep.simxGetVisionSensorImage2(clientID,h(4),0,vrep.simx_opmode_buffer);
        % inicializacion de variables
        state=true;
        k=1;
        estados=zeros(1,4);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        while state==true
            tic
            % leer sensores
             %[~,pos]=vrep.simxGetObjectPosition(clientID,h(3),-1,vrep.simx_opmode_buffer);
            pos=pos(1:2)
            [~,ang1]=vrep.simxGetJointPosition(clientID,h(1),vrep.simx_opmode_buffer);
            [~,ang2]=vrep.simxGetJointPosition(clientID,h(2),vrep.simx_opmode_buffer);
            [~,lineal,angular]=vrep.simxGetObjectVelocity(clientID,h(3),vrep.simx_opmode_buffer);
            [~,res,img]=vrep.simxGetVisionSensorImage2(clientID,h(4),0,vrep.simx_opmode_buffer);
            % inicializacion y lectura de los sensores
            
%             [~,pos]=vrep.simxGetObjectPosition(clientID,h(3),-1,vrep.simx_opmode_oneshot_wait);
%             [~,ang1]=vrep.simxGetJointPosition(clientID,h(1),vrep.simx_opmode_oneshot_wait);
%             [~,ang2]=vrep.simxGetJointPosition(clientID,h(2),vrep.simx_opmode_oneshot_wait);
%             [~,lineal,angular]=vrep.simxGetObjectVelocity(clientID,h(3),vrep.simx_opmode_oneshot_wait);
%             [~,res,img]=vrep.simxGetVisionSensorImage2(clientID,h(4),0,vrep.simx_opmode_oneshot_wait);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
            % estimación de la posición de la bola por medio de una imagen
            res=double(res)'; % pasar de int a double
            imgd=binarizar(img);
            centroide=centroid(imgd)';
            pos(1)=(-res(1)/2+centroide(1))*(0.225/res(1))% posición estimada en x
            pos(2)=(-centroide(2)+res(2)/2)*(0.225/res(2))% posición estimada en y
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             imshow(img)
            % corrección de la posicion usando funiones trigonometricas
            % cuando cambia el angulo
            
            x1(2)=pos(1)/cos(ang1);
            y1(2)=pos(2)/cos(ang2);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
             % letura de los estados sin observador
            estados=[x1(2) lineal(1) y1(2) lineal(2)];
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if k==50
               vrep.simxSetJointTargetPosition(clientID,h(1),((2*pi)/180),vrep.simx_opmode_blocking);
            end
            if k==100
               vrep.simxSetJointTargetPosition(clientID,h(1),((-2*pi)/180),vrep.simx_opmode_blocking);
               vrep.simxSetJointTargetPosition(clientID,h(2),((2*pi)/180),vrep.simx_opmode_blocking);
            end
            if k==150;
               vrep.simxSetJointTargetPosition(clientID,h(1),((2*pi)/180),vrep.simx_opmode_blocking);
               vrep.simxSetJointTargetPosition(clientID,h(2),((-2*pi)/180),vrep.simx_opmode_blocking);
            end
            if k==200;
               state=false;
            end
            % vectores para graficar 
            estados1(k,:)=estados;
            posx(k)=x1(2)*100;
            posy(k)=y1(2)*100;
            tiempo(k)=k*T;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            k=k+1; % contador
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ts=toc;
            pause(T-ts);
        end
         
        vrep.simxFinish(-1);
end
vrep.delete();  

%% graficas 

    figure(1)
    subplot(1,2,1);
    plot(tiempo,posx);
    xlabel('tiempo: [s]');
    ylabel('posicion bola en x:[cm]');
    title('posicion de la bola en x')
    grid on;
    subplot(1,2,2);
    plot(tiempo,posy);
    xlabel('tiempo: [s]');
    ylabel('posicion bola en y:[cm]');
    title('posicion de la bola en y')
    grid on;
    
    figure(2)
    
    plot(posx,posy);
    xlabel('posicion en x:[cm]');
    ylabel('posicion en y:[cm]');
    title('posicion de la bola en el plano (x,y)')
    grid on;

