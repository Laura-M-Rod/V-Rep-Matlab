
/*
 * Include Files
 *
 */
#include "simstruc.h"



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include <time.h>

#define NON_MATLAB_PARSING
#define SS_STDIO_AVAILABLE 
#define MAX_EXT_API_CONNECTIONS 255
#include <extApi.h>
#include <extApi.c>
#include <extApiPlatform.h>
#include <extApiPlatform.c>
#include <shared_memory.h>
#include <shared_memory.c>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
int clientID;
int ObjectHandle[4];
float Pos[3],Vel[3],Ang[3],Ang1[3],Ori[3];
float Phi,Phi1;
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void V_Rep_Start_wrapper(real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */




int Valido[4];
simxFinish(-1);
clientID=simxStart("127.0.0.1",19997,true,true,100,5); //Conectar con la simulacion
if(clientID==-1)
{
      mexPrintf ("ERROR: No se ha podido conectar con el robot");
      simxFinish(clientID);
}
else
{
      mexPrintf("Connected to remote API server\n");
      switch((int) Modo[0])
      {
          case 0:
              Valido[0]=simxGetObjectHandle(clientID,"motord",&ObjectHandle[0],simx_opmode_blocking);
              Valido[1]=simxGetObjectHandle(clientID,"motori",&ObjectHandle[1],simx_opmode_blocking);
              Valido[2]=simxGetObjectHandle(clientID,"Pioneer_p3dx",&ObjectHandle[2],simx_opmode_blocking);
              Valido[3]=0;
              break;
          case 1:
              Valido[0]=simxGetObjectHandle(clientID,"motor1",&ObjectHandle[0],simx_opmode_blocking);
              Valido[1]=simxGetObjectHandle(clientID,"motor2",&ObjectHandle[1],simx_opmode_blocking);
              Valido[2]=simxGetObjectHandle(clientID,"Bola",&ObjectHandle[2],simx_opmode_blocking);
              Valido[3]=0;
              break;
          case 2:
              Valido[0]=simxGetObjectHandle(clientID,"abase",&ObjectHandle[0],simx_opmode_blocking);
              Valido[1]=simxGetObjectHandle(clientID,"Proximity_sensor",&ObjectHandle[1],simx_opmode_blocking);
              Valido[2]=simxGetObjectHandle(clientID,"ball",&ObjectHandle[2],simx_opmode_blocking);
              Valido[3]=0;
              break;
          case 3:
              Valido[0]=simxGetObjectHandle(clientID,"motor",&ObjectHandle[0],simx_opmode_blocking);
              Valido[1]=simxGetObjectHandle(clientID,"Potenciometro",&ObjectHandle[1],simx_opmode_blocking);
              Valido[2]=simxGetObjectHandle(clientID,"rotating",&ObjectHandle[2],simx_opmode_blocking);
              Valido[3]=simxGetObjectHandle(clientID,"pendulum",&ObjectHandle[3],simx_opmode_blocking);;
              break;
      }
      bool flag=true;
      for(int i=1;i<4;i++)
      {
          if(Valido[i]!=0)
          {
              flag=false;
              mexPrintf("Remote API function call returned with error code \n");
              simxFinish(clientID);
          }
      }
      
      if(flag)
      {
          mexPrintf("Objetos obtenidos exitosamente");
          if((int) Modo[0]==0)
          {
            simxGetObjectPosition(clientID,ObjectHandle[2],-1,&Pos,simx_opmode_streaming);  
            simxGetObjectOrientation(clientID,ObjectHandle[2],-1,&Ori,simx_opmode_streaming);         
            Sleep(1);
            simxGetObjectPosition(clientID,ObjectHandle[2],-1,&Pos,simx_opmode_buffer);
            simxGetObjectOrientation(clientID,ObjectHandle[2],-1,&Ori,simx_opmode_buffer);   
          }
          if((int) Modo[0]==2)
          {
            simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_streaming);
            simxReadProximitySensor(clientID,ObjectHandle[1],NULL,&Pos,NULL,NULL,simx_opmode_streaming);
            simxGetObjectVelocity(clientID,ObjectHandle[2],&Vel,NULL,simx_opmode_streaming);         
            Sleep(1);
            simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_buffer);
            simxReadProximitySensor(clientID,ObjectHandle[1],NULL,&Pos,NULL,NULL,simx_opmode_buffer);
            simxGetObjectVelocity(clientID,ObjectHandle[2],&Vel,NULL,simx_opmode_buffer);
          }
          if((int) Modo[0]==3)
          {
            simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_streaming);
            simxGetJointPosition(clientID,ObjectHandle[1],&Phi1,simx_opmode_streaming);
            simxGetObjectVelocity(clientID,ObjectHandle[2],NULL,&Ang,simx_opmode_streaming);
            simxGetObjectVelocity(clientID,ObjectHandle[2],NULL,&Ang1,simx_opmode_streaming);         
            Sleep(1);
            simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_buffer);
            simxGetJointPosition(clientID,ObjectHandle[1],&Phi1,simx_opmode_buffer);
            simxGetObjectVelocity(clientID,ObjectHandle[2],NULL,&Ang,simx_opmode_buffer);
            simxGetObjectVelocity(clientID,ObjectHandle[3],NULL,&Ang1,simx_opmode_buffer);
          }
      }
      
}
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void V_Rep_Outputs_wrapper(const real_T *u0,
			real_T *x0,
			const real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
switch ((int) Modo[0])
{
    case 0:
        simxGetObjectPosition(clientID,ObjectHandle[2],-1,&Pos,simx_opmode_buffer);
        simxGetObjectOrientation(clientID,ObjectHandle[2],-1,&Ori,simx_opmode_buffer); 
        x0[0]=Pos[0];
        x0[1]=Pos[1];
        x0[2]=Ori[2];
        break;
    case 1:
        simxGetObjectPosition(clientID,ObjectHandle[2],-1,&Pos,simx_opmode_oneshot_wait);
        simxGetObjectVelocity(clientID,ObjectHandle[2],&Vel,NULL,simx_opmode_oneshot_wait);
        simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_oneshot_wait);
        simxGetJointPosition(clientID,ObjectHandle[1],&Phi1,simx_opmode_oneshot_wait);
        x0[0]=Pos[0]/cos(Phi);
        x0[1]=Vel[0];
        x0[2]=Pos[1]/cos(Phi1);
        x0[3]=Vel[1];
        break;
    case 2:
        simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_buffer);
        simxReadProximitySensor(clientID,ObjectHandle[1],NULL,&Pos,NULL,NULL,simx_opmode_buffer);
        simxGetObjectVelocity(clientID,ObjectHandle[2],&Vel,NULL,simx_opmode_buffer);    
        x0[0]=Pos[2]-0.5;
        x0[1]=Phi;
        x0[2]=Vel[1];
        x0[3]=0;
        break;
    case 3:
        simxGetJointPosition(clientID,ObjectHandle[0],&Phi,simx_opmode_buffer);
        simxGetJointPosition(clientID,ObjectHandle[1],&Phi1,simx_opmode_buffer);
        simxGetObjectVelocity(clientID,ObjectHandle[2],NULL,&Ang,simx_opmode_buffer);
        simxGetObjectVelocity(clientID,ObjectHandle[3],NULL,&Ang1,simx_opmode_buffer);
        x0[0]=Phi;
        x0[1]=Ang[2];
        x0[2]=Phi1;
        x0[3]=Ang1[0];
        break;
}
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}

/*
 * Updates function
 *
 */
void V_Rep_Update_wrapper(const real_T *u0,
			real_T *x0,
			real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Update_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Code example
 *   xD[0] = u0[0];
 */
switch ((int) Modo[0])
{
    case 0:
        simxSetJointTargetVelocity(clientID,ObjectHandle[0],u0[0],simx_opmode_blocking);
        simxSetJointTargetVelocity(clientID,ObjectHandle[1],u0[1],simx_opmode_blocking);
        break;
    case 1:
        simxSetJointTargetPosition(clientID,ObjectHandle[0],u0[0],simx_opmode_oneshot);
        simxSetJointTargetPosition(clientID,ObjectHandle[1],u0[1],simx_opmode_oneshot);
        break;
    case 2:
        simxSetJointTargetVelocity(clientID,ObjectHandle[0],u0[0],simx_opmode_blocking);
        break;
    case 3:
        simxSetJointTargetVelocity(clientID,ObjectHandle[0],u0[0],simx_opmode_blocking);
        break;
        
}
/* %%%-SFUNWIZ_wrapper_Update_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Terminate function
 *
 */
void V_Rep_Terminate_wrapper(real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S)
{
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Terminate code goes here.
 */
if(clientID!=-1)
{
      simxFinish(clientID);
      mexPrintf ("Simulación terminada con éxito");
}
/* %%%-SFUNWIZ_wrapper_Terminate_Changes_END --- EDIT HERE TO _BEGIN */
}

