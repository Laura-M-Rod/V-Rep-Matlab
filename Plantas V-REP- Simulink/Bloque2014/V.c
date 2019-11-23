
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME V
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
/* %%%-SFUNWIZ_defines_Changes_BEGIN --- EDIT HERE TO _END */
#define NUM_INPUTS            1
/* Input Port  0 */
#define IN_PORT_0_NAME        u0
#define INPUT_0_WIDTH         DYNAMICALLY_SIZED
#define INPUT_DIMS_0_COL      1
#define INPUT_0_DTYPE         real_T
#define INPUT_0_COMPLEX       COMPLEX_NO
#define IN_0_FRAME_BASED      FRAME_NO
#define IN_0_BUS_BASED        0
#define IN_0_BUS_NAME         
#define IN_0_DIMS             1-D
#define INPUT_0_FEEDTHROUGH   0
#define IN_0_ISSIGNED         0
#define IN_0_WORDLENGTH       8
#define IN_0_FIXPOINTSCALING  1
#define IN_0_FRACTIONLENGTH   9
#define IN_0_BIAS             0
#define IN_0_SLOPE            0.125

#define NUM_OUTPUTS           1
/* Output Port  0 */
#define OUT_PORT_0_NAME       x0
#define OUTPUT_0_WIDTH        DYNAMICALLY_SIZED
#define OUTPUT_DIMS_0_COL     1
#define OUTPUT_0_DTYPE        real_T
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUT_0_FRAME_BASED     FRAME_NO
#define OUT_0_BUS_BASED       0
#define OUT_0_BUS_NAME        
#define OUT_0_DIMS            1-D
#define OUT_0_ISSIGNED        1
#define OUT_0_WORDLENGTH      8
#define OUT_0_FIXPOINTSCALING 1
#define OUT_0_FRACTIONLENGTH  3
#define OUT_0_BIAS            0
#define OUT_0_SLOPE           0.125

#define NPARAMS               3
/* Parameter 0 */
#define PARAMETER_0_NAME      Modo
#define PARAMETER_0_DTYPE     real_T
#define PARAMETER_0_COMPLEX   COMPLEX_NO
/* Parameter 1 */
#define PARAMETER_1_NAME      Tiempo
#define PARAMETER_1_DTYPE     real_T
#define PARAMETER_1_COMPLEX   COMPLEX_NO
/* Parameter 1 */
#define PARAMETER_2_NAME      Salidas
#define PARAMETER_2_DTYPE     real_T
#define PARAMETER_2_COMPLEX   COMPLEX_NO

#define SAMPLE_TIME_0         Tiempo
#define NUM_DISC_STATES       1
#define DISC_STATES_IC        [0]
#define NUM_CONT_STATES       0
#define CONT_STATES_IC        [0]
#define S_FUNCTIONS_LIB
#define SFUNWIZ_GENERATE_TLC  0
#define PANELINDEX            8
#define USE_SIMSTRUCT         1
#define SHOW_COMPILE_STEPS    1
#define CREATE_DEBUG_MEXFILE  1
#define SAVE_CODE_ONLY        0
#define SFUNWIZ_REVISION      3.0
/* %%%-SFUNWIZ_defines_Changes_END --- EDIT HERE TO _BEGIN */
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#include "simstruc.h"
#define PARAM_DEF0(S) ssGetSFcnParam(S, 0)
#define PARAM_DEF1(S) ssGetSFcnParam(S, 1)

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define IS_PARAM_UINT32(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsUint32(pVal))

extern void V_Rep_Start_wrapper(real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S);
extern void V_Rep_Outputs_wrapper(const real_T *u0,
			real_T *x0,
			const real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S);
extern void V_Rep_Update_wrapper(const real_T *u0,
			real_T *x0,
			real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S);
extern void V_Rep_Terminate_wrapper(real_T *xD,
			const real_T *Modo, const int_T p_width0,
			const real_T *Tiempo, const int_T p_width1,
			SimStruct *S);
/*====================*
 * S-function methods *
 *====================*/
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
/* Function: mdlCheckParameters =============================================
 * Abstract:
 *     Verify parameter definitions and types.
 */
static void mdlCheckParameters(SimStruct *S)
{
    int paramIndex  = 0;
    bool invalidParam = false;
    /* All parameters must match the S-function Builder Dialog */

    {
        const mxArray *pVal0 = ssGetSFcnParam(S, 0);
        if (!IS_PARAM_DOUBLE(pVal0)) {
            invalidParam = true;
            paramIndex = 0;
            goto EXIT_POINT;
        }
    }

    {
        const mxArray *pVal1 = ssGetSFcnParam(S, 1);
        if (!mxIsDouble(pVal1)) {
            ssSetErrorStatus(S, "Sample time parameter Tiempo must be of type double");
            return;
        }
    }

    {
        const mxArray *pVal1 = ssGetSFcnParam(S, 1);
        if (!IS_PARAM_DOUBLE(pVal1)) {
            invalidParam = true;
            paramIndex = 1;
            goto EXIT_POINT;
        }
    }
    
    {
        const mxArray *pVal2 = ssGetSFcnParam(S, 2);
        if (!mxIsDouble(pVal2)) {
            ssSetErrorStatus(S, "Sample time parameter Tiempo must be of type double");
            return;
        }
    }

    {
        const mxArray *pVal2 = ssGetSFcnParam(S, 2);
        if (!IS_PARAM_DOUBLE(pVal2)) {
            invalidParam = true;
            paramIndex = 1;
            goto EXIT_POINT;
        }
    }
    


    EXIT_POINT:
    if (invalidParam) {
        char parameterErrorMsg[1024];
        sprintf(parameterErrorMsg, "The data type and or complexity of parameter %d does not match the "
                "information specified in the S-function Builder dialog. "
                "For non-double parameters you will need to cast them using int8, int16, "
                "int32, uint8, uint16, uint32 or boolean.", paramIndex + 1);
        ssSetErrorStatus(S, parameterErrorMsg);
    }
    return;
}
#endif /* MDL_CHECK_PARAMETERS */
/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{

    DECL_AND_INIT_DIMSINFO(inputDimsInfo);
    DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NPARAMS); /* Number of expected parameters */
    #if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    #endif


    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetNumContStates(S, NUM_CONT_STATES);
    ssSetNumDiscStates(S, NUM_DISC_STATES);
    
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;
    ssSetInputPortWidth(S, 0, INPUT_0_WIDTH);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, INPUT_0_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/

    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;
    ssSetOutputPortWidth(S, 0, *mxGetPr(ssGetSFcnParam(S, 2)));
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);
    ssSetNumPWork(S, 0);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetSimulinkVersionGeneratedIn(S, "9.1");

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_WORKS_WITH_CODE_REUSE));
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_WIDTH
static void mdlSetInputPortWidth(SimStruct *S, int_T port,
                                 int_T inputPortWidth)
{
    ssSetInputPortWidth(S, port, inputPortWidth);
}
#define MDL_SET_OUTPUT_PORT_WIDTH
static void mdlSetOutputPortWidth(SimStruct *S, int_T port,
                                  int_T outputPortWidth)
{
    ssSetOutputPortWidth(S, port, *mxGetPr(ssGetSFcnParam(S, 2)));
}
#endif
/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, *mxGetPr(ssGetSFcnParam(S, 1)));
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    ssSetOffsetTime(S, 0, 0.0);
}
#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize the states
 */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *xD = ssGetRealDiscStates(S);

    xD[0] = 0;
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType(S, 0, dType);
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetOutputPortDataType(S, 0, dType);
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
}

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)

static void mdlSetWorkWidths(SimStruct *S)
{

}

#endif

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
/* Function: mdlStart =======================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
    real_T *xD = ssGetDiscStates(S);
    const int_T   p_width0  = mxGetNumberOfElements(PARAM_DEF0(S));
    const int_T   p_width1  = mxGetNumberOfElements(PARAM_DEF1(S));
    const real_T *Modo = (const real_T *) mxGetData(PARAM_DEF0(S));
    const real_T *Tiempo = (const real_T *) mxGetData(PARAM_DEF1(S));
    
    V_Rep_Start_wrapper(xD, Modo, p_width0, Tiempo, p_width1, S);
}
#endif /*  MDL_START */

/* Function: mdlOutputs =======================================================
 *
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *u0 = (real_T *) ssGetInputPortRealSignal(S, 0);
    real_T *x0 = (real_T *) ssGetOutputPortRealSignal(S, 0);
    const real_T *xD = ssGetDiscStates(S);
    const int_T   p_width0  = mxGetNumberOfElements(PARAM_DEF0(S));
    const int_T   p_width1  = mxGetNumberOfElements(PARAM_DEF1(S));
    const real_T *Modo = (const real_T *) mxGetData(PARAM_DEF0(S));
    const real_T *Tiempo = (const real_T *) mxGetData(PARAM_DEF1(S));
        const int_T u_width = ssGetInputPortWidth(S, 0);

    V_Rep_Outputs_wrapper(u0, x0, xD, Modo, p_width0, Tiempo, p_width1, S);

}

#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is
 *    useful for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    const real_T *u0 = (real_T *) ssGetInputPortRealSignal(S, 0);
    real_T *x0 = (real_T *) ssGetOutputPortRealSignal(S, 0);
    real_T *xD = ssGetDiscStates(S);
    const int_T   p_width0  = mxGetNumberOfElements(PARAM_DEF0(S));
    const int_T   p_width1  = mxGetNumberOfElements(PARAM_DEF1(S));
    const real_T *Modo = (const real_T *) mxGetData(PARAM_DEF0(S));
    const real_T *Tiempo = (const real_T *) mxGetData(PARAM_DEF1(S));
        const int_T u_width = ssGetInputPortWidth(S, 0);

    V_Rep_Update_wrapper(u0, x0, xD, Modo, p_width0, Tiempo, p_width1, S);

}
#endif /* MDL_UPDATE */
/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    real_T *xD = ssGetDiscStates(S);
    const int_T   p_width0  = mxGetNumberOfElements(PARAM_DEF0(S));
    const int_T   p_width1  = mxGetNumberOfElements(PARAM_DEF1(S));
    const real_T *Modo = (const real_T *) mxGetData(PARAM_DEF0(S));
    const real_T *Tiempo = (const real_T *) mxGetData(PARAM_DEF1(S));
    
    V_Rep_Terminate_wrapper(xD, Modo, p_width0, Tiempo, p_width1, S);
}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



