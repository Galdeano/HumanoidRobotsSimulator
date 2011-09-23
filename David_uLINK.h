#ifndef H_David_uLink
#define H_David_uLink


typedef struct
{
    char name[8];
    unsigned char sister,child,mother;
    float m,q,dq,ddq,u,ug,u_joint;
    float a[3];
    float b[3];
    float c[3];
    float p[3];
    float hw[3];
    float hv[3];
    float R[9];
} Struct_uLINK ;


typedef struct
{
    int desired_support;
    unsigned char ddl,support,right_foot_ID,left_foot_ID; /*  support: 0:none,1:right,2:left,3:both*/
    float distribution_y;
    float com_old[3];
    float FootCenter_R[3];
    float FootCenter_L[3];
} Struct_State ;

/* //    uLINK(n).dq     = 0;            % joint velocity [rad/s]
 * //    uLINK(n).ddq    = 0;            % joint acceleration [rad/s^2]
 * //    %uLINK(n).c      = [0 0 0]';     % center of gravity [m]
 * //    uLINK(n).c      = uLINK(n).b/2;     % center of gravity [m]
 * //    uLINK(n).I      = zeros(3,3);   % around the center of gravity inertia tensor [kg.m^2]
 * //    uLINK(n).Ir     = 0.0;          % child electric motor moment of inertia [kg.m^2]
 * //    uLINK(n).gr     = 0.0;          % reduction ratio of the motor [-]
 * //    uLINK(n).u      = 0.0;          % joint torque [Nm]
 */


void David_SetupRobot(Struct_uLINK uLINK[],Struct_State *Status);


#endif


