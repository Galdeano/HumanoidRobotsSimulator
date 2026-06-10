#ifndef H_DrawLight
#define H_DrawLight


typedef struct
{
GLfloat Ka[4];
GLfloat Kd[4];
GLfloat Ks[4];
GLfloat n;
} materialStruct ;

struct LightVertex
{
    float nx, ny, nz;
    float x, y, z;
};



#ifdef __cplusplus
extern "C" {
#endif

extern float g_shadowMatrix[16];

extern materialStruct test;
extern materialStruct red_shiny_plastic;
extern materialStruct white_shiny_plastic;
extern materialStruct shiny_gold;

extern materialStruct emerald;
extern materialStruct jade;
extern materialStruct obsidian;
extern materialStruct pearl;
extern materialStruct ruby;
extern materialStruct turquoise;
extern materialStruct brass;
extern materialStruct bronze;
extern materialStruct chrome;
extern materialStruct copper;
extern materialStruct gold;
extern materialStruct silver;
extern materialStruct black_plastic;
extern materialStruct cyan_plastic;
extern materialStruct green_plastic;
extern materialStruct red_plastic;
extern materialStruct white_plastic;
extern materialStruct yellow_plastic;
extern materialStruct black_rubber;
extern materialStruct cyan_rubber;
extern materialStruct green_rubber;
extern materialStruct red_rubber;
extern materialStruct white_rubber;
extern materialStruct yellow_rubber;

#ifdef __cplusplus
}
#endif

void init_material();
void set_material(materialStruct *mat);

void buildShadowMatrix( float fMatrix[16], float fLightPos[4], float fPlane[4] );

void findPlane( GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3] );

//void renderFloor();

void Init_light();

#endif




