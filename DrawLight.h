#ifndef H_DrawLight
#define H_DrawLight


typedef struct
{
GLfloat Ka [4];
GLfloat Kd[4];
GLfloat Ks[4];
GLfloat n;
} materialStruct ;

struct Vertex
{
    float nx, ny, nz;
    float x, y, z;
};



float g_shadowMatrix[16];

materialStruct red_shiny_plastic;
materialStruct white_shiny_plastic;
materialStruct shiny_gold;

materialStruct emerald;
materialStruct jade;
materialStruct obsidian;
materialStruct pearl;
materialStruct ruby;
materialStruct turquoise;
materialStruct brass;
materialStruct bronze;
materialStruct chrome;
materialStruct copper;
materialStruct gold;
materialStruct silver;
materialStruct black_plastic;
materialStruct cyan_plastic;
materialStruct green_plastic;
materialStruct red_plastic;
materialStruct white_plastic;
materialStruct yellow_plastic;
materialStruct black_rubber;
materialStruct cyan_rubber;
materialStruct green_rubber;
materialStruct red_rubber;
materialStruct white_rubber;
materialStruct yellow_rubber;

void init_material();
void set_material(materialStruct *mat);

void buildShadowMatrix( float fMatrix[16], float fLightPos[4], float fPlane[4] );

void findPlane( GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3] );

//void renderFloor();

void Init_light();

#endif




