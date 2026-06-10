#include "OpenGLHeaders.h"
#include "Shader.h"
#include "DrawLight.h"

float g_shadowMatrix[16];

materialStruct test;
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


#include "Setup.h"

void init_material()
{
//set_material(&emerald);
//set_material(&jade);
//set_material(&obsidian);
//set_material(&pearl);
//set_material(&ruby);
//set_material(&turquoise);
//set_material(&brass);
//set_material(&bronze);
//set_material(&chrome);
//set_material(&copper);
//set_material(&gold);
//set_material(&silver);
//set_material(&black_plastic);
//set_material(&cyan_plastic);
//set_material(&green_plastic);
//set_material(&red_plastic);
//set_material(&white_plastic);
//set_material(&yellow_plastic);
//set_material(&black_rubber);
//set_material(&cyan_rubber);
//set_material(&green_rubber);
//set_material(&red_rubber);
//set_material(&white_rubber);
//set_material(&yellow_rubber);

    test.Ka[0]=0.35;
    test.Ka[1]=0.35;
    test.Ka[2]=0.25;
    test.Ka[3]=1.0;
    test.Kd[0]=0.5;
    test.Kd[1]=0.5;
    test.Kd[2]=0.5;
    test.Kd[3]=1.0;
    test.Ks[0]=0.5;
    test.Ks[1]=0.5;
    test.Ks[2]=0.5;
    test.Ks[3]=1.0;
    test.n=50;

    red_shiny_plastic.Ka[0]=0.3;
    red_shiny_plastic.Ka[1]=0.0;
    red_shiny_plastic.Ka[2]=0.0;
    red_shiny_plastic.Ka[3]=1.0;
    red_shiny_plastic.Kd[0]=0.6;
    red_shiny_plastic.Kd[1]=0.0;
    red_shiny_plastic.Kd[2]=0.0;
    red_shiny_plastic.Kd[3]=1.0;
    red_shiny_plastic.Ks[0]=0.8;
    red_shiny_plastic.Ks[1]=0.6;
    red_shiny_plastic.Ks[2]=0.6;
    red_shiny_plastic.Ks[3]=1.0;
    red_shiny_plastic.n=100.0;

    white_shiny_plastic.Ka[0]=0;
    white_shiny_plastic.Ka[1]=0;
    white_shiny_plastic.Ka[2]=0;
    white_shiny_plastic.Ka[3]=1.0;
    white_shiny_plastic.Kd[0]=0.55;
    white_shiny_plastic.Kd[1]=0.55;
    white_shiny_plastic.Kd[2]=0.55;
    white_shiny_plastic.Kd[3]=1.0;
    white_shiny_plastic.Ks[0]=0.7;
    white_shiny_plastic.Ks[1]=0.7;
    white_shiny_plastic.Ks[2]=0.7;
    white_shiny_plastic.Ks[3]=1.0;
    white_shiny_plastic.n=100;

    shiny_gold.Ka[0]=0.24725*3;
    shiny_gold.Ka[1]=0.1995*3;
    shiny_gold.Ka[2]=0.0745*3;
    shiny_gold.Ka[3]=1.0;
    shiny_gold.Kd[0]=0.75164;
    shiny_gold.Kd[1]=0.60648;
    shiny_gold.Kd[2]=0.22648;
    shiny_gold.Kd[3]=1.0;
    shiny_gold.Ks[0]=0.62828;
    shiny_gold.Ks[1]=0.5558;
    shiny_gold.Ks[2]=0.36606;
    shiny_gold.Ks[3]=1.0;
    shiny_gold.n=100;

    emerald.Ka[0]=0.0215;
    emerald.Ka[1]=0.1745;
    emerald.Ka[2]=0.0215;
    emerald.Ka[3]=1.0;
    emerald.Kd[0]=0.07568;
    emerald.Kd[1]=0.61424;
    emerald.Kd[2]=0.07568;
    emerald.Kd[3]=1.0;
    emerald.Ks[0]=0.633;
    emerald.Ks[1]=0.72781;
    emerald.Ks[2]=0.633;
    emerald.Ks[3]=1.0;
    emerald.n=76.8;

    jade.Ka[0]=0.135;
    jade.Ka[1]=0.2225;
    jade.Ka[2]=0.1575;
    jade.Ka[3]=1.0;
    jade.Kd[0]=0.54;
    jade.Kd[1]=0.89;
    jade.Kd[2]=0.63;
    jade.Kd[3]=1.0;
    jade.Ks[0]=0.31623;
    jade.Ks[1]=0.31623;
    jade.Ks[2]=0.31623;
    jade.Ks[3]=1.0;
    jade.n=12.8;

    obsidian.Ka[0]=0.05375;
    obsidian.Ka[1]=0.05;
    obsidian.Ka[2]=0.06625;
    obsidian.Ka[3]=1.0;
    obsidian.Kd[0]=0.18275;
    obsidian.Kd[1]=0.17;
    obsidian.Kd[2]=0.22525;
    obsidian.Kd[3]=1.0;
    obsidian.Ks[0]=0.33274;
    obsidian.Ks[1]=0.32863;
    obsidian.Ks[2]=0.34643;
    obsidian.Ks[3]=1.0;
    obsidian.n=38.4;

    pearl.Ka[0]=0.25;
    pearl.Ka[1]=0.20725;
    pearl.Ka[2]=0.20725;
    pearl.Ka[3]=1.0;
    pearl.Kd[0]=1;
    pearl.Kd[1]=0.829;
    pearl.Kd[2]=0.829;
    pearl.Kd[3]=1.0;
    pearl.Ks[0]=0.29665;
    pearl.Ks[1]=0.29665;
    pearl.Ks[2]=0.29665;
    pearl.Ks[3]=1.0;
    pearl.n=11.264;

    ruby.Ka[0]=0.1745;
    ruby.Ka[1]=0.01175;
    ruby.Ka[2]=0.01175;
    ruby.Ka[3]=1.0;
    ruby.Kd[0]=0.61424;
    ruby.Kd[1]=0.04136;
    ruby.Kd[2]=0.04136;
    ruby.Kd[3]=1.0;
    ruby.Ks[0]=0.72781;
    ruby.Ks[1]=0.62696;
    ruby.Ks[2]=0.62696;
    ruby.Ks[3]=1.0;
    ruby.n=76.8;

    turquoise.Ka[0]=0.1;
    turquoise.Ka[1]=0.18725;
    turquoise.Ka[2]=0.1745;
    turquoise.Ka[3]=1.0;
    turquoise.Kd[0]=0.396;
    turquoise.Kd[1]=0.74151;
    turquoise.Kd[2]=0.69102;
    turquoise.Kd[3]=1.0;
    turquoise.Ks[0]=0.29725;
    turquoise.Ks[1]=0.30829;
    turquoise.Ks[2]=0.30668;
    turquoise.Ks[3]=1.0;
    turquoise.n=12.8;

    brass.Ka[0]=0.32941;
    brass.Ka[1]=0.22353;
    brass.Ka[2]=0.027451;
    brass.Ka[3]=1.0;
    brass.Kd[0]=0.78039;
    brass.Kd[1]=0.56863;
    brass.Kd[2]=0.11373;
    brass.Kd[3]=1.0;
    brass.Ks[0]=0.99216;
    brass.Ks[1]=0.94118;
    brass.Ks[2]=0.80784;
    brass.Ks[3]=1.0;
    brass.n=27.8974;

    bronze.Ka[0]=0.2125;
    bronze.Ka[1]=0.1275;
    bronze.Ka[2]=0.054;
    bronze.Ka[3]=1.0;
    bronze.Kd[0]=0.714;
    bronze.Kd[1]=0.4284;
    bronze.Kd[2]=0.18144;
    bronze.Kd[3]=1.0;
    bronze.Ks[0]=0.39355;
    bronze.Ks[1]=0.27191;
    bronze.Ks[2]=0.16672;
    bronze.Ks[3]=1.0;
    bronze.n=25.6;

    chrome.Ka[0]=0.25;
    chrome.Ka[1]=0.25;
    chrome.Ka[2]=0.25;
    chrome.Ka[3]=1.0;
    chrome.Kd[0]=0.4;
    chrome.Kd[1]=0.4;
    chrome.Kd[2]=0.4;
    chrome.Kd[3]=1.0;
    chrome.Ks[0]=0.7746;
    chrome.Ks[1]=0.7746;
    chrome.Ks[2]=0.7746;
    chrome.Ks[3]=1.0;
    chrome.n=76.8;

    copper.Ka[0]=0.19125;
    copper.Ka[1]=0.0735;
    copper.Ka[2]=0.0225;
    copper.Ka[3]=1.0;
    copper.Kd[0]=0.7038;
    copper.Kd[1]=0.27048;
    copper.Kd[2]=0.0828;
    copper.Kd[3]=1.0;
    copper.Ks[0]=0.25678;
    copper.Ks[1]=0.13762;
    copper.Ks[2]=0.086014;
    copper.Ks[3]=1.0;
    copper.n=12.8;

    gold.Ka[0]=0.24725;
    gold.Ka[1]=0.1995;
    gold.Ka[2]=0.0745;
    gold.Ka[3]=1.0;
    gold.Kd[0]=0.75164;
    gold.Kd[1]=0.60648;
    gold.Kd[2]=0.22648;
    gold.Kd[3]=1.0;
    gold.Ks[0]=0.62828;
    gold.Ks[1]=0.5558;
    gold.Ks[2]=0.36606;
    gold.Ks[3]=1.0;
    gold.n=51.2;

    silver.Ka[0]=0.19225;
    silver.Ka[1]=0.19225;
    silver.Ka[2]=0.19225;
    silver.Ka[3]=1.0;
    silver.Kd[0]=0.50754;
    silver.Kd[1]=0.50754;
    silver.Kd[2]=0.50754;
    silver.Kd[3]=1.0;// transparence
    silver.Ks[0]=0.50827;
    silver.Ks[1]=0.50827;
    silver.Ks[2]=0.50827;
    silver.Ks[3]=1.0;
    silver.n=51.2;

    black_plastic.Ka[0]=0;
    black_plastic.Ka[1]=0;
    black_plastic.Ka[2]=0;
    black_plastic.Ka[3]=1.0;
    black_plastic.Kd[0]=0.01;
    black_plastic.Kd[1]=0.01;
    black_plastic.Kd[2]=0.01;
    black_plastic.Kd[3]=1.0;
    black_plastic.Ks[0]=0.5;
    black_plastic.Ks[1]=0.5;
    black_plastic.Ks[2]=0.5;
    black_plastic.Ks[3]=1.0;
    black_plastic.n=32;

    cyan_plastic.Ka[0]=0;
    cyan_plastic.Ka[1]=0.1;
    cyan_plastic.Ka[2]=0.06;
    cyan_plastic.Ka[3]=1.0;
    cyan_plastic.Kd[0]=0;
    cyan_plastic.Kd[1]=0.5098;
    cyan_plastic.Kd[2]=0.5098;
    cyan_plastic.Kd[3]=1.0;
    cyan_plastic.Ks[0]=0.50196;
    cyan_plastic.Ks[1]=0.50196;
    cyan_plastic.Ks[2]=0.50196;
    cyan_plastic.Ks[3]=1.0;
    cyan_plastic.n=32;

    green_plastic.Ka[0]=0;
    green_plastic.Ka[1]=0;
    green_plastic.Ka[2]=0;
    green_plastic.Ka[3]=1.0;
    green_plastic.Kd[0]=0.1;
    green_plastic.Kd[1]=0.35;
    green_plastic.Kd[2]=0.1;
    green_plastic.Kd[3]=1.0;
    green_plastic.Ks[0]=0.45;
    green_plastic.Ks[1]=0.55;
    green_plastic.Ks[2]=0.45;
    green_plastic.Ks[3]=1.0;
    green_plastic.n=32;

    red_plastic.Ka[0]=0;
    red_plastic.Ka[1]=0;
    red_plastic.Ka[2]=0;
    red_plastic.Ka[3]=1.0;
    red_plastic.Kd[0]=0.5;
    red_plastic.Kd[1]=0;
    red_plastic.Kd[2]=0;
    red_plastic.Kd[3]=1.0;
    red_plastic.Ks[0]=0.7;
    red_plastic.Ks[1]=0.6;
    red_plastic.Ks[2]=0.6;
    red_plastic.Ks[3]=1.0;
    red_plastic.n=32;

    white_plastic.Ka[0]=0;
    white_plastic.Ka[1]=0;
    white_plastic.Ka[2]=0;
    white_plastic.Ka[3]=1.0;
    white_plastic.Kd[0]=0.55;
    white_plastic.Kd[1]=0.55;
    white_plastic.Kd[2]=0.55;
    white_plastic.Kd[3]=1.0;
    white_plastic.Ks[0]=0.7;
    white_plastic.Ks[1]=0.7;
    white_plastic.Ks[2]=0.7;
    white_plastic.Ks[3]=1.0;
    white_plastic.n=32;

    yellow_plastic.Ka[0]=0;
    yellow_plastic.Ka[1]=0;
    yellow_plastic.Ka[2]=0;
    yellow_plastic.Ka[3]=1.0;
    yellow_plastic.Kd[0]=0.5;
    yellow_plastic.Kd[1]=0.5;
    yellow_plastic.Kd[2]=0;
    yellow_plastic.Kd[3]=1.0;
    yellow_plastic.Ks[0]=0.6;
    yellow_plastic.Ks[1]=0.6;
    yellow_plastic.Ks[2]=0.5;
    yellow_plastic.Ks[3]=1.0;
    yellow_plastic.n=32;

    black_rubber.Ka[0]=0.02;
    black_rubber.Ka[1]=0.02;
    black_rubber.Ka[2]=0.02;
    black_rubber.Ka[3]=1.0;
    black_rubber.Kd[0]=0.01;
    black_rubber.Kd[1]=0.01;
    black_rubber.Kd[2]=0.01;
    black_rubber.Kd[3]=1.0;
    black_rubber.Ks[0]=0.4;
    black_rubber.Ks[1]=0.4;
    black_rubber.Ks[2]=0.4;
    black_rubber.Ks[3]=1.0;
    black_rubber.n=10;

    cyan_rubber.Ka[0]=0;
    cyan_rubber.Ka[1]=0.05;
    cyan_rubber.Ka[2]=0.05;
    cyan_rubber.Ka[3]=1.0;
    cyan_rubber.Kd[0]=0.4;
    cyan_rubber.Kd[1]=0.5;
    cyan_rubber.Kd[2]=0.5;
    cyan_rubber.Kd[3]=1.0;
    cyan_rubber.Ks[0]=0.04;
    cyan_rubber.Ks[1]=0.7;
    cyan_rubber.Ks[2]=0.7;
    cyan_rubber.Ks[3]=1.0;
    cyan_rubber.n=10;

    green_rubber.Ka[0]=0;
    green_rubber.Ka[1]=0.05;
    green_rubber.Ka[2]=0;
    green_rubber.Ka[3]=1.0;
    green_rubber.Kd[0]=0.4;
    green_rubber.Kd[1]=0.5;
    green_rubber.Kd[2]=0.4;
    green_rubber.Kd[3]=1.0;
    green_rubber.Ks[0]=0.04;
    green_rubber.Ks[1]=0.7;
    green_rubber.Ks[2]=0.04;
    green_rubber.Ks[3]=1.0;
    green_rubber.n=10;

    red_rubber.Ka[0]=0.05;
    red_rubber.Ka[1]=0;
    red_rubber.Ka[2]=0;
    red_rubber.Ka[3]=1.0;
    red_rubber.Kd[0]=0.5;
    red_rubber.Kd[1]=0.4;
    red_rubber.Kd[2]=0.4;
    red_rubber.Kd[3]=1.0;
    red_rubber.Ks[0]=0.7;
    red_rubber.Ks[1]=0.04;
    red_rubber.Ks[2]=0.04;
    red_rubber.Ks[3]=1.0;
    red_rubber.n=10;

    white_rubber.Ka[0]=0.05;
    white_rubber.Ka[1]=0.05;
    white_rubber.Ka[2]=0.05;
    white_rubber.Ka[3]=1.0;
    white_rubber.Kd[0]=0.5;
    white_rubber.Kd[1]=0.5;
    white_rubber.Kd[2]=0.5;
    white_rubber.Kd[3]=1.0;
    white_rubber.Ks[0]=0.7;
    white_rubber.Ks[1]=0.7;
    white_rubber.Ks[2]=0.7;
    white_rubber.Ks[3]=1.0;
    white_rubber.n=10;

    yellow_rubber.Ka[0]=0.05;
    yellow_rubber.Ka[1]=0.05;
    yellow_rubber.Ka[2]=0;
    yellow_rubber.Ka[3]=1.0;
    yellow_rubber.Kd[0]=0.5;
    yellow_rubber.Kd[1]=0.5;
    yellow_rubber.Kd[2]=0.4;
    yellow_rubber.Kd[3]=1.0;
    yellow_rubber.Ks[0]=0.7;
    yellow_rubber.Ks[1]=0.7;
    yellow_rubber.Ks[2]=0.04;
    yellow_rubber.Ks[3]=1.0;
    yellow_rubber.n=10;

    glShadeModel(GL_FLAT);
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glEnable(GL_NORMALIZE);

}

extern Shader defaultShader;

void set_material(materialStruct *mat)
{
    defaultShader.use();
    defaultShader.setVec3("material_Ambient", glm::vec3(mat->Ka[0], mat->Ka[1], mat->Ka[2]));
    defaultShader.setVec3("material_Diffuse", glm::vec3(mat->Kd[0], mat->Kd[1], mat->Kd[2]));
    defaultShader.setVec3("material_Specular", glm::vec3(mat->Ks[0], mat->Ks[1], mat->Ks[2]));
    defaultShader.setFloat("material_Shininess", mat->n);
    defaultShader.setBool("use_Base_Color", false);

    // Sync activeColor with material's diffuse color so that flat lines use it too
    extern glm::vec3 activeColor;
    activeColor = glm::vec3(mat->Kd[0], mat->Kd[1], mat->Kd[2]);
}


//-----------------------------------------------------------------------------
// Name: buildShadowMatrix()
// Desc:
//-----------------------------------------------------------------------------
void buildShadowMatrix( float fMatrix[16], float fLightPos[4], float fPlane[4] )
{
    float dotp;

    // Calculate the dot-product between the plane and the light's position
    dotp = fPlane[0] * fLightPos[0] +
           fPlane[1] * fLightPos[1] +
           fPlane[2] * fLightPos[2] +
           fPlane[3] * fLightPos[3];

    // First column
    fMatrix[0]  = dotp - fLightPos[0] * fPlane[0];
    fMatrix[4]  = 0.0f - fLightPos[0] * fPlane[1];
    fMatrix[8]  = 0.0f - fLightPos[0] * fPlane[2];
    fMatrix[12] = 0.0f - fLightPos[0] * fPlane[3];

    // Second column
    fMatrix[1]  = 0.0f - fLightPos[1] * fPlane[0];
    fMatrix[5]  = dotp - fLightPos[1] * fPlane[1];
    fMatrix[9]  = 0.0f - fLightPos[1] * fPlane[2];
    fMatrix[13] = 0.0f - fLightPos[1] * fPlane[3];

    // Third column
    fMatrix[2]  = 0.0f - fLightPos[2] * fPlane[0];
    fMatrix[6]  = 0.0f - fLightPos[2] * fPlane[1];
    fMatrix[10] = dotp - fLightPos[2] * fPlane[2];
    fMatrix[14] = 0.0f - fLightPos[2] * fPlane[3];

    // Fourth column
    fMatrix[3]  = 0.0f - fLightPos[3] * fPlane[0];
    fMatrix[7]  = 0.0f - fLightPos[3] * fPlane[1];
    fMatrix[11] = 0.0f - fLightPos[3] * fPlane[2];
    fMatrix[15] = dotp - fLightPos[3] * fPlane[3];
}

//-----------------------------------------------------------------------------
// Name: findPlane()
// Desc: find the plane equation given 3 points
//-----------------------------------------------------------------------------
void findPlane( GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3] )
{
    GLfloat vec0[3], vec1[3];



    // Need 2 vectors to find cross product
    vec0[0] = v1[0] - v0[0];
    vec0[1] = v1[1] - v0[1];
    vec0[2] = v1[2] - v0[2];

    vec1[0] = v2[0] - v0[0];
    vec1[1] = v2[1] - v0[1];
    vec1[2] = v2[2] - v0[2];

    // Find cross product to get A, B, and C of plane equation
    plane[0] =   vec0[1] * vec1[2] - vec0[2] * vec1[1];
    plane[1] = -(vec0[0] * vec1[2] - vec0[2] * vec1[0]);
    plane[2] =   vec0[0] * vec1[1] - vec0[1] * vec1[0];

    plane[3] = -(plane[0] * v0[0] + plane[1] * v0[1] + plane[2] * v0[2]);
}

////-----------------------------------------------------------------------------
//// Name: renderFloor()
//// Desc:
////-----------------------------------------------------------------------------
//void renderFloor()
//{
//    glColor3f( 1.0f, 1.0f, 1.0f );
//    glInterleavedArrays( GL_N3F_V3F, 0, g_floorQuad );
//    glDrawArrays( GL_QUADS, 0, 4 );
//}




void Init_light()
{
    init_material();
}


