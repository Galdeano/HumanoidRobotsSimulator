#ifndef H_ObjLoader
#define H_ObjLoader

#include <GL/gl.h>
#include <GL/glu.h>
#include "uLink.h"
#include "Setup.h"

typedef struct
{

    unsigned int v1;
    unsigned int v2;
    unsigned int v3;

} Triangle ;

typedef struct
{

    GLfloat x;
    GLfloat y;
    GLfloat z;

} Vertex ;

typedef struct
{

    GLfloat vn1;
    GLfloat vn2;
    GLfloat vn3;

} Normals ;

typedef struct
{
    float x;
    float y;
    float z;
    float a;
} FloatVector ;

typedef struct
{
    FloatVector coul;
    char name[16];
} Material;

typedef struct
{
    GLuint texture;
    GLuint displayList;
    int n_data;
    int firstTime;
    Vertex *v;
    Triangle *t;
    Normals *vn;
    GLfloat *gl_v;
    unsigned int *gl_t;
    GLfloat *gl_vn;
    //float *vertice,*normals,*textures,*colours;
    int vertexCount, triangleCount, normalsCount;
    Material materiaux[10];
} MeshObj;


void load_obj(char *fName,MeshObj *obj);
/* void charger_obj(std::string,MeshObj *first=NULL);
   Charge un fichier OBJ et son MTL, prend en arguments le nom du modèle à charger et le pointeur de la première frame si le modèle appartient à une animation (sinon laissez-le à NULL). Cette fonction est appelée par le constructeur.
   Aucune valeur de retour.
*/
//MeshObj obj;
//load_obj("./cube.obj",&obj);

void load_mtl(char *fName);
/* void charger_mtl(std::string);
   Charge un fichier MTL, prend en argument le nom du fichier à charger. Cette fonction est appelée par charger_obj.
   Aucune valeur de retour.
*/

void draw_model(MeshObj *obj);
/* void draw_model(bool nor=true,bool tex=false);
   Dessine le modèle, prend en arguments deux booléens représentant respectivement les normales et la texture. Si nor vaut true alors on prend en compte les normales, et si tex vaut true alors on applique la texture.
   Aucune valeur de retour.
*/
//draw_model(&obj);

void setMaterialsAndTex(Material mats,GLuint tex);
/* void setMaterialsAndTex(std::vector<Material*> mats,GLuint tex);
   Définit directement les matériaux et la texture du modèle, prend en arguments un vector<Material*> et la texture. Cette fonction est appelée par giveMaterialsAndTex.
   Aucune valeur de retour.
*/

void DrawOBJ(char *fName);
//DrawOBJ("./cube.obj");

#endif
