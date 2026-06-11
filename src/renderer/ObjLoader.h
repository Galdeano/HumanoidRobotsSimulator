#ifndef H_ObjLoader
#define H_ObjLoader

#include "OpenGLHeaders.h"
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

#include <utility>
#include <cstring>

struct MeshObj
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
    // Modern OpenGL buffer handles
    unsigned int vao;
    unsigned int vbo;
    unsigned int vbo_normal;
    unsigned int ebo;
    int index_count;
    int has_buffers;

    MeshObj() {
        texture = 0;
        displayList = 0;
        n_data = 0;
        firstTime = 0;
        v = nullptr;
        t = nullptr;
        vn = nullptr;
        gl_v = nullptr;
        gl_t = nullptr;
        gl_vn = nullptr;
        vertexCount = 0;
        triangleCount = 0;
        normalsCount = 0;
        memset(materiaux, 0, sizeof(materiaux));
        vao = 0;
        vbo = 0;
        vbo_normal = 0;
        ebo = 0;
        index_count = 0;
        has_buffers = 0;
    }

    ~MeshObj() {
        if (v) ::free(v);
        if (t) ::free(t);
        if (vn) ::free(vn);
        if (gl_v) ::free(gl_v);
        if (gl_t) ::free(gl_t);
        if (gl_vn) ::free(gl_vn);
        if (has_buffers) {
            glDeleteVertexArrays(1, &vao);
            glDeleteBuffers(1, &vbo);
            glDeleteBuffers(1, &vbo_normal);
            glDeleteBuffers(1, &ebo);
        }
    }

    // Disable copy semantics to prevent double free of pointers and buffers
    MeshObj(const MeshObj&) = delete;
    MeshObj& operator=(const MeshObj&) = delete;

    // Enable move semantics
    MeshObj(MeshObj&& other) noexcept {
        *this = std::move(other);
    }
    MeshObj& operator=(MeshObj&& other) noexcept {
        if (this != &other) {
            this->~MeshObj();

            texture = other.texture;
            displayList = other.displayList;
            n_data = other.n_data;
            firstTime = other.firstTime;
            v = other.v; other.v = nullptr;
            t = other.t; other.t = nullptr;
            vn = other.vn; other.vn = nullptr;
            gl_v = other.gl_v; other.gl_v = nullptr;
            gl_t = other.gl_t; other.gl_t = nullptr;
            gl_vn = other.gl_vn; other.gl_vn = nullptr;
            vertexCount = other.vertexCount;
            triangleCount = other.triangleCount;
            normalsCount = other.normalsCount;
            memcpy(materiaux, other.materiaux, sizeof(materiaux));
            vao = other.vao; other.vao = 0;
            vbo = other.vbo; other.vbo = 0;
            vbo_normal = other.vbo_normal; other.vbo_normal = 0;
            ebo = other.ebo; other.ebo = 0;
            index_count = other.index_count;
            has_buffers = other.has_buffers; other.has_buffers = 0;
        }
        return *this;
    }
};


void load_obj(char *fName,MeshObj *obj);
void buffer_model(MeshObj *obj);
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

#include "Shader.h"
void draw_model(MeshObj *obj, const glm::mat4 &Model);
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
