#ifndef H_ObjLoader
#define H_ObjLoader

typedef struct {

       int v1;
       int v2;
       int v3;

} Triangle ;

typedef struct {

      float x;
      float y;
      float z;

} Vertex ;

typedef struct {

      float vn1;
      float vn2;
      float vn3;

} Normals ;

void DrawOBJ(char *fName);

#endif
