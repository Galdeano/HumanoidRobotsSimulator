#ifndef _MGL_CONFIG_H_
#define _MGL_CONFIG_H_

/* This define enables double precision in MathGL */
#define MGL_USE_DOUBLE 0
#define MGL_NO_DATA_A 0

#define MGL_HAVE_GSL 1
#define MGL_HAVE_LTDL
#if defined(_MSC_VER) || defined(__BORLANDC__)
#define MGL_HAVE_PTHREAD 0
#else
#define MGL_HAVE_PTHREAD 0
#endif
#define MGL_HAVE_HDF4 0
#define MGL_HAVE_HDF5 0
#define MGL_HAVE_HDF5_18
#define MGL_HAVE_JPEG 1
#define MGL_HAVE_PNG 1
#define MGL_HAVE_ZLIB 0
#define MGL_HAVE_U3D
#define MGL_HAVE_PDF 0
#define MGL_HAVE_GIF 1
#define MGL_HAVE_OPENGL 1
#define MGL_HAVE_GLUT 0
#define MGL_HAVE_FLTK 1
#define MGL_HAVE_WX 0
#define MGL_HAVE_QT 1
#define MGL_HAVE_PYTHON 0
#define MGL_HAVE_OCTAVE 0
#define MGL_HAVE_DOC 0
#define MGL_HAVE_MPI 0

#endif
