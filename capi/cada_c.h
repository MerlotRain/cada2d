#ifndef CADA_C_H
#define CADA_C_H

typedef void* CadaShapeH;

typedef enum {
  CADA_POINT,
  CADA_LINE,
  CADA_ARC,
  CADA_CIRCLE,
  CADA_ELLIPSE,
  CADA_POLYLINE,
  CADA_XLINE,
  CADA_RAY,
  CADA_SPLINE,
} CadaShapeType;

CadaShapeType GetShapeType(CadaShapeH shape);

CadaShapeH CadaShape_CreatePoint_r();
CadaShapeH CadaShape_CreatePointFromXY_r();


#endif