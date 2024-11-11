#ifndef TG_H
#define TG_H

typedef int boolean;
#define TG_TRUE  (1)
#define TG_FALSE (0)

typedef struct tg_point {
    double x, y;
} tg_point;

typedef struct tg_line {
    tg_point begin_point;
    tg_point end_point;
} tg_line;

typedef struct tg_arc {
    tg_point center;
    double radius;
    double start_angle;
    double end_angle;
    boolean revserse;
} tg_arc;

typedef struct tg_circle {
    tg_point center;
    double radius;
} tg_circle;

typedef struct tg_ellipse {
    tg_point center;
    tg_point major_axis;
    double ratio;
    double start_angle;
    double end_angle;
    boolean revserse;
} tg_ellipse;

typedef struct tg_vertex {
    tg_point point;
    double bulge;
} tg_vertex;

typedef struct tg_polyline {
    int vertex_count;
    boolean closed;
    tg_vertex *vertex;
} tg_polyline;

typedef struct tg_object {
    int type;
    union {
        tg_point point;
        tg_line line;
        tg_arc arc;
        tg_circle circle;
        tg_ellipse ellipse;
        tg_polyline polyline;
    };
} tg_object;

#endif