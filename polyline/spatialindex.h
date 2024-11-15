#ifndef SPATIAL_INDRX_H
#define SPATIAL_INDRX_H

typedef struct spatialIndex_s spatialIndex_t;


spatialIndex_t *create_spidx(size_t numItems, size_t nodeSize);
void free_spidx(spatialIndex_t *idx);

void spidx_add(spatialIndex_t *idx, double minx, double miny, double maxx, double maxy);
void spidx_finish(spaitalIndex_t *idx);



#endif