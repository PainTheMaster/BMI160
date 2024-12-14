#ifndef _GRAPHIC_H
#define _GRAPHIC_H

#include "graphic.h"

#define FLAG_CHECK_COMMENCED (1<<0)

#define FLAG_OPERATION_OUT (1<<0)
#define FLAG_OPERATION_BOARDER (1<<1)
#define FLAG_OPERATION_FILL (1<<2)

extern void scoping(const point_t* p_center,
                    unsigned int radius,
                    point_t* p_left_top,
                    point_t* p_right_bottom);

extern unsigned int dist_sq(int pt1_x, int pt1_y, int pt2_x, int pt2_y);


extern void dominant_quadrant(const point_t* p_center,
                              const point_t* p_left_top,
                              const point_t* p_right_bottom,
                              point_t* quad,
                              point_t* start);

extern void copy(const point_t* p_center,
                 const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int x,
                 int y,
                 uint8_t color);

extern int is_in_scope(const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int x,
                 int y);

extern int is_in_scope_x(const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int x);

extern int is_in_scope_y(const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int y);

extern void hrizontal_scan(const point_t* p_center,
                        unsigned int radius,
                        const point_t* p_left_top,
                        const point_t* p_right_bottom,
                        const point_t* p_dominant_quad,
                        const point_t* start,
                        int y,
                        int16_t color_rim,
                        int16_t color_inside);


#endif