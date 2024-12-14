
#ifndef GRAPHIC_H
#define GRAPHIC_H

#include "oled.h"

#define GRAPHIC_HEIGHT_PX OLED_HEIGHT_PX
#define GRAPHIC_WIDTH_PX OLED_WIDTH_PX

#define GRAPHIC_RED_256 (uint8_t)(0b11100000)
#define GRAPHIC_GREEN_256 (uint8_t)(0b11100)
#define GRAPHIC_BLUE_256 (uint8_t)(0b11)
#define GRAPHIC_WHITE_256 (uint8_t)(0b11111111)
#define GRAPHIC_BLACK_256 (uint8_t)(0)

typedef struct {
    int row;
    int col;
} point_t;

extern void display(void);
extern void set_background(int16_t color);
extern void set_background_sect(int bnd, uint8_t col_left, uint8_t col_right);
extern void clear(void);
extern void circle(point_t* p_center, unsigned int radius, int16_t color_rim, int16_t color_inside);
extern void line(const point_t* p1, const point_t* p2, int16_t color);

#endif //GRAPHIC_H