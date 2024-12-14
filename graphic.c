
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "oled.h"
#include "graphic.h"
#include "_graphic.h"


uint8_t pix[GRAPHIC_HEIGHT_PX][GRAPHIC_WIDTH_PX];

void display(void){
    sendDataContinu(pix, GRAPHIC_HEIGHT_PX*GRAPHIC_WIDTH_PX);
}

extern void clear(void){
    memset(pix, 0, sizeof(pix[0][0])*GRAPHIC_HEIGHT_PX*GRAPHIC_WIDTH_PX);

}

/*
void set_background(uint8_t color){
    int x, y;
    for(y=0; y <= GRAPHIC_HEIGHT_PX-1; y++){
        for(x=0; x<=GRAPHIC_WIDTH_PX-1; x++){
            pix[y][x] = color;
        }
    }
}
*/

void set_background(int16_t color){
    int x, y;
    if(color >= 0){
        for(y=0; y <= GRAPHIC_HEIGHT_PX-1; y++){
            for(x=0; x<=GRAPHIC_WHITE_256-1; x++){
                pix[y][x] = color;
            }
        }
    }
}



void circle(point_t* p_center, unsigned int radius, int16_t color_rim, int16_t color_inside){
    point_t left_top, right_bottom, dominant_quad, start;

    //scopingで計算する範囲を絞る。
    scoping(p_center, radius, &left_top, &right_bottom);

    //計算を行う象限を決定する。例えば第一象限に円の中心があれば、円の第３象限が計算対象になる。
    dominant_quadrant(p_center, &left_top, &right_bottom, &dominant_quad, &start);


    //いよいよ計算を行う。
    int y, temp_term;
//    y = start.row;
    const unsigned int rad_sq = radius * radius; 

    /*TODO: yの適切な終点をhorizontal_scan()に倣って実装する*/
    const int sign_y = dominant_quad.row;

    if(start.row==left_top.row){
        temp_term = right_bottom.row;
    } else {
        temp_term = left_top.row;
    }
    if(temp_term*sign_y < (p_center->row)*sign_y){
        temp_term = p_center->row;
    }
    const int terminal = temp_term;

    for(y=start.row;
        terminal*sign_y <= y*sign_y;
        y -= dominant_quad.row){
        
        hrizontal_scan(p_center, radius, &left_top, &right_bottom, &dominant_quad, &start, y, color_rim, color_inside);
    }
}

void line(const point_t* p1, const point_t* p2, int16_t color){
    const point_t left_top = {
        .col=0,
        .row =0,
    };
    const point_t right_down = {
        .col=GRAPHIC_WIDTH_PX-1,
        .row=GRAPHIC_HEIGHT_PX-1,
    };

    const point_t vec = {
        .col = (p2->col)-(p1->col),
        .row =  (p2->row)-(p1->row),
    };

    int x, y;
    int step;
    int sq_x = (vec.col) * (vec.col);
    int sq_y = (vec.row) * (vec.row);
    int sign;

    if(sq_x <= sq_y){
        //縦に移動したいとき
        if(0 <= vec.row){
            sign = 1;
        } else {
            sign = -1;
        }
        for(step=0; (p1->row)*sign+step <= (p2->row)*sign; step++){
            y = p1->row + step*sign;
            x = p1->col + (vec.col * sign*step)/(vec.row);
            if(is_in_scope(&left_top, &right_down, x, y) && 0 <= color){
                pix[y][x] = color;
            }
        }
    } else {    //横に移動するとき
        if(0 <= vec.col){
            sign = 1;
        } else {
            sign = -1;
        }
        for(step=0; (p1->col)*sign+step <= (p2->col)*sign; step++){
            x = p1->col + sign*step;
            y = p1->row + (vec.row*sign*step)/(vec.col);
            if(is_in_scope(&left_top, &right_down, x, y) && 0 <= color){
                pix[y][x] = color;
            }
        }
    } 

}