#include "graphic.h"
#include "_graphic.h"

extern uint8_t pix[GRAPHIC_HEIGHT_PX][GRAPHIC_WIDTH_PX];
uint8_t record[GRAPHIC_HEIGHT_PX][GRAPHIC_WIDTH_PX];

void scoping(const point_t* p_center,
                    unsigned int radius,
                    point_t* p_left_top,
                    point_t* p_right_bottom) {

    p_left_top->row = p_center->row-radius;
    p_left_top->col = p_center->col-radius;
    p_right_bottom->row = p_center->row+radius;
    p_right_bottom->col = p_center->col+radius;
    
    //縦方向の処理
    if(p_left_top->row < 0){   //上辺が上にはみ出している時
        if(p_right_bottom->row < 0){ //さらに下辺も上にはみ出し
            p_left_top->row=-1;
            p_left_top->col=-1;
            p_right_bottom->row=-1;
            p_right_bottom->col=-1;
        } else if(p_right_bottom->row > GRAPHIC_HEIGHT_PX-1) {     //下辺が下にはみ出しているとき
            p_left_top->row=0;
            p_right_bottom->row=GRAPHIC_HEIGHT_PX-1;
        } else {    //下辺が範囲内。上辺のみ処理が必要
            p_left_top->row=0;   
        }
    } else if (0 <= p_left_top->row && p_left_top->row <= GRAPHIC_HEIGHT_PX -1){  //上辺が範囲内
        if (GRAPHIC_HEIGHT_PX - 1 < p_right_bottom->row) {     //下辺が下にはみ出し
            p_right_bottom->row = GRAPHIC_HEIGHT_PX - 1;
        } else {}  //上辺、下辺がともに範囲内であれば上下の処理は必要なし。わかりやすさのためにからのelse分
    } else {    //上辺が下にはみ出している時
        p_left_top->row=-1;
        p_left_top->col=-1;
        p_right_bottom->row=-1;
        p_right_bottom->col=-1;
    }
    
    //横方向の処理
    if(p_left_top->col < 0){   //左辺が左にはみ出しているとき
        if(p_right_bottom->col < 0){   //右辺も左にはみ出しているとき
            p_left_top->row=-1;
            p_left_top->col=-1;
            p_right_bottom->row=-1;
            p_right_bottom->col=-1;
        } else if (p_right_bottom->col > GRAPHIC_WIDTH_PX-1){   //同時に右辺が右側にはみ出している時
            p_left_top->col = 0;
            p_right_bottom->col = GRAPHIC_WIDTH_PX-1;
        } else{     //残りのケース=左がはみ出していて右辺が範囲内の時。左のみ処理が必要
            p_left_top->col = 0;
        }
    } else if(0 <= p_left_top->col && p_left_top->col <= GRAPHIC_WIDTH_PX-1){    //左辺が範囲内
        if(GRAPHIC_WIDTH_PX-1 < p_right_bottom->col){       //左辺が範囲内で右辺が範囲右側にはみ出し。
            p_right_bottom->col = GRAPHIC_WIDTH_PX-1;
        } else {}   //左辺が範囲内で右辺が残りのケース=右辺は範囲内の時は処理は必要なし。
    } else {    //残りのケース=左辺が右側にはみ出している時。
        p_left_top->row=-1;
        p_left_top->col=-1;
        p_right_bottom->row=-1;
        p_right_bottom->col=-1;
    }
}

unsigned int dist_sq(int pt1_x, int pt1_y, int pt2_x, int pt2_y){
    int dx = pt2_x - pt1_x;
    int dy = pt2_y - pt1_y;
    return (unsigned int)(dx*dx+dy*dy);
}

void dominant_quadrant(const point_t* p_center,
                              const point_t* p_left_top,
                              const point_t* p_right_bottom,
                              point_t* quad,
                              point_t* start){

    int boarder_x = GRAPHIC_WIDTH_PX/2-1;
    int boarder_y = GRAPHIC_HEIGHT_PX/2-1;
    if(p_center->row <= boarder_y){   //the center placed in the upper
        if(p_center->col <= boarder_x){   //the center place in the left
            quad->row = 1;
            quad->col = 1;
            start->row = p_right_bottom->row;
            start->col = p_right_bottom->col;
        }else{  //the center placed in the right
            quad->row = 1;
            quad->col = -1;
            start->row = p_right_bottom->row;
            start->col = p_left_top->col;
        }
    }else{  //the center place in the lower
        if(p_center->col <= boarder_x){   //the center place in the left
            quad->row = -1;
            quad->col = 1;
            start->row = p_left_top->row;
            start->col = p_right_bottom->col;
        }else{  //the center place in the right
            quad->row = -1;
            quad->col = -1;
            start->row = p_left_top->row;
            start->col = p_left_top->col;            
        }
    }
}


void copy(const point_t* p_center,
                 const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int x,
                 int y,
                 uint8_t color){
    int vec_x = x - p_center->col;
    int vec_y = y - p_center->row;
    int temp_x, temp_y;

    //horizontal
    temp_x = p_center->col - vec_x;
    temp_y = y;
    if(is_in_scope(p_left_top, p_right_bottom, temp_x, temp_y)){
        pix[temp_y][temp_x] = color;
    }

    //vertical
    temp_x = x;
    temp_y = p_center->row - vec_y;
    if(is_in_scope(p_left_top, p_right_bottom, temp_x, temp_y)){
        pix[temp_y][temp_x] = color;
    }

    temp_x = p_center->col - vec_x;
    temp_y = p_center->row - vec_y;
    if(is_in_scope(p_left_top, p_right_bottom, temp_x, temp_y)){
        pix[temp_y][temp_x] = color;
    }
}

int is_in_scope(const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int x,
                 int y){
    if(p_left_top->col <= x &&
       x <= p_right_bottom->col &&
       p_left_top->row <= y &&
       y <= p_right_bottom->row){
        return true;
       } else {
        return false;
       }
}

int is_in_scope_x(const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int x){
    if(p_left_top->col <= x && x <= p_right_bottom->col){
        return true;
       } else {
        return false;
       }
}

int is_in_scope_y(const point_t* p_left_top,
                 const point_t* p_right_bottom,
                 int y){
    if(p_left_top->row <= y &&
       y <= p_right_bottom->row){
        return true;
       } else {
        return false;
       }
}

void hrizontal_scan(const point_t* p_center,
                    unsigned int radius,
                    const point_t* p_left_top,
                    const point_t* p_right_bottom,
                    const point_t* p_dominant_quad,
                    const point_t* p_start,
                    int y,
                    int16_t color_rim,
                    int16_t color_inside){

    unsigned int status=0;
    const unsigned int radius_sq = radius*radius;

    int x, temp_term;
    const int sign_x = p_dominant_quad->col;
    const int sign_y = p_dominant_quad->row;

    if(p_start->col == p_left_top->col){
        temp_term = p_right_bottom->col;
    } else {
        temp_term = p_left_top->col;
    }
    if(temp_term*sign_x < (p_center->col)*sign_x){
        temp_term = p_center->col;
    }
    const int terminal = temp_term;

    status &= ~FLAG_CHECK_COMMENCED;
    //外から中にスキャン
    //処理: フラグが0で範囲内に入ったら円周の色塗りしてフラグ立てる。
    //各色塗るごとにほかの象限にもコピーする。
    //       
    if(y==p_left_top->row ||  y==p_right_bottom->row){
            for(x=p_start->col;
            terminal*sign_x <= x*sign_x;
            x -=1*sign_x){
            //中か外か判定する。
            if(dist_sq(p_center->col, p_center->row, x, y) <= radius_sq){
                //commencedのフラグが立っていたらそれは境界
                if(status & FLAG_CHECK_COMMENCED){
                    if(0 <= color_rim){
                        pix[y][x] = color_rim;
                        copy(p_center, p_left_top, p_right_bottom, x, y, color_rim);
                    }
                    record[y][x] = FLAG_OPERATION_BOARDER;
                } else{ //commencedのフラグが立っていなかったら境界なのか内部なのかを判定する。
                    status |= FLAG_CHECK_COMMENCED;
                    int x_outer = x + 1*sign_x;   //一つ外側の座標
                    if(radius_sq < dist_sq(p_center->col, p_center->row, x_outer, y)){  //一つ外側はぎりぎり範囲外なので注目している点は境界
                        if(0 <= color_rim){
                            pix[y][x] = color_rim;
                            copy(p_center, p_left_top, p_right_bottom, x, y, color_rim);
                        }
                        record[y][x] = FLAG_OPERATION_BOARDER;
                    } else {
                        if(0 <= color_inside){
                            pix[y][x] = color_inside;
                            copy(p_center, p_left_top, p_right_bottom, x, y, color_inside);
                        }
                        record[y][x] = FLAG_OPERATION_FILL;
                    }
                }
                if(0 <= color_inside){
                    for(x -= 1*sign_x; terminal*sign_x <= x*sign_x; x -= 1*sign_x){
                        // あとは内部塗りつぶし
                        pix[y][x] = color_inside;
                        record[y][x] = FLAG_OPERATION_FILL;
                        copy(p_center, p_left_top, p_right_bottom, x, y, color_inside);
                    }
                } else {
                    for(x -= 1*sign_x; terminal*sign_x <= x*sign_x; x -= 1*sign_x){
                        // あとは内部塗りつぶし（透明だった場合）
                        record[y][x] = FLAG_OPERATION_FILL;
                    }
                }
            } else {
                record[y][x] = FLAG_OPERATION_OUT; 
                status |= FLAG_CHECK_COMMENCED;
            }
        }
    } else {
        for(x=p_start->col;
            terminal*sign_x <= x*sign_x;
            x -=1*sign_x){
            //中か外か判定する。
            if(dist_sq(p_center->col, p_center->row, x, y) <= radius_sq){
                //commencedのフラグが立っていたらそれは境界
                if(status & FLAG_CHECK_COMMENCED){
                    if(0 <= color_rim){
                        pix[y][x] = color_rim;
                        copy(p_center, p_left_top, p_right_bottom, x, y, color_rim);
                    }
                    record[y][x] = FLAG_OPERATION_BOARDER;
                } else{ //commencedのフラグが立っていなかったら境界なのか内部なのかを判定する。
                    status |= FLAG_CHECK_COMMENCED;
                    int x_outer = x + 1*sign_x;   //一つ外側の座標
                    if(radius_sq < dist_sq(p_center->col, p_center->row, x_outer, y)){  //一つ外側はぎりぎり範囲外なので注目している点は境界
                        if(0 <= color_rim){
                            pix[y][x] = color_rim;
                            copy(p_center, p_left_top, p_right_bottom, x, y, color_rim);
                        }
                        record[y][x] = FLAG_OPERATION_BOARDER;
                    } else {
                        if(0 <= color_inside){
                            pix[y][x] = color_inside;
                            copy(p_center, p_left_top, p_right_bottom, x, y, color_inside);
                        }
                        record[y][x] = FLAG_OPERATION_FILL;
                        if((record[y+1*sign_y][x] & FLAG_OPERATION_OUT) && 0 <= color_rim){
                            pix[y+1*sign_y][x] = color_rim;
                            copy(p_center, p_left_top, p_right_bottom, x, y+1*sign_y, color_rim);
                        }
                    }
                }
                if(0 <= color_inside){
                    for(x -= 1*sign_x; terminal*sign_x <= x*sign_x; x -= 1*sign_x){
                        // あとは内部塗りつぶし
                        pix[y][x] = color_inside;
                        record[y][x] = FLAG_OPERATION_FILL;
                        copy(p_center, p_left_top, p_right_bottom, x, y, color_inside);
                        if(record[y+1*sign_y][x] & FLAG_OPERATION_OUT && 0 <= color_rim){
                            pix[y+1*sign_y][x] = color_rim;
                            copy(p_center, p_left_top, p_right_bottom, x, y+1*sign_y, color_rim);
                        }
                    }
                } else {
                    for(x -= 1*sign_x; terminal*sign_x <= x*sign_x; x -= 1*sign_x){
                        // あとは内部塗りつぶし
//                        pix[y][x] = color_inside;
                        record[y][x] = FLAG_OPERATION_FILL;
//                        copy(p_center, p_left_top, p_right_bottom, x, y, color_inside);
                        if(record[y+1*sign_y][x] & FLAG_OPERATION_OUT && 0 <= color_rim){
                            pix[y+1*sign_y][x] = color_rim;
                            copy(p_center, p_left_top, p_right_bottom, x, y+1*sign_y, color_rim);
                        }
                    }
                }
            }else{
                record[y][x] = FLAG_OPERATION_OUT; 
                status |= FLAG_CHECK_COMMENCED;
            }
        }
    }

}