// https://github.com/Sylaina/oled-display

#include <oled.h>

int main(void)
{
    oled_init(OLED_DISP_ON);
    oled_clrscr();

    oled_charMode(DOUBLESIZE);
    oled_puts("OLED disp.");

    oled_charMode(NORMALSIZE);

    // oled_gotoxy(x, y)
    oled_gotoxy(0, 2);
    oled_puts("128x64, SHH1106");

    // oled_drawLine(x1, y1, x2, y2, color)
    oled_drawLine(0, 25, 120, 64, WHITE);

    oled_gotoxy(0, 4);
    oled_puts("BPC-DE2, Brno");


    //oled_drawCircle(0,5,2,1);
  
    oled_drawCircle(64,64,7,WHITE); // draw circle to buffer
    // Copy buffer to display RAM
    oled_display();

    

    while (1) {
        ;
    }

    return 0;
}