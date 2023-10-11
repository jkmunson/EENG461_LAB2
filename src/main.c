#include "main.h"
#include "setup.h"
#include "timers.h"
#include <stdint.h>
#include <common/tm4c123gh6pm.h>

int main (void) {

    setup();
    configureTimer();

    Enable_Interrupts(); //Enable Global Interrupts

    while (1) {
	
        switch (sec_count) {
            case 0:
                /*
                 * Time = 0 starts with blue led turning on
                 */
                GPIO_PORTF_DATA_BITS_R[BLUE_LED] = BLUE_LED;
                break;
            case 1:
                /*
                 * After 1 second of blue LED being on, turn on the red LED (leaving blue LED on)
                 */
                GPIO_PORTF_DATA_BITS_R[RED_LED] = RED_LED;
                break;
            case 3:
                /*
                 * After 2 seconds of red and blue LEDs being on, turn both of them off
                 */
                GPIO_PORTF_DATA_BITS_R[RED_LED | BLUE_LED] =  0;
                break;
        }
    }

    return (0);
}

/*
 * Taken from Lab Assignment
 */
void Disable_Interrupts(void) {
    __asm ("  CPSID    I\n");
}

void Enable_Interrupts(void) {
    __asm ("  CPSIE    I\n");
}
