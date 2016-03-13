#include <stdio.h>
#include <stdint.h>
#include "libservodrv.h"
 
int main ( int argc, char **argv )
{
    printf("servodrv test app\n");
    int drv = servodrv_open();
    if(drv > -1){
        printf("servodrv opened successfully\n");
    }
    else{
        printf("failed to open servodrv!\n");
        return 1;
    }
    
    uint16_t buf[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 ,17 ,18};
    
    servodrv_write(drv, buf, sizeof(buf));
    
    printf("write successful\n");
    
    servodrv_close(drv);
    
    printf("test app successfully completed!\n");
    return 0;
}