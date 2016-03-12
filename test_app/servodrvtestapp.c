#include <stdio.h>
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
    
    servodrv_write(drv, "this is a test", 14);
    
    printf("write successful\n");
    
    servodrv_close(drv);
    
    printf("test app successfully completed!\n");
    return 0;
}