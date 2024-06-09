#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>


struct gpiod_chip  *gpiochip;
struct gpiod_line  *gpioline;;

#define GETPIN gpiod_line_get_value(gpioline)
#define SETPIN(V) gpiod_line_set_value(gpioline,V)
#define CHECKPIN(V) printf(GETPIN == V ? " OK    " : " ERROR ")

int main(void)
{
  bool flag;

  gpiochip = gpiod_chip_open_by_name("gpiochip4");
  if(gpiochip == NULL)
      gpiochip = gpiod_chip_open_by_name("gpiochip0");

  if(gpiochip == NULL)
   {
     printf("Unable to access GPIO\n");
     return -1;
   }

  for(int gpioPin=0;gpioPin<28;gpioPin++)
  {

   printf("GPIO %02d  ",gpioPin);

   gpioline= gpiod_chip_get_line(gpiochip,gpioPin);

   if(gpioline != NULL)
   {
       if(gpiod_line_request_output(gpioline,"Test",0)!=0)
         {
           gpiod_line_release(gpioline);
           printf("Unable to  set Output! Maybe in Used\n");
           continue;
         }
       printf("OUT 0:");
       // test OUTPUT 0
       SETPIN(1);
       SETPIN(0);
       CHECKPIN(0);
       // test OUTPUT 1
       printf(" 1:");
       SETPIN(1);
       CHECKPIN(1);
       // test  IN pull down
       printf("   IN PULL DOWN:");
       gpiod_line_release(gpioline);
       gpiod_line_request_input_flags(gpioline,"Test",GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN);
       CHECKPIN(0);
       // test  IN pull UP
       printf("   IN PULL UP:");
       gpiod_line_release(gpioline);
       gpiod_line_request_input_flags(gpioline,"Test",GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
       CHECKPIN(1);
       printf("\n");
       gpiod_line_release(gpioline);
    }
    else
       printf("In used or error\n");
  }
 gpiod_chip_close(gpiochip);
 return 0;
}