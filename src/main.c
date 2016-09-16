#include "ch.h"
#include "hal.h"
#include "chprintf.h"

int main(void)
{
  halInit();
  chSysInit();

  sdStart(&SD2, NULL);
  chprintf((BaseSequentialStream *)&SD2, "Yep, hi there!!\r\n");

  while (true)
  {
    if (palReadPad(GPIOA, GPIOA_BUTTON))
      palSetPad(GPIOA, GPIOA_LED);
    else
      palClearPad(GPIOA, GPIOA_LED);
  }
}
