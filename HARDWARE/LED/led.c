#include "led.h"

void Led_Flash(uint16_t time)
{
	/* PA12 is now USB D+; keep the legacy API as a no-op. */
	(void)time;
}
