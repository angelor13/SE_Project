#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>

int xprintf(const char* fmt, ...) {
    uint8_t rc = USBD_OK;
    char tmp[128];
	va_list ptr;

	va_start(ptr, fmt);
	int len = vsprintf(tmp, fmt, ptr);
    va_end(ptr);

    do {
        rc = CDC_Transmit_FS((uint8_t*)tmp, len);
    } while (USBD_BUSY == rc);

    if (USBD_FAIL == rc) {
        return 0;
    }

    return len;
}
