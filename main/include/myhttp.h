#ifndef MYHTTP_H
#define MYHTTP_H

#include "floorlamp.h"

httpd_handle_t start_webserver(void);

void stop_webserver(httpd_handle_t server);

#endif
