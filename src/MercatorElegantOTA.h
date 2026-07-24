#ifndef _MercatorElegantOta_h_
#define _MercatorElegantOta_h_

#include <AsyncElegantOTA.h>
#include "HttpRequestQueue.h"

class MercatorElegantOtaClass : public AsyncElegantOtaClass
{
    public:

        void begin(HttpRequestQueue* httpQueue, AsyncWebServer *server, const char* username = "", const char* password = "");

    private:
        HttpRequestQueue* _httpRequestQueue;

};
#endif