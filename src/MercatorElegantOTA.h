#ifndef _MercatorElegantOta_h_
#define _MercatorElegantOta_h_

#include <AsyncElegantOTA.h>
#include <queue>

class MercatorElegantOtaClass : public AsyncElegantOtaClass
{
    public:

        void begin(std::queue<std::string>* httpQueue, AsyncWebServer *server, const char* username = "", const char* password = "");

    private:
        std::queue<std::string>* _httpRequestQueue;

};
#endif