#include <MercatorElegantOTA.h>

extern char track_and_trace_html_content[];

void MercatorElegantOtaClass::begin(std::queue<std::string>* httpQueue, AsyncWebServer *server, const char* username, const char* password)
{
    AsyncElegantOtaClass::begin(server, username, password);

    _httpRequestQueue = httpQueue;

    _server->on("/boot", HTTP_GET, [&](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Rebooting...");
        ESP.restart();
    });

    _server->on("/track", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("track"));
        request->send(200, "text/plain", "Switch to dive track test");
    });

    _server->on("/trace", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("trace"));
        request->send(200, "text/plain", "Switch to trace test");
    });

    _server->on("/up", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("u"));
        request->send(200, "text/plain", "Up");
    });

    _server->on("/do", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("d"));
        request->send(200, "text/plain", "Down");
    });

    _server->on("/le", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("l"));
        request->send(200, "text/plain", "Left");
    });

    _server->on("/ri", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("r"));
        request->send(200, "text/plain", "Right");
    });

    _server->on("/stop", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("stop"));
        request->send(200, "text/plain", "stop");
    });

    _server->on("/reset", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("reset"));
        request->send(200, "text/plain", "reset");
    });

    _server->on("/dim", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("dim"));
        request->send(200, "text/plain", "Dim screen");
    });

    _server->on("/night", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("night"));
        request->send(200, "text/plain", "Night screen");
    });

    _server->on("/bright", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("bright"));
        request->send(200, "text/plain", "Bright screen");
    });

    _server->on("/sleep", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("sleep"));
        request->send(200, "text/plain", "Sleep System");
    });

    _server->on("/test", HTTP_GET, [&](AsyncWebServerRequest *request){
        _httpRequestQueue->push(std::string("test"));
        request->send(200, "text/html", track_and_trace_html_content);
    });

    _server->on("/test", HTTP_POST, [&](AsyncWebServerRequest *request){
            AsyncWebParameter* p = request->getParam("button",true,false);
            if (p)
            {
                _httpRequestQueue->push(p->value().c_str());
                request->send(200, "text/html", "ok");
            }
            else
            {
            request->send(200, "text/plain", "invalid");
            }
    });

}