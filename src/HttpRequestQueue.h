#ifndef _HttpRequestQueue_h_
#define _HttpRequestQueue_h_

#include <Arduino.h>
#include <queue>
#include <string>

// FIFO of command strings from the on-screen web buttons (/test POST handler
// and the other MercatorElegantOTA routes) to the Arduino loop() task.
// push() is called from the AsyncWebServer "async_tcp" task; pop() is called
// from loop(). A plain std::queue is not thread-safe across those two tasks,
// so access is serialised with a spinlock.
class HttpRequestQueue
{
    public:
        void push(const std::string& command)
        {
            portENTER_CRITICAL(&_mux);
            _queue.push(command);
            portEXIT_CRITICAL(&_mux);
        }

        // Pops the oldest command into 'command'. Returns false if the queue was empty.
        bool pop(std::string& command)
        {
            bool haveItem = false;
            portENTER_CRITICAL(&_mux);
            if (!_queue.empty())
            {
                command = _queue.front();
                _queue.pop();
                haveItem = true;
            }
            portEXIT_CRITICAL(&_mux);
            return haveItem;
        }

    private:
        std::queue<std::string> _queue;
        portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
};

#endif
