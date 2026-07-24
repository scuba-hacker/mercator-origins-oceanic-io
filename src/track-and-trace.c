const char track_and_trace_html_content[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Directional Buttons Control</title>
    <style>
        .container {
            text-align: center;
            margin-top: 25px;
            margin-bottom: 25px;
        }
        .button {
            display: inline-block;
            padding: 10px 20px;
            margin: 10px;
            font-size: 18px;
            cursor: pointer;
            border: none;
            border-radius: 5px;
            transition: background-color 0.3s;
        }
        .button-active {
            background-color: #2980b9 !important;
            color: #fff;
        }
        .button-red {
            background-color: #e74c3c;
            color: #fff;
        }
        .button-orange {
            background-color: #e67e22;
            color: #fff;
        }
        .button-green {
            background-color: #2ecc71;
            color: #fff;
        }
        .button-blue {
            background-color: #3498db;
            color: #fff;
        }
        .dropdown {
            display: inline-block;
            padding: 10px 20px;
            margin: 10px;
            font-size: 18px;
            border-radius: 5px;
        }
    </style>
</head>
<body>

<div class="container">
    <button class="button button-green" id="allButton">All</button>
    <button class="button button-green" id="x1Button">x1</button>
    <button class="button button-green" id="x2Button">x2</button>
    <button class="button button-green" id="x3Button">x3</button>
    <button class="button button-green" id="x4Button">x4</button>
</div>

<div class="container">
    <label for="deltaScaleSlider">Step: <span id="deltaScaleValue">1</span></label>
    <input type="range" id="deltaScaleSlider" min="1" max="5" step="1" value="1">
</div>

<div class="container">
    <button class="button button-blue" id="upButton">&#8593;</button>
</div>

<div class="container" style="margin-top: -10px; margin-bottom: -10px;">
    <button class="button button-grey" id="slowerButton">-</button>
    <button class="button button-blue" id="leftButton">&#8592;</button>
    <button class="button button-blue" id="rightButton">&#8594;</button>
    <button class="button button-grey" id="fasterButton">+</button>
</div>

<div class="container">
    <button class="button button-blue" id="downButton">&#8595;</button>
</div>

<div class="container">
    <select class="dropdown" id="zWaypointSelect">
        <option value="">Trace start: Z waypoint...</option>
    </select>
</div>

<div class="container">
    <button class="button" id="trackButton">Track</button>
    <button class="button" id="traceButton">Trace</button>
    <button class="button" id="updateButton">Update</button>
    <button class="button button-green" id="startSerialButton">Start Serial</button>
    <button class="button button-green" id="stopSerialButton">Stop Serial</button>
</div>

<div class="container">
    <button class="button button-red" id="stopButton">Stop</button>
    <button class="button button-orange" id="resetButton">Reset</button>
</div>
<div class="container">
    <button class="button button-red" id="rebootButton">Reboot</button>
</div>
<iframe src="/webserial" width="1000" height="1000" frameborder="0"></iframe>

<script>
    // Function to handle button activation
    function activateButton(buttonId) {
        var button = document.getElementById(buttonId);
        if (!button) return;
        button.classList.add("button-active");
        setTimeout(function() {
            button.classList.remove("button-active");
        }, 100);
    }
    // Function to send a POST request
    function sendPostRequest(url, buttonId) {
        activateButton(buttonId);
        // Drop focus from whatever control triggered this so a later keypress
        // (space, arrow keys) is never captured or re-actioned by that control.
        if (document.activeElement && document.activeElement.blur) {
            document.activeElement.blur();
        }
        fetch(url, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/x-www-form-urlencoded'
            },
            body: 'button=' + encodeURIComponent(buttonId)
        });
    }

    // Function to handle button clicks
    function handleButtonClick(buttonId) {
        sendPostRequest(window.location.href, buttonId);
    }

    // Attach event listeners to each button
    document.getElementById("allButton").addEventListener("click", function() {
        handleButtonClick("allButton");
    });

    document.getElementById("x1Button").addEventListener("click", function() {
        handleButtonClick("x1Button");
    });

    document.getElementById("x2Button").addEventListener("click", function() {
        handleButtonClick("x2Button");
    });

    document.getElementById("x3Button").addEventListener("click", function() {
        handleButtonClick("x3Button");
    });

    document.getElementById("x4Button").addEventListener("click", function() {
        handleButtonClick("x4Button");
    });

    document.getElementById("upButton").addEventListener("click", function() {
        handleButtonClick("upButton");
    });

    document.getElementById("leftButton").addEventListener("click", function() {
        handleButtonClick("leftButton");
    });

    document.getElementById("rightButton").addEventListener("click", function() {
        handleButtonClick("rightButton");
    });

    document.getElementById("downButton").addEventListener("click", function() {
        handleButtonClick("downButton");
    });

    document.getElementById("slowerButton").addEventListener("click", function() {
        handleButtonClick("slowerButton");
    });
    
    document.getElementById("fasterButton").addEventListener("click", function() {
        handleButtonClick("fasterButton");
    });
    
    document.getElementById("trackButton").addEventListener("click", function() {
        handleButtonClick("trackButton");
    });

    // Trace uses the selected Z waypoint as the start position, if one is selected
    // (ie. the dropdown is not still on its placeholder option).
    function triggerTrace() {
        var select = document.getElementById("zWaypointSelect");
        if (select && select.value !== "") {
            handleButtonClick("zwp:" + select.value);
        } else {
            handleButtonClick("traceButton");
        }
    }

    document.getElementById("traceButton").addEventListener("click", function() {
        triggerTrace();
    });

    document.getElementById("stopButton").addEventListener("click", function() {
        handleButtonClick("stopButton");
    });

    document.getElementById("resetButton").addEventListener("click", function() {
        handleButtonClick("resetButton");
    });
    document.getElementById("startSerialButton").addEventListener("click", function() {
        handleButtonClick("startSerialButton");
    });
    document.getElementById("stopSerialButton").addEventListener("click", function() {
        handleButtonClick("stopSerialButton");
    });
    document.getElementById("rebootButton").addEventListener("click", function() {
        handleButtonClick("rebootButton");
    });

    // Populate the Z waypoint dropdown and wire up selection to set the trace start position
    function loadZWaypoints() {
        fetch('/zwaypoints')
            .then(response => response.json())
            .then(waypoints => {
                var select = document.getElementById("zWaypointSelect");
                waypoints.forEach(function(wp) {
                    var option = document.createElement("option");
                    option.value = wp.i;
                    option.textContent = wp.l;
                    select.appendChild(option);
                });
            })
            .catch(error => console.error('Error loading Z waypoints:', error));
    }

    document.getElementById("zWaypointSelect").addEventListener("change", function() {
        if (this.value !== "") {
            handleButtonClick("zwp:" + this.value);
        }
    });

    loadZWaypoints();

    // Delta scale factor - controls the step size taken on each trace increment
    var deltaScaleSlider = document.getElementById("deltaScaleSlider");
    var deltaScaleValue = document.getElementById("deltaScaleValue");

    deltaScaleSlider.addEventListener("input", function() {
        // Live-update the label while dragging, without spamming the server.
        deltaScaleValue.textContent = this.value;
    });

    deltaScaleSlider.addEventListener("change", function() {
        // Fires once the value is committed (mouse released / arrow key press).
        handleButtonClick("dsf:" + this.value);
    });

    // Function to handle update button click
    function handleUpdateButtonClick() {
        window.location.href = "/update";
    }

    // Attach event listener to the update button
    document.getElementById("updateButton").addEventListener("click", handleUpdateButtonClick);

    // Function to send a GET request
    function sendGetRequest(url) {
        fetch(url, {
            method: 'GET',
            headers: {
                'Content-Type': 'text/html'
            }
        })
        .then(response => response.text())
        .then(data => document.documentElement.innerHTML = data)
        .catch(error => console.error('Error:', error));
    }

    // Function to handle keydown events
    function handleKeyDown(event) {
        var handled = true;
        switch(event.key) {
            case 'ArrowUp':
                handleButtonClick("upButton");
                break;
            case 'ArrowLeft':
                handleButtonClick("leftButton");
                break;
            case 'ArrowRight':
                handleButtonClick("rightButton");
                break;
            case 'ArrowDown':
                handleButtonClick("downButton");
                break;
            case '-':
                handleButtonClick("slowerButton");
                break;
            case '=':
                handleButtonClick("fasterButton");
                break;
            case 'r':
                handleButtonClick("resetButton");
                break;
            case 's':
                handleButtonClick("stopButton");
                break;
            case 'k':
                handleButtonClick("trackButton");
                break;
            case 'c':
                triggerTrace();
                break;
            case 'u':
                handleUpdateButtonClick();
                break;
            case '0':
                handleButtonClick("allButton");
                break;
            case '1':
                handleButtonClick("x1Button");
                break;
            case '2':
                handleButtonClick("x2Button");
                break;
            case '3':
                handleButtonClick("x3Button");
                break;
            case '4':
                handleButtonClick("x4Button");
                break;
            default:
                handled = false;
        }
        // Stop the browser acting on these keys itself (e.g. arrow keys /
        // space scrolling the page, or re-triggering a focused control).
        if (handled) {
            event.preventDefault();
        }
    }

    // Attach keydown event listener to document
    document.addEventListener("keydown", handleKeyDown);
</script>

</body>
</html>
)rawliteral";
