#include <WiFi.h>
#include <WebServer.h>
#include "vec2.hpp"

char CSS[] = R"(
    body {
        font-family: Arial, sans-serif;
        display: flex;
        flex-direction: column;
        align-items: center;
        margin-top: 50px;
    }
    .controls {
        display: flex;
        flex-direction: column;
        align-items: center;
        margin-top: 20px;
    }
    .controls button {
        margin: 5px;
        padding: 10px 20px;
        font-size: 16px;
    }
    .coordinate-display, .mode-selection, .start-stop-buttons {
        margin: 20px;
    }
)";
char HTML[] = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Control Interface</title>
</head>
<body>
    <link rel="stylesheet" type="text/css" href="style.css"/>

    <div class="coordinate-display">
        <p>X: <span id="x-coordinate">0</span></p>
        <p>Y: <span id="y-coordinate">0</span></p>
    </div>

    <div class="start-stop-buttons">
        <button id="start-button">Start</button>
        <button id="stop-button">Stop</button>
    </div>

    <div class="mode-selection">
        <label>
            <input type="radio" name="mode" value="manual" checked> Manual
        </label>
        <label>
            <input type="radio" name="mode" value="automatic"> Automatic
        </label>
    </div>

    <div class="controls">
        <button id="up-button">Forward</button>
        <div>
            <button id="left-button">Left</button>
            <button id="right-button">Right</button>
        </div>
        <button id="down-button">Back</button>
    </div>

    <script>     
        let n = 0;
        let e = 0;
        let s = 0;
        let w = 0;
        let interval_id = null;

        document.getElementById('start-button').addEventListener('click', () => {
            console.log('Start button clicked');
        });

        document.getElementById('stop-button').addEventListener('click', () => {
            console.log('Stop button clicked');
        });

        document.querySelectorAll('input[name="mode"]').forEach((elem) => {
            elem.addEventListener('change', (event) => {
                let value = event.target.value;

                if (value === "manual") {
                    fetch(`/mode?m=1`)
                    .then(response => response.text())
                    .then(data => {
                        console.log('Response:', data);
                    })
                    .catch(error => console.error('Error:', error));

                    interval_id = setInterval(async() => {
                        fetch(`/control?n=${n}&e=${e}&s=${s}&w=${w}`)
                        .then(response => response.text())
                        .then(data => {
                            console.log('Response:', data);
                        })
                        .catch(error => console.error('Error:', error));
                    }, 500);
                } else {
                    fetch(`/mode?m=0`)
                    .then(response => response.text())
                    .then(data => {
                        console.log('Response:', data);
                    })
                    .catch(error => console.error('Error:', error));

                    clearInterval(interval_id);
                }
            });
        });

        document.getElementById('up-button').addEventListener('mousedown', () => {
            n = 1;
        });

        document.getElementById('up-button').addEventListener('mouseup', () => {
            n = 0;
        });

        document.getElementById('down-button').addEventListener('mousedown', () => {
            s = 1;
        });

        document.getElementById('down-button').addEventListener('mouseup', () => {
            s = 0;
        });

        document.getElementById('left-button').addEventListener('mousedown', () => {
            w = 1;
        });

        document.getElementById('left-button').addEventListener('mouseup', () => {
            w = 0;
        });

        document.getElementById('right-button').addEventListener('mousedown', () => {
            e = 1;
        });

        document.getElementById('right-button').addEventListener('mouseup', () => {
            e = 0;
        });

    </script>
</body>
</html>
)";

// Replace with your network credentials
const char* ssid = "jesus";
const char* password = "opeixedotobias1";

// Create an instance of the web server
WebServer server(80); 

// Function to handle the root path ("/")
void handleRoot() {
    server.send(200, "text/html", HTML);
}

// Function to handle the path ("/style.css")
void handleCSS() {
    server.send(200, "text/css", CSS);
}

extern void handleSwitchMode();

void handleCommand() {
    String north = server.hasArg("n") ? server.arg("n") : "0";
    String east = server.hasArg("e") ? server.arg("e") : "0";
    String south = server.hasArg("s") ? server.arg("s") : "0";
    String west = server.hasArg("w") ? server.arg("w") : "0";

    vec2 move = vec2(east.toInt() - west.toInt(), north.toInt() - south.toInt());

    Serial.printf("%f, %f\n", move.x, move.y);
    server.send(200, "text/plain", "OK");

    // do move shit

}

// Function to handle 404 (Not Found) errors
void handleNotFound() {
    server.send(404, "text/plain", "404: Not found");
}

void taskServerHandle(void) {
    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println(" connected");

    // Start the web server
    server.on("/", handleRoot);
    server.on("/style.css", handleCSS);
    server.on("/control", handleCommand);
    server.on("/mode", handleSwitchMode);
    server.onNotFound(handleNotFound);

    server.begin();
    server.enableCrossOrigin();
    Serial.println("HTTP server started");

    // Print the IP address
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    while (1) {
        server.handleClient();
        delay(100);
    }
}