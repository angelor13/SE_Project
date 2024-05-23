#include "server.h"
#include "vec2.hpp"

// Function to handle the root path ("/")
void handleRoot() {
    server.send(200, "text/html", HTML);
}

// Function to handle the path ("/style.css")
void handleCSS() {
    server.send(200, "text/css", CSS);
}

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

void taskServer() {
    Serial.begin(115200);

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
    }
}

