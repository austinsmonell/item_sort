// =====================================================================
// Network: WiFi + WebSocket + OTA setup, and the WS event handler.
// =====================================================================
// Same reasoning as the note in 04_homing.ino: the copy of this in
// config.h is invisible to Arduino's auto-prototype scanner, so it needs
// its own hand-written prototype here too, or the auto-generated one
// (which lands before WStype_t is declared) breaks the build.
static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len);

// A WebSocket frame is one command line, byte for byte what the serial
// monitor would take. Keeping one parser for both transports is the whole
// reason the protocol is line based.
static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {

    case WStype_CONNECTED: {
      wsClients++;
      IPAddress ip = ws.remoteIP(num);
      outf("# ws client %u connected from %s", num, ip.toString().c_str());
      emitStatus();                         // new client should not wait for the tick
      break;
    }

    case WStype_DISCONNECTED:
      if (wsClients) wsClients--;
      Serial.printf("# ws client %u disconnected\n", num);
      // Last one out while the machine is moving: the operator has lost
      // sight of it and can no longer press stop, so stop for them. The
      // heartbeat below is what makes this fire on a dead link and not
      // just on a clean browser close.
      if (wsClients == 0 && anyMoving()) {
        stopAll();
        outLine("# link lost with motion in progress - stopped");
      }
      break;

    case WStype_TEXT: {
      String line;
      line.reserve(len + 1);
      for (size_t i = 0; i < len; i++) line += (char)payload[i];
      execLine(line);
      break;
    }

    default: break;
  }
}

static void setupNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // modem sleep adds ~100 ms of jitter to jogs
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(OTA_HOST);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("# wifi connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  // Not fatal. USB serial still drives the machine, so come up either way
  // rather than sitting in a retry loop with the motors unattended.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("# wifi FAILED - serial control only");
    return;
  }

  wifiUp = true;
  Serial.printf("# wifi %s  ip %s  rssi %d\n",
                WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI());

  if (MDNS.begin(OTA_HOST)) {
    MDNS.addService("ws", "tcp", WS_PORT);
    Serial.printf("# mdns %s.local\n", OTA_HOST);
  }

  ws.begin();
  ws.onEvent(onWsEvent);
  // Ping every 2 s, expect a pong inside 1 s, give up after 2 misses. This
  // is what turns a yanked power lead or a dead AP into a DISCONNECTED
  // event in ~5 s instead of hanging a half-open socket indefinitely.
  ws.enableHeartbeat(2000, 1000, 2);
  Serial.printf("# websocket on ws://%s:%d\n", WiFi.localIP().toString().c_str(), WS_PORT);

  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() {
    // The board reboots at the end of this, so stop the motors while the
    // sketch still can: an axis left running would keep its last commanded
    // direction with nothing servicing the step line. E-stop is engaged to
    // block anything arriving during the transfer, not to persist - the
    // reboot clears it, and it clears isHomed with it. Re-home after OTA.
    // Drop the drives too, same as the E command: nothing should be
    // energised through a reflash.
    killDrives();
    estopActive = true;
    Serial.println("# OTA starting - motion stopped, drives OFF, re-home after reboot");
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("# OTA error %u\n", e); });
  ArduinoOTA.begin();
  Serial.printf("# OTA ready as '%s'\n", OTA_HOST);
}
