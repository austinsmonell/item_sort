/* =====================================================================
   esp32_4axis_serial.ino
   ---------------------------------------------------------------------
   ESP32-WROOM-32U  ->  3x HB808C closed-loop + 1x open-loop stepper
   USB serial control plus WiFi, and wireless reflash over OTA.

   Drive it any of three ways - all speak the same line protocol:
     - Arduino Serial Monitor at 115200, line ending "Newline". Type HELP.
     - stepper_gui.html over Web Serial (USB cable).
     - stepper_gui.html over WiFi, to ws://<board-ip>:81.

   Output is broadcast to every connected transport, so the serial monitor
   and the browser always see the same thing.

   WIFI IS NOT AN E-STOP PATH. A dropped link cannot stop a moving machine
   any faster than its timeout, so the board watches for that itself: if the
   last WebSocket client goes away while an axis is moving, motion stops.
   That is a backstop, not a safety device. Keep a hardware e-stop in the
   motor supply.

   THE -U MODULE HAS NO ANTENNA ON IT. WROOM-32U brings the RF out to a
   U.FL connector instead of a PCB trace. Without a pigtail antenna fitted
   it will associate from a few inches away and nowhere else.

   AXIS PAIRING:
     Axes 1 and 2 are two motors on opposite ends of one rail, so nothing
     may command one without the other. Jog, move-to, zero and home all go
     out to both ends; the motion config keys mirror across the pair; and a
     fault on one end stops the other. Homing drives both ends into their
     own hard stop, which is what squares the rail.

     Check that the two ends move the same way before homing. If they
     fight, flip one with "C 2 inv 1" rather than rewiring.

   WIRING ASSUMPTION (rev 5 - common cathode, no buffer chip):
     - all PUL- / DIR- tied to ground, shared with the ESP32
     - GPIO drives the plus lines directly -> LOGIC IS NORMAL
       GPIO HIGH = optocoupler conducting = signal asserted
     - Encoders EA+/EB+ tapped through a TXS0108E (5 V -> 3V3)
     - ALM not wired

   WHY THIS IS THE RIGHT WAY ROUND. The previous rev tied the plus lines
   to 5 V and sank the minus lines. A 3.3 V GPIO high then left 1.7 V
   across the opto - above the LED forward voltage, so a couple of mA kept
   flowing and a de-asserted input never actually cleared. Edge-triggered
   PUL survived that; a level-sensitive DIR did not, which is what pinned
   an axis to one direction. Common cathode gives a true 0 V off state.

   The cost is drive current: 3.3 V into an input resistor sized for 5 V
   is roughly 8 mA where the driver expected 14 mA. That is inside spec
   for the usual opto inputs. If one driver is marginal, lower its series
   resistor - do not go back to a 5 V anode.

   PULSE POLARITY NOTE:
     FastAccelStepper idles its step pin LOW and pulses HIGH, which is now
     exactly what the hardware wants: dark at idle, conducting during the
     pulse. Nothing to flip in the vendor software any more, and the input
     LEDs no longer run warm doing nothing.

   LIBRARIES (Library Manager):
     - FastAccelStepper   by gin66
     - ESP32Encoder       by Kevin Harrington
     - WebSockets         by Markus Sattler   ("arduinoWebSockets")

   WIFI SETUP: fill in WIFI_SSID / WIFI_PASS in 01_config.ino. The first
   flash has to go over USB; after that the board appears in the Arduino
   IDE under Tools > Port as a network port and OTA_PASS unlocks it.

   FLASH WITH THE MOTOR SUPPLY OFF. Every upload resets the board and
   leaves the unbuffered PUL lines floating, which can emit stray steps.

   ---------------------------------------------------------------------
   FILE LAYOUT. Arduino concatenates every .ino tab in this folder into
   one translation unit - this file first, then the rest alphabetically -
   so plain top-to-bottom C++ ordering rules apply across tabs. That is
   why the numeric prefixes exist and must stay in this order:

     config.h             pin map, per-axis config, all shared globals,
                          HomeState enum. A real header, included below,
                          not a .ino tab - see the note inside it for why
                          any type used in a function signature has to
                          live here rather than in a tab.
     02_output.ino        serial/WS output helpers, the JSON status line
     03_drives.ino        ENA/direction/speed, enable/disable, e-stop
                          fault handling, idle timeout
     04_homing.ino        homing state machine, stall + lost-sync watch
     05_motion.ino        soft limits, jog/move/zero command paths
     06_persistence.ino   NVS load/save of config and machine settings
     07_commands.ino      serial/WS line parser, HELP text
     08_network.ino       WiFi/WebSocket/OTA setup and the WS event handler
     09_main.ino          setup() / loop() - kept last on purpose so every
                          type and function above is already visible
   ===================================================================== */

#include <FastAccelStepper.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include "config.h"
