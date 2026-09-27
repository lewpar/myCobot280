// pump_test.ino
// Stand-alone bench test for the Elephant Robotics Suction Pump 2.0, driven from the
// ATOM (ESP32) in the myCobot 280 end effector.
//
// This is a TEST sketch, not a replacement for atom_led_matrix.ino: while it's flashed
// the ATOM does not listen on the servo bus, so ID 7 (the LED panel) stops answering.
// The servos are unaffected. Reflash atom_led_matrix.ino when you're done.
//
// ---- Background ------------------------------------------------------------------
// Elephant's docs for the 280 Pi wire the pump to the Pi's GPIO header in the base
// (BCM 21 = pump/solenoid, BCM 20 = air release, both active LOW). On the 280 M5 the
// box's G5 pin is the pump and G2 the release (set_basic_output(5, 0) = suck). The box
// takes 3.3 V logic, so the ATOM can drive it directly through its Grove port.
//
// ---- Wiring (one Grove / HY2.0-4P cable, pump box -> ATOM Grove port) -------------
// Both connectors are 4-pin Grove, labelled in the same order:
//   Pump box:  GND  5V  G2  G5
//   ATOM:      GND  5V  G26 G32
// so a straight cable gives:
//   G5 (pump: motor + solenoid, LOW = sucking) -> G32  (PUMP_PIN)
//   G2 (air-release valve,      LOW = venting) -> G26  (RELEASE_PIN)
//   5V / GND                                   -> the pump runs off the ATOM's 5V
// G26/G32 aren't used by atom_led_matrix.ino (bus G19/G22, LEDs G27).
// If "s" only clicks a valve instead of running the motor, swap the two defines;
// the "a"/"b" serial commands drive each pin on its own so you can tell which is which.
// The motor draws a lot more than the ESP32: if the ATOM browns out (resets, LEDs
// flicker) when the pump starts, power the pump from its own supply instead.
//
// ---- Controls --------------------------------------------------------------------
//   ATOM button (the screen, G39): toggle suck / release.
//   USB serial, 115200 baud, one letter + Enter:
//     s  suck      (pump on)
//     r  release   (pump off, then vent the cup for RELEASE_MS)
//     x  all off   (both pins HIGH, no vent)
//     a  toggle PUMP_PIN alone      (wiring check)
//     b  toggle RELEASE_PIN alone   (wiring check)
//     ?  print state
//   LEDs: green = sucking, blue = venting, dim red = idle.
//
// Safety: every output is driven HIGH (off) before anything else runs, and the pump
// switches itself off after PUMP_MAX_MS so a forgotten test can't run it forever.

#include <Adafruit_NeoPixel.h>

// ---- Pin config ------------------------------------------------------------------
#define PUMP_PIN     32     // Grove pin 4, pump box G5
#define RELEASE_PIN  26     // Grove pin 3, pump box G2
#define BUTTON_PIN   39     // ATOM Matrix screen button, active LOW, external pull-up
#define LED_PIN      27
#define BUS_RX       19     // servo bus: left as inputs so this sketch never drives it
#define BUS_TX       22

// ---- Constants -------------------------------------------------------------------
#define NUM_LEDS       25
#define ACTIVE         LOW   // the pump box's inputs are active LOW
#define IDLE           HIGH
#define RELEASE_GAP_MS 50    // pump off -> vent open (Elephant's example timing)
#define RELEASE_MS     1000  // how long the vent stays open
#define PUMP_MAX_MS    30000 // auto-off
#define DEBOUNCE_MS    40

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

enum State { ST_IDLE, ST_SUCK, ST_GAP, ST_VENT, ST_MANUAL };
State    state      = ST_IDLE;
uint32_t state_ms   = 0;      // millis() when the state was entered
bool     pump_on    = false;  // current pin levels, for the manual commands
bool     release_on = false;

// ---- Outputs ---------------------------------------------------------------------

void set_pins(bool pump, bool release) {
    pump_on    = pump;
    release_on = release;
    digitalWrite(PUMP_PIN,    pump    ? ACTIVE : IDLE);
    digitalWrite(RELEASE_PIN, release ? ACTIVE : IDLE);
}

void show(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
    strip.show();
}

const char* state_name() {
    switch (state) {
        case ST_IDLE:   return "idle";
        case ST_SUCK:   return "sucking";
        case ST_GAP:    return "releasing (pump off)";
        case ST_VENT:   return "releasing (venting)";
        case ST_MANUAL: return "manual";
    }
    return "?";
}

void print_state() {
    Serial.printf("[pump] %s  (G%d=%s, G%d=%s)\n", state_name(),
                  PUMP_PIN, pump_on ? "LOW" : "HIGH",
                  RELEASE_PIN, release_on ? "LOW" : "HIGH");
}

void enter(State s) {
    state    = s;
    state_ms = millis();
    switch (s) {
        case ST_IDLE: set_pins(false, false); show(12, 0, 0);  break;
        case ST_SUCK: set_pins(true,  false); show(0, 60, 0);  break;
        case ST_GAP:  set_pins(false, false); show(0, 0, 20);  break;
        case ST_VENT: set_pins(false, true);  show(0, 0, 60);  break;
        case ST_MANUAL: break;  // pins set by the caller
    }
    print_state();
}

// ---- Actions ---------------------------------------------------------------------

void suck()    { enter(ST_SUCK); }
void release() { if (state == ST_IDLE) return; enter(ST_GAP); }

void manual(bool pump, bool rel) {
    set_pins(pump, rel);
    show(pump ? 60 : 0, rel ? 60 : 0, 0);   // red = pump pin, green = release pin
    enter(pump || rel ? ST_MANUAL : ST_IDLE);
}

// ---- Setup / loop ----------------------------------------------------------------

void setup() {
    // Outputs off before anything else: an ESP32 pin floats at boot.
    digitalWrite(PUMP_PIN, IDLE);
    digitalWrite(RELEASE_PIN, IDLE);
    pinMode(PUMP_PIN, OUTPUT);
    pinMode(RELEASE_PIN, OUTPUT);
    set_pins(false, false);

    pinMode(BUS_RX, INPUT);
    pinMode(BUS_TX, INPUT);
    pinMode(BUTTON_PIN, INPUT);

    Serial.begin(115200);
    strip.begin();
    strip.setBrightness(40);
    enter(ST_IDLE);
    Serial.println("[pump] ready: s=suck r=release x=off a/b=toggle one pin ?=state");
}

void loop() {
    uint32_t now = millis();

    // Timed transitions
    if (state == ST_SUCK && now - state_ms >= PUMP_MAX_MS) {
        Serial.println("[pump] auto-off after PUMP_MAX_MS");
        release();
    } else if (state == ST_GAP && now - state_ms >= RELEASE_GAP_MS) {
        enter(ST_VENT);
    } else if (state == ST_VENT && now - state_ms >= RELEASE_MS) {
        enter(ST_IDLE);
    } else if (state == ST_MANUAL && pump_on && now - state_ms >= PUMP_MAX_MS) {
        Serial.println("[pump] auto-off after PUMP_MAX_MS");
        enter(ST_IDLE);
    }

    // Button: toggle on the press edge
    static bool     last_btn = HIGH;
    static uint32_t btn_ms   = 0;
    bool btn = digitalRead(BUTTON_PIN);
    if (btn != last_btn && now - btn_ms >= DEBOUNCE_MS) {
        btn_ms   = now;
        last_btn = btn;
        if (btn == LOW) {
            if (state == ST_SUCK) release();
            else if (state == ST_IDLE || state == ST_MANUAL) suck();
        }
    }

    // Serial commands
    while (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 's': case 'S': suck(); break;
            case 'r': case 'R': release(); break;
            case 'x': case 'X': enter(ST_IDLE); break;
            case 'a': case 'A': manual(!pump_on, release_on); break;
            case 'b': case 'B': manual(pump_on, !release_on); break;
            case '?': print_state(); break;
            default: break;  // ignore newlines etc.
        }
    }
}
