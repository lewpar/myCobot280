// atom_led_matrix.ino
// Firmware for the ATOM ESP32 at the end of the myCobot280 servo bus.
//
// Listens on the half-duplex UART bus for Feetech-protocol frames (0xFF 0xFF)
// addressed to ID 7. Servos use IDs 1-6 so they ignore ID 7 traffic.
//
// Wiring:
//   LED data pin -> GPIO 27
//   Bus RX/TX    -> See BUS_RX / BUS_TX defines below
//
// Feetech frame format (command to us):
//   FF FF 07 <LEN> 03 <ADDR> <DATA...> <CHK>
//
// Feetech status packet format (response from us):
//   FF FF 07 02 00 <CHK>                    (ping / ack with no data)
//   FF FF 07 <2+N> 00 <DATA...> <CHK>        (response with N data bytes)

#include <Adafruit_NeoPixel.h>

// ---- Pin config -----------------------------------------------------------
#define LED_PIN      27
#define BUS_RX       32
#define BUS_TX       26

// ---- Constants ------------------------------------------------------------
#define OUR_ID        7
#define NUM_LEDS     25
#define MATRIX_W      5
#define MATRIX_H      5
#define UART_BAUD    1000000
#define MAX_FRAME    64

// ATOM command addresses (sent via Feetech WRITE instruction 0x03)
#define ADDR_PING      0x00
#define ADDR_SET_COLOR 0x01
#define ADDR_SET_PIXEL 0x02

// Feetech protocol
#define FEETECH_HEADER 0xFF
#define FEETECH_WRITE  0x03

// ---- Hardware -------------------------------------------------------------
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
HardwareSerial BusSerial(1);

// ---- Frame buffer ---------------------------------------------------------
uint8_t buf[MAX_FRAME];
int     buf_pos   = 0;
bool    commanded = false;   // true once the first ATOM command arrives

// ---- Helpers --------------------------------------------------------------

uint8_t checksum(const uint8_t* data, int len) {
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return (~sum) & 0xFF;
}

// Build and send a Feetech status packet: FF FF 07 <LEN> 00 [data] <CHK>
void send_status(const uint8_t* data = nullptr, int len = 0) {
    uint8_t hdr[4] = {FEETECH_HEADER, FEETECH_HEADER, OUR_ID, (uint8_t)(len + 2)};
    uint8_t err = 0;

    BusSerial.write(hdr, 4);
    BusSerial.write(err);
    if (len > 0 && data != nullptr) {
        BusSerial.write(data, len);
    }

    // checksum over ID..data
    int chk_len = 1 + 1 + 1 + len;  // ID + LEN + ERR + data
    uint8_t chk_buf[chk_len];
    chk_buf[0] = OUR_ID;
    chk_buf[1] = len + 2;
    chk_buf[2] = err;
    if (len > 0) memcpy(chk_buf + 3, data, len);
    uint8_t chk = checksum(chk_buf, chk_len);
    BusSerial.write(chk);
    BusSerial.flush();
}

void process_frame(const uint8_t* frame, int frame_len) {
    if (frame_len < 6) return;                       // need header(2)+id+len+instr+chk
    if (frame[2] != OUR_ID) return;                   // not for us
    if (frame[4] != FEETECH_WRITE) return;             // we only handle WRITE

    commanded = true;   // stop the startup animation

    int len   = frame[3];                             // payload length after LEN byte
    int dlen  = len - 2;                              // minus INSTR + ADDR
    uint8_t addr = frame[5];
    const uint8_t* d = frame + 6;                     // data starts after ADDR

    switch (addr) {

        case ADDR_PING:
            send_status();
            break;

        case ADDR_SET_COLOR:
            if (dlen >= 3) {
                for (int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(d[0], d[1], d[2]));
                }
                strip.show();
                send_status();
            }
            break;

        case ADDR_SET_PIXEL:
            if (dlen >= 5 && d[0] < MATRIX_W && d[1] < MATRIX_H) {
                int idx = d[1] * MATRIX_W + d[0];
                strip.setPixelColor(idx, strip.Color(d[2], d[3], d[4]));
                strip.show();
                send_status();
            }
            break;
    }
}

// ---- Setup ----------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    BusSerial.begin(UART_BAUD, SERIAL_8N1, BUS_RX, BUS_TX);

    strip.begin();
    strip.clear();
    strip.show();

    Serial.println("ATOM LED Matrix ready (Feetech ID 7)");
    Serial.printf("Bus: %d baud, RX=%d TX=%d\n", UART_BAUD, BUS_RX, BUS_TX);
}

// ---- Loop -----------------------------------------------------------------

void loop() {
    // Startup animation — cycles colours until the first ATOM command arrives
    if (!commanded) {
        static unsigned long last_frame = 0;
        static uint8_t      hue        = 0;
        if (millis() - last_frame > 30) {
            last_frame = millis();
            uint32_t rgb = strip.ColorHSV(hue << 8);   // HSV hue 0-255 → 16-bit
            for (int i = 0; i < NUM_LEDS; i++) {
                strip.setPixelColor(i, rgb);
            }
            strip.show();
            hue++;
        }
    }

    while (BusSerial.available()) {
        uint8_t b = BusSerial.read();

        // ---- look for Feetech header 0xFF 0xFF ----
        if (buf_pos < 2) {
            if (b == FEETECH_HEADER) {
                buf[buf_pos++] = b;
            } else {
                buf_pos = 0;
            }
            continue;
        }

        buf[buf_pos++] = b;

        // Once we have LEN (buf[3]), we know the total frame size
        if (buf_pos >= 4) {
            int total = buf[3] + 4;   // Feetech frame: 2 headers + ID + LEN + payload + checksum
            if (total > MAX_FRAME) {
                buf_pos = 0;
                continue;
            }
            if (buf_pos >= total) {
                // Validate checksum (over ID..last byte before checksum)
                uint8_t chk = checksum(buf + 2, total - 3);
                if (chk == buf[total - 1] && buf[2] == OUR_ID) {
                    process_frame(buf, total);
                }
                buf_pos = 0;
            }
        }
    }
}
