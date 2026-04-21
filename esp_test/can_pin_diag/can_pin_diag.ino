/**
 * @file   can_pin_diag.ino
 * @brief  Slow toggle test — blinks the LED on the second transceiver
 *
 * Toggles CTX (GPIO 5) slowly so you can SEE the LED on the
 * second transceiver's CRX blink on and off.
 *
 * CTX LOW  (dominant) → CANH high, CANL low → CRX2 = LOW  → LED OFF
 * CTX HIGH (recessive) → CANH ≈ CANL        → CRX2 = HIGH → LED ON
 *
 * Also reads back CRX on GPIO 4 from the FIRST transceiver.
 */

#define CAN_TX  5
#define CAN_RX  4

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== Slow CAN Bus Toggle Test ===");
    Serial.println("Watch the LED on the second transceiver!\n");

    pinMode(CAN_TX, OUTPUT);
    pinMode(CAN_RX, INPUT);

    /* Phase 1: Recessive (LED should be ON) */
    digitalWrite(CAN_TX, HIGH);
    Serial.println(">>> RECESSIVE — LED should be ON");
    Serial.printf("    CRX1 (GPIO 4) = %d\n", digitalRead(CAN_RX));
    delay(3000);

    /* Phase 2: Dominant (LED should be OFF) */
    digitalWrite(CAN_TX, LOW);
    Serial.println(">>> DOMINANT  — LED should be OFF");
    Serial.printf("    CRX1 (GPIO 4) = %d\n", digitalRead(CAN_RX));
    delay(3000);

    /* Phase 3: Back to recessive (LED ON) */
    digitalWrite(CAN_TX, HIGH);
    Serial.println(">>> RECESSIVE — LED should be ON again");
    Serial.printf("    CRX1 (GPIO 4) = %d\n\n", digitalRead(CAN_RX));
    delay(2000);

    Serial.println("Now blinking every 1 second...\n");
}

void loop() {
    /* Dominant — LED OFF */
    digitalWrite(CAN_TX, LOW);
    Serial.printf("TX=LOW  (dominant)  CRX=%d  | LED should be OFF\n",
                  digitalRead(CAN_RX));
    delay(1000);

    /* Recessive — LED ON */
    digitalWrite(CAN_TX, HIGH);
    Serial.printf("TX=HIGH (recessive) CRX=%d  | LED should be ON\n",
                  digitalRead(CAN_RX));
    delay(1000);
}
