#include <LibLanc.h>

#define LANC_INPUT_PIN  2 // TIP
#define LANC_OUTPUT_PIN  3 // SLEEVE
Lanc lanc(LANC_INPUT_PIN, LANC_OUTPUT_PIN);

void setup() {
    lanc.begin();
}

void loop() {
    // get next command to execute
    // call lanc.Zoom(value);
    lanc.Zoom(5);
    lanc.loop();
}