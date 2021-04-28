// #pragma once
#include "images.h"

#include <Arduino.h>

// BITMAP-BILDER BITMAP-BILDER BITMAP-BILDER BITMAP-BILDER BITMAP-BILDER

const uint8_t settingsbutton10[] = {
  0x00, 0x00, 0xfe, 0x01, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x01, 0x00, 0x00,
  0x00, 0x00, 0xfe, 0x01, 0x00, 0x00, 0x00, 0x00
};
const uint8_t settingsbutton40[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x80, 0xff, 0xff,
  0xff, 0x01, 0x80, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x3c, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x80,
  0xff, 0xff, 0xff, 0x01, 0x80, 0xff, 0xff, 0xff, 0x01, 0x00, 0x3c, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff,
  0x01, 0x80, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const uint8_t settingsbutton30[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0xc0, 0xff, 0xff, 0x00,
  0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
  0xc0, 0xff, 0xff, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00,
  0xc0, 0xff, 0xff, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t espresso_cup40[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x01, 0x80, 0x00, 0x00,
  0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01,
  0x80, 0x00, 0x00, 0x00, 0x0f, 0x80, 0x00, 0x00, 0x00, 0x1b, 0x80, 0x01,
  0x00, 0x80, 0x31, 0x00, 0x01, 0x00, 0x80, 0x20, 0x00, 0x03, 0x00, 0xc0,
  0x20, 0x00, 0x02, 0x00, 0xc0, 0x31, 0x00, 0x06, 0x00, 0x60, 0x1b, 0x00,
  0x04, 0x00, 0x20, 0x0e, 0x00, 0x0c, 0x00, 0x30, 0x00, 0x00, 0x08, 0x00,
  0x10, 0x00, 0x00, 0x18, 0x00, 0x18, 0x00, 0x00, 0x10, 0x00, 0x08, 0x00,
  0x00, 0x10, 0x00, 0x08, 0x00, 0xfe, 0xff, 0xff, 0xff, 0x7f, 0x0e, 0x00,
  0x00, 0x00, 0x70, 0x38, 0x00, 0x00, 0x00, 0x1c, 0xe0, 0x00, 0x00, 0x00,
  0x07, 0x80, 0x03, 0x00, 0xc0, 0x01, 0x00, 0xfe, 0xff, 0x7f, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t espresso_cup30[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x7f, 0x00,
  0x80, 0x00, 0x40, 0x00, 0x80, 0x00, 0xc0, 0x01, 0x80, 0x00, 0x60, 0x03,
  0x80, 0x01, 0x20, 0x02, 0x00, 0x01, 0x20, 0x02, 0x00, 0x03, 0x70, 0x03,
  0x00, 0x02, 0xd0, 0x01, 0x00, 0x06, 0x18, 0x00, 0x00, 0x04, 0x08, 0x00,
  0xfe, 0xff, 0xff, 0x1f, 0x06, 0x00, 0x00, 0x18, 0x1c, 0x00, 0x00, 0x0e,
  0x70, 0x00, 0x80, 0x03, 0xc0, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t back_10[] PROGMEM = {
  0xc0, 0x00, 0xe0, 0x00, 0x70, 0x00, 0x38, 0x00, 0x1c, 0x00, 0x1c, 0x00,
  0x38, 0x00, 0x70, 0x00, 0xe0, 0x00, 0xc0, 0x00
};

const uint8_t round_10[] PROGMEM = {
  0x78, 0x00, 0xcc, 0x00, 0x86, 0x01, 0x03, 0x03, 0x01, 0x02, 0x01, 0x02,
  0x03, 0x03, 0x86, 0x01, 0xcc, 0x00, 0x78, 0x00 
};
const uint8_t roundfilled_10[] PROGMEM = {
  0x78, 0x00, 0xfc, 0x00, 0xfe, 0x01, 0xff, 0x03, 0xff, 0x03, 0xff, 0x03,
  0xff, 0x03, 0xfe, 0x01, 0xfc, 0x00, 0x78, 0x00
};

const uint8_t home_10[] PROGMEM = {
   0x30, 0x00, 0x48, 0x00, 0x84, 0x00, 0x02, 0x01, 0x01, 0x02, 0x02, 0x01,
   0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0xfe, 0x01 
};

const uint8_t home_10_filled[] PROGMEM = {
   0x30, 0x00, 0x78, 0x00, 0xfc, 0x00, 0xfe, 0x01, 0xff, 0x03, 0xfe, 0x01,
   0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01 
};

const uint8_t espresso_10[] PROGMEM = {
  0x00, 0x00, 0xfc, 0x00, 0x84, 0x03, 0x84, 0x02, 0xcc, 0x03, 0x48, 0x00,
  0x7b, 0x03, 0xfe, 0x01, 0x00, 0x00, 0x00, 0x00 
};
const uint8_t trash_10 [] PROGMEM = {
  0x30, 0x00, 0xff, 0x03, 0x02, 0x01, 0x4a, 0x01, 0x4a, 0x01, 0x4a, 0x01,
  0x4a, 0x01, 0x4a, 0x01, 0x02, 0x01, 0xfe, 0x01
};

const uint8_t trash_10_filled [] PROGMEM = {
  0x30, 0x00, 0xff, 0x03, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01,
  0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01, 0xfe, 0x01
};

const uint8_t trash_20 [] PROGMEM = {
  0x00, 0x0f, 0x00, 0x00, 0x0f, 0x00, 0xfe, 0xff, 0x07, 0x08, 0x00, 0x01,
  0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x88, 0x10, 0x01,
  0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x88, 0x10, 0x01,
  0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x88, 0x10, 0x01,
  0x88, 0x10, 0x01, 0x88, 0x10, 0x01, 0x08, 0x00, 0x01, 0xf8, 0xff, 0x01
};

const uint8_t home_20 [] PROGMEM = {
   0x00, 0x06, 0x00, 0x00, 0x0f, 0x00, 0x80, 0x19, 0x00, 0xc0, 0x30, 0x00,
   0x60, 0x60, 0x00, 0x30, 0xc0, 0x00, 0x18, 0x80, 0x01, 0x0c, 0x00, 0x03,
   0x06, 0x00, 0x06, 0x07, 0x00, 0x0e, 0x05, 0x00, 0x0a, 0x04, 0x00, 0x02,
   0x04, 0x00, 0x02, 0x04, 0x00, 0x02, 0x04, 0x00, 0x02, 0x04, 0x00, 0x02,
   0x04, 0x00, 0x02, 0x04, 0x00, 0x02, 0x04, 0x00, 0x02, 0xfc, 0xff, 0x03 
};

const uint8_t hourglass_10 [] PROGMEM = {
   0xfe, 0x01, 0x02, 0x01, 0xb4, 0x00, 0x78, 0x00, 0x30, 0x00, 0x30, 0x00,
   0x48, 0x00, 0x84, 0x00, 0x02, 0x01, 0xfe, 0x01 
};

const uint8_t hourglass_10_filled [] PROGMEM = {
   0xfe, 0x01, 0xfe, 0x01, 0xfc, 0x00, 0x78, 0x00, 0x30, 0x00, 0x30, 0x00,
   0x78, 0x00, 0xfc, 0x00, 0xfe, 0x01, 0xfe, 0x01 
};