#include "rtthread.h"
#include "rotary_encoder.h"

typedef struct rotary_knob_config {
    int clk_pin;
    int dt_pin;
    int sw_pin;
} rotary_knob_config_t;

typedef struct rotary_knob rotary_knob_t; 

rotary_knob_t * rotary_knob_create(rotary_knob_config_t *cfg);

void rotary_knob_destroy(rotary_knob_t *rotary_knob);