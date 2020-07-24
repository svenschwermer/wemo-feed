#ifndef INSIGHT_H_
#define INSIGHT_H_

struct insight_state;
struct insight_data
{
    float int_temperature; // °C
    float ext_temperature; // °C
    float rms_voltage;     // V
    float rms_current;     // A
    float active_power;    // W
    float average_power;   // W
    float power_factor;
    float line_frequency;  // Hz
    float active_energy;   // kWh
};

struct insight_state *insight_open(const char *dev);
int insight_free(struct insight_state *s);
const struct insight_data *insight_borrow_data(struct insight_state *s);
void insight_return_data(struct insight_state *s);
void print_data(const struct insight_state *s);

#endif
