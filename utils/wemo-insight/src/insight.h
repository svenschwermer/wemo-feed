#ifndef INSIGHT_H_
#define INSIGHT_H_

struct insight_state;
struct insight_data
{
    float int_temperature;
    float ext_temperature;
    float rms_voltage;
    float rms_current;
    float active_power;
    float average_power;
    float power_factor;
    float line_frequency;
    float active_energy;
};

struct insight_state *insight_open(const char *dev);
int insight_free(struct insight_state *s);
const struct insight_data *insight_borrow_data(struct insight_state *s);
void insight_return_data(struct insight_state *s);
void print_data(const struct insight_state *s);

#endif
