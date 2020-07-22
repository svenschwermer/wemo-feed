#ifndef INSIGHT_H_
#define INSIGHT_H_

struct insight_data;

struct insight_data *insight_open(const char *dev);
int insight_free(struct insight_data *);

#endif
