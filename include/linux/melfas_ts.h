#ifndef _LINUX_MELFAS_TS_H
#define _LINUX_MELFAS_TS_H

#define MELFAS_TS_NAME "synaptics-rmi-ts"

struct melfas_tsi_platform_data {
int x_size;
int y_size;
int version;
};

#endif /* _LINUX_MELFAS_TS_H */