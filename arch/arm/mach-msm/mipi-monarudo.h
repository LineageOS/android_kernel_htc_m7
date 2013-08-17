#ifndef MIPI_MONARUDO_H
#define MIPI_MONARUDO_H

#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>

int mipi_monarudo_device_register(struct msm_panel_info *pinfo,
                                 u32 channel, u32 panel);

#endif /* !MIPI_MONARUDO_H */

