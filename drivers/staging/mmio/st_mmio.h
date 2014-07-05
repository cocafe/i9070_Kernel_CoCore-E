
#ifndef __ST_MMIO_H__
#define __ST_MMIO_H__

extern struct class *sec_class;
extern struct class *camera_class;

/* Flash IC Lux Value Setting */
void mmio_cam_flash_rt8515(int lux_val);
void mmio_cam_flash_ktd262(int lux_val);

/* NCP6914 Camera Sub-PMIC */
int  NCP6914_subPMIC_module_init(void);
void NCP6914_subPMIC_module_exit(void);
int  NCP6914_subPMIC_PowerOn(int opt);
int  NCP6914_subPMIC_PowerOff(int opt);
int  NCP6914_subPMIC_PinOnOff(int pin, int on_off);

/* SM5103 Camera Sub-PMIC */
int  SM5103_subPMIC_module_init(void);
void SM5103_subPMIC_module_exit(void);
int  SM5103_subPMIC_PowerOn(int opt);
int  SM5103_subPMIC_PowerOff(int opt);
int  SM5103_subPMIC_PinOnOff(int pin, int on_off);

void check_VT_CAM_ID(int pin);

#endif
