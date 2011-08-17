#ifndef __BOARD_SBC51_H__
#define __BOARD_SBC51_H__

struct blt_gps_platform_data {
	char *reg;
	int uV;
};

struct blt_gsm_platform_data {
	void (*power) (int on);
	void (*reset) (void);
};

struct blt_io_platform_data {
	void (*power) (int on);
	void (*reset) (void);
};


#endif
