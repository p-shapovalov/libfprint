/*
 * Elan SPI driver for libfprint
 *
 * Copyright (C) 2021 Matthew Mirvish <matthew@mm12.xyz>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "elanspi"

#include "drivers_api.h"
#include "elanspi.h"
#include "fpi-spi-transfer.h"

#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <errno.h>

struct _FpiDeviceElanSpi {
	FpImageDevice parent;

	/* sensor info */
	unsigned char sensor_width, sensor_height, sensor_ic_version, sensor_id;
	gboolean sensor_otp;
	/* end sensor info */

	/* init info */
	unsigned char sensor_raw_version, sensor_init_reg17;
	/* end init info */

	/* background / calibration parameters */
	guint16 *bg_image;

	/* active SPI status info */
	int spi_fd;
};

G_DECLARE_FINAL_TYPE(FpiDeviceElanSpi, fpi_device_elanspi, FPI, DEVICE_ELANSPI, FpImageDevice);
G_DEFINE_TYPE(FpiDeviceElanSpi, fpi_device_elanspi, FP_TYPE_IMAGE_DEVICE);

static void elanspi_do_hwreset(FpiDeviceElanSpi *self, GError **err) {
	/*
	 * TODO: Make this also work with the non-HID cases
	 */

	int fd = open((char *)fpi_device_get_udev_data(FP_DEVICE(self), FP_DEVICE_UDEV_SUBTYPE_HIDRAW), O_RDWR);
	if (fd < 0) {
		g_set_error(err, G_IO_ERROR, g_io_error_from_errno(errno), "unable to open hid");
		return;
	}

	guint8 buf[5] = {
		0xe, 0, 0, 0, 0
	};

	if (ioctl(fd, HIDIOCSFEATURE(5), &buf) != 5) {
		g_set_error(err, G_IO_ERROR, g_io_error_from_errno(errno), "unable to reset via hid");
		return;
	}

	close(fd);
}

/*
 * Three main processes involved in driving these sensors:
 *   - initialization (device type detection)
 *   - calibration
 *      - image capture (single)
 *   - image capture (stitched)
 */

enum elanspi_init_state {
	ELANSPI_INIT_READ_STATUS1,
	ELANSPI_INIT_HWSWRESET, // fused b.c. hw reset is currently sync
	ELANSPI_INIT_READ_HEIGHT,
	ELANSPI_INIT_READ_WIDTH,
	ELANSPI_INIT_READ_REG17,   // both of these states finish setting up sensor settings
	ELANSPI_INIT_READ_VERSION, // can jump straight to calibrate
	ELANSPI_INIT_SWRESET2,
	ELANSPI_INIT_OTP_READ_VREF1,
	ELANSPI_INIT_OTP_WRITE_VREF1,
	ELANSPI_INIT_OTP_WRITE_0x28,
	ELANSPI_INIT_OTP_LOOP_READ_0x28, // may loop
	ELANSPI_INIT_OTP_LOOP_READ_0x27,
	ELANSPI_INIT_OTP_LOOP_UPDATEDAC_READ_DAC2,
	ELANSPI_INIT_OTP_LOOP_UPDATEDAC_WRITE_DAC2,
	ELANSPI_INIT_OTP_LOOP_UPDATEDAC_WRITE_10,
	// exit loop
	ELANSPI_INIT_OTP_WRITE_0xb,
	ELANSPI_INIT_OTP_WRITE_0xc,
	// do calibration (mutexc)
	ELANSPI_INIT_CALIBRATE,
	ELANSPI_INIT_NSTATES
};

enum elanspi_calibrate_old_state {
	ELANSPI_CALIBOLD_UNPROTECT,
	ELANSPI_CALIBOLD_WRITE_STARTCALIB,
	ELANSPI_CALIBOLD_SEND_REGTABLE,
	// calibrate dac base value
	ELANSPI_CALIBOLD_DACBASE_CAPTURE,
	ELANSPI_CALIBOLD_DACBASE_WRITE_DAC1,
	// check for finger
	ELANSPI_CALIBOLD_CHECKFIN_CAPTURE,
	// increase gain
	ELANSPI_CALIBOLD_WRITE_GAIN,
	// calibrate dac stage2
	ELANSPI_CALIBOLD_DACFINE_CAPTURE,
	ELANSPI_CALIBOLD_DACFINE_WRITE_DAC1,
	// exit ok
	ELANSPI_CALIBOLD_PROTECT,
	// capture bg image
	ELANSPI_CALIBOLD_BG_CAPTURE, // jumps to end of ssm on ok
	// jump to reset to 0 but fail too
	ELANSPI_CALIBOLD_EXIT_ERR,
	ELANSPI_CALIBOLD_NSTATES
};

enum elanspi_capture_old_state {
	ELANSPI_CAPTOLD_WRITE_CAPTURE,
	ELANSPI_CAPTOLD_CHECK_LINEREADY,
	ELANSPI_CAPTOLD_RECV_LINE,

	ELANSPI_CAPTOLD_NSTATES
};

enum elanspi_calibrate_hv_state {
	ELANSPI_CALIBHV_SELECT_PAGE0_0,
	ELANSPI_CALIBHV_WRITE_STARTCALIB,
	ELANSPI_CALIBHV_UNPROTECT,
	ELANSPI_CALIBHV_SEND_REGTABLE0,
	ELANSPI_CALIBHV_SELECT_PAGE1,
	ELANSPI_CALIBHV_SEND_REGTABLE1,
	ELANSPI_CALIBHV_SELECT_PAGE0_1,
	ELANSPI_CALIBHV_WRITE_GDAC_H,
	ELANSPI_CALIBHV_WRITE_GDAC_L,
	ELANSPI_CALIBHV_CAPTURE,
	ELANSPI_CALIBHV_WRITE_BEST_GDAC_H,
	ELANSPI_CALIBHV_WRITE_BEST_GDAC_L,
	
	ELANSPI_CALIBHV_NSTATES
};

enum elanspi_capture_hv_state {
	ELANSPI_CAPTHV_WRITE_CAPTURE,
	ELANSPI_CHECK_READY,
	ELANSPI_RECV_IMAGE
};

enum elanspi_write_regtable_state {
	ELANSPI_WRTABLE_WRITE,
	ELANSPI_WRTABLE_ITERATE,
	ELANSPI_WRTABLE_NSTATES
};
