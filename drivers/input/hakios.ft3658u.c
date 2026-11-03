/*
 * The touch screen of the realme GT Neo 3T (spartan, SM8250-AC): a FocalTech FT3658U on I2C, serial engine 13 of the SoC
 * (QUP wrapper 1, engine 5, at 0xA94000), as a source of touches for /dev/hakios-touch (sys/hakios.input.h). Only with
 * boot-arg ft3658u=1: every address below belongs to this phone.
 *
 * As the POCO X5 Pro's driver (drivers/input/hakios.goodix.c), whose RPMh, clock and serial-engine code this one follows,
 * there is no clock, pin, regulator or I2C framework to lean on, so it does all of it itself, in the order realme's Linux
 * driver does:
 *   power    the chip's 3.0 V supply is LDO11 of the PM8150A ("ldoc11", 3008 mV, the device tree's vdd_2v8_volt) and its
 *            1.8 V supply LDO1 ("ldoc1"), both switched through RPMh; reset low, 1 ms, both supplies, 10 ms, reset high,
 *            200 ms (POWEWRUP_TO_RESET_TIME, RESET_TO_NORMAL_TIME)
 *   clocks   the serial engine's three clocks, by vote in the global clock controller (the same registers as SC7280's)
 *   pins     GPIO36/37 to the serial engine (function qup13), GPIO38 reset, GPIO39 interrupt, all on the TLMM's south tile
 *   I2C      the serial engine ("GENI") in FIFO mode, polled, at 400 kHz from the 19.2 MHz crystal; a register is read as a
 *            write of its number with the stop held back (a repeated start), then a read
 *   chip     its ID at 0xA3 (0x56); the touch report at 0x01: 21 bytes, and 41 more from 0x16 when the 21st is not 0xFF;
 *            a finger is 6 bytes from the 3rd on: event (7:6) and X (11:8), X (7:0), ID (7:4) and Y (11:8), Y (7:0),
 *            the 8-times finer bits, the area. Its firmware is in its own flash: nothing is uploaded
 * No interrupt is used: a thread looks at the interrupt line every 8 ms (boot-arg ft_poll_us=N).
 *
 * Coordinates: 12 bits, the panel taken as 1080 x 2400 (ft_max_x=, ft_max_y=); ft_x8=1 reads the 8-times finer form
 * (15 bits, 8640 x 19200: the device tree's touchpanel,panel-coords). The first touches are logged as the chip sent them,
 * so that the scale can be checked on the phone. ft_flip=1/2/3 mirrors X, Y or both.
 *
 * Where the numbers come from: the phone's own device tree (focaltech@38 on i2c@a94000, rpmh-regulator-ldoc11 and -ldoc1,
 * pmx_ts_int_active_oppo, pmx_ts_reset_active_oppo, qupv3_se13_i2c_pins), realme's Linux driver (drivers/input/touchscreen/
 * oplus_touchscreen/Focal/ft3658u/), Linux drivers/i2c/busses/i2c-qcom-geni.c, drivers/soc/qcom/qcom-geni-se.c,
 * drivers/clk/qcom/gcc-sm8250.c and drivers/pinctrl/qcom/pinctrl-sm8250.c.
 *
 * Status: works, seen on an RMX3371, 3 November 2026: both supplies on, the clocks on (the wrapper's slave AHB clock under
 * the hardware's own gating, 0x80000002), the engine's I2C firmware (0x303), chip ID 0x56 0x52, firmware 0x31, vendor 0x48;
 * touches in the 12-bit form inside 1080 x 2400, no mirroring, 0 bad reports in 1617; Settings reached and opened by touch.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

/* RPMh's sequencer (TCS 0 of the RSC): the other drivers that write it take the same lock (drivers/core/hakios.platform.c) */
extern lck_mtx_t hakios_rsc_lock;

static vm_offset_t tlmm, gcc, se, rsc, cmd_db;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* ---- pins (TLMM, south tile) --------------------------------------------------------------------- */

#define TLMM_SOUTH_PHYS         0x0f500000UL    /* pinctrl@f000000 + the south tile (pinctrl-sm8250.c: gpio36-39 are SOUTH) */
#define TLMM_SOUTH_SIZE         0x100000UL
#define GPIO_CFG(n)             (tlmm + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + 0x1000UL * (n) + 4)     /* bit 0 in, bit 1 out */
#define PULL_NONE               0
#define PULL_UP                 3

#define PIN_SDA                 36
#define PIN_SCL                 37
#define PIN_RESET               38      /* low = in reset */
#define PIN_INT                 39      /* low = the chip has a report */
#define PIN_FUNCTION_I2C        1       /* "qup13" */

static void
gpio_configure(unsigned pin, unsigned function, unsigned pull, unsigned ma, bool output)
{
	wr(GPIO_CFG(pin), pull | function << 2 | (ma / 2 - 1) << 6 | (output ? 1u << 9 : 0));
}

static void
gpio_set(unsigned pin, bool high)
{
	wr(GPIO_IO(pin), high ? 2 : 0);
}

static bool
gpio_get(unsigned pin)
{
	return rd(GPIO_IO(pin)) & 1;
}

static void
gpio_output(unsigned pin, bool high, unsigned pull, unsigned ma)
{
	gpio_set(pin, high);            /* the level first, so that the pin never drives the wrong one */
	gpio_configure(pin, 0, pull, ma, true);
}

/* ---- the supplies (RPMh) ------------------------------------------------------------------------- */

#define CMD_DB_PHYS             0x80860000UL    /* the device tree's reserved-memory@80860000 */
#define CMD_DB_SIZE             0x20000UL
#define RSC_PHYS                0x18220000UL    /* rsc@18200000's "drv-2", the application processor's */
#define RSC_SIZE                0x10000UL
#define TCS(reg)                (rsc + 0xd00 + (reg))           /* qcom,tcs-offset = 0xd00: command slot set 0 */
#define TCS_IRQ_ENABLE          0x00
#define TCS_IRQ_STATUS          0x04
#define TCS_IRQ_CLEAR           0x08
#define TCS_WAIT_FOR_COMPLETION 0x10
#define TCS_CONTROL             0x14
#define TCS_CONTROL_ENABLE      (1u << 16)
#define TCS_CONTROL_TRIGGER     (1u << 24)
#define TCS_STATUS              0x18            /* 0 while busy */
#define TCS_CMD_ENABLE          0x1c
#define TCS_CMD_MSGID(n)        (0x30 + 20 * (n))
#define TCS_CMD_ADDR(n)         (0x34 + 20 * (n))
#define TCS_CMD_DATA(n)         (0x38 + 20 * (n))
#define MSGID_WRITE_4_BYTES     0x10008
#define MSGID_RESPONSE          0x100

#define VRM_VOLTAGE             0               /* a regulator's registers, from its address: millivolts */
#define VRM_ENABLE              4
#define VRM_MODE                8
#define VRM_MODE_LDO_HPM        7

struct cmd_db_slave {
	uint16_t id, header_offset, data_offset, count, version, reserved[3];
};
struct cmd_db_header {
	uint32_t version;
	uint8_t magic[4];
	struct cmd_db_slave slave[8];
	uint32_t checksum, reserved;
	uint8_t data[];
};
struct cmd_db_entry {
	char id[8];
	uint32_t priority[2];
	uint32_t address;
	uint16_t length, offset;
};

/* The RPMh address of a resource by its name in the command database the boot loader left; 0 = none. */
static uint32_t
rpmh_address(const char *name)
{
	const struct cmd_db_header *db = (const struct cmd_db_header *)cmd_db;
	char id[8] = { 0 };

	if (db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		printf("ft3658u: no command database (magic %02x %02x %02x %02x)\n", db->magic[0], db->magic[1], db->magic[2],
		    db->magic[3]);
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct cmd_db_slave *slave = &db->slave[s];
		const struct cmd_db_entry *entry = (const struct cmd_db_entry *)(db->data + slave->header_offset);

		if (slave->id == 0 || slave->header_offset >= CMD_DB_SIZE - 0x1000 || slave->count > 512) {
			continue;
		}
		for (unsigned i = 0; i < slave->count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

static bool
rpmh_wait(uint32_t reg, uint32_t mask, uint32_t value, const char *what)
{
	for (int i = 0; i < 2000; i++) {
		if ((rd(TCS(reg)) & mask) == value) {
			return true;
		}
		sleep_us(100);
	}
	printf("ft3658u: RPMh: timeout waiting for %s (register %#x = %#x)\n", what, reg, rd(TCS(reg)));
	return false;
}

/* Send up to three register writes to RPMh and wait until they are done. */
static bool
rpmh_write(const uint32_t *address, const uint32_t *data, unsigned count)
{
	uint32_t irq_enable;
	bool done;

	lck_mtx_lock(&hakios_rsc_lock);
	irq_enable = rd(TCS(TCS_IRQ_ENABLE));
	for (int i = 0; rd(TCS(TCS_STATUS)) == 0; i++) {
		if (i > 2000) {
			printf("ft3658u: RPMh: the command slots stay busy\n");
			lck_mtx_unlock(&hakios_rsc_lock);
			return false;
		}
		sleep_us(100);
	}
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_TRIGGER);
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_ENABLE);
	wr(TCS(TCS_IRQ_CLEAR), 1);
	for (unsigned i = 0; i < count; i++) {
		wr(TCS(TCS_CMD_MSGID(i)), MSGID_WRITE_4_BYTES | MSGID_RESPONSE);
		wr(TCS(TCS_CMD_ADDR(i)), address[i]);
		wr(TCS(TCS_CMD_DATA(i)), data[i]);
	}
	wr(TCS(TCS_WAIT_FOR_COMPLETION), (1u << count) - 1);
	wr(TCS(TCS_CMD_ENABLE), (1u << count) - 1);
	wr(TCS(TCS_IRQ_ENABLE), irq_enable | 1);        /* the status bit follows the enable; the interrupt itself stays masked */
	wr(TCS(TCS_CONTROL), TCS_CONTROL_ENABLE);
	wr(TCS(TCS_CONTROL), TCS_CONTROL_ENABLE | TCS_CONTROL_TRIGGER);

	done = rpmh_wait(TCS_IRQ_STATUS, 1, 1, "the request");
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_TRIGGER);
	wr(TCS(TCS_CONTROL), rd(TCS(TCS_CONTROL)) & ~TCS_CONTROL_ENABLE);
	wr(TCS(TCS_CMD_ENABLE), 0);
	wr(TCS(TCS_WAIT_FOR_COMPLETION), 0);
	wr(TCS(TCS_IRQ_CLEAR), 1);
	wr(TCS(TCS_IRQ_ENABLE), irq_enable);
	lck_mtx_unlock(&hakios_rsc_lock);
	return done;
}

/* One of the chip's regulators on at its voltage; false if the command database does not name it as a regulator. */
static bool
regulator_on(const char *name, uint32_t millivolts)
{
	uint32_t base = rpmh_address(name);
	uint32_t address[3] = { base + VRM_VOLTAGE, base + VRM_ENABLE, base + VRM_MODE };
	uint32_t data[3] = { millivolts, 1, VRM_MODE_LDO_HPM };

	/* addresses of regulators ("VRM") have 4 in bits 19:16; anything else is not what we are looking for */
	if (base == 0 || (base >> 16 & 0xf) != 4) {
		printf("ft3658u: RPMh: no regulator \"%s\" (address %#x)\n", name, base);
		return false;
	}
	if (!rpmh_write(address, data, 3)) {
		return false;
	}
	printf("ft3658u: %s on at %u mV (RPMh address %#x)\n", name, millivolts, base);
	return true;
}

/* ---- clocks (GCC: gcc-sm8250.c, the same registers as SC7280's) ----------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x60000UL
#define GCC_VOTE                (gcc + 0x52008)         /* the application processor's votes */
#define VOTE_QUP1_CORE_2X       (1u << 18)
#define VOTE_QUP1_CORE          (1u << 19)
#define VOTE_QUP1_M_AHB         (1u << 20)
#define VOTE_QUP1_S_AHB         (1u << 21)
#define VOTE_QUP1_SE5           (1u << 27)             /* serial engine 13 is the 6th of the second group */
#define GCC_QUP1_M_AHB_CBCR     (gcc + 0x18004)         /* bit 31: the clock is off */
#define GCC_QUP1_S_AHB_CBCR     (gcc + 0x18008)
#define GCC_QUP1_SE5_CBCR       (gcc + 0x185fc)
#define GCC_QUP1_SE5_CMD        (gcc + 0x18600)         /* bit 0: take the new configuration */
#define GCC_QUP1_SE5_CFG        (gcc + 0x18604)         /* [4:0] 2*divider-1, [10:8] source, [13:12] mode */
#define GCC_QUP1_SE5_DFSR       (gcc + 0x18614)         /* bit 0: the engine chooses among preset rates itself */
#define GCC_QUP1_SE5_PERF_CFG(l) (gcc + 0x1861c + 4 * (l))
#define GCC_QUP1_SE5_PERF_M(l)  (gcc + 0x1865c + 4 * (l))
#define GCC_QUP1_SE5_PERF_N(l)  (gcc + 0x1869c + 4 * (l))
#define CLOCK_OFF               (1u << 31)
#define CLOCK_HW_CTL            (1u << 1)       /* the hardware gates the clock itself: "off" while idle is no fault */
#define XO_HZ                   19200000

/* A branch is usable when it runs, or when the hardware gates it (gcc-sm8250.c: hwcg_bit 1 on the wrapper's AHB clocks,
 * which clk-branch.c then does not wait for: seen on the phone as 0x80000002 for the slave AHB clock with every vote in). */
static bool
branch_usable(uint32_t cbcr)
{
	return !(cbcr & CLOCK_OFF) || (cbcr & CLOCK_HW_CTL);
}

static uint32_t se_clock_select;

static uint32_t
rcg_rate(uint32_t cfg, uint32_t m, uint32_t not_n_minus_m)
{
	static const uint32_t parent[8] = { XO_HZ, 600000000, 0, 0, 0, 0, 300000000, 0 };
	uint64_t hz = parent[cfg >> 8 & 7];
	uint32_t divider = cfg & 0x1f, mode = cfg >> 12 & 3;

	if (divider) {
		hz = hz * 2 / (divider + 1);
	}
	if (mode) {
		uint32_t n = (~not_n_minus_m & 0xffff) + (m & 0xffff);

		if (n) {
			hz = hz * (m & 0xffff) / n;
		}
	}
	return (uint32_t)hz;
}

/* The engine's clock at 19.2 MHz, what the I2C timings below are worked out for (i2c-qcom-geni.c, geni_i2c_clk_map). */
static bool
clocks_on(void)
{
	bool found = false;

	wr(GCC_VOTE, rd(GCC_VOTE) | VOTE_QUP1_CORE_2X | VOTE_QUP1_CORE | VOTE_QUP1_M_AHB | VOTE_QUP1_S_AHB);
	if (rd(GCC_QUP1_SE5_DFSR) & 1) {
		/* preset rates: the one that is the crystal's */
		for (uint32_t level = 0; level < 8 && !found; level++) {
			if (rcg_rate(rd(GCC_QUP1_SE5_PERF_CFG(level)), rd(GCC_QUP1_SE5_PERF_M(level)),
			    rd(GCC_QUP1_SE5_PERF_N(level))) == XO_HZ) {
				se_clock_select = level;
				found = true;
			}
		}
		if (!found) {
			printf("ft3658u: no preset clock rate of 19.2 MHz\n");
			return false;
		}
	} else {
		/* the crystal, undivided: nothing else has to be running */
		wr(GCC_QUP1_SE5_CFG, (rd(GCC_QUP1_SE5_CFG) & ~0x371fu) | 1);
		wr(GCC_QUP1_SE5_CMD, rd(GCC_QUP1_SE5_CMD) | 1);
		for (int i = 0; i < 500 && (rd(GCC_QUP1_SE5_CMD) & 1); i++) {
			sleep_us(10);
		}
		if (rd(GCC_QUP1_SE5_CMD) & 1) {
			printf("ft3658u: the serial engine's clock does not take its configuration (%#x)\n", rd(GCC_QUP1_SE5_CMD));
			return false;
		}
		se_clock_select = 0;
	}
	wr(GCC_VOTE, rd(GCC_VOTE) | VOTE_QUP1_SE5);
	for (int i = 0; i < 1000; i++) {
		/* the engine's own clock has to run; the wrapper's two bus clocks may be gated by the hardware */
		if (!(rd(GCC_QUP1_SE5_CBCR) & CLOCK_OFF) && branch_usable(rd(GCC_QUP1_M_AHB_CBCR)) &&
		    branch_usable(rd(GCC_QUP1_S_AHB_CBCR))) {
			printf("ft3658u: clocks on: %#x %#x %#x\n", rd(GCC_QUP1_M_AHB_CBCR), rd(GCC_QUP1_S_AHB_CBCR),
			    rd(GCC_QUP1_SE5_CBCR));
			return true;
		}
		sleep_us(10);
	}
	printf("ft3658u: clocks stay off: %#x %#x %#x\n", rd(GCC_QUP1_M_AHB_CBCR), rd(GCC_QUP1_S_AHB_CBCR), rd(GCC_QUP1_SE5_CBCR));
	return false;
}

/* ---- I2C through the serial engine (GENI) --------------------------------------------------------- */

#define SE_PHYS                 0x00a94000UL    /* i2c@a94000 */
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      (se + 0x020)
#define GENI_OUTPUT_CTRL        (se + 0x024)
#define GENI_CGC_CTRL           (se + 0x028)
#define GENI_STATUS             (se + 0x040)    /* bit 0: a command is running */
#define GENI_SER_M_CLK_CFG      (se + 0x048)    /* bit 0 enable, [15:4] divider */
#define GENI_IF_DISABLE_RO      (se + 0x064)    /* bit 0: no FIFO interface */
#define GENI_FW_REVISION_RO     (se + 0x068)    /* [15:8] protocol: 3 = I2C */
#define GENI_CLK_SEL            (se + 0x07c)
#define GENI_BYTE_GRAN          (se + 0x254)
#define GENI_DMA_MODE_EN        (se + 0x258)
#define GENI_TX_PACKING_CFG0    (se + 0x260)
#define GENI_TX_PACKING_CFG1    (se + 0x264)
#define I2C_TX_TRANS_LEN        (se + 0x26c)
#define I2C_RX_TRANS_LEN        (se + 0x270)
#define I2C_SCL_COUNTERS        (se + 0x278)    /* [29:20] high, [19:10] low, [9:0] cycle */
#define GENI_RX_PACKING_CFG0    (se + 0x284)
#define GENI_RX_PACKING_CFG1    (se + 0x288)
#define GENI_M_CMD0             (se + 0x600)    /* [31:27] operation, [26:0] its parameters */
#define GENI_M_CMD_CTRL         (se + 0x604)    /* bit 2 cancel, bit 1 abort */
#define GENI_M_IRQ_STATUS       (se + 0x610)
#define GENI_M_IRQ_EN           (se + 0x614)
#define GENI_M_IRQ_CLEAR        (se + 0x618)
#define GENI_S_IRQ_CLEAR        (se + 0x648)
#define GENI_TX_FIFO            (se + 0x700)
#define GENI_RX_FIFO            (se + 0x780)
#define GENI_TX_FIFO_STATUS     (se + 0x800)    /* [27:0] words */
#define GENI_RX_FIFO_STATUS     (se + 0x804)    /* bit 31 last, [30:28] bytes of the last word, [24:0] words */
#define GENI_TX_WATERMARK       (se + 0x80c)
#define GENI_RX_WATERMARK       (se + 0x810)
#define GENI_RX_RFR_WATERMARK   (se + 0x814)
#define SE_DMA_TX_IRQ_CLR       (se + 0xc44)
#define SE_DMA_RX_IRQ_CLR       (se + 0xd44)
#define SE_GSI_EVENT_EN         (se + 0xe18)
#define SE_IRQ_EN               (se + 0xe1c)
#define SE_HW_PARAM_0           (se + 0xe24)    /* [21:16] FIFO depth in words */
#define SE_DMA_GENERAL_CFG      (se + 0xe30)

#define M_CMD_DONE              (1u << 0)
#define M_CMD_OVERRUN           (1u << 1)
#define M_ILLEGAL_CMD           (1u << 2)
#define M_CMD_FAILURE           (1u << 3)
#define M_CMD_CANCEL            (1u << 4)
#define M_CMD_ABORT             (1u << 5)
#define M_GP_IRQ_NACK           (1u << 10)      /* GP IRQ 1: the chip did not acknowledge */
#define M_GP_IRQ_BUS_PROTO      (1u << 12)      /* GP IRQ 3 */
#define M_GP_IRQ_ARB_LOST       (1u << 13)      /* GP IRQ 4 */
#define M_ERRORS                (M_CMD_OVERRUN | M_ILLEGAL_CMD | M_CMD_FAILURE | M_GP_IRQ_NACK | M_GP_IRQ_BUS_PROTO | \
	                         M_GP_IRQ_ARB_LOST)

#define I2C_WRITE               (0x1u << 27)
#define I2C_READ                (0x2u << 27)
#define I2C_STOP_STRETCH        (1u << 2)       /* no stop at the end: the next command starts again (repeated start) */
#define I2C_ADDRESS(a)          ((uint32_t)(a) << 9)

#define FT_ADDRESS              0x38            /* focaltech@38 */

static uint32_t fifo_depth;

/* 0 = ready; otherwise why the engine cannot be used */
static const char *
geni_i2c_init(uint32_t khz)
{
	/* i2c-qcom-geni.c's table for a 19.2 MHz source: divider, high, low, cycle */
	uint32_t divider = 2, high = 5, low = 12, cycle = 24;

	if ((rd(GENI_FW_REVISION_RO) >> 8 & 0xff) != 3) {
		return "the serial engine has no I2C firmware";
	}
	if (rd(GENI_IF_DISABLE_RO) & 1) {
		return "the serial engine's FIFO interface is disabled";
	}
	fifo_depth = rd(SE_HW_PARAM_0) >> 16 & 0x3f;
	if (fifo_depth < 4) {
		return "the serial engine reports no FIFO";
	}
	if (khz <= 100) {
		divider = 7, high = 10, low = 11, cycle = 26;
	}
	wr(SE_GSI_EVENT_EN, 0);
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	wr(GENI_S_IRQ_CLEAR, 0xffffffff);
	wr(SE_DMA_TX_IRQ_CLR, 0xffffffff);
	wr(SE_DMA_RX_IRQ_CLR, 0xffffffff);
	wr(GENI_CGC_CTRL, rd(GENI_CGC_CTRL) | 0x7f);
	wr(SE_DMA_GENERAL_CFG, rd(SE_DMA_GENERAL_CFG) | 0xf);
	wr(GENI_OUTPUT_CTRL, 0x7f);
	wr(GENI_FORCE_DEFAULT, 1);
	wr(GENI_DMA_MODE_EN, rd(GENI_DMA_MODE_EN) & ~1u);
	wr(GENI_RX_WATERMARK, fifo_depth - 3);
	wr(GENI_RX_RFR_WATERMARK, fifo_depth - 2);
	wr(GENI_TX_WATERMARK, 0);
	/* the status bits we poll follow these enables; the interrupt itself stays masked at the GIC */
	wr(SE_IRQ_EN, rd(SE_IRQ_EN) | 0xf);
	wr(GENI_M_IRQ_EN, rd(GENI_M_IRQ_EN) | 0x7fc0007f | M_GP_IRQ_NACK | M_GP_IRQ_BUS_PROTO | M_GP_IRQ_ARB_LOST);

	/* 8 bits a byte, four bytes a FIFO word, most significant first (geni_se_config_packing(8, 4, true, ...)) */
	wr(GENI_TX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_TX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_RX_PACKING_CFG0, 0x0007f8fe);
	wr(GENI_RX_PACKING_CFG1, 0x000ffefe);
	wr(GENI_BYTE_GRAN, 0);

	wr(GENI_CLK_SEL, se_clock_select & 7);
	wr(GENI_SER_M_CLK_CFG, divider << 4 | 1);
	wr(I2C_SCL_COUNTERS, high << 20 | low << 10 | cycle);
	return NULL;
}

static void
geni_recover(void)
{
	wr(GENI_M_CMD_CTRL, 1u << 2);
	for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL); i++) {
		sleep_us(100);
	}
	if (!(rd(GENI_M_IRQ_STATUS) & M_CMD_CANCEL)) {
		wr(GENI_M_CMD_CTRL, 1u << 1);
		for (int i = 0; i < 2000 && !(rd(GENI_M_IRQ_STATUS) & M_CMD_ABORT); i++) {
			sleep_us(100);
		}
	}
	for (int i = 0; i < 64 && (rd(GENI_RX_FIFO_STATUS) & 0x01ffffff); i++) {
		(void)rd(GENI_RX_FIFO);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
}

static void
geni_drain(uint8_t *rx, uint32_t length, uint32_t *received)
{
	uint32_t status = rd(GENI_RX_FIFO_STATUS), words = status & 0x01ffffff, bytes = words * 4;

	if (status & 0x80000000u) {
		uint32_t valid = status >> 28 & 7;

		if (valid && valid < 4) {
			bytes -= 4 - valid;
		}
	}
	for (uint32_t i = 0; i < words; i++) {
		uint32_t word = rd(GENI_RX_FIFO);

		for (int j = 0; j < 4 && bytes; j++, bytes--) {
			if (*received < length) {
				rx[*received] = (uint8_t)(word >> (8 * j));
				(*received)++;
			}
		}
	}
}

/* One I2C command to the chip: `tx` out (a write) or `rx` in (a read); stretch = no stop after it. */
static bool
geni_i2c_command(const uint8_t *tx, uint8_t *rx, uint32_t length, bool stretch)
{
	static unsigned complaints;
	uint32_t sent = 0, received = 0, status = 0;
	uint64_t deadline;

	for (int i = 0; rd(GENI_STATUS) & 1; i++) {
		if (i > 2000) {
			geni_recover();
			break;
		}
		sleep_us(100);
	}
	wr(GENI_M_IRQ_CLEAR, 0xffffffff);
	if (tx) {
		wr(I2C_TX_TRANS_LEN, length);
		wr(GENI_M_CMD0, I2C_WRITE | I2C_ADDRESS(FT_ADDRESS) | (stretch ? I2C_STOP_STRETCH : 0));
	} else {
		wr(I2C_RX_TRANS_LEN, length);
		wr(GENI_M_CMD0, I2C_READ | I2C_ADDRESS(FT_ADDRESS) | (stretch ? I2C_STOP_STRETCH : 0));
	}
	clock_interval_to_deadline(50, NSEC_PER_MSEC, &deadline);
	while (mach_absolute_time() < deadline) {
		while (tx && sent < length && (rd(GENI_TX_FIFO_STATUS) & 0x0fffffff) + 1 < fifo_depth) {
			uint32_t word = 0;

			for (int j = 0; j < 4 && sent < length; j++, sent++) {
				word |= (uint32_t)tx[sent] << (8 * j);
			}
			wr(GENI_TX_FIFO, word);
		}
		if (rx) {
			geni_drain(rx, length, &received);
		}
		status = rd(GENI_M_IRQ_STATUS);
		if (status & M_ERRORS) {
			break;
		}
		if (status & M_CMD_DONE) {
			if (rx) {
				geni_drain(rx, length, &received);
			}
			wr(GENI_M_IRQ_CLEAR, status);
			if (tx ? sent == length : received == length) {
				return true;
			}
			break;
		}
	}
	if (complaints++ < 8) {
		printf("ft3658u: I2C %s of %u bytes failed: status %#x%s, %u of them done\n", tx ? "write" : "read", length, status,
		    (status & M_GP_IRQ_NACK) ? " (no acknowledge: the chip is not there, or not powered, or in reset)" : "",
		    tx ? sent : received);
	}
	geni_recover();
	return false;
}

/* The chip's registers from `reg` on: a write of the register's number, a repeated start, a read. */
static bool
ft_read(uint8_t reg, uint8_t *data, uint32_t length)
{
	return geni_i2c_command(&reg, NULL, 1, true) && geni_i2c_command(NULL, data, length, false);
}

/* ---- the chip ------------------------------------------------------------------------------------- */

#define FT_REG_POINTS           0x01            /* the report: FTS_POINTS_ONE bytes from here ... */
#define FT_REG_POINTS_N         0x16            /* ... and FTS_POINTS_TWO from here */
#define FT_POINTS_ONE           21
#define FT_POINTS_TWO           41
#define FT_REG_CHIP_ID          0xa3
#define FT_REG_CHIP_ID2         0x9f
#define FT_REG_FW_VER           0xa6
#define FT_REG_VENDOR_ID        0xa8
#define FT_CHIP_ID              0x56
#define FT_CHIP_ID2             0x52
#define FT_MAX_TOUCHES          10
#define FT_EVENT_DOWN           0
#define FT_EVENT_UP             1
#define FT_EVENT_CONTACT        2

static uint32_t max_x = 1080, max_y = 2400, flip, x8;

static struct {
	uint32_t polls, reports, bad, touches, last_x, last_y;
} seen;

static struct {
	bool down, seen;
	uint32_t x, y;
} contact[FT_MAX_TOUCHES];

static bool
ft_probe(void)
{
	uint8_t id = 0, id2 = 0, fw = 0, vendor = 0;

	for (int attempt = 0; attempt < 3; attempt++) {
		if (ft_read(FT_REG_CHIP_ID, &id, 1) && ft_read(FT_REG_CHIP_ID2, &id2, 1)) {
			break;
		}
		sleep_ms(20);
	}
	(void)ft_read(FT_REG_FW_VER, &fw, 1);
	(void)ft_read(FT_REG_VENDOR_ID, &vendor, 1);
	printf("ft3658u: chip ID %#04x %#04x (FT3658U: %#04x %#04x), firmware %#04x, vendor %#04x\n", id, id2, FT_CHIP_ID,
	    FT_CHIP_ID2, fw, vendor);
	if (id != FT_CHIP_ID) {
		printf("ft3658u: this is not the FT3658U answering, or it is in its boot loader (no firmware); not going on\n");
		return false;
	}
	return true;
}

static void
contact_update(uint32_t id, uint32_t event, uint32_t raw_x, uint32_t raw_y)
{
	static unsigned shown;
	uint32_t x = raw_x, y = raw_y;

	if (shown++ < 16) {
		printf("ft3658u: finger %u event %u raw %u, %u (taken as of %u x %u)\n", id, event, raw_x, raw_y, max_x, max_y);
	}
	if (event == FT_EVENT_UP) {
		contact[id].seen = true;
		if (contact[id].down) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
		return;
	}
	if (event != FT_EVENT_DOWN && event != FT_EVENT_CONTACT) {
		return;
	}
	seen.touches++;
	if (x > max_x - 1) {
		x = max_x - 1;
	}
	if (y > max_y - 1) {
		y = max_y - 1;
	}
	switch (flip) {
	case 1: x = max_x - 1 - x; break;
	case 2: y = max_y - 1 - y; break;
	case 3: x = max_x - 1 - x; y = max_y - 1 - y; break;
	}
	x = (uint32_t)(((uint64_t)x * 65535) / (max_x - 1));
	y = (uint32_t)(((uint64_t)y * 65535) / (max_y - 1));
	seen.last_x = x;
	seen.last_y = y;
	contact[id].seen = true;
	if (!contact[id].down) {
		contact[id].down = true;
		hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
	} else if (contact[id].x != x || contact[id].y != y) {
		hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
	}
	contact[id].x = x;
	contact[id].y = y;
}

static void
ft_poll(void)
{
	static uint8_t report[FT_POINTS_ONE + FT_POINTS_TWO];
	static unsigned complaints;

	seen.polls++;
	if (!ft_read(FT_REG_POINTS, report, FT_POINTS_ONE) ||
	    (report[FT_POINTS_ONE - 1] != 0xff && !ft_read(FT_REG_POINTS_N, report + FT_POINTS_ONE, FT_POINTS_TWO))) {
		seen.bad++;
		if (complaints++ < 8) {
			printf("ft3658u: the touch report could not be read\n");
		}
		return;
	}
	seen.reports++;
	for (uint32_t i = 0; i < FT_MAX_TOUCHES; i++) {
		contact[i].seen = false;
	}
	for (uint32_t i = 0; i < FT_MAX_TOUCHES; i++) {
		const uint8_t *t = report + 2 + 6 * i;
		uint32_t id = t[2] >> 4, event = t[0] >> 6, x, y;

		if (id >= FT_MAX_TOUCHES) {
			break;          /* no more fingers in this report */
		}
		if (x8) {
			x = (uint32_t)(t[0] & 0x0f) << 11 | (uint32_t)t[1] << 3 | (t[4] >> 5 & 7);
			y = (uint32_t)(t[2] & 0x0f) << 11 | (uint32_t)t[3] << 3 | (t[4] >> 2 & 7);
		} else {
			x = (uint32_t)(t[0] & 0x0f) << 8 | t[1];
			y = (uint32_t)(t[2] & 0x0f) << 8 | t[3];
		}
		contact_update(id, event, x, y);
	}
	/* a finger that is not in the report has lifted */
	for (uint32_t i = 0; i < FT_MAX_TOUCHES; i++) {
		if (!contact[i].seen && contact[i].down) {
			contact[i].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, i, contact[i].x, contact[i].y);
		}
	}
}

/* ---- all of it together --------------------------------------------------------------------------- */

static void
show_state(const char *when)
{
	printf("ft3658u: %s: votes %#x, clocks %#x %#x %#x, rate cfg %#x dfs %#x; pins sda/scl cfg %#x %#x, reset %#x/%u, "
	    "int %#x/%u\n", when, rd(GCC_VOTE), rd(GCC_QUP1_M_AHB_CBCR), rd(GCC_QUP1_S_AHB_CBCR), rd(GCC_QUP1_SE5_CBCR),
	    rd(GCC_QUP1_SE5_CFG), rd(GCC_QUP1_SE5_DFSR), rd(GPIO_CFG(PIN_SDA)), rd(GPIO_CFG(PIN_SCL)), rd(GPIO_CFG(PIN_RESET)),
	    rd(GPIO_IO(PIN_RESET)), rd(GPIO_CFG(PIN_INT)), rd(GPIO_IO(PIN_INT)));
}

/* Power the chip up from nothing, in realme's order. False if a supply could not be switched on. */
static bool
power_on(void)
{
	gpio_output(PIN_RESET, false, PULL_NONE, 2);
	gpio_configure(PIN_INT, 0, PULL_UP, 2, false);
	sleep_ms(1);
	/* without its supplies the chip must not come out of reset: stop at the first that does not switch on */
	if (!regulator_on("ldoc11", 3008) || !regulator_on("ldoc1", 1800)) {
		return false;
	}
	sleep_ms(10);
	gpio_set(PIN_RESET, true);
	sleep_ms(200);
	return true;
}

static bool
bring_up(void)
{
	const char *problem;
	uint32_t khz = 400;

	show_state("as found");
	if (!power_on()) {
		printf("ft3658u: the supplies are not on; the chip is left in reset\n");
		return false;
	}
	if (!clocks_on()) {
		return false;
	}
	printf("ft3658u: serial engine: firmware %#x, interface %#x, parameters %#x, clock select %u\n", rd(GENI_FW_REVISION_RO),
	    rd(GENI_IF_DISABLE_RO), rd(SE_HW_PARAM_0), se_clock_select);
	PE_parse_boot_argn("ft_i2c_khz", &khz, sizeof(khz));
	problem = geni_i2c_init(khz);
	if (problem) {
		printf("ft3658u: %s; not going on\n", problem);
		return false;
	}
	gpio_configure(PIN_SDA, PIN_FUNCTION_I2C, PULL_NONE, 2, false);
	gpio_configure(PIN_SCL, PIN_FUNCTION_I2C, PULL_NONE, 2, false);
	printf("ft3658u: the bus at %u kHz\n", khz <= 100 ? 100 : 400);
	show_state("powered, on the serial engine");
	return ft_probe();
}

static void
ft3658u_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 8000;
	unsigned idle = 0, idle_looks;

	PE_parse_boot_argn("ft_poll_us", &poll_us, sizeof(poll_us));
	if (poll_us < 1000 || poll_us > 100000) {
		poll_us = 8000;
	}
	idle_looks = 80000 / poll_us;   /* a report read every 80 ms even with the line high, in case it is missed */

	tlmm = ml_io_map(TLMM_SOUTH_PHYS, TLMM_SOUTH_SIZE);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	se = ml_io_map(SE_PHYS, SE_SIZE);
	rsc = ml_io_map(RSC_PHYS, RSC_SIZE);
	cmd_db = ml_io_map_wcomb(CMD_DB_PHYS, CMD_DB_SIZE);      /* memory, not registers: no alignment rules */
	if (!tlmm || !gcc || !se || !rsc || !cmd_db) {
		printf("ft3658u: could not map the registers; off\n");
		thread_terminate(current_thread());
		return;
	}
	if (!bring_up()) {
		printf("ft3658u: no touch screen\n");
		thread_terminate(current_thread());
		return;
	}
	printf("ft3658u: touch screen ready; the interrupt line is looked at every %u us\n", poll_us);
	for (unsigned tick = 1;; tick++) {
		if (!gpio_get(PIN_INT)) {
			idle = 0;
			ft_poll();
		} else if (++idle >= idle_looks) {
			idle = 0;
			ft_poll();
		}
		if (tick % (60 * (1000000 / poll_us)) == 0) {
			printf("ft3658u: so far %u reports in %u looks (%u bad), %u finger positions, the last at %u, %u; "
			    "interrupt line %s\n", seen.reports, seen.polls, seen.bad, seen.touches, seen.last_x, seen.last_y,
			    gpio_get(PIN_INT) ? "high" : "low");
		}
		sleep_us(poll_us);
	}
}

static thread_call_t start_call;

static void
ft3658u_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(ft3658u_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("ft3658u: could not start its thread\n");
	}
}

__startup_func
static void
ft3658u_init(void)
{
	uint32_t enabled = 0, value;
	uint64_t deadline;

	if (!PE_parse_boot_argn("ft3658u", &enabled, sizeof(enabled)) || !enabled) {
		return;                 /* nothing is touched unless the boot-argument asks */
	}
	if (PE_parse_boot_argn("ft_max_x", &value, sizeof(value)) && value > 1 && value <= 65536) {
		max_x = value;
	}
	if (PE_parse_boot_argn("ft_max_y", &value, sizeof(value)) && value > 1 && value <= 65536) {
		max_y = value;
	}
	PE_parse_boot_argn("ft_flip", &flip, sizeof(flip));
	PE_parse_boot_argn("ft_x8", &x8, sizeof(x8));
	if (x8 && !PE_parse_boot_argn("ft_max_x", &value, sizeof(value))) {
		max_x = 8640;
		max_y = 19200;
	}
	/* a little after boot, from a thread of its own: the touch queue and the rest of the kernel are up by then */
	start_call = thread_call_allocate(ft3658u_start, NULL);
	clock_interval_to_deadline(2, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start_call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, ft3658u_init);
