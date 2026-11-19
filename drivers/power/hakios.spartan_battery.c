/*
 * The battery of the realme GT Neo 3T (spartan, SM8250-AC), for the power source Darwin reads (hakios_battery_report() in
 * drivers/power/hakios.battery.c, then hakios.battery.cpp: the status bar's percentage, Settings > Battery). Only with
 * boot-arg spartan_battery=1: every address below belongs to this phone.
 *
 * The phone's Android reads a TI fuel gauge, "bq27541-battery" at I2C address 0x55 on i2c@888000 (/sys/bus/i2c/devices/
 * 10-0055, seen 19 November 2026), the same gauge and bus as the OnePlus 8 (instantnoodle_battery.c of the OnePlus 8 Pro
 * port, whose register map this follows). That bus is serial engine 16 of the SoC: QUP wrapper 2, engine 2, pins GPIO48 (data)
 * and GPIO49 (clock), function qup16, on the TLMM's south tile (pinctrl-sm8250.c). Its clocks (gcc-sm8250.c): the wrapper's
 * core, 2x core, master and slave AHB clocks and the engine's own, by vote in 0x52010 (bits 0, 3, 2, 1 and 6); the engine's
 * branch at 0x1e26c and its clock generator at 0x1e270. The engine is driven as drivers/input/hakios.ft3658u.c drives
 * serial engine 13 (FIFO mode, polled), at 100 kHz.
 *
 * Only reads: a register is read as a write of its number with the stop held back, then a read of two bytes. Nothing is
 * written to the gauge's control register, and nothing to the charger (the PM8150B is not touched at all), nor to the
 * fast-charge chip on the same bus (rk826 at 0x0a). Every 5 s; a line in the log every minute. Whether a charger is in is
 * taken from the current (more than 0: the battery takes charge).
 */
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

static vm_offset_t tlmm, gcc, se;

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

/* ---- pins (TLMM, south tile) --------------------------------------------------------------------- */

#define TLMM_SOUTH_PHYS         0x0f500000UL
#define TLMM_SOUTH_SIZE         0x100000UL
#define GPIO_CFG(n)             (tlmm + 0x1000UL * (n))         /* [1:0] pull, [5:2] function, [8:6] drive, [9] output */
#define GPIO_IO(n)              (tlmm + 0x1000UL * (n) + 4)     /* bit 0 in */
#define PULL_UP                 3
#define PIN_SDA                 48
#define PIN_SCL                 49
#define PIN_FUNCTION_I2C        1       /* "qup16" */

/* ---- clocks (GCC) -------------------------------------------------------------------------------- */

#define GCC_PHYS                0x00100000UL
#define GCC_SIZE                0x60000UL
#define GCC_VOTE2               (gcc + 0x52010)         /* the application processor's votes, third register */
#define VOTE_QUP2_CORE          (1u << 0)
#define VOTE_QUP2_S_AHB         (1u << 1)
#define VOTE_QUP2_M_AHB         (1u << 2)
#define VOTE_QUP2_CORE_2X       (1u << 3)
#define VOTE_QUP2_SE2           (1u << 6)
#define GCC_QUP2_M_AHB_CBCR     (gcc + 0x1e004)
#define GCC_QUP2_S_AHB_CBCR     (gcc + 0x1e008)
#define GCC_QUP2_SE2_CBCR       (gcc + 0x1e26c)
#define GCC_QUP2_SE2_CMD        (gcc + 0x1e270)
#define GCC_QUP2_SE2_CFG        (gcc + 0x1e274)
#define GCC_QUP2_SE2_DFSR       (gcc + 0x1e284)
#define GCC_QUP2_SE2_PERF_CFG(l) (gcc + 0x1e28c + 4 * (l))
#define GCC_QUP2_SE2_PERF_M(l)  (gcc + 0x1e2cc + 4 * (l))
#define GCC_QUP2_SE2_PERF_N(l)  (gcc + 0x1e30c + 4 * (l))
#define CLOCK_OFF               (1u << 31)
#define CLOCK_HW_CTL            (1u << 1)
#define XO_HZ                   19200000

static uint32_t se_clock_select;

static bool
branch_usable(uint32_t cbcr)
{
	return !(cbcr & CLOCK_OFF) || (cbcr & CLOCK_HW_CTL);
}

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

static bool
clocks_on(void)
{
	bool found = false;

	wr(GCC_VOTE2, rd(GCC_VOTE2) | VOTE_QUP2_CORE_2X | VOTE_QUP2_CORE | VOTE_QUP2_M_AHB | VOTE_QUP2_S_AHB);
	if (rd(GCC_QUP2_SE2_DFSR) & 1) {
		for (uint32_t level = 0; level < 8 && !found; level++) {
			if (rcg_rate(rd(GCC_QUP2_SE2_PERF_CFG(level)), rd(GCC_QUP2_SE2_PERF_M(level)),
			    rd(GCC_QUP2_SE2_PERF_N(level))) == XO_HZ) {
				se_clock_select = level;
				found = true;
			}
		}
		if (!found) {
			printf("spartan_battery: no preset clock rate of 19.2 MHz\n");
			return false;
		}
	} else {
		wr(GCC_QUP2_SE2_CFG, (rd(GCC_QUP2_SE2_CFG) & ~0x371fu) | 1);
		wr(GCC_QUP2_SE2_CMD, rd(GCC_QUP2_SE2_CMD) | 1);
		for (int i = 0; i < 500 && (rd(GCC_QUP2_SE2_CMD) & 1); i++) {
			sleep_us(10);
		}
		if (rd(GCC_QUP2_SE2_CMD) & 1) {
			printf("spartan_battery: the serial engine's clock does not take its configuration (%#x)\n", rd(GCC_QUP2_SE2_CMD));
			return false;
		}
		se_clock_select = 0;
	}
	wr(GCC_VOTE2, rd(GCC_VOTE2) | VOTE_QUP2_SE2);
	for (int i = 0; i < 1000; i++) {
		if (!(rd(GCC_QUP2_SE2_CBCR) & CLOCK_OFF) && branch_usable(rd(GCC_QUP2_M_AHB_CBCR)) &&
		    branch_usable(rd(GCC_QUP2_S_AHB_CBCR))) {
			printf("spartan_battery: clocks on: %#x %#x %#x (preset %u)\n", rd(GCC_QUP2_M_AHB_CBCR), rd(GCC_QUP2_S_AHB_CBCR),
			    rd(GCC_QUP2_SE2_CBCR), se_clock_select);
			return true;
		}
		sleep_us(10);
	}
	printf("spartan_battery: clocks stay off: %#x %#x %#x\n", rd(GCC_QUP2_M_AHB_CBCR), rd(GCC_QUP2_S_AHB_CBCR),
	    rd(GCC_QUP2_SE2_CBCR));
	return false;
}

/* ---- I2C through the serial engine (GENI), as hakios.ft3658u.c ------------------------------------ */

#define SE_PHYS                 0x00888000UL    /* i2c@888000 */
#define SE_SIZE                 0x4000UL
#define GENI_FORCE_DEFAULT      (se + 0x020)
#define GENI_OUTPUT_CTRL        (se + 0x024)
#define GENI_CGC_CTRL           (se + 0x028)
#define GENI_STATUS             (se + 0x040)
#define GENI_SER_M_CLK_CFG      (se + 0x048)
#define GENI_IF_DISABLE_RO      (se + 0x064)
#define GENI_FW_REVISION_RO     (se + 0x068)
#define GENI_CLK_SEL            (se + 0x07c)
#define GENI_BYTE_GRAN          (se + 0x254)
#define GENI_DMA_MODE_EN        (se + 0x258)
#define GENI_TX_PACKING_CFG0    (se + 0x260)
#define GENI_TX_PACKING_CFG1    (se + 0x264)
#define I2C_TX_TRANS_LEN        (se + 0x26c)
#define I2C_RX_TRANS_LEN        (se + 0x270)
#define I2C_SCL_COUNTERS        (se + 0x278)
#define GENI_RX_PACKING_CFG0    (se + 0x284)
#define GENI_RX_PACKING_CFG1    (se + 0x288)
#define GENI_M_CMD0             (se + 0x600)
#define GENI_M_CMD_CTRL         (se + 0x604)
#define GENI_M_IRQ_STATUS       (se + 0x610)
#define GENI_M_IRQ_EN           (se + 0x614)
#define GENI_M_IRQ_CLEAR        (se + 0x618)
#define GENI_S_IRQ_CLEAR        (se + 0x648)
#define GENI_TX_FIFO            (se + 0x700)
#define GENI_RX_FIFO            (se + 0x780)
#define GENI_TX_FIFO_STATUS     (se + 0x800)
#define GENI_RX_FIFO_STATUS     (se + 0x804)
#define GENI_TX_WATERMARK       (se + 0x80c)
#define GENI_RX_WATERMARK       (se + 0x810)
#define GENI_RX_RFR_WATERMARK   (se + 0x814)
#define SE_DMA_TX_IRQ_CLR       (se + 0xc44)
#define SE_DMA_RX_IRQ_CLR       (se + 0xd44)
#define SE_GSI_EVENT_EN         (se + 0xe18)
#define SE_IRQ_EN               (se + 0xe1c)
#define SE_HW_PARAM_0           (se + 0xe24)
#define SE_DMA_GENERAL_CFG      (se + 0xe30)

#define M_CMD_DONE              (1u << 0)
#define M_CMD_OVERRUN           (1u << 1)
#define M_ILLEGAL_CMD           (1u << 2)
#define M_CMD_FAILURE           (1u << 3)
#define M_CMD_CANCEL            (1u << 4)
#define M_CMD_ABORT             (1u << 5)
#define M_GP_IRQ_NACK           (1u << 10)
#define M_GP_IRQ_BUS_PROTO      (1u << 12)
#define M_GP_IRQ_ARB_LOST       (1u << 13)
#define M_ERRORS                (M_CMD_OVERRUN | M_ILLEGAL_CMD | M_CMD_FAILURE | M_GP_IRQ_NACK | M_GP_IRQ_BUS_PROTO | \
	                         M_GP_IRQ_ARB_LOST)

#define I2C_WRITE               (0x1u << 27)
#define I2C_READ                (0x2u << 27)
#define I2C_STOP_STRETCH        (1u << 2)
#define I2C_ADDRESS(a)          ((uint32_t)(a) << 9)

#define GAUGE_ADDRESS           0x55            /* bq27541-battery@55 */

static uint32_t fifo_depth;

static const char *
geni_i2c_init(void)
{
	/* i2c-qcom-geni.c's table for a 19.2 MHz source at 100 kHz: divider, high, low, cycle */
	uint32_t divider = 7, high = 10, low = 11, cycle = 26;

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
	wr(SE_IRQ_EN, rd(SE_IRQ_EN) | 0xf);
	wr(GENI_M_IRQ_EN, rd(GENI_M_IRQ_EN) | 0x7fc0007f | M_GP_IRQ_NACK | M_GP_IRQ_BUS_PROTO | M_GP_IRQ_ARB_LOST);
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
		wr(GENI_M_CMD0, I2C_WRITE | I2C_ADDRESS(GAUGE_ADDRESS) | (stretch ? I2C_STOP_STRETCH : 0));
	} else {
		wr(I2C_RX_TRANS_LEN, length);
		wr(GENI_M_CMD0, I2C_READ | I2C_ADDRESS(GAUGE_ADDRESS) | (stretch ? I2C_STOP_STRETCH : 0));
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
		printf("spartan_battery: I2C %s of %u bytes failed: status %#x%s, %u of them done\n", tx ? "write" : "read", length,
		    status, (status & M_GP_IRQ_NACK) ? " (no acknowledge)" : "", tx ? sent : received);
	}
	geni_recover();
	return false;
}

/* A 16-bit register of the gauge (little-endian): its number written with the stop held back, then two bytes read. */
static bool
gauge_word(uint8_t reg, int *value)
{
	uint8_t b[2];

	if (!geni_i2c_command(&reg, NULL, 1, true) || !geni_i2c_command(NULL, b, 2, false)) {
		return false;
	}
	*value = b[0] | b[1] << 8;
	return true;
}

/* ---- the gauge ------------------------------------------------------------------------------------ */

/* OnePlus's/realme's gauge_set_cmd_addr(): the bq27541's registers, or the bq27411's */
static struct {
	const char *name;
	uint8_t temp, volt, flags, rm, fcc, ai, soc, cycles;
} map = { "bq27541", 0x06, 0x08, 0x0a, 0x10, 0x12, 0x14, 0x2c, 0x2a };

#define FLAG_FC                 (1u << 9)

static bool
plausible(int soc, int mv, int temp_dk)
{
	return soc >= 0 && soc <= 100 && mv >= 2500 && mv <= 5000 && temp_dk >= 2330 && temp_dk <= 3630;
}

static int raw_soc = -1, raw_mv = -1, raw_temp = -1, raw_i2c = -1;     /* the last reading as it came, for the log */

static bool
sample(struct hakios_battery_state *b, int *flags_out)
{
	int soc = -1, mv = -1, ma = 0, temp_dk = -1, flags = 0, rm = 0, fcc = 0, cycles = 0;

	raw_i2c = gauge_word(map.soc, &soc) + 2 * gauge_word(map.volt, &mv) + 4 * gauge_word(map.ai, &ma) +
	    8 * gauge_word(map.temp, &temp_dk);
	raw_soc = soc;
	raw_mv = mv;
	raw_temp = temp_dk;
	if (raw_i2c != 15) {
		return false;
	}
	gauge_word(map.flags, &flags);
	gauge_word(map.rm, &rm);
	gauge_word(map.fcc, &fcc);
	if (map.cycles) {
		gauge_word(map.cycles, &cycles);
	}
	ma = (int16_t)ma;                               /* TI's sign: more than 0 while the battery takes charge */
	if (mv > 5500) {
		/*
		 * Two cells in series (19 November: the gauge, a ZY0603 in bq28z610 mode, said 8701 mV and 1658/1658 mAh where
		 * Android says 4.2 V and 3316 mAh full, 1414 cycles at 0x2a and 100% at 0x2c as here): a cell's voltage, the pack's
		 * charge in what the two hold, as Android shows them.
		 */
		mv /= 2;
		rm *= 2;
		fcc *= 2;
	}
	bzero(b, sizeof(*b));
	b->size = sizeof(*b);
	b->percent_hundredths = (soc > 100 ? 100 : soc) * 100;
	b->voltage_uv = mv * 1000;
	b->current_ua = ma * 1000;
	b->temperature_hundredths = temp_dk * 10 - 27315;
	b->present = 1;
	b->external = ma > 0;
	b->charge_uah = rm * 1000;
	b->full_uah = fcc * 1000;
	b->cycles = cycles < 10000 ? cycles : 0;
	b->status = !b->external ? 2 : (soc >= 100 || (flags & FLAG_FC)) ? 4 : 1;
	*flags_out = flags;
	return plausible(soc, mv, temp_dk);
}

static void
battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct hakios_battery_state b;
	const char *why;
	unsigned failed = 0, reported = 0;
	int flags = 0;

	tlmm = ml_io_map(TLMM_SOUTH_PHYS, TLMM_SOUTH_SIZE);
	gcc = ml_io_map(GCC_PHYS, GCC_SIZE);
	se = ml_io_map(SE_PHYS, SE_SIZE);
	if (tlmm == 0 || gcc == 0 || se == 0) {
		printf("spartan_battery: registers not mapped\n");
		return;
	}
	printf("spartan_battery: pins as the boot chain left them: gpio%u cfg %#x in %u, gpio%u cfg %#x in %u; vote %#x\n", PIN_SDA,
	    rd(GPIO_CFG(PIN_SDA)), rd(GPIO_IO(PIN_SDA)) & 1, PIN_SCL, rd(GPIO_CFG(PIN_SCL)), rd(GPIO_IO(PIN_SCL)) & 1, rd(GCC_VOTE2));
	if (!clocks_on()) {
		return;
	}
	/* both lines to the engine, pulled up, 2 mA (0x7, as the OnePlus 8's boot chain leaves the same pins) */
	wr(GPIO_CFG(PIN_SDA), PULL_UP | PIN_FUNCTION_I2C << 2);
	wr(GPIO_CFG(PIN_SCL), PULL_UP | PIN_FUNCTION_I2C << 2);
	if ((why = geni_i2c_init()) != NULL) {
		printf("spartan_battery: %s (firmware %#x)\n", why, rd(GENI_FW_REVISION_RO));
		return;
	}
	printf("spartan_battery: serial engine 16 ready (firmware %#x, FIFO %u words); the gauge at %#x\n", rd(GENI_FW_REVISION_RO),
	    fifo_depth, GAUGE_ADDRESS);

	for (unsigned tick = 0;; tick++) {
		bool good = sample(&b, &flags);

		if (tick % 120 == 0) {
			/* (every ten minutes) every 16-bit register from 0x00 to 0x3e as the gauge gives it (reads only): which map this gauge has */
			char dump[400];
			unsigned n = 0;

			for (uint8_t reg = 0; reg < 0x40 && n < sizeof(dump); reg += 2) {
				int value = -1;

				n += snprintf(dump + n, sizeof(dump) - n, " %02x:%04x", reg, gauge_word(reg, &value) ? value & 0xffff : 0xdead);
			}
			printf("spartan_battery: registers%s\n", dump);
		}
		if (!good && tick == 0 && map.soc == 0x2c && raw_i2c == 15) {
			/* not a bq27541's numbers: the bq27411's map, once */
			map.name = "bq27411";
			map.temp = 0x02; map.volt = 0x04; map.flags = 0x06; map.rm = 0x0c; map.fcc = 0x0e;
			map.ai = 0x10; map.soc = 0x1c; map.cycles = 0;
			good = sample(&b, &flags);
		}
		if (good) {
			failed = 0;
			hakios_battery_report(&b);
			if (reported++ == 0 || tick % 12 == 0) {
				printf("spartan_battery: %s: %d%%, %d mV, %d mA, %d.%02d C, %d/%d mAh, %d cycles, flags %#x\n", map.name,
				    b.percent_hundredths / 100, b.voltage_uv / 1000, b.current_ua / 1000, b.temperature_hundredths / 100,
				    (b.temperature_hundredths < 0 ? -b.temperature_hundredths : b.temperature_hundredths) % 100,
				    b.charge_uah / 1000, b.full_uah / 1000, b.cycles, flags);
			}
		} else if (failed++ < 3 || tick % 12 == 0) {
			printf("spartan_battery: no plausible reading (%s map, %u in a row): registers answered %#x of 0xf, soc %d, %d mV, "
			    "%d dK; engine fw %#x, vote %#x, cbcr %#x; pins cfg %#x %#x, lines %u %u\n", map.name, failed, raw_i2c, raw_soc,
			    raw_mv, raw_temp, rd(GENI_FW_REVISION_RO), rd(GCC_VOTE2), rd(GCC_QUP2_SE2_CBCR), rd(GPIO_CFG(PIN_SDA)),
			    rd(GPIO_CFG(PIN_SCL)), rd(GPIO_IO(PIN_SDA)) & 1, rd(GPIO_IO(PIN_SCL)) & 1);
		}
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

static thread_call_t start_call;

static void
battery_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(battery_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
spartan_battery_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("spartan_battery", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	start_call = thread_call_allocate(battery_start, NULL);
	clock_interval_to_deadline(3, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start_call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, spartan_battery_init);
