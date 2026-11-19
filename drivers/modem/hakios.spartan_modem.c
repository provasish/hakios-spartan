/*
 * The OnePlus 8 Pro's cellular modem and the bus it is on. The SM8250 has no modem of its own: the phone's is a Snapdragon X55 (SDX55M), a chip
 * of its own that Android's device tree calls qcom,mdm0 (compatible qcom,ext-sdx55m) and reaches through PCIe 2 of the SoC (qcom,pcie@1c10000, endpoint
 * "esoc0": an MHI device). Its firmware is the phone's own (sdx55m/ in the modem partition's image directory: sbl1.mbn, tz.mbn, hyp.mbn, aop.mbn,
 * apps.mbn, qdsp6sw.mbn ...), which the application processor hands over: sbl1.mbn through the MHI's boot interface (BHI), the rest through the
 * Sahara protocol on MHI channels once sbl1 runs; then the modem resets itself and comes up as an MHI device in "mission mode", whose QRTR/QMI
 * services are what the radio is driven with (SIM, network, SMS, calls) and whose IP channels carry the data.
 *
 * What Android does on this phone (its kernel log of a boot, op8pro-data/android-logs/kmsg.txt, and its device tree): when the radio interface layer
 * asks for the subsystem "esoc0", the SoC's PCIe 2 is enabled ("Assert the reset of endpoint of RC2", "PHY is ready", "Release the reset", "link
 * initialized" 16 ms after the release: the modem's boot ROM is already waiting), the device 0002:01:00.0 gets its BAR 0 at 0x64300000 (4 KB), sbl1.mbn
 * is loaded, 2.6 s later PCIe 2 is taken down and brought up again (the modem has restarted into the images the Sahara conversation gave it) and the
 * rmnet interface and QMI's connection to the modem's SSCTL service follow.
 *
 * This file does the first part of that on the OnePlus 8 Pro, in steps behind boot-arg modem=N (nothing is touched unless it is set; board=instantnoodle only):
 *   1  reads   the six GPIOs of the modem's control lines and of PCIe 2 (PERST 85, CLKREQ 86, WAKE 87; AP2MDM status 56 and error 57, MDM2AP error 1 and status 3), and
 *              the state of PCIe 2's power domain and clocks in GCC; writes nothing
 *   2  writes  PCIe 2 up in the order of instantnoodle_wifi.c (the radio's PCIe 0), with PCIe 2's numbers from the device tree: PERST held, power domain,
 *              clocks, resets, PARF, the PHY's sequence of 136 writes, PERST let go, the link; then reads the modem's MHI registers (BAR 0) and its boot interface
 *              (BHI): which execution environment it is in (PBL, SBL, mission mode), its serial number; then takes PCIe 2 down again (modem=2) or leaves it (modem_keep=1)
 *   3  writes  as step 2, and: the modem's stream (0x1d01) of the application processors' SMMU sent through a context with page tables of ours, a DMA window of
 *              32 MB at 0x20000000, sbl1.mbn read from the phone's own modem partition (modem_b of LUN 4, FAT16, read only: bsd/dev/instantnoodle_ufs.c), and
 *              handed to the boot ROM through BHI; then the execution environment and the boot interface's status are watched (the modem should go from PBL to SBL)
 *   4  writes  as step 3, but the boot by hakios.mhi.c (a copy of the Wi-Fi's MHI host): BHI, READY, M0, the SBL's channels 2 and 3 started, and the Sahara
 *              protocol answered on them from the same partition (image ids to files: sahara_files[]); it stops when the modem goes away, has all its images, or modem_secs (60) pass
 * The next steps (mission mode after the modem's restart, QRTR, QMI) are in docs/TODO.md T06.
 *
 * A register of PCIe 2 is not read before its domain and clocks are on: on this SoC that is a NoC error and Qualcomm's crash dump mode.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/errno.h>
#include <sys/hakios.input.h>
#include <sys/hakios.mhi.h>
#include <sys/hakios.net.h>
#include <stdarg.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <vm/vm_kern.h>

extern uint64_t kvtophys(vm_offset_t va);
extern lck_mtx_t hakios_smmu_lock;                  /* (drivers/hakios.wifi.c) whoever looks for a free match register and context of the apps SMMU takes it */
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern int hakios_wifi_tell(const char *format, ...) __printflike(1, 2);
extern kern_return_t thread_terminate(thread_t thread);

static uint64_t mdm_t0;
static void
mdm_say(const char *format, ...) __printflike(1, 2);
static void
mdm_say(const char *format, ...)
{
	char line[420];
	va_list ap;
	uint64_t abs, ns;
	int n;

	clock_get_uptime(&abs);
	absolutetime_to_nanoseconds(abs, &ns);
	if (mdm_t0 == 0) {
		mdm_t0 = ns;
	}
	va_start(ap, format);
	n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	(void)n;
	if (strncmp(line, "modem: ", 7) == 0) {
		hakios_wifi_tell("modem: [%6llu ms] %s", (ns - mdm_t0) / 1000000, line + 7);
	} else {
		hakios_wifi_tell("%s", line);
	}
}
#define say mdm_say

/*
 * ---- spartan: what the OnePlus 8 port took from its board's shared files (sys/instantnoodle.h) ----------------------
 * The pins are in the TLMM's south tile as on the OnePlus 8 (spartan's device tree: qcom,mdm0 and qcom,pcie@1c10000 name the
 * same GPIOs 1, 3, 56, 57, 85, 86, 87); RPMh votes go through command slot set 0 under hakios_rsc_lock, as the other drivers'.
 * The audio DSP's voice bridge (hakios_adsp_*) is not here: those calls answer "no".
 */
#define INSTANTNOODLE_TLMM_SOUTH        0x0f500000UL
#define INSTANTNOODLE_PULL_NONE         0
#define INSTANTNOODLE_PULL_DOWN         1
#define INSTANTNOODLE_PULL_UP           3
struct instantnoodle_pin {
	uint64_t tile;
	unsigned pin;
};
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern lck_mtx_t hakios_rsc_lock;

static vm_offset_t spartan_pin_page[200];

static vm_offset_t
spartan_pin_base(struct instantnoodle_pin p)
{
	if (p.pin >= 200) {
		return 0;
	}
	if (spartan_pin_page[p.pin] == 0) {
		spartan_pin_page[p.pin] = ml_io_map(p.tile + 0x1000UL * p.pin, 0x1000);
	}
	return spartan_pin_page[p.pin];
}

static uint32_t
instantnoodle_gpio_cfg(struct instantnoodle_pin p)
{
	vm_offset_t b = spartan_pin_base(p);

	return b ? *(volatile uint32_t *)b : 0;
}

static bool
instantnoodle_gpio_level(struct instantnoodle_pin p)
{
	vm_offset_t b = spartan_pin_base(p);

	return b && (*(volatile uint32_t *)(b + 4) & 1);
}

static void
instantnoodle_gpio_input(struct instantnoodle_pin p, unsigned pull)
{
	vm_offset_t b = spartan_pin_base(p);

	if (b) {
		*(volatile uint32_t *)b = pull | 0u << 6;
		__asm__ volatile ("dsb sy" ::: "memory");
	}
}

static void
instantnoodle_gpio_output(struct instantnoodle_pin p, bool high, unsigned pull, unsigned ma)
{
	vm_offset_t b = spartan_pin_base(p);

	if (b) {
		*(volatile uint32_t *)(b + 4) = high ? 2 : 0;   /* the level first, so that the pin never drives the wrong one */
		*(volatile uint32_t *)b = pull | (ma / 2 - 1) << 6 | 1u << 9;
		__asm__ volatile ("dsb sy" ::: "memory");
	}
}

static void
instantnoodle_gpio_function(struct instantnoodle_pin p, unsigned function, unsigned pull, unsigned ma)
{
	vm_offset_t b = spartan_pin_base(p);

	if (b) {
		*(volatile uint32_t *)b = pull | (function & 0xf) << 2 | (ma / 2 - 1) << 6;
		__asm__ volatile ("dsb sy" ::: "memory");
	}
}

static bool
instantnoodle_board(void)
{
	return true;                    /* (this file is built only into spartan's kernel) */
}

/* RPMh: one write through command slot set 0 (as hakios.spartan_wifi.c), for the PCIe NoC's bus clock manager */
#define SP_CMD_DB_PHYS          0x80860000UL
#define SP_CMD_DB_SIZE          0x20000UL
#define SP_RSC_PHYS             0x18220000UL
#define SP_TCS(reg)             (sp_rsc + 0xd00 + (reg))
static vm_offset_t sp_rsc, sp_cmd_db;

static uint32_t
sp_rpmh_address(const char *name)
{
	struct sp_slave { uint16_t id, header_offset, data_offset, count, version, reserved[3]; };
	struct sp_header { uint32_t version; uint8_t magic[4]; struct sp_slave slave[8]; uint32_t checksum, reserved; uint8_t data[]; };
	struct sp_entry { char id[8]; uint32_t priority[2]; uint32_t address; uint16_t length, offset; };
	const struct sp_header *db;
	char id[8] = { 0 };

	if (sp_cmd_db == 0) {
		sp_cmd_db = ml_io_map_wcomb(SP_CMD_DB_PHYS, SP_CMD_DB_SIZE);
	}
	db = (const struct sp_header *)sp_cmd_db;
	if (db == NULL || db->magic[0] != 0xdb || db->magic[1] != 0x30 || db->magic[2] != 0x03 || db->magic[3] != 0x0c) {
		return 0;
	}
	strlcpy(id, name, sizeof(id));
	for (unsigned s = 0; s < 8; s++) {
		const struct sp_entry *entry = (const struct sp_entry *)(db->data + db->slave[s].header_offset);

		if (db->slave[s].id == 0 || db->slave[s].header_offset >= SP_CMD_DB_SIZE - 0x1000 || db->slave[s].count > 512) {
			continue;
		}
		for (unsigned i = 0; i < db->slave[s].count; i++) {
			if (memcmp(entry[i].id, id, sizeof(id)) == 0) {
				return entry[i].address;
			}
		}
	}
	return 0;
}

static bool
instantnoodle_bcm_vote(const char *name, uint32_t x, uint32_t y)
{
	uint32_t base = sp_rpmh_address(name), irq_enable;
	uint32_t data = (x | y) == 0 ? 1u << 30 : 1u << 30 | 1u << 29 | (x & 0x3fff) << 14 | (y & 0x3fff);
	bool done = false;

	if (base == 0 || (base >> 16 & 0xf) != 5) {
		say("modem: RPMh: no BCM \"%s\" (address %#x)\n", name, base);
		return false;
	}
	if (sp_rsc == 0) {
		sp_rsc = ml_io_map(SP_RSC_PHYS, 0x10000);
	}
	lck_mtx_lock(&hakios_rsc_lock);
	irq_enable = *(volatile uint32_t *)SP_TCS(0x00);
	for (int i = 0; *(volatile uint32_t *)SP_TCS(0x18) == 0 && i < 2000; i++) {
		delay_for_interval(100, NSEC_PER_USEC);
	}
	*(volatile uint32_t *)SP_TCS(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)SP_TCS(0x14) &= ~(1u << 16);
	*(volatile uint32_t *)SP_TCS(0x08) = 1;
	*(volatile uint32_t *)SP_TCS(0x30) = 0x10008 | 0x100;
	*(volatile uint32_t *)SP_TCS(0x34) = base;
	*(volatile uint32_t *)SP_TCS(0x38) = data;
	*(volatile uint32_t *)SP_TCS(0x10) = 1;
	*(volatile uint32_t *)SP_TCS(0x1c) = 1;
	*(volatile uint32_t *)SP_TCS(0x00) = irq_enable | 1;
	*(volatile uint32_t *)SP_TCS(0x14) = 1u << 16;
	*(volatile uint32_t *)SP_TCS(0x14) = 1u << 16 | 1u << 24;
	for (int i = 0; i < 2000 && !done; i++) {
		done = (*(volatile uint32_t *)SP_TCS(0x04) & 1) != 0;
		if (!done) {
			delay_for_interval(100, NSEC_PER_USEC);
		}
	}
	*(volatile uint32_t *)SP_TCS(0x14) &= ~(1u << 24);
	*(volatile uint32_t *)SP_TCS(0x14) &= ~(1u << 16);
	*(volatile uint32_t *)SP_TCS(0x1c) = 0;
	*(volatile uint32_t *)SP_TCS(0x10) = 0;
	*(volatile uint32_t *)SP_TCS(0x08) = 1;
	*(volatile uint32_t *)SP_TCS(0x00) = irq_enable;
	lck_mtx_unlock(&hakios_rsc_lock);
	return done;
}

/* the audio DSP's side of a call (the OnePlus 8's hk_adsp.c): not on spartan yet */
void hakios_adsp_qrtr_from_modem(uint32_t type, uint32_t from_node, uint32_t from_port, uint32_t to_node, uint32_t to_port,
    bool confirm, const void *data, uint32_t length);
void
hakios_adsp_qrtr_from_modem(__unused uint32_t type, __unused uint32_t from_node, __unused uint32_t from_port,
    __unused uint32_t to_node, __unused uint32_t to_port, __unused bool confirm, __unused const void *data, __unused uint32_t length)
{
}
void hakios_adsp_qrtr_announce(void);
void
hakios_adsp_qrtr_announce(void)
{
}
uint32_t hakios_adsp_ssctl_port(void);
uint32_t
hakios_adsp_ssctl_port(void)
{
	return 0;
}
bool hakios_adsp_sat_open(void);
bool
hakios_adsp_sat_open(void)
{
	return false;
}
int hakios_adsp_sat_send(const void *data, uint32_t length);
int
hakios_adsp_sat_send(__unused const void *data, __unused uint32_t length)
{
	return ENXIO;
}

#define PIN(n)                ((struct instantnoodle_pin){ INSTANTNOODLE_TLMM_SOUTH, (n) })     /* pinctrl-kona.c: all of these are in the south tile */
#define PIN_MDM2AP_ERRFATAL     PIN(1)
#define PIN_MDM2AP_STATUS       PIN(3)
#define PIN_AP2MDM_STATUS       PIN(56)
#define PIN_AP2MDM_ERRFATAL     PIN(57)
#define PIN_PERST               PIN(85)
#define PIN_CLKREQ              PIN(86)
#define PIN_WAKE                PIN(87)
#define FUNCTION_PCI_E2         1

/* GCC (0x100000), a page at a time (offsets of gcc-kona.c) */
#define GCC_PHYS                0x00100000ul
#define GCC_VOTE                0x52010         /* APCS's branch votes: bit 10 slv_q2a, 11 slv_axi, 12 mstr_axi, 13 cfg_ahb, 14 aux, 15 pipe (PCIe 2's) */
#define GCC_PCIE_2_BCR          0x06000
#define GCC_PCIE_2_GDSCR        0x06004
#define GCC_PCIE_2_SLV_Q2A_CBCR 0x06010
#define GCC_PCIE_2_SLV_AXI_CBCR 0x06014
#define GCC_PCIE_2_MSTR_AXI_CBCR 0x0601c
#define GCC_PCIE_2_CFG_AHB_CBCR 0x06024
#define GCC_PCIE_2_AUX_CBCR     0x06028
#define GCC_PCIE_2_PIPE_CBCR    0x0602c
#define GCC_PCIE_2_AUX_RCG      0x06038         /* command, then configuration at +4 */
#define GCC_PCIE_2_PHY_BCR      0x1f01c
#define GCC_PCIE_REFGEN_RCG     0x6f014
#define GCC_PCIE2_REFGEN_CBCR   0x6f034         /* gcc_pcie2_phy_refgen_clk (PCIe 0's is 0x6f02c) */
#define GCC_PCIE_MDM_CLKREF     0x8c00c         /* gcc_pcie_mdm_clkref_en: the modem's reference clock buffer (dt: pcie_2_ldo) */
#define GCC_DDRSS_PCIE_SF_TBU   0x8d058
#define GCC_AGGRE_NOC_PCIE_TBU  0x9000c
#define CBCR_OFF                (1u << 31)
#define GDSC_COLLAPSE           (1u << 0)
#define GDSC_ON                 (1u << 31)
#define RCG_UPDATE              (1u << 0)
#define VOTE_SLV_Q2A            (1u << 10)
#define VOTE_SLV_AXI            (1u << 11)
#define VOTE_MSTR_AXI           (1u << 12)
#define VOTE_CFG_AHB            (1u << 13)
#define VOTE_AUX                (1u << 14)
#define VOTE_PIPE               (1u << 15)
#define VOTES_ALL               0xfc00u

/* PCIe 2 (dt: reg of qcom,pcie@1c10000) */
#define PARF_PHYS               0x01c10000ul    /* 0x3000 */
#define PHY_PHYS                0x01c16000ul
#define DBI_PHYS                0x64000000ul    /* the root complex's own configuration space, and ELBI at 0xf20 */
#define IATU_PHYS               0x64001000ul
#define CONF_PHYS               0x64100000ul    /* the device's configuration space, through iATU region 0 */
#define BAR_PHYS                0x64300000ul    /* the memory window: 0x64300000..0x67ffffff (dt ranges) */
#define BAR_WINDOW              0x03d00000ul
#define PARF_SYS_CTRL           0x000
#define PARF_PM_CTRL            0x020
#define PARF_PHY_CTRL           0x040
#define PARF_MHI_CLOCK_RESET_CTRL 0x174
#define PARF_AXI_MSTR_WR_ADDR_HALT 0x1a8
#define PARF_LTSSM              0x1b0
#define PARF_INT_ALL_MASK       0x22c
#define PARF_DBI_BASE_ADDR      0x350
#define PARF_SLV_ADDR_SPACE_SIZE 0x358
#define PARF_DEVICE_TYPE        0x1000
#define PARF_BDF_TO_SID         0x2000
#define ELBI_SYS_STTS           0xf28
#define XMLH_LINK_UP            0x400
#define DBI_LINK_STATUS         0x080           /* the express capability's link control and status (0x70 + 0x10) */
#define DBI_DEVICE_CONTROL      0x078
#define DBI_LINK_CONTROL2       0x0a0
#define DBI_SPCIE_CAP           0x154
#define DBI_ACK_F_ASPM_CTRL     0x70c
#define DBI_GEN2_CTRL           0x80c
#define DBI_GEN3_RELATED        0x890
#define DBI_MISC_CONTROL        0x8bc
#define DBI_AUX_CLK_FREQ        0xb40
#define IATU_CTRL1              0x00
#define IATU_CTRL2              0x04
#define IATU_LBAR               0x08
#define IATU_UBAR               0x0c
#define IATU_LAR                0x10
#define IATU_LTAR               0x14
#define IATU_UTAR               0x18
#define PHY_STATUS              0xa14           /* dt qcom,phy-status-offset: bit 6 set: not ready */
#define PHY_POWER_DOWN          0xa40           /* dt qcom,phy-power-down-offset */
#define SID_BASE                0x1d00
#define EP_BDF                  0x0100          /* bus 1, device 0, function 0 */

/* The PHY's sequence (dt: qcom,phy-sequence of qcom,pcie@1c10000, none of its delays is set): offset, value. */
static const uint16_t phy_sequence[][2] = {
	{ 0xa40, 0x03 }, { 0x010, 0x01 }, { 0x01c, 0x31 }, { 0x020, 0x01 }, { 0x024, 0xde }, { 0x028, 0x07 }, { 0x030, 0x4c },
	{ 0x034, 0x06 }, { 0x048, 0x90 }, { 0x058, 0x0f }, { 0x074, 0x06 }, { 0x078, 0x06 }, { 0x07c, 0x16 }, { 0x080, 0x16 },
	{ 0x084, 0x36 }, { 0x088, 0x36 }, { 0x094, 0x08 }, { 0x0a4, 0x42 }, { 0x0ac, 0x0a }, { 0x0b0, 0x1a }, { 0x0b4, 0x14 },
	{ 0x0b8, 0x34 }, { 0x0bc, 0x82 }, { 0x0c4, 0x68 }, { 0x0cc, 0x55 }, { 0x0d0, 0x55 }, { 0x0d4, 0x03 }, { 0x0d8, 0xab },
	{ 0x0dc, 0xaa }, { 0x0e0, 0x02 }, { 0x10c, 0x02 }, { 0x110, 0x24 }, { 0x118, 0xb4 }, { 0x11c, 0x03 }, { 0x154, 0x34 },
	{ 0x158, 0x01 }, { 0x16c, 0x08 }, { 0x1ac, 0xca }, { 0x1b0, 0x1e }, { 0x1b4, 0xa2 }, { 0x1b8, 0x18 }, { 0x1bc, 0x11 },
	{ 0x23c, 0x11 }, { 0x284, 0x75 }, { 0x29c, 0x12 }, { 0x304, 0x02 }, { 0x408, 0x0c }, { 0x414, 0x03 }, { 0x434, 0x7f },
	{ 0x444, 0x70 }, { 0x460, 0x30 }, { 0x4d4, 0x04 }, { 0x4d8, 0x07 }, { 0x4dc, 0x1b }, { 0x4e8, 0x04 }, { 0x4ec, 0x0e },
	{ 0x4f0, 0x4a }, { 0x4f4, 0x0f }, { 0x4f8, 0xc0 }, { 0x4fc, 0x00 }, { 0x510, 0x17 }, { 0x518, 0x1c }, { 0x51c, 0x03 },
	{ 0x524, 0x1e }, { 0x570, 0xbf }, { 0x574, 0x3f }, { 0x578, 0xff }, { 0x57c, 0x7f }, { 0x580, 0x15 }, { 0x584, 0x24 },
	{ 0x588, 0xe4 }, { 0x58c, 0xec }, { 0x590, 0x3b }, { 0x594, 0x36 }, { 0x598, 0xd4 }, { 0x59c, 0x54 }, { 0x5a0, 0xdb },
	{ 0x5a4, 0x3b }, { 0x5a8, 0x31 }, { 0x5bc, 0x0c }, { 0x5b8, 0x38 }, { 0x63c, 0x11 }, { 0x684, 0x75 }, { 0x69c, 0x12 },
	{ 0x704, 0x20 }, { 0x808, 0x0c }, { 0x814, 0x03 }, { 0x834, 0x7f }, { 0x844, 0x70 }, { 0x860, 0x30 }, { 0x8d4, 0x04 },
	{ 0x8d8, 0x07 }, { 0x8dc, 0x1b }, { 0x8e8, 0x04 }, { 0x8ec, 0x0e }, { 0x8f0, 0x4a }, { 0x8f4, 0x0f }, { 0x8f8, 0xc0 },
	{ 0x8fc, 0x00 }, { 0x910, 0x17 }, { 0x918, 0x1c }, { 0x91c, 0x03 }, { 0x924, 0x1e }, { 0x970, 0xbf }, { 0x974, 0x3f },
	{ 0x978, 0xff }, { 0x97c, 0x7f }, { 0x980, 0x15 }, { 0x984, 0x24 }, { 0x988, 0xe4 }, { 0x98c, 0xec }, { 0x990, 0x3b },
	{ 0x994, 0x36 }, { 0x998, 0xd4 }, { 0x99c, 0x54 }, { 0x9a0, 0xdb }, { 0x9a4, 0x3b }, { 0x9a8, 0x31 }, { 0x9bc, 0x0c },
	{ 0x9b8, 0x38 }, { 0xadc, 0x05 }, { 0xb88, 0x77 }, { 0xb98, 0x0b }, { 0xba4, 0x01 }, { 0xbe0, 0x0f }, { 0xe0c, 0x0d },
	{ 0xe14, 0x07 }, { 0xe1c, 0xc1 }, { 0xe40, 0x01 }, { 0xe48, 0x01 }, { 0xe90, 0x00 }, { 0xeb4, 0x33 }, { 0xebc, 0x00 },
	{ 0xee0, 0x58 }, { 0xa00, 0x00 }, { 0xa44, 0x03 },
};

static unsigned option_chans = 0x881;   /* modem_chans=MASK: which channel pairs are started (LOOPBACK, DIAG, QDSS, EFS, QMI0, QMI1, IP_CTRL, IPCR, TF, DCI, DUN, IP_HW0: bits 0..11): LOOPBACK, IPCR and IP_HW0 */
static bool option_ipa;                 /* modem_ipa=1: the IPA's QMI handshake as the application processor's side (our service announced, INIT_MODEM_DRIVER sent) */
static bool option_efs;                 /* modem_efs=1: the EFS images (Sahara ids 16, 17, 20) are the phone's mdm1m9kefs partitions, not the firmware's empty stubs */
static bool option_service;             /* modem_service=1: after the registration the thread stays, keeps the sysctls kern.hakios.modem_* true and sends/receives SMS */
static bool cell_started;
static bool qmi_find(uint32_t service, uint32_t *node, uint32_t *port);
/* the modem's latest position report (QMI LOC 0x0024, its TLVs as they came; the modem's GNSS: userland/src/hakios_location.m starts it, reads this and tells iOS) */
static struct {
	uint32_t count, length;
	uint8_t data[512];
} gps_report, gps_sats;
static volatile uint32_t modem_generation;      /* how many runs of the modem have come up: what was set up for it (hakios.mic.c's mailbox) is set up again */
static volatile bool cell_mt_pending;  /* a new message is waiting in the card or the modem's memory (WMS event report) */
static uint8_t cell_mt_storage;
static uint32_t cell_mt_index;
static volatile bool cell_route_pending;   /* a message came in the event report itself (WMS transfer route): decoded and acknowledged by the service loop */
static bool cell_route_ack;
static uint32_t cell_route_tid, cell_route_len;
static uint8_t cell_route_format, cell_route_pdu[200];
static struct {
	char state[120];                /* "off", "starting", "registered LTE", ... */
	int bars;                       /* 0..4 */
	int dbm;
	int registered;
	char operator_name[64];
	char qmi_request[420];          /* "SERVICE MSGID PORT HEXBYTES" written to kern.hakios.modem_qmi: a QMI request typed by hand, for experiments */
	volatile bool qmi_pending;
	char qmi_answer[1800];
	char sms_request[700];          /* "NUMBER TEXT" written to kern.hakios.modem_sms_send, taken by the modem's thread */
	volatile bool sms_pending;
	char sms_status[120];
	char sms_last[240];             /* the latest message received: "NUMBER: TEXT" */
	uint32_t sms_received;
	char sms_ring[8][240];          /* the last eight messages received, entry n % 8 for the n-th: "NUMBER: TEXT" (kern.hakios.modem_sms_ring) */
	bool event_report_on;
	uint64_t next_poll;
} cell = { .state = "off", .sms_status = "none", .sms_last = "" };
static bool option_sms_self;            /* modem_sms_self=1: one test SMS to the SIM's own number once registered */
static bool option_sms;                 /* modem_sms=1: one test SMS to the owner once registered */
static bool option_bringup;             /* modem_bringup=1: the radio brought online and the SIM and network watched */
static bool option_hw;                  /* (unused) */
static unsigned option_lanes;           /* modem_lanes=2: the controller told it has two lanes */
static LCK_GRP_DECLARE(modem_pcie_lck_grp, "instantnoodle-modem");
static LCK_MTX_DECLARE(modem_pcie_lock, &modem_pcie_lck_grp);

static struct {
	vm_offset_t gcc06, gcc52, gcc1f, gcc6f, gcc8c, gcc8d, gcc90;
	vm_offset_t parf, phy, dbi, iatu, conf, bar;
	bool mapped;
	bool clocked;                   /* PCIe 2's domain and clocks: its registers may be touched */
	volatile bool up;               /* the link: the device's configuration space may be touched */
	uint64_t bar_size;
	uint32_t bhi;                   /* the boot interface's offset in BAR 0 */
} pcie;

static inline uint32_t
rd(vm_offset_t address)
{
	return *(volatile uint32_t *)address;
}

static inline void
wr(vm_offset_t address, uint32_t value)
{
	*(volatile uint32_t *)address = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
set_bits(vm_offset_t address, uint32_t clear, uint32_t set)
{
	wr(address, (rd(address) & ~clear) | set);
	(void)rd(address);
}

static void
sleep_us(uint32_t us)
{
	delay_for_interval(us, NSEC_PER_USEC);
}

static vm_offset_t
gcc(uint32_t offset)
{
	vm_offset_t page = (offset & ~0xfffu) == 0x06000 ? pcie.gcc06 : (offset & ~0xfffu) == 0x52000 ? pcie.gcc52 :
	    (offset & ~0xfffu) == 0x1f000 ? pcie.gcc1f : (offset & ~0xfffu) == 0x6f000 ? pcie.gcc6f :
	    (offset & ~0xfffu) == 0x8c000 ? pcie.gcc8c : (offset & ~0xfffu) == 0x8d000 ? pcie.gcc8d : pcie.gcc90;

	return page + (offset & 0xfff);
}

static bool
map_gcc(void)
{
	if (pcie.gcc06 != 0) {
		return true;
	}
	pcie.gcc06 = ml_io_map(GCC_PHYS + 0x06000, 0x1000);
	pcie.gcc52 = ml_io_map(GCC_PHYS + 0x52000, 0x1000);
	pcie.gcc1f = ml_io_map(GCC_PHYS + 0x1f000, 0x1000);
	pcie.gcc6f = ml_io_map(GCC_PHYS + 0x6f000, 0x1000);
	pcie.gcc8c = ml_io_map(GCC_PHYS + 0x8c000, 0x1000);
	pcie.gcc8d = ml_io_map(GCC_PHYS + 0x8d000, 0x1000);
	pcie.gcc90 = ml_io_map(GCC_PHYS + 0x90000, 0x1000);
	return pcie.gcc06 && pcie.gcc52 && pcie.gcc1f && pcie.gcc6f && pcie.gcc8c && pcie.gcc8d && pcie.gcc90;
}

static bool
map_pcie(void)
{
	if (pcie.mapped) {
		return true;
	}
	pcie.parf = ml_io_map(PARF_PHYS, 0x3000);
	pcie.phy = ml_io_map(PHY_PHYS, 0x1000);
	pcie.dbi = ml_io_map(DBI_PHYS, 0x1000);
	pcie.iatu = ml_io_map(IATU_PHYS, 0x1000);
	pcie.conf = ml_io_map(CONF_PHYS, 0x1000);
	pcie.mapped = pcie.parf && pcie.phy && pcie.dbi && pcie.iatu && pcie.conf;
	if (!pcie.mapped) {
		say("modem: pcie: the registers could not be mapped\n");
	}
	return pcie.mapped;
}

/* A branch clock on (its vote bit in the vote register) and running (its CBCR, unless 0: the pipe clock, which the PHY makes). */
static bool
branch_on(const char *name, vm_offset_t enable, uint32_t bit, uint32_t cbcr)
{
	set_bits(enable, 0, bit);
	for (unsigned us = 0; cbcr != 0 && (rd(gcc(cbcr)) & CBCR_OFF); us += 10) {
		if (us > 2000) {
			say("modem: pcie: the clock %s does not start (cbcr %#x)\n", name, rd(gcc(cbcr)));
			return false;
		}
		sleep_us(10);
	}
	return true;
}

/* An RCG to `source` divided by `half_divider` (2 x divider - 1, as gcc's tables have it). */
static bool
rcg_set(uint32_t rcg, uint32_t source, uint32_t half_divider)
{
	uint32_t cfg = rd(gcc(rcg + 4));

	if ((cfg & 0x71f) == (source << 8 | half_divider)) {
		return true;
	}
	wr(gcc(rcg + 4), (cfg & ~0x371fu) | source << 8 | half_divider);
	set_bits(gcc(rcg), 0, RCG_UPDATE);
	for (unsigned us = 0; rd(gcc(rcg)) & RCG_UPDATE; us += 10) {
		if (us > 2000) {
			say("modem: pcie: the clock source at %#x does not take its configuration\n", rcg);
			return false;
		}
		sleep_us(10);
	}
	return true;
}

static void
reset_pulse(uint32_t bcr)
{
	wr(gcc(bcr), 1);
	sleep_us(1000);
	wr(gcc(bcr), 0);
	sleep_us(1000);
}

/* ---- step 1: what the lines and the clocks are, nothing written ----------------------------------------- */

static void
state_read(void)
{
	static const struct { struct instantnoodle_pin pin; const char *what; } pins[] = {
		{ PIN_MDM2AP_ERRFATAL, "MDM2AP error fatal (1)" }, { PIN_MDM2AP_STATUS, "MDM2AP status (3)" },
		{ PIN_AP2MDM_STATUS, "AP2MDM status (56)" }, { PIN_AP2MDM_ERRFATAL, "AP2MDM error fatal (57)" },
		{ PIN_PERST, "PCIe 2 PERST (85)" }, { PIN_CLKREQ, "PCIe 2 CLKREQ (86)" }, { PIN_WAKE, "PCIe 2 WAKE (87)" },
	};

	say("modem: step 1 (reads only): the modem's control lines and PCIe 2's power and clocks\n");
	for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
		say("modem: gpio %-24s config %#010x, level %u\n", pins[i].what, instantnoodle_gpio_cfg(pins[i].pin),
		    instantnoodle_gpio_level(pins[i].pin) ? 1u : 0u);
	}
	if (!map_gcc()) {
		say("modem: GCC could not be mapped\n");
		return;
	}
	say("modem: GCC: PCIe 2's domain %#x (bit 31: on, bit 0: collapse), votes %#x, aux %#x, cfg ahb %#x, mstr axi %#x, slv axi %#x, pipe %#x\n",
	    rd(gcc(GCC_PCIE_2_GDSCR)), rd(gcc(GCC_VOTE)), rd(gcc(GCC_PCIE_2_AUX_CBCR)), rd(gcc(GCC_PCIE_2_CFG_AHB_CBCR)),
	    rd(gcc(GCC_PCIE_2_MSTR_AXI_CBCR)), rd(gcc(GCC_PCIE_2_SLV_AXI_CBCR)), rd(gcc(GCC_PCIE_2_PIPE_CBCR)));
}

/* ---- power and clocks ------------------------------------------------------------------------------ */

static void
clocks_off(void)
{
	set_bits(gcc(GCC_VOTE), VOTES_ALL, 0);
	set_bits(gcc(GCC_PCIE2_REFGEN_CBCR), 1, 0);
	set_bits(gcc(GCC_PCIE_MDM_CLKREF), 1, 0);
	sleep_us(100);
	set_bits(gcc(GCC_PCIE_2_GDSCR), 0, GDSC_COLLAPSE);
	pcie.clocked = false;
}

static bool
clocks_on(void)
{
	/* PCIe 0 (the Wi-Fi's radio) may be running: the NoC's bus clock manager is voted by it as well; the TBU clocks are shared with it and left on */
	instantnoodle_bcm_vote("SN12", 1, 1);
	set_bits(gcc(GCC_PCIE_2_GDSCR), GDSC_COLLAPSE, 0);
	for (unsigned us = 0; !(rd(gcc(GCC_PCIE_2_GDSCR)) & GDSC_ON); us += 10) {
		if (us > 5000) {
			say("modem: pcie: PCIe 2's power domain does not come on (%#x)\n", rd(gcc(GCC_PCIE_2_GDSCR)));
			return false;
		}
		sleep_us(10);
	}
	pcie.clocked = true;
	/* aux 19.2 MHz from the crystal; the PHY's reference 100 MHz from GPLL0 (600 / 6) */
	if (!rcg_set(GCC_PCIE_2_AUX_RCG, 0, 1) || !rcg_set(GCC_PCIE_REFGEN_RCG, 1, 11)) {
		return false;
	}
	if (!branch_on("aggre noc pcie tbu", gcc(GCC_AGGRE_NOC_PCIE_TBU), 1, GCC_AGGRE_NOC_PCIE_TBU) ||
	    !branch_on("ddrss pcie sf tbu", gcc(GCC_DDRSS_PCIE_SF_TBU), 1, GCC_DDRSS_PCIE_SF_TBU) ||
	    !branch_on("pcie mdm clkref", gcc(GCC_PCIE_MDM_CLKREF), 1, GCC_PCIE_MDM_CLKREF) ||
	    !branch_on("pcie2 phy refgen", gcc(GCC_PCIE2_REFGEN_CBCR), 1, GCC_PCIE2_REFGEN_CBCR) ||
	    !branch_on("pcie_2_aux", gcc(GCC_VOTE), VOTE_AUX, GCC_PCIE_2_AUX_CBCR) ||
	    !branch_on("pcie_2_cfg_ahb", gcc(GCC_VOTE), VOTE_CFG_AHB, GCC_PCIE_2_CFG_AHB_CBCR) ||
	    !branch_on("pcie_2_mstr_axi", gcc(GCC_VOTE), VOTE_MSTR_AXI, GCC_PCIE_2_MSTR_AXI_CBCR) ||
	    !branch_on("pcie_2_slv_axi", gcc(GCC_VOTE), VOTE_SLV_AXI, GCC_PCIE_2_SLV_AXI_CBCR) ||
	    !branch_on("pcie_2_slv_q2a_axi", gcc(GCC_VOTE), VOTE_SLV_Q2A, GCC_PCIE_2_SLV_Q2A_CBCR)) {
		return false;
	}
	reset_pulse(GCC_PCIE_2_BCR);
	reset_pulse(GCC_PCIE_2_PHY_BCR);
	return true;
}

/* ---- the link ------------------------------------------------------------------------------------ */

/* CRC-8, polynomial 0x07, most significant bit first: the controller's hash of a BDF (big endian) into its SID table. */
static uint8_t
bdf_hash(uint16_t bdf)
{
	uint8_t bytes[2] = { (uint8_t)(bdf >> 8), (uint8_t)bdf }, crc = 0;

	for (unsigned b = 0; b < 2; b++) {
		crc ^= bytes[b];
		for (unsigned i = 0; i < 8; i++) {
			crc = (uint8_t)(crc & 0x80 ? crc << 1 ^ 0x07 : crc << 1);
		}
	}
	return crc;
}

/* The SMMU streams of the bus's two functions (dt iommu-map: the bridge 0x1d00, the modem 0x1d01), as pci-msm.c enters them. */
static void
sid_table(void)
{
	static const uint16_t bdfs[2] = { 0x0000, EP_BDF };

	for (unsigned i = 0; i < 256; i++) {
		wr(pcie.parf + PARF_BDF_TO_SID + 4 * i, 0);
	}
	for (unsigned i = 0; i < 2; i++) {
		uint8_t hash = bdf_hash(bdfs[i]);

		while (rd(pcie.parf + PARF_BDF_TO_SID + 4u * hash) != 0) {
			uint32_t entry = rd(pcie.parf + PARF_BDF_TO_SID + 4u * hash);
			uint8_t following = (uint8_t)(hash + 1);

			if ((entry & 0xff) == 0) {
				wr(pcie.parf + PARF_BDF_TO_SID + 4u * hash, entry | following);
			}
			hash = following;
		}
		wr(pcie.parf + PARF_BDF_TO_SID + 4u * hash, (uint32_t)bdfs[i] << 16 | (uint32_t)i << 8);
	}
}

static bool
link_train(void)
{
	uint32_t status = 0, link = 0;

	set_bits(pcie.dbi + DBI_GEN2_CTRL, 0x1f00, (option_lanes == 2 ? 2u : 1u) << 8);       /* one lane to begin with (modem_lanes=2: two, the link is x2) */
	if (option_lanes == 2) {
		set_bits(pcie.dbi + 0x710, 0x3f0000, 0x30000);              /* the port logic's link mode: x2 */
	}
	set_bits(pcie.dbi + DBI_GEN3_RELATED, 1, 0);
	set_bits(pcie.dbi + DBI_MISC_CONTROL, 0, 1);                    /* (read-only registers writable) */
	wr(pcie.dbi + DBI_SPCIE_CAP, 0x55555555);                       /* the Gen3 presets */
	set_bits(pcie.dbi + DBI_MISC_CONTROL, 1, 0);
	if (!(option_lanes & 0x10)) {
		set_bits(pcie.dbi + DBI_LINK_CONTROL2, 0xf, 2);         /* target: 5 GT/s (modem_lanes bit 4: none, as Android's driver, which writes the target only when the device tree has one) */
	}
	set_bits(pcie.dbi + DBI_DEVICE_CONTROL, 0x7000, 5u << 12);      /* the largest read request: 4 KB */
	set_bits(pcie.parf + PARF_LTSSM, 0, 1u << 8);
	for (unsigned tries = 0; tries <= 40; tries++) {
		sleep_us(5000);
		status = rd(pcie.dbi + ELBI_SYS_STTS);
		link = rd(pcie.dbi + DBI_LINK_STATUS);
		if ((status & XMLH_LINK_UP) && (link & 1u << 29) && rd(pcie.dbi) != 0xffffffffu) {
			say("modem: pcie: THE LINK IS UP after %u ms: %s, x%u (LTSSM %#x)\n", 5 * (tries + 1), (link >> 16 & 0xf) == 1 ?
			    "2.5 GT/s" : (link >> 16 & 0xf) == 2 ? "5 GT/s" : "8 GT/s", link >> 20 & 0x3f, status >> 12 & 0x3f);
			return true;
		}
	}
	say("modem: pcie: the link does not come up (ELBI status %#x, LTSSM %#x, link status %#x)\n", status, status >> 12 & 0x3f, link);
	return false;
}

static void
iatu_config_window(void)
{
	wr(pcie.iatu + IATU_CTRL2, 0);
	wr(pcie.iatu + IATU_CTRL1, 0x4);                                /* configuration requests, type 0 */
	wr(pcie.iatu + IATU_LBAR, (uint32_t)CONF_PHYS);
	wr(pcie.iatu + IATU_UBAR, 0);
	wr(pcie.iatu + IATU_LAR, (uint32_t)CONF_PHYS + 0xfff);
	wr(pcie.iatu + IATU_LTAR, (uint32_t)EP_BDF << 16);             /* to bus 1, device 0 */
	wr(pcie.iatu + IATU_UTAR, 0);
	wr(pcie.iatu + IATU_CTRL2, 1u << 31);
}

/* The bridge's bus numbers and window, and the device's BAR 0 placed in it. */
static bool
enumerate(void)
{
	uint32_t ids, bar, size_mask;
	bool wide;

	for (unsigned ms = 0; (ids = rd(pcie.conf)) == 0xffffffffu; ms++) {
		if (ms > 1000) {
			say("modem: pcie: the device's configuration space does not answer\n");
			return false;
		}
		sleep_us(1000);
	}
	wr(pcie.dbi + 0x18, 0x00010100);                                /* buses: primary 0, secondary and subordinate 1 */
	wr(pcie.dbi + 0x1c, 0x000000f0);                                /* no I/O window */
	wr(pcie.dbi + 0x20, (uint32_t)((BAR_PHYS + BAR_WINDOW - 1) >> 16 & 0xfff0) << 16 | (uint32_t)(BAR_PHYS >> 16 & 0xfff0));
	wr(pcie.dbi + 0x24, 0x0000fff0);                                /* no prefetchable window */
	set_bits(pcie.dbi + 0x04, 0, 0x6);                              /* memory, bus master */

	bar = rd(pcie.conf + 0x10);
	wide = (bar & 0x6) == 0x4;
	wr(pcie.conf + 0x10, 0xffffffffu);
	size_mask = rd(pcie.conf + 0x10) & ~0xfu;
	pcie.bar_size = size_mask == 0 ? 0 : (uint64_t)(~size_mask) + 1;
	if (pcie.bar_size == 0 || pcie.bar_size > BAR_WINDOW) {
		say("modem: pcie: the device's BAR 0 is not one this places (%#x, size %#llx)\n", bar, pcie.bar_size);
		return false;
	}
	wr(pcie.conf + 0x10, (uint32_t)BAR_PHYS | (bar & 0xf));
	if (wide) {
		wr(pcie.conf + 0x14, 0);
	}
	/* (the device's command register: memory space and bus master) */
	set_bits(pcie.conf + 0x04, 0, 0x6);
	say("modem: pcie: device %04x:%04x behind the bridge (class %#x); its BAR 0 (%s) at %#lx, %llu KB\n", ids & 0xffff, ids >> 16,
	    rd(pcie.conf + 0x08) >> 8, wide ? "64-bit" : "32-bit", BAR_PHYS, pcie.bar_size >> 10);
	return true;
}


/*
 * Android's "msm_pcie_disable" + "msm_pcie_enable" when the modem says it has booted (MDM2AP status high): PERST low for about 15 ms and released again with the
 * controller's own state (clocks, PHY, config window) left as it is, the link trained again: the modem's AMSS initialises its PCIe endpoint on that reset. Tried
 * `tries` times; returns whether it came up. LTSSM states are said while it trains.
 */
static bool
pcie_soft_cycle(unsigned tries)
{
	for (unsigned attempt = 0; attempt < tries; attempt++) {
		uint32_t status = 0, link = 0;
		bool up = false;

		pcie.up = false;
		set_bits(pcie.parf + PARF_LTSSM, 1u << 8, 0);               /* link training off */
		set_bits(pcie.dbi + DBI_GEN2_CTRL, 0x1f00, (option_lanes == 2 ? 2u : 1u) << 8);
		if (option_lanes == 2) {
			set_bits(pcie.dbi + 0x710, 0x3f0000, 0x30000);
		}
		instantnoodle_gpio_output(PIN_PERST, false, INSTANTNOODLE_PULL_DOWN, 2);
		sleep_us(15000);
		instantnoodle_gpio_output(PIN_PERST, true, INSTANTNOODLE_PULL_DOWN, 2);
		sleep_us(10000);
		set_bits(pcie.parf + PARF_LTSSM, 0, 1u << 8);               /* and on again */
		for (unsigned ms = 0; ms < 600 && !up; ms += 5) {
			sleep_us(5000);
			status = rd(pcie.dbi + ELBI_SYS_STTS);
			link = rd(pcie.dbi + DBI_LINK_STATUS);
			up = (status & XMLH_LINK_UP) && (link & 1u << 29) && rd(pcie.dbi) != 0xffffffffu;
			if (!up && ms % 100 == 0) {
				say("modem: pcie: soft cycle %u, %u ms: LTSSM %#x, ELBI %#x\n", attempt + 1, ms, status >> 12 & 0x3f, status);
			}
		}
		if (up) {
			say("modem: pcie: the link is UP again after the soft cycle %u: %s x%u\n", attempt + 1, (link >> 16 & 0xf) == 1 ? "2.5 GT/s" : (link >> 16 & 0xf) == 2 ? "5 GT/s" : "8 GT/s",
			    link >> 20 & 0x3f);
			sid_table();
			iatu_config_window();
			set_bits(pcie.dbi + DBI_ACK_F_ASPM_CTRL, 0, 1u << 15);
			wr(pcie.dbi + DBI_AUX_CLK_FREQ, 0x14);
			if (enumerate()) {
				pcie.up = true;
				return true;
			}
		} else {
			say("modem: pcie: soft cycle %u: no link (LTSSM %#x)\n", attempt + 1, status >> 12 & 0x3f);
		}
	}
	return false;
}

static void
pcie_off(void)
{
	pcie.up = false;
	__asm__ volatile ("dmb ish" ::: "memory");
	sleep_us(2000);         /* (a register access of another thread that had already looked at pcie.up: mhi_op_write()) */
	instantnoodle_gpio_output(PIN_PERST, false, INSTANTNOODLE_PULL_DOWN, 2);
	if (pcie.clocked) {
		wr(pcie.phy + PHY_POWER_DOWN, 0);
		set_bits(pcie.parf + PARF_PHY_CTRL, 0, 1);
		clocks_off();
	}
}

static bool
pcie_on(void)
{
	uint32_t ready = 0;

	/* the modem in reset while everything comes up */
	instantnoodle_gpio_output(PIN_PERST, false, INSTANTNOODLE_PULL_DOWN, 2);
	instantnoodle_gpio_function(PIN_CLKREQ, FUNCTION_PCI_E2, INSTANTNOODLE_PULL_UP, 2);
	instantnoodle_gpio_input(PIN_WAKE, INSTANTNOODLE_PULL_UP);
	sleep_us(1000);
	if (!clocks_on()) {
		return false;
	}
	say("modem: pcie: PARF %#x, PHY status %#x: the controller set up as a root complex\n", rd(pcie.parf + PARF_SYS_CTRL),
	    rd(pcie.phy + PHY_STATUS));
	wr(pcie.parf + PARF_DEVICE_TYPE, 0x4);
	set_bits(pcie.parf + PARF_PM_CTRL, 0, 1u << 5);                 /* L1 not entered (REQ_NOT_ENTR_L1: Android clears it only when the device tree says the controller supports L1, and PCIe 2's does not) */
	set_bits(pcie.parf + PARF_PHY_CTRL, 1, 0);                      /* the PHY powered */
	wr(pcie.parf + PARF_DBI_BASE_ADDR, 0);
	wr(pcie.parf + PARF_SYS_CTRL, 0x365e);
	set_bits(pcie.parf + PARF_MHI_CLOCK_RESET_CTRL, 0, 1u << 4);
	wr(pcie.parf + PARF_INT_ALL_MASK, 0);                           /* (the controller's own interrupts are not taken) */
	wr(pcie.parf + PARF_SLV_ADDR_SPACE_SIZE, 0x4000000);
	set_bits(pcie.parf + PARF_AXI_MSTR_WR_ADDR_HALT, 0, 1u << 31);

	for (unsigned i = 0; i < sizeof(phy_sequence) / sizeof(phy_sequence[0]); i++) {
		wr(pcie.phy + phy_sequence[i][0], phy_sequence[i][1]);
		(void)rd(pcie.phy + phy_sequence[i][0]);
	}
	sleep_us(1000);
	branch_on("pcie_2_pipe", gcc(GCC_VOTE), VOTE_PIPE, 0);
	for (unsigned tries = 0; tries <= 10 && ((ready = rd(pcie.phy + PHY_STATUS)) & 1u << 6); tries++) {
		sleep_us(1000);
	}
	if (ready & 1u << 6) {
		say("modem: pcie: the PHY is not ready (status %#x)\n", ready);
		return false;
	}
	say("modem: pcie: the PHY is ready\n");
	sleep_us(10000);                                                /* dt qcom,ep-latency */
	instantnoodle_gpio_output(PIN_PERST, true, INSTANTNOODLE_PULL_DOWN, 2);
	sleep_us(1000);
	if (!link_train()) {
		return false;
	}
	sid_table();
	iatu_config_window();
	set_bits(pcie.dbi + DBI_ACK_F_ASPM_CTRL, 0, 1u << 15);
	wr(pcie.dbi + DBI_AUX_CLK_FREQ, 0x14);                          /* the aux clock is 19.2 MHz */
	if (!enumerate()) {
		return false;
	}
	pcie.up = true;
	return true;
}

/* ---- the modem's MHI registers (BAR 0): read only ------------------------------------------------- */

#define MHI_VERSION             0x08
#define MHI_CFG                 0x10
#define MHI_CHDBOFF             0x18
#define MHI_ERDBOFF             0x20
#define MHI_BHIOFF              0x28
#define MHI_BHIEOFF             0x2c
#define MHI_STATUS              0x48
#define BHI_STATUS              0x2c
#define BHI_ERRCODE             0x30
#define BHI_IMGADDR_LOW         0x08
#define BHI_IMGADDR_HIGH        0x0c
#define BHI_IMGSIZE             0x10
#define BHI_IMGTXDB             0x18
#define BHI_INTVEC              0x20
#define BHI_EXECENV             0x28
#define BHI_SERIAL              0x40
#define BHI_MSMHWID             0x4c

static void
mhi_read(void)
{
	vm_offset_t bar;
	uint32_t bhi, bhie, execenv;
	static const char *const env[] = { "PBL", "SBL", "AMSS (mission mode)", "RDDM", "WFW", "PASS THROUGH", "EDL" };

	bar = ml_io_map(BAR_PHYS, 0x1000);
	if (bar == 0) {
		say("modem: mhi: BAR 0 could not be mapped\n");
		return;
	}
	pcie.bar = bar;
	say("modem: mhi: BAR 0: MHIREGLEN %#x, MHIVER %#x, MHICFG %#x, CHDBOFF %#x, ERDBOFF %#x, BHIOFF %#x, BHIEOFF %#x, MHISTATUS %#x\n", rd(bar), rd(bar + MHI_VERSION),
	    rd(bar + MHI_CFG), rd(bar + MHI_CHDBOFF), rd(bar + MHI_ERDBOFF), rd(bar + MHI_BHIOFF), rd(bar + MHI_BHIEOFF), rd(bar + MHI_STATUS));
	bhi = rd(bar + MHI_BHIOFF);
	if (bhi == 0xffffffffu || bhi == 0 || bhi >= 0x1000 - 0x80) {
		say("modem: mhi: the boot interface's offset (%#x) is not one to read at\n", bhi);
		return;
	}
	execenv = rd(bar + bhi + BHI_EXECENV);
	say("modem: bhi: EXECENV %u (%s), STATUS %#x, ERRCODE %#x, serial %#x, MSM HW id %#x %#x\n", execenv, execenv < sizeof(env) / sizeof(env[0]) ? env[execenv] : "?",
	    rd(bar + bhi + BHI_STATUS), rd(bar + bhi + BHI_ERRCODE), rd(bar + bhi + BHI_SERIAL), rd(bar + bhi + BHI_MSMHWID), rd(bar + bhi + BHI_MSMHWID + 4));
	bhie = rd(bar + MHI_BHIEOFF);
	(void)bhie;
	pcie.bhi = bhi;
}

/* ---- the modem's view of memory: its stream in the application processors' SMMU ------------------------ */

/*
 * What the modem reads and writes in our memory by itself goes through the apps SMMU (0x15000000) as stream 0x1d01 (dt iommu-map of qcom,pcie@1c10000:
 * BDF 0x100 -> 0x1d01; the bridge is 0x1d00). As for the Wi-Fi's radio (hakios.wifi.c) and the UFS controller (instantnoodle_ufs.c) the hypervisor takes a
 * single-stream match register and a context of ours, and refuses a bypass: so a context with page tables of ours, a window of addresses handed out
 * upwards from DMA_IOVA (dt qcom,iommu-dma-addr-pool 0x20000000 + 512 MB), stream 0x1d01 sent through it. The pages are mapped non-cacheable
 * (MAIR index 0: the same attributes as the Wi-Fi's and the UFS's); the processor's side cleans and invalidates what it shares (dma_sync()).
 */
#define SMMU_PHYS               0x15000000ul
#define DMA_IOVA                0x20000000ull
#define DMA_SIZE                0x02000000u     /* 32 MB: the root table, one of the second level, 16 of the third (2 MB each) */
#define DMA_TABLES              18
#define DMA_PTE_NC              (3ull | 1ull << 6 | 2ull << 8 | 1ull << 10 | 1ull << 11)           /* non-cacheable (MAIR index 0) */
#define DMA_PTE_WB              (3ull | 1ull << 2 | 1ull << 6 | 3ull << 8 | 1ull << 10 | 1ull << 11) /* write-back, inner shareable (MAIR index 1): the coherent mapping Android's tree asks for */
#define DMA_PTE_DEVICE          (3ull | 2ull << 2 | 1ull << 6 | 2ull << 8 | 1ull << 10 | 1ull << 11 | 1ull << 53 | 1ull << 54)   /* a device's register (MAIR index 2), never executed */
#define MSI_REGISTER            0x17a10040ull   /* the GIC's SETSPI register, where the radio's MSIs of hakios.wifi.c go: a write of an interrupt's number raises it */
#define MSI_IOVA                (DMA_IOVA + DMA_SIZE - 4096)    /* the last page of the window: the modem's address of that register's page */
#define MSI_SPI                 960u            /* interrupts nothing here enables (960..991: 32 vectors, the low five bits of the data are the vector): a write only becomes pending */
#define MODEM_STREAM            0x1d01
#define PTE_ADDRESS             0x0000fffffffff000ull

static bool option_nc;                  /* modem_nc=1: the buffers mapped non-cacheable, as the first tries had them */

static struct {
	bool on;
	volatile uint32_t *global, *second, *cb;
	uint32_t match, context, pages, old_smr, old_s2cr, old_sctlr;
	vm_offset_t tables;
	uint64_t next;                  /* of the window, the first byte not handed out */
} smmu;

static void
dma_sync(const void *p, size_t n)
{
	for (vm_offset_t line = (vm_offset_t)p & ~63ul; line < (vm_offset_t)p + n; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy" ::: "memory");
}

static bool
smmu_on(void)
{
	uint32_t idr1, n, contexts, ctx = 0, m = ~0u;
	bool used[256] = { false };
	uint64_t *root, *second_level, root_pa;

	lck_mtx_lock(&hakios_smmu_lock);
	smmu.global = (volatile uint32_t *)ml_io_map(SMMU_PHYS, 0x2000);
	idr1 = smmu.global[0x24 / 4];
	n = smmu.global[0x20 / 4] & 0xff;
	contexts = idr1 & 0xff;
	smmu.pages = 2u << (idr1 >> 28 & 7);
	if ((idr1 >> 31) != 0 || smmu.pages * 0x1000 * 2 != 0x100000 || (smmu.global[0] & (1u << 3)) != 0) {
		say("modem: smmu: not the SMMU this is written for (IDR1 %#x, sCR0 %#x)\n", idr1, smmu.global[0]);
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	for (uint32_t i = 0; i < n && 0x800 + 4 * i < 0xc00; i++) {
		uint32_t smr = smmu.global[(0x800 + 4 * i) / 4];

		if ((smr >> 31) != 0) {
			used[smmu.global[(0xc00 + 4 * i) / 4] & 0xff] = true;
		} else if (m == ~0u) {
			m = i;
		}
	}
	while (ctx < contexts && used[ctx]) {
		ctx++;
	}
	if (m == ~0u || ctx >= contexts) {
		say("modem: smmu: no free match register (%d) or context (%u of %u)\n", (int)m, ctx, contexts);
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	smmu.cb = (volatile uint32_t *)ml_io_map(SMMU_PHYS + (uint64_t)(smmu.pages + ctx) * 0x1000, 0x1000);
	if ((smmu.cb[0] & 1) != 0) {
		say("modem: smmu: context %u is switched on although no stream goes through it: left alone\n", ctx);
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	if (smmu.tables == 0 && kmem_alloc(kernel_map, &smmu.tables, round_page(4096 * DMA_TABLES), KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		lck_mtx_unlock(&hakios_smmu_lock);
		return false;
	}
	smmu.second = smmu.global + 0x1000 / 4;
	smmu.match = m;
	smmu.context = ctx;
	smmu.old_smr = smmu.global[(0x800 + 4 * m) / 4];
	smmu.old_s2cr = smmu.global[(0xc00 + 4 * m) / 4];
	smmu.old_sctlr = smmu.cb[0];
	root = (uint64_t *)smmu.tables;
	second_level = (uint64_t *)(smmu.tables + 4096);
	root[DMA_IOVA >> 30 & 0x1ff] = kvtophys((vm_offset_t)second_level) | 3;
	for (unsigned t = 0; t < DMA_SIZE >> 21; t++) {
		second_level[(DMA_IOVA >> 21 & 0x1ff) + t] = kvtophys(smmu.tables + 4096 * (2 + t)) | 3;
	}
	{
		uint64_t at = MSI_IOVA - DMA_IOVA;

		((uint64_t *)(smmu.tables + 4096 * (2 + (at >> 21))))[at >> 12 & 0x1ff] = (MSI_REGISTER & ~0xfffull) | DMA_PTE_DEVICE;
	}
	dma_sync((void *)smmu.tables, 4096 * DMA_TABLES);
	root_pa = kvtophys(smmu.tables) | (uint64_t)ctx << 48;
	say("modem: smmu: THE SMMU IS ABOUT TO BE WRITTEN: context %u gets tables of ours (root %#llx) and is switched on; match register %u sends stream %#x through it\n",
	    ctx, root_pa & PTE_ADDRESS, m, MODEM_STREAM);
	smmu.cb[0x00 / 4] = 0;
	smmu.second[(0x800 + 4 * ctx) / 4] = 1;
	smmu.second[(4 * ctx) / 4] = (1u << 16 | 3u << 8 | 0xfu << 12);
	smmu.cb[0x10 / 4] = (1u | 1u << 4 | 7u << 15);
	smmu.cb[0x30 / 4] = (25u | 1u << 8 | 1u << 10 | 3u << 12 | 1u << 23);
	smmu.cb[0x20 / 4] = (uint32_t)root_pa;
	smmu.cb[0x24 / 4] = (uint32_t)(root_pa >> 32);
	smmu.cb[0x28 / 4] = 0;
	smmu.cb[0x2c / 4] = 0;
	smmu.cb[0x38 / 4] = 0xf404ff44;
	smmu.cb[0x3c / 4] = 0;
	__asm__ volatile ("dsb sy" ::: "memory");
	smmu.cb[0x58 / 4] = 0xc00001fe;
	smmu.cb[0x610 / 4] = ctx;
	smmu.cb[0x7f0 / 4] = 1;
	for (int w = 0; w < 1000 && (smmu.cb[0x7f4 / 4] & 1) != 0; w++) {
		sleep_us(10);
	}
	smmu.cb[0x00 / 4] = (1u << 0 | 1u << 1 | 1u << 2 | 1u << 5 | 1u << 8 | 1u << 12);
	smmu.global[(0xc00 + 4 * m) / 4] = ctx;
	smmu.global[(0x800 + 4 * m) / 4] = 1u << 31 | MODEM_STREAM;
	__asm__ volatile ("dsb sy" ::: "memory");
	smmu.on = (smmu.global[(0x800 + 4 * m) / 4] >> 31) != 0 && (smmu.global[(0xc00 + 4 * m) / 4] & 0xff) == ctx && (smmu.cb[0] & 1) != 0 &&
	    smmu.cb[0x20 / 4] == (uint32_t)root_pa;
	say("modem: smmu: smr reads %#x, s2cr %#x, context sctlr %#x, ttbr0 %#x: %s\n", smmu.global[(0x800 + 4 * m) / 4], smmu.global[(0xc00 + 4 * m) / 4], smmu.cb[0],
	    smmu.cb[0x20 / 4], smmu.on ? "THE MODEM'S STREAM GOES THROUGH CONTEXT OF OURS" : "NOT TAKEN: the stream is not sent through our context");
	if (!smmu.on) {
		smmu.global[(0x800 + 4 * m) / 4] = smmu.old_smr;
		smmu.global[(0xc00 + 4 * m) / 4] = smmu.old_s2cr;
		smmu.cb[0] = smmu.old_sctlr;
	}
	lck_mtx_unlock(&hakios_smmu_lock);
	return smmu.on;
}

static void
smmu_faults(void)
{
	if (!smmu.on) {
		return;
	}
	if ((smmu.cb[0x58 / 4] & 0xc00001fe) != 0 || smmu.global[0x48 / 4] != 0) {
		say("modem: smmu: A REQUEST OF THE MODEM'S WAS REFUSED: context fsr %#x, far %#llx, syndrome %#x; global sgfsr %#x, syndromes %#x %#x\n", smmu.cb[0x58 / 4],
		    (uint64_t)smmu.cb[0x64 / 4] << 32 | smmu.cb[0x60 / 4], smmu.cb[0x68 / 4], smmu.global[0x48 / 4], smmu.global[0x50 / 4], smmu.global[0x54 / 4]);
	} else {
		say("modem: smmu: the modem's requests were all served (context fsr 0)\n");
	}
}

static void
smmu_off(void)
{
	if (!smmu.on) {
		return;
	}
	lck_mtx_lock(&hakios_smmu_lock);
	smmu_faults();
	smmu.global[(0x800 + 4 * smmu.match) / 4] = smmu.old_smr;
	smmu.global[(0xc00 + 4 * smmu.match) / 4] = smmu.old_s2cr;
	smmu.cb[0] = smmu.old_sctlr;
	__asm__ volatile ("dsb sy" ::: "memory");
	smmu.on = false;
	smmu.next = 0;
	say("modem: smmu: the modem's stream and context %u put back as they were found\n", smmu.context);
	lck_mtx_unlock(&hakios_smmu_lock);
}

/*
 * Memory somebody else has allocated, physically in one piece, entered in the modem's window (for the mailbox of a call's sound, drivers/hakios.mic.c: the audio DSP sees the same
 * memory by its physical address). False while the window is not up.
 */
void hakios_modem_dma_sync(void *address, size_t size);
void
hakios_modem_dma_sync(void *address, size_t size)
{
	dma_sync(address, size);
}

uint32_t hakios_modem_generation(void);
uint32_t
hakios_modem_generation(void)
{
	return modem_generation;
}

bool hakios_modem_map_contig(uint64_t phys, size_t size, uint64_t *iova);
bool
hakios_modem_map_contig(uint64_t phys, size_t size, uint64_t *iova)
{
	size = (size + 4095) & ~(size_t)4095;
	if (!smmu.on || !cell_started || smmu.next + size > DMA_SIZE - 4096) {
		return false;
	}
	for (size_t off = 0; off < size; off += 4096) {
		uint64_t at = smmu.next + off;
		uint64_t *third = (uint64_t *)(smmu.tables + 4096 * (2 + (at >> 21)));

		third[at >> 12 & 0x1ff] = (phys + off) | (option_nc ? DMA_PTE_NC : DMA_PTE_WB);
	}
	dma_sync((void *)smmu.tables, 4096 * DMA_TABLES);
	*iova = DMA_IOVA + smmu.next;
	smmu.next += size;
	return true;
}

/* `size` bytes of kernel memory, zeroed, that the modem sees at *iova (aligned to `align`, a power of two): pages entered one by one in the third-level tables. NULL if there is no room. */
static uint8_t *
dma_alloc_aligned(size_t size, size_t align, uint64_t *iova)
{
	vm_offset_t va;

	size = (size + 4095) & ~(size_t)4095;
	if (align > 4096) {
		smmu.next = (smmu.next + align - 1) & ~(uint64_t)(align - 1);
	}
	if (!smmu.on || smmu.next + size > DMA_SIZE - 4096 ||
	    kmem_alloc(kernel_map, &va, size, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return NULL;
	}
	for (size_t off = 0; off < size; off += 4096) {
		uint64_t at = smmu.next + off;
		uint64_t *third = (uint64_t *)(smmu.tables + 4096 * (2 + (at >> 21)));

		third[at >> 12 & 0x1ff] = kvtophys(va + off) | (option_nc ? DMA_PTE_NC : DMA_PTE_WB);
	}
	dma_sync((void *)smmu.tables, 4096 * DMA_TABLES);
	dma_sync((void *)va, size);
	*iova = DMA_IOVA + smmu.next;
	smmu.next += size;
	return (uint8_t *)va;
}

static uint8_t *
dma_alloc(size_t size, uint64_t *iova)
{
	return dma_alloc_aligned(size, 4096, iova);
}

/*
 * ---- the firmware: the phone's firmware partition's image/ tree, on the root (/fw/modem), read only ----------------
 * On the OnePlus 8 this read the modem partition's FAT through its own UFS driver. On spartan the tree is copied, once, from
 * Android's /vendor/firmware_mnt/image (sdx55m/, modem_pr/, sdx55/: fw-modem/image of the build, into the system image's
 * /fw/modem by userland/build.sh), and read from there: no access to the phone's storage. The modem's names
 * ("image/sdx55m/sbl1.mbn", and over TFTP "/readonly/firmware/image/...") are paths under /fw/modem. The functions keep the
 * FAT's names and shape: fat_open() gives a handle (in `cluster`) and the size, fat_read() reads by handle and offset.
 * The sequence runs on its own thread, which waits here for the root to be mounted.
 */
#include <sys/vnode.h>
#include <sys/vnode_internal.h>
#include <sys/mount.h>
#include <sys/kauth.h>

#define FW_ROOT                 "/fw/modem/"
#define FW_HANDLES              64
static const char *fw_partition = "/fw/modem (the system image)";

static struct {
	bool ok;
} fat;
static struct {
	char path[192];
	uint32_t size;
} fw_handles[FW_HANDLES];
static uint32_t fw_handle_count;
static LCK_GRP_DECLARE(fw_lock_group, "spartan-modem-fw");
static LCK_MTX_DECLARE(fw_lock, &fw_lock_group);

/*
 * The modem's EFS (modem_efs=1): on the OnePlus 8 its mdm1m9kefs1..3 partitions, read from the storage; on spartan copies of
 * them (read once, with dd, from Android's /dev/block/by-name: fw-modem/image/hakios-efs/, which are the phone's own and in
 * no repository), here as files of the tree. The OnePlus 8's code asks for a partition's base and then reads base + offset:
 * the base is the file's handle shifted past any offset (EFS_HANDLE_SHIFT), so that code stays as it is.
 */
#define EFS_HANDLE_SHIFT        40
static bool fat_open(const char *path, uint32_t *cluster, uint32_t *size);
static int fat_read(uint32_t handle, uint32_t size, uint64_t offset, uint8_t *dst, uint64_t len);

static bool
gpt_find(__unused uint8_t lun, const char *want, uint64_t *base, uint64_t *bytes)
{
	char path[96];
	uint32_t handle, size;

	snprintf(path, sizeof(path), "image/hakios-efs/%s", want);
	if (!fat_open(path, &handle, &size)) {
		say("modem: fw: no copy of the partition %s (fw-modem/image/hakios-efs/): the firmware's stub is sent\n", want);
		return false;
	}
	*base = (uint64_t)handle << EFS_HANDLE_SHIFT;
	*bytes = size;
	return true;
}

static int
instantnoodle_ufs_read(__unused uint8_t lun, uint64_t offset, uint8_t *dst, uint64_t len)
{
	uint32_t handle = (uint32_t)(offset >> EFS_HANDLE_SHIFT);

	offset &= (1ull << EFS_HANDLE_SHIFT) - 1;
	if (handle == 0 || handle > fw_handle_count) {
		return EINVAL;
	}
	return fat_read(handle, fw_handles[handle - 1].size, offset, dst, len);
}

/* A file's vnode and size; the vnode is to be put. */
static int
fw_lookup(const char *path, vnode_t *vp, uint32_t *size)
{
	char full[256];
	struct vnode_attr va;
	vfs_context_t context = vfs_context_kernel();
	int error;

	while (*path == '/') {
		path++;
	}
	snprintf(full, sizeof(full), FW_ROOT "%s", path);
	if ((error = vnode_lookup(full, 0, vp, context)) != 0) {
		return error;
	}
	VATTR_INIT(&va);
	VATTR_WANTED(&va, va_data_size);
	VATTR_WANTED(&va, va_type);
	if ((error = vnode_getattr(*vp, &va, context)) != 0 || vnode_vtype(*vp) != VREG || va.va_data_size > UINT32_MAX) {
		vnode_put(*vp);
		return error != 0 ? error : EISDIR;
	}
	*size = (uint32_t)va.va_data_size;
	return 0;
}

/* The root mounted and /fw/modem on it: up to two minutes after the start, from the sequence's thread. */
static bool
fat_mount(void)
{
	vnode_t vp;

	for (unsigned waited = 0; !fat.ok && waited < 1200; waited++) {
		if (rootvnode != NULL && vnode_lookup(FW_ROOT "image/sdx55m", 0, &vp, vfs_context_kernel()) == 0) {
			vnode_put(vp);
			fat.ok = true;
			say("modem: fw: the firmware is in %s, %u s after it was first looked for\n", FW_ROOT, waited / 10);
			break;
		}
		delay_for_interval(100, NSEC_PER_MSEC);
	}
	if (!fat.ok) {
		say("modem: fw: NO %simage/sdx55m on the root: put the phone's /vendor/firmware_mnt/image/{sdx55m,modem_pr,sdx55} "
		    "into fw-modem/image and build the system image\n", FW_ROOT);
	}
	return fat.ok;
}

static bool
fat_open(const char *path, uint32_t *cluster, uint32_t *size)
{
	vnode_t vp;
	uint32_t handle = 0;

	if (!fat_mount() || fw_lookup(path, &vp, size) != 0) {
		say("modem: fw: %s: no %s\n", fw_partition, path);
		return false;
	}
	vnode_put(vp);
	lck_mtx_lock(&fw_lock);
	for (uint32_t i = 0; i < fw_handle_count && handle == 0; i++) {
		if (strcmp(fw_handles[i].path, path) == 0) {
			handle = i + 1;
		}
	}
	if (handle == 0 && fw_handle_count < FW_HANDLES) {
		strlcpy(fw_handles[fw_handle_count].path, path, sizeof(fw_handles[0].path));
		handle = ++fw_handle_count;
	}
	if (handle != 0) {
		fw_handles[handle - 1].size = *size;
	}
	lck_mtx_unlock(&fw_lock);
	if (handle == 0) {
		say("modem: fw: more than %u files open\n", FW_HANDLES);
		return false;
	}
	*cluster = handle;
	return true;
}

uint8_t *hakios_fw_partition_file(const char *path, uint64_t *size_out);
uint8_t *
hakios_fw_partition_file(const char *path, uint64_t *size_out);

/* `len` bytes of the file from `offset`; 0 or an errno. */
static int
fat_read(uint32_t handle, uint32_t size, uint64_t offset, uint8_t *dst, uint64_t len)
{
	char path[192];
	vfs_context_t context = vfs_context_kernel();
	vnode_t vp;
	uint32_t now;
	int error, left = 0;

	if (handle == 0 || handle > fw_handle_count || offset + len > size) {
		return EINVAL;
	}
	lck_mtx_lock(&fw_lock);
	strlcpy(path, fw_handles[handle - 1].path, sizeof(path));
	lck_mtx_unlock(&fw_lock);
	if ((error = fw_lookup(path, &vp, &now)) != 0) {
		return error;
	}
	while (len > 0 && error == 0) {
		int take = len > (64u << 20) ? (64 << 20) : (int)len;

		error = vn_rdwr(UIO_READ, vp, (caddr_t)dst, take, (off_t)offset, UIO_SYSSPACE, 0, vfs_context_ucred(context), &left,
		    vfs_context_proc(context));
		if (error == 0 && left != 0) {
			error = EIO;
		}
		dst += take;
		offset += (uint64_t)take;
		len -= (uint64_t)take;
	}
	vnode_put(vp);
	return error;
}

uint8_t *
hakios_fw_partition_file(const char *path, uint64_t *size_out)
{
	uint32_t cluster = 0, size = 0;
	uint8_t *data;

	if (!fat_open(path, &cluster, &size) || size == 0) {
		return NULL;
	}
	data = kalloc_data(size, Z_WAITOK);
	if (data == NULL) {
		return NULL;
	}
	if (fat_read(cluster, size, 0, data, size) != 0) {
		kfree_data(data, size);
		return NULL;
	}
	*size_out = size;
	return data;
}

/* ---- the boot interface: sbl1.mbn into the boot ROM ------------------------------------------------ */

#define BHI_STATUS_SHIFT        30
#define BHI_SBL_MAX             0x80000u
static bool option_probe;
static unsigned option_quiet;           /* modem_quiet=1: the mission's channels started on the first link and nothing said; 2: hellos answered, no lookups */
static uint32_t option_modes;           /* modem_modes=MASK: the radio technologies allowed (NAS mode preference; 0: as the modem has it) */
static bool option_full_cycle;          /* modem_cycle=1: the link reset by switching the controller off and on, not by PERST alone */               /* modem_probe=1: a first attempt with an address nothing maps */
static uint32_t
bhi_execenv(void)
{
	return rd(pcie.bar + pcie.bhi + BHI_EXECENV);
}

/* The device's MSI capability set up and switched on: its writes go to the GIC's SETSPI register through our context. Returns whether there is one. */
static bool
msi_enable(void)
{
	uint32_t cap = rd(pcie.conf + 0x34) & 0xff;

	for (unsigned guard = 0; cap != 0 && guard < 48; guard++) {
		uint32_t head = rd(pcie.conf + (cap & ~3u));
		unsigned id = head >> (8 * (cap & 3)) & 0xff, next = head >> (8 * (cap & 3) + 8) & 0xff;

		if (id == 0x05) {
			uint32_t control = rd(pcie.conf + cap) >> 16;
			bool wide = (control & 0x80) != 0;
			uint64_t address = MSI_IOVA + (MSI_REGISTER & 0xfff);

			wr(pcie.conf + cap + 4, (uint32_t)address);
			if (wide) {
				wr(pcie.conf + cap + 8, (uint32_t)(address >> 32));
				wr(pcie.conf + cap + 12, MSI_SPI);
			} else {
				wr(pcie.conf + cap + 8, MSI_SPI);
			}
			wr(pcie.conf + cap, (rd(pcie.conf + cap) & 0xffffu) | ((control & ~0x70u) | (control >> 1 & 7u) << 4 | 1u) << 16);   /* all the vectors it offers */
			say("modem: msi: capability at %#x (control %#x, %s): address %#llx, data %u, enabled\n", cap, rd(pcie.conf + cap) >> 16, wide ? "64-bit" : "32-bit", address, MSI_SPI);
			return true;
		}
		cap = next;
	}
	say("modem: msi: the device has no MSI capability\n");
	return false;
}

static bool
bhi_load(const char *path)
{
	uint32_t cluster, size, status = 0, ee = 0;
	uint64_t iova;
	uint8_t *image;
	vm_offset_t bar = pcie.bar, bhi = bar + pcie.bhi;

	if (!fat_open(path, &cluster, &size)) {
		say("modem: bhi: %s cannot be read from the modem partition\n", path);
		return false;
	}
	say("modem: bhi: %s is %u bytes, at cluster %u\n", path, size, cluster);
	if (!smmu_on()) {
		return false;
	}
	image = dma_alloc(size, &iova);
	if (image == NULL || fat_read(cluster, size, 0, image, size) != 0) {
		say("modem: bhi: the image did not get into memory the modem can be shown\n");
		return false;
	}
	dma_sync(image, size);
	msi_enable();
	{
		uint32_t sum = 0;

		for (uint32_t i = 0; i < size / 4; i++) {
			sum += ((const uint32_t *)image)[i];
		}
		say("modem: bhi: the image in memory: sum of its words %u (the file's, on the Mac: sbl1.mbn 1658231051), first words %08x %08x %08x %08x\n", sum,
		    ((const uint32_t *)image)[0], ((const uint32_t *)image)[1], ((const uint32_t *)image)[2], ((const uint32_t *)image)[3]);
	}
	if (option_probe) {
		/* a first try at an address nothing maps: if the modem's stream is ours, the context records a translation fault */
		uint64_t bad = DMA_IOVA + 0x10000000ull;

		say("modem: bhi: PROBE: the boot rom is first told the address %#llx, which nothing maps\n", bad);
		wr(bhi + BHI_STATUS, 0);
		wr(bhi + BHI_IMGADDR_HIGH, (uint32_t)(bad >> 32));
		wr(bhi + BHI_IMGADDR_LOW, (uint32_t)bad);
		wr(bhi + BHI_IMGSIZE, size);
		wr(bhi + BHI_INTVEC, 0);
		wr(bhi + BHI_IMGTXDB, 0x1234560);
		sleep_us(500000);
		say("modem: bhi: PROBE: STATUS %#x, ERRCODE %#x, ERRDBG %#x %#x %#x\n", rd(bhi + BHI_STATUS), rd(bhi + BHI_ERRCODE), rd(bhi + 0x34), rd(bhi + 0x38), rd(bhi + 0x3c));
		smmu_faults();
	}
	say("modem: bhi: THE BOOT ROM IS ABOUT TO BE HANDED %u bytes at its address %#llx (EXECENV %u, STATUS %#x)\n", size, iova, bhi_execenv(), rd(bhi + BHI_STATUS));
	wr(bhi + BHI_STATUS, 0);
	wr(bhi + BHI_IMGADDR_HIGH, (uint32_t)(iova >> 32));
	wr(bhi + BHI_IMGADDR_LOW, (uint32_t)iova);
	/* what the boot ROM is handed is at most the SBL's size (downstream mhi: sbl_size, 512 KB: "the maximum size, not necessarily the image size"); the rest of the file
	   is for the full-boot-chain download through BHIe once SBL runs */
	wr(bhi + BHI_IMGSIZE, size);
	wr(bhi + BHI_INTVEC, 0);
	wr(bhi + BHI_IMGTXDB, 0x1234567);
	for (unsigned ms = 0; ms < 5000; ms++) {
		sleep_us(1000);
		status = rd(bhi + BHI_STATUS) >> BHI_STATUS_SHIFT;
		ee = bhi_execenv();
		if (status != 0 || ee != 0) {
			break;
		}
	}
	say("modem: bhi: after the transfer: STATUS %#x (%u: 2 success, 3 error), EXECENV %u, ERRCODE %#x, ERRDBG %#x %#x %#x, MHISTATUS %#x\n", rd(bhi + BHI_STATUS), status, ee,
	    rd(bhi + BHI_ERRCODE), rd(bhi + 0x34), rd(bhi + 0x38), rd(bhi + 0x3c), rd(bar + MHI_STATUS));
	for (unsigned ms = 0; ms < 3000 && ee == 0; ms += 100) {
		sleep_us(100000);
		ee = bhi_execenv();
	}
	smmu_faults();
	say("modem: bhi: the modem is in execution environment %u (%s); MDM2AP status %u, error %u; MHISTATUS %#x\n", ee,
	    ee == 0 ? "PBL" : ee == 1 ? "SBL" : ee == 2 ? "AMSS" : "other", instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u,
	    instantnoodle_gpio_level(PIN_MDM2AP_ERRFATAL) ? 1u : 0u, rd(bar + MHI_STATUS));
	return status == 2;
}

/* ---- MHI (hakios.mhi.c) on the modem's BAR 0, and Sahara on its SBL's channels ------------------------------------ */

static uint32_t last_step;              /* (defined again below with its neighbours: the boot-arg's step) */
static uint32_t sahara_watch = 15, sahara_qrtr_seconds = 20;     /* modem_watch=S: after the images, how long to watch the modem */
static struct hakios_mhi mhi;
static LCK_MTX_DECLARE(mhi_mutex, &modem_pcie_lck_grp);

static uint32_t
mhi_op_read(void *context, uint32_t offset)
{
	(void)context;
	return pcie.up ? rd(pcie.bar + offset) : 0xffffffffu;
}

static void
mhi_op_write(void *context, uint32_t offset, uint32_t value)
{
	(void)context;
	/*
	 * Other threads get here (en1's uplink rings channel 100's doorbell): once the controller's clocks are off, an access to its BAR is an error on the bus that the
	 * whole phone goes to CrashDump for (2026-11-02: SYSTEM_NOC at 0x64300624, the modem started again by hand with mobile data up).
	 */
	if (pcie.up) {
		wr(pcie.bar + offset, value);
	}
}

static void *
mhi_op_alloc(void *context, size_t size, size_t align, uint64_t *address)
{
	(void)context;
	return dma_alloc_aligned(size, align, address);
}

static void
mhi_op_free(void *context, void *memory, size_t size)
{
	(void)context;
	(void)memory;
	(void)size;                     /* (the window is handed out upwards and given back whole, with the SMMU context) */
}

static uint64_t
mhi_op_nanoseconds(void *context)
{
	uint64_t abs = mach_absolute_time(), ns;

	(void)context;
	absolutetime_to_nanoseconds(abs, &ns);
	return ns;
}

static void
mhi_op_lock(void *context)
{
	(void)context;
	lck_mtx_lock(&mhi_mutex);
}

static void
mhi_op_unlock(void *context)
{
	(void)context;
	lck_mtx_unlock(&mhi_mutex);
}

static void
mhi_op_pause(void *context, unsigned microseconds)
{
	(void)context;
	sleep_us(microseconds);
}

static void
mhi_op_random(void *context, void *bytes, size_t length)
{
	uint64_t x = mach_absolute_time() * 6364136223846793005ull + 1442695040888963407ull;

	(void)context;
	for (size_t i = 0; i < length; i++) {
		x = x * 6364136223846793005ull + 1442695040888963407ull;
		((uint8_t *)bytes)[i] = (uint8_t)(x >> 56);
	}
}

static void
mhi_op_say(void *context, const char *line)
{
	(void)context;
	say("modem: %s\n", line);
}

static void
mhi_op_status(void *context, enum hakios_mhi_status what)
{
	(void)context;
	say("modem: mhi: the device reports %s\n", what == HAKIOS_MHI_MISSION ? "its mission" : what == HAKIOS_MHI_SYS_ERROR ? "a SYSTEM ERROR" : "a crash dump (RDDM)");
}

/* what the device sends on the Sahara channel: queued by the host's event service (which holds no lock then), read by the server's loop */
#define SAHARA_QUEUE            16
#define SAHARA_PACKET           0x400
#define IPCR_PACKET             0x2000
static struct {
	uint8_t data[SAHARA_QUEUE][IPCR_PACKET];
	uint32_t length[SAHARA_QUEUE], channel[SAHARA_QUEUE];
	volatile uint32_t head, tail;
} sahara_in;

/* mobile data: what the modem sends on its hardware IP channel (101), in a queue of its own so that the control channels' packets are not crowded out */
#define DATA_UL                 100
#define DATA_DL                 101
#define DATA_QUEUE              32
static struct {
	uint8_t data[DATA_QUEUE][IPCR_PACKET];
	uint32_t length[DATA_QUEUE];
	volatile uint32_t head, tail;
	uint64_t dropped;
} data_in;

static void
mhi_op_receive(void *context, unsigned channel, const void *data, size_t length)
{
	uint32_t at;

	(void)context;
	if (channel == DATA_DL) {
		at = data_in.head % DATA_QUEUE;
		if (data_in.head - data_in.tail >= DATA_QUEUE) {
			data_in.dropped++;
			return;
		}
		if (length > IPCR_PACKET) {
			length = IPCR_PACKET;
		}
		memcpy(data_in.data[at], data, length);
		data_in.length[at] = (uint32_t)length;
		__asm__ volatile ("dmb ish" ::: "memory");
		data_in.head++;
		return;
	}
	at = sahara_in.head % SAHARA_QUEUE;
	if (sahara_in.head - sahara_in.tail >= SAHARA_QUEUE) {
		say("modem: mhi: a packet of %zu bytes on channel %u dropped\n", length, channel);
		return;
	}
	if (length > IPCR_PACKET) {
		length = IPCR_PACKET;
	}
	memcpy(sahara_in.data[at], data, length);
	sahara_in.length[at] = (uint32_t)length;
	sahara_in.channel[at] = channel;
	__asm__ volatile ("dmb ish" ::: "memory");
	sahara_in.head++;
}

#pragma push_macro("say")
#undef say                              /* (the ops' member is named say) */
static const struct hakios_mhi_ops mhi_ops = {
	.read = mhi_op_read, .write = mhi_op_write, .dma_alloc = mhi_op_alloc, .dma_free = mhi_op_free, .nanoseconds = mhi_op_nanoseconds,
	.lock = mhi_op_lock, .unlock = mhi_op_unlock, .pause = mhi_op_pause, .random = mhi_op_random, .say = mhi_op_say, .status = mhi_op_status,
	.receive = mhi_op_receive,
};
#pragma pop_macro("say")

/* Sahara (Qualcomm's image transfer protocol): little-endian words, a header of command and length first */
enum { SAHARA_HELLO = 1, SAHARA_HELLO_RESP, SAHARA_READ_DATA, SAHARA_END_IMAGE_TX, SAHARA_DONE, SAHARA_DONE_RESP, SAHARA_RESET, SAHARA_RESET_RESP,
	SAHARA_MEM_DEBUG, SAHARA_MEM_READ, SAHARA_CMD_READY, SAHARA_CMD_SWITCH_MODE, SAHARA_CMD_EXEC, SAHARA_CMD_EXEC_RESP, SAHARA_CMD_EXEC_DATA,
	SAHARA_64BIT_MEM_DEBUG, SAHARA_64BIT_MEM_READ, SAHARA_READ_DATA64 };

/* image ids -> files of image/sdx55m/ (the ids are Qualcomm's: Linux's table for the QDU100, the same family, and the usual ones; the others are learnt from the requests) */
#define SAHARA_MAP_MAX 32
static struct { uint32_t id; char file[32]; } sahara_files[SAHARA_MAP_MAX] = {
	{ 21, "sbl1.mbn" }, { 25, "tz.mbn" }, { 23, "aop.mbn" }, { 8, "qdsp6sw.mbn" }, { 34, "mdmddr.mbn" }, { 36, "multi_image_qti.mbn" },
	{ 37, "multi_image.mbn" }, { 38, "xbl_cfg.elf" }, { 41, "devcfg.mbn" }, { 33, "hyp.mbn" }, { 16, "efs1.bin" }, { 17, "efs2.bin" }, { 20, "efs3.bin" },
	/* learnt on the phone, 2026-10-28 (the modem accepted all of them, its END_IMAGE_TX status 0): */
	{ 40, "apdp.mbn" }, { 42, "sec.elf" }, { 29, "acdb.mbn" }, { 6, "apps.mbn" },
};
static unsigned sahara_mapped = 17;
/* what an id the table lacks may be, in the order tried (an id's file is chosen once; modem_ids=ID=FILE,ID=FILE overrides and adds, to iterate without a build) */
static const char *const sahara_candidates[] = { "apdp.mbn", "sec.elf", "acdb.mbn", "apps.mbn" };

static struct {
	uint32_t served_images, bytes_hi, bytes_lo;
	bool done, finished;
	uint32_t unknown;
} sahara;

static int
sahara_send(const void *data, size_t length)
{
	return hakios_mhi_send(&mhi, 2, data, length);
}

static void
sahara_word_packet(uint32_t cmd, const uint32_t *words, uint32_t count)
{
	uint32_t packet[16] = { 0 };

	packet[0] = cmd;
	packet[1] = 8 + 4 * count;
	for (uint32_t i = 0; i < count && i < 14; i++) {
		packet[2 + i] = words[i];
	}
	if (sahara_send(packet, 8 + 4 * count) != 0) {
		say("modem: sahara: sending command %u failed\n", cmd);
	}
}

/* an image's bytes [offset, offset + length), to the device in pieces of what a transfer element holds */
static void
sahara_read_data(uint32_t id, uint64_t offset, uint64_t length)
{
	const char *file = NULL;
	char path[64];
	uint32_t cluster, size;
	static uint8_t chunk[0x2000];

	if (option_efs && (id == 16 || id == 17 || id == 20)) {
		/* the EFS images: what the phone's Android keeps in its mdm1m9kefs partitions, read only (the stubs of the firmware partition are 512 bytes of nothing) */
		static uint8_t part_chunk[0x2000];
		static const char *const names[] = { "mdm1m9kefs1", "mdm1m9kefs2", NULL, NULL, "mdm1m9kefs3" };
		uint64_t base, bytes;

		if (!gpt_find(5, names[id - 16], &base, &bytes)) {
			sahara.done = true;
			return;
		}
		say("modem: sahara: image %u = partition %s (%llu bytes): %llu bytes from %llu\n", id, names[id - 16], bytes, length, offset);
		while (length > 0) {
			uint64_t take = length < sizeof(part_chunk) ? length : sizeof(part_chunk);
			uint64_t have = offset >= bytes ? 0 : (bytes - offset < take ? bytes - offset : take);

			memset(part_chunk, 0, (size_t)take);
			if (have > 0 && instantnoodle_ufs_read(5, base + offset, part_chunk, have) != 0) {
				say("modem: sahara: partition %s could not be read at %llu\n", names[id - 16], offset);
				sahara.done = true;
				return;
			}
			if (sahara_send(part_chunk, (size_t)take) != 0) {
				say("modem: sahara: sending %llu bytes of the partition failed\n", take);
				sahara.done = true;
				return;
			}
			offset += take;
			length -= take;
			sahara.bytes_lo += (uint32_t)take;
			if (sahara.bytes_lo < take) {
				sahara.bytes_hi++;
			}
		}
		return;
	}
	for (unsigned i = 0; i < sahara_mapped; i++) {
		if (sahara_files[i].id == id) {
			file = sahara_files[i].file;
		}
	}
	for (unsigned c = 0; file == NULL && c < sizeof(sahara_candidates) / sizeof(sahara_candidates[0]) && sahara_mapped < SAHARA_MAP_MAX; c++) {
		bool taken = false;

		for (unsigned i = 0; i < sahara_mapped; i++) {
			taken = taken || strcmp(sahara_files[i].file, sahara_candidates[c]) == 0;
		}
		if (!taken) {
			sahara_files[sahara_mapped].id = id;
			strlcpy(sahara_files[sahara_mapped].file, sahara_candidates[c], sizeof(sahara_files[0].file));
			file = sahara_files[sahara_mapped++].file;
			say("modem: sahara: image %u is not in the table: GUESSED %s\n", id, file);
		}
	}
	if (file == NULL) {
		say("modem: sahara: THE DEVICE ASKS FOR IMAGE %u (offset %llu, length %llu), which this does not know: not answered\n", id, offset, length);
		sahara.unknown = id;
		sahara.done = true;
		return;
	}
	snprintf(path, sizeof(path), "image/sdx55m/%s", file);
	if (!fat_open(path, &cluster, &size)) {
		sahara.done = true;
		return;
	}
	{
		static unsigned logged;

		if (logged++ < 120 || length >= 0x100000) {
			say("modem: sahara: image %u = %s (%u bytes): %llu bytes from %llu\n", id, file, size, length, offset);
		}
	}
	while (length > 0) {
		uint64_t take = length < sizeof(chunk) ? length : sizeof(chunk);
		uint64_t have = offset >= size ? 0 : (size - offset < take ? size - offset : take);

		memset(chunk, 0, (size_t)take);
		if (have > 0 && fat_read(cluster, size, offset, chunk, have) != 0) {
			say("modem: sahara: %s could not be read at %llu\n", file, offset);
			sahara.done = true;
			return;
		}
		if (sahara_send(chunk, (size_t)take) != 0) {
			say("modem: sahara: sending %llu bytes of %s failed\n", take, file);
			sahara.done = true;
			return;
		}
		offset += take;
		length -= take;
		sahara.bytes_lo += (uint32_t)take;
		if (sahara.bytes_lo < take) {
			sahara.bytes_hi++;
		}
	}
}

static void
sahara_packet(const uint8_t *p, uint32_t length)
{
	const uint32_t *w = (const uint32_t *)p;
	uint32_t cmd = w[0];

	if (length < 8) {
		return;
	}
	switch (cmd) {
	case SAHARA_HELLO: {
		uint32_t reply[10] = { 2, 1, 0, w[5], 0, 0, 0, 0, 0, 0 };      /* version, compatible version, status (0), mode, and six reserved words: 0x30 bytes with the header */

		say("modem: sahara: HELLO: version %u (compatible from %u), command packets up to %u bytes, mode %u\n", w[2], w[3], w[4], w[5]);
		sahara_word_packet(SAHARA_HELLO_RESP, reply, 10);
		break;
	}
	case SAHARA_READ_DATA:
		sahara_read_data(w[2], w[3], w[4]);
		break;
	case SAHARA_READ_DATA64:
		sahara_read_data(w[2], (uint64_t)w[5] << 32 | w[4], (uint64_t)w[7] << 32 | w[6]);
		break;
	case SAHARA_END_IMAGE_TX:
		say("modem: sahara: image %u ends with status %u\n", w[2], w[3]);
		if (w[3] == 0) {
			sahara.served_images++;
			sahara_word_packet(SAHARA_DONE, NULL, 0);
		} else {
			sahara.done = true;
		}
		break;
	case SAHARA_DONE_RESP:
		say("modem: sahara: DONE answered: image transfer status %u (0: more images, 1: complete)\n", w[2]);
		if (w[2] != 0) {
			sahara.finished = true;
		}
		break;
	case SAHARA_CMD_READY: {
		uint32_t mode = 0;                      /* back to "image transfer pending" */

		say("modem: sahara: command mode ready: switching it to image transfer\n");
		sahara_word_packet(SAHARA_CMD_SWITCH_MODE, &mode, 1);
		break;
	}
	case SAHARA_RESET_RESP:
		say("modem: sahara: command %u\n", cmd);
		break;
	case SAHARA_MEM_DEBUG:
	case SAHARA_64BIT_MEM_DEBUG:
		/* the modem has crashed and offers its memory (Sahara mode 2): not taken; it is asked to start again, or it waits here for ever and the phone has no network until the next boot */
		say("modem: sahara: the modem offers a crash dump: not taken, it is reset (what it said: the \"sfr:\" line)\n");
		sahara_word_packet(SAHARA_RESET, NULL, 0);
		break;
	default:
		say("modem: sahara: UNEXPECTED command %u, %u bytes, first words %08x %08x %08x %08x\n", cmd, length, w[2], w[3], w[4], w[5]);
		break;
	}
}


/* ---- QRTR on IPCR: the modem's services ------------------------------------------------------------ */

#define QRTR_US                 1u              /* we are node 1, as Android is on this phone */
#define QRTR_CONTROL            0xfffffffeu
enum { QRTR_DATA = 1, QRTR_HELLO = 2, QRTR_BYE = 3, QRTR_NEW_SERVER = 4, QRTR_DEL_SERVER = 5, QRTR_DEL_CLIENT = 6, QRTR_RESUME_TX = 7, QRTR_NEW_LOOKUP = 10 };

struct qrtr_header {
	uint32_t version, type, from_node, from_port, confirm, size, to_node, to_port;
};

static bool qrtr_v2;
static unsigned qrtr_channel = 32;
/* one link per node of the modem: the MHI channel pair its packets use and the header version (node 2: QMI0 20/21, version 2; node 3: DUN 50/51, version 1) */
static struct { uint32_t node; unsigned ul; bool v2; bool used; } qrtr_routes[4];
static struct { bool on; unsigned ul; bool v2; } qrtr_force;       /* a try of a route other than the node's own */      /* the channel QRTR leaves on: IPCR's 32, or QMI0's 20 once the modem has shown it speaks QRTR there */
static struct {
	uint32_t node, hellos, packets_in, packets_out;
	unsigned servers;
	struct { uint32_t service, instance, node, port; } server[64];
} qrtr;

/* (from_node and confirm: what the bridge between the modem and the audio DSP forwards keeps its own: qrtr_bridge_drain()) */
static int
qrtr_send_as(uint32_t from_node, bool confirm, uint32_t type, uint32_t from_port, uint32_t to_node, uint32_t to_port, const void *data, uint32_t length)
{
	/* the link of the node asked for (a broadcast, or an unknown node, goes by the default) */
	for (unsigned r = 0; r < 4; r++) {
		if (qrtr_routes[r].used && qrtr_routes[r].node == to_node) {
			qrtr_channel = qrtr_routes[r].ul;
			qrtr_v2 = qrtr_routes[r].v2;
		}
	}
	if (qrtr_force.on) {
		qrtr_channel = qrtr_force.ul;
		qrtr_v2 = qrtr_force.v2;
	}
	static uint8_t packet[sizeof(struct qrtr_header) + IPCR_PACKET];
	struct qrtr_header *h = (struct qrtr_header *)packet;
	uint32_t padded = (length + 3) & ~3u;

	if (length == 0 || padded > IPCR_PACKET) {
		return EMSGSIZE;
	}
	if (qrtr_v2) {
		/* version 2: version 3, type, flags, option length, size (32 bits), then source node and port, destination node and port, of 16 bits each */
		memset(packet, 0, 16 + padded);
		packet[0] = 3;
		packet[1] = (uint8_t)type;
		packet[2] = confirm ? 1 : 0;
		packet[3] = 0;
		memcpy(packet + 4, &length, 4);
		packet[8] = (uint8_t)from_node;
		packet[9] = (uint8_t)(from_node >> 8);
		packet[10] = (uint8_t)(from_port == QRTR_CONTROL ? 0xfe : from_port);
		packet[11] = (uint8_t)((from_port == QRTR_CONTROL ? 0xff : from_port >> 8));
		packet[12] = (uint8_t)to_node;
		packet[13] = (uint8_t)(to_node >> 8);
		packet[14] = (uint8_t)(to_port == QRTR_CONTROL ? 0xfe : to_port);
		packet[15] = (uint8_t)(to_port == QRTR_CONTROL ? 0xff : to_port >> 8);
		memcpy(packet + 16, data, length);
		qrtr.packets_out++;
		return hakios_mhi_send(&mhi, qrtr_channel, packet, 16 + padded);
	}
	memset(packet, 0, sizeof(*h) + padded);
	h->version = 1;
	h->type = type;
	h->from_node = from_node;
	h->from_port = from_port;
	h->confirm = confirm;
	h->size = length;
	h->to_node = to_node;
	h->to_port = to_port;
	memcpy(packet + sizeof(*h), data, length);
	qrtr.packets_out++;
	return hakios_mhi_send(&mhi, qrtr_channel, packet, sizeof(*h) + padded);
}

static int
qrtr_send(uint32_t type, uint32_t from_port, uint32_t to_node, uint32_t to_port, const void *data, uint32_t length)
{
	return qrtr_send_as(QRTR_US, false, type, from_port, to_node, to_port, data, length);
}

/*
 * The bridge between the modem and the audio DSP, boot-arg voice_bridge=1. In Android's device tree they are on different QRTR networks (modem net-id 3, DSP net-id 2) and the application
 * processor's router forwards between them (net/qrtr, source: Android's; inferred from the device tree that a call's sound needs it: a call was silent with every voice command accepted, and
 * neither side said anything). What the DSP sends for the modem's nodes comes here from the DSP link's thread (hakios_modem_qrtr_from_dsp()) and goes out from the mission's thread: this
 * side's sending is not to be entered twice. One writer, one reader. What the modem sends for the DSP's node: hakios_adsp_qrtr_from_modem(), hakios.adsp.c. The first lines of each say what crossed.
 */
#define BRIDGE_DSP_NODE         5u
#define BRIDGE_SLOTS            32u
#define BRIDGE_BYTES            1536u
extern void hakios_adsp_qrtr_from_modem(uint32_t type, uint32_t from_node, uint32_t from_port, uint32_t to_node, uint32_t to_port, bool confirm, const void *data, uint32_t length);
extern void hakios_adsp_qrtr_announce(void);
static bool option_bridge;
static unsigned bridge_said;
static struct {
	uint32_t type, from_node, from_port, to_node, to_port, length;
	bool confirm;
	uint8_t data[BRIDGE_BYTES];
} bridge[BRIDGE_SLOTS];
static volatile uint32_t bridge_in, bridge_out;

void hakios_modem_qrtr_from_dsp(uint32_t type, uint32_t from_node, uint32_t from_port, uint32_t to_node, uint32_t to_port, bool confirm, const void *data, uint32_t length);
void
hakios_modem_qrtr_from_dsp(uint32_t type, uint32_t from_node, uint32_t from_port, uint32_t to_node, uint32_t to_port, bool confirm, const void *data, uint32_t length)
{
	uint32_t in = bridge_in;

	if (!option_bridge) {
		return;
	}
	if (in - bridge_out >= BRIDGE_SLOTS || length > BRIDGE_BYTES) {
		say("modem: bridge: dropped %u bytes from the DSP (%u waiting)\n", length, in - bridge_out);
		return;
	}
	bridge[in % BRIDGE_SLOTS].type = type;
	bridge[in % BRIDGE_SLOTS].from_node = from_node;
	bridge[in % BRIDGE_SLOTS].from_port = from_port;
	bridge[in % BRIDGE_SLOTS].to_node = to_node;
	bridge[in % BRIDGE_SLOTS].to_port = to_port;
	bridge[in % BRIDGE_SLOTS].confirm = confirm;
	bridge[in % BRIDGE_SLOTS].length = length;
	memcpy(bridge[in % BRIDGE_SLOTS].data, data, length);
	__atomic_store_n(&bridge_in, in + 1, __ATOMIC_RELEASE);
}

/*
 * Each told that the other is up, once the modem runs, as Android's sysmon does (mainline Linux: remoteproc/qcom_sysmon.c, ssctl_send_event(): SSCTL, service 43, message 0x23, TLV 1 the
 * subsystem's name, TLV 2 the event, 3 after power-up, TLV 0x10 how it came about, 0 forced). Hypothesis (2026-11-02): the DSP started first, never heard that the modem exists, and so answers
 * nothing of its voice commands. The modem's name for the DSP is "adsp"; the DSP's for the modem is one of the two below, by what Android's device tree calls an external modem.
 */
extern uint32_t hakios_adsp_ssctl_port(void);
#define SSCTL_SERVICE           43u
#define BRIDGE_CLIENT_PORT      0x4005u

static uint32_t
ssctl_event(uint8_t *q, uint16_t transaction, const char *name)
{
	uint32_t n = (uint32_t)strlen(name), at = 7;

	q[0] = 0;
	q[1] = (uint8_t)transaction;
	q[2] = (uint8_t)(transaction >> 8);
	q[3] = 0x23;
	q[4] = 0;
	q[at++] = 0x01;
	q[at++] = (uint8_t)(n + 1);
	q[at++] = 0;
	q[at++] = (uint8_t)n;
	memcpy(q + at, name, n);
	at += n;
	q[at++] = 0x02;
	q[at++] = 4;
	q[at++] = 0;
	q[at++] = 3;
	q[at++] = 0;
	q[at++] = 0;
	q[at++] = 0;
	q[at++] = 0x10;
	q[at++] = 4;
	q[at++] = 0;
	q[at++] = 0;
	q[at++] = 0;
	q[at++] = 0;
	q[at++] = 0;
	q[5] = (uint8_t)(at - 7);
	q[6] = 0;
	return at;
}

static void
qrtr_bridge_tell_up(void)
{
	static bool told;
	static const char * const dsp_names[] = { "esoc0", "modem" };
	uint8_t q[64];
	uint32_t dsp = hakios_adsp_ssctl_port();

	if (told || !cell_started || dsp == 0) {
		return;
	}
	told = true;
	{
		/*
		 * The modem's nodes introduced to the DSP, and the DSP to the modem: the DSP answers node 2 and has never sent anything to node 3, where the modem's voice commands come from
		 * (hypothesis: it has no route to it; a router learns a neighbour from its HELLO). And the modem's services told again now that the DSP knows the nodes.
		 */
		uint32_t hello[5] = { QRTR_HELLO, 0, 0, 0, 0 };

		for (unsigned r = 0; r < 4; r++) {
			if (qrtr_routes[r].used) {
				say("modem: bridge: node %u says hello to the DSP\n", qrtr_routes[r].node);
				hakios_adsp_qrtr_from_modem(QRTR_HELLO, qrtr_routes[r].node, QRTR_CONTROL, BRIDGE_DSP_NODE, QRTR_CONTROL, false, hello, sizeof(hello));
			}
		}
		say("modem: bridge: the DSP says hello to the modem: %d\n", qrtr_send_as(BRIDGE_DSP_NODE, false, QRTR_HELLO, QRTR_CONTROL, 2, QRTR_CONTROL, hello, sizeof(hello)));
		for (unsigned i = 0; i < qrtr.servers && i < 64; i++) {
			uint32_t again[5] = { QRTR_NEW_SERVER, qrtr.server[i].service, qrtr.server[i].instance, qrtr.server[i].node, qrtr.server[i].port };

			hakios_adsp_qrtr_from_modem(QRTR_NEW_SERVER, qrtr.server[i].node, QRTR_CONTROL, BRIDGE_DSP_NODE, QRTR_CONTROL, false, again, sizeof(again));
		}
	}
	for (unsigned i = 0; i < 2; i++) {
		uint32_t n = ssctl_event(q, (uint16_t)(0x100 + i), dsp_names[i]);

		say("modem: bridge: telling the DSP that \"%s\" is up (its SSCTL, port %u)\n", dsp_names[i], dsp);
		hakios_adsp_qrtr_from_modem(QRTR_DATA, QRTR_US, BRIDGE_CLIENT_PORT, BRIDGE_DSP_NODE, dsp, false, q, n);
	}
	for (unsigned i = 0; i < qrtr.servers && i < 64; i++) {
		if (qrtr.server[i].service == SSCTL_SERVICE) {
			uint32_t n = ssctl_event(q, 0x200, "adsp");

			say("modem: bridge: telling the modem that the DSP is up (node %u port %u): %d\n", qrtr.server[i].node, qrtr.server[i].port,
			    qrtr_send(QRTR_DATA, BRIDGE_CLIENT_PORT, qrtr.server[i].node, qrtr.server[i].port, q, n));
		}
	}
}

/*
 * The audio DSP's own road to the modem, boot-arg voice_sat=1: Android's drivers/bus/mhi/devices/mhi_satellite.c. Seen on Android on this phone (2026-11-02, kprobes on a real call): no voice
 * command of the modem's passes this processor's QRTR, the DSP and the modem talk over MHI channels of their own, ADSP_0 (50), ADSP_1 (51), ADSP_2 (70), ADSP_3 (71), event ring 4, "offload"
 * in the device tree. This processor only lets them: on the GLINK channel "mhi_sat" it tells the DSP where the modem is (the BAR, the event ring, M0, the mission mode), and does what the DSP
 * asks: maps memory into the modem's window, writes context entries that say where the DSP keeps the real ones, starts channels; each message is answered with the same elements
 * made completions, the message's sequence number in reply_seq.
 */
#define SAT_DEVICE_ID           (0x0306u << 16 | 2u << 12 | 1u << 4 | 0u)       /* MHI_SAT_CREATE_DEVICE_ID: device 0306, domain 2, bus 1, slot 0 ("0306_02.01.00" on Android) */
#define SAT_EVENT_RING          4u
#define SAT_SLOTS               8u
#define SAT_BYTES               1024u
#define SAT_ID_CMD              0xc
#define SAT_ID_EVT              0xe
#define SAT_MHI_M0              2u
#define SAT_MHI_EE_AMSS         2u
#define SAT_CC_SUCCESS          1u
#define SAT_CC_BAD_TRE          0x11u
extern bool hakios_adsp_sat_open(void);
extern int hakios_adsp_sat_send(const void *data, uint32_t length);

struct __attribute__((packed)) sat_header {
	uint16_t major, minor, id, seq, reply_seq, payload;
	uint32_t device;
	uint8_t reserved[8];
};
struct __attribute__((packed)) sat_tre {
	uint64_t pointer;
	uint32_t word[2];
};
static bool option_sat;
static uint16_t sat_seq;
static unsigned sat_said;
static struct {
	uint32_t length;
	uint8_t data[SAT_BYTES];
} sat_in[SAT_SLOTS];
static volatile uint32_t sat_in_head, sat_in_tail;

void hakios_modem_sat_from_dsp(const void *data, uint32_t length);
void
hakios_modem_sat_from_dsp(const void *data, uint32_t length)
{
	uint32_t head = sat_in_head;

	if (head - sat_in_tail >= SAT_SLOTS || length > SAT_BYTES) {
		say("modem: sat: dropped %u bytes from the DSP\n", length);
		return;
	}
	sat_in[head % SAT_SLOTS].length = length;
	memcpy(sat_in[head % SAT_SLOTS].data, data, length);
	__atomic_store_n(&sat_in_head, head + 1, __ATOMIC_RELEASE);
}

static void
sat_send(uint16_t id, uint16_t reply_seq, void *message, uint32_t length)
{
	struct sat_header *header = message;

	sat_seq = sat_seq + 1 == 0xffff ? 0 : sat_seq + 1;
	header->major = 1;
	header->minor = 0;
	header->id = id;
	header->seq = sat_seq;
	header->reply_seq = reply_seq;
	header->payload = (uint16_t)(length - sizeof(*header));
	header->device = SAT_DEVICE_ID;
	bzero(header->reserved, sizeof(header->reserved));
	int error = hakios_adsp_sat_send(message, length);

	if (error != 0) {
		say("modem: sat: not sent to the DSP: %d\n", error);
	}
}

/* on the mission's thread */
static bool sat_hello;                  /* the DSP has been told of this run of the modem */

/*
 * The modem has gone: the DSP is told as Android tells it (mhi_satellite.c, mhi_sat_send_sys_err(): one command, type 0xff), so that it lets go of the channels it had, and
 * the next run's hello finds it ready. Its answer is not waited for: the modem is not there to serve anything it would ask.
 */
static void
sat_modem_gone(void)
{
	struct {
		struct sat_header header;
		struct sat_tre tre;
	} __attribute__((packed)) message;

	if (!sat_hello) {
		return;
	}
	sat_hello = false;
	bzero(&message, sizeof(message));
	message.tre.word[1] = 0xffu << 16;
	say("modem: sat: the DSP told that the modem has gone\n");
	sat_send(SAT_ID_CMD, 0xffff, &message, sizeof(message));
}

static bool
sat_poll(void)
{
	bool any = false;

	if (!sat_hello && cell_started && hakios_adsp_sat_open()) {
		struct {
			struct sat_header header;
			struct sat_tre tre[3];
		} __attribute__((packed)) message;

		sat_hello = true;
		bzero(&message, sizeof(message));
		message.tre[0].pointer = BAR_PHYS;
		message.tre[0].word[0] = SAT_EVENT_RING << 16 | 1u;
		message.tre[0].word[1] = 0x6eu << 16;
		message.tre[1].word[0] = SAT_MHI_M0 << 24;
		message.tre[1].word[1] = 0x20u << 16;
		message.tre[2].word[0] = SAT_MHI_EE_AMSS << 24;
		message.tre[2].word[1] = 0x40u << 16;
		say("modem: sat: hello to the DSP: the modem at %#lx, event ring %u, M0, mission mode\n", BAR_PHYS, SAT_EVENT_RING);
		sat_send(SAT_ID_EVT, 0xffff, &message, sizeof(message));
		any = true;
	}
	while (sat_in_tail != __atomic_load_n(&sat_in_head, __ATOMIC_ACQUIRE)) {
		uint8_t *data = sat_in[sat_in_tail % SAT_SLOTS].data;
		uint32_t length = sat_in[sat_in_tail % SAT_SLOTS].length;
		struct sat_header *header = (struct sat_header *)data;

		any = true;
		if (length < sizeof(*header) || length != sizeof(*header) + header->payload || header->major != 1) {
			say("modem: sat: a message that is none (%u bytes)\n", length);
		} else if (header->id == SAT_ID_CMD) {
			struct sat_tre *tre = (struct sat_tre *)(data + sizeof(*header));

			for (uint32_t i = 0; i < header->payload / sizeof(*tre); i++, tre++) {
				uint32_t type = tre->word[1] >> 16 & 0xff, number = tre->word[1] >> 24, code = SAT_CC_BAD_TRE;
				uint64_t pointer = tre->pointer, answer = 0;
				uint32_t size = tre->word[0];
				bool event = (tre->word[1] & 1) != 0;

				switch (type) {
				case 0x65: {    /* map: the memory of the DSP's rings into the modem's window */
					uint64_t iova = 0, start = pointer & ~(uint64_t)4095;

					if (hakios_modem_map_contig(start, (size_t)(pointer - start) + size, &iova)) {
						answer = iova + (pointer - start);
						code = SAT_CC_SUCCESS;
					}
					break;
				}
				case 0x64:      /* a context entry */
					if (hakios_mhi_sat_context(&mhi, event, number, pointer, size) == 0) {
						code = SAT_CC_SUCCESS;
					}
					break;
				case 0x12:      /* start a channel */
					if (hakios_mhi_sat_command(&mhi, number, true) == 0) {
						code = SAT_CC_SUCCESS;
					}
					break;
				case 0x10:      /* reset a channel */
					hakios_mhi_sat_command(&mhi, number, false);
					code = SAT_CC_SUCCESS;
					break;
				default:
					break;
				}
				if (sat_said++ < 80) {
					say("modem: sat: the DSP asks %#x: %#llx, %u bytes, id %u%s: %s, %#llx\n", type, pointer, size, number, event ? " (event)" : "", code == SAT_CC_SUCCESS ? "done" : "FAILED", answer);
				}
				tre->pointer = answer;
				tre->word[0] = code << 24;
				tre->word[1] = 0x21u << 16;
			}
			sat_send(SAT_ID_EVT, header->seq, data, length);
		} else if (sat_said++ < 80) {
			say("modem: sat: from the DSP: id %#x, seq %u, %u bytes\n", header->id, header->seq, header->payload);
		}
		__atomic_store_n(&sat_in_tail, sat_in_tail + 1, __ATOMIC_RELEASE);
	}
	return any;
}

/* on the mission's thread: true if there was something (a control message for "the modem" goes to each of its nodes we know) */
static bool
qrtr_bridge_drain(void)
{
	bool any = false;

	qrtr_bridge_tell_up();

	while (bridge_out != __atomic_load_n(&bridge_in, __ATOMIC_ACQUIRE)) {
		typeof(&bridge[0]) b = &bridge[bridge_out % BRIDGE_SLOTS];
		int error = 0;

		if (b->to_port == QRTR_CONTROL) {
			for (unsigned r = 0; r < 4; r++) {
				if (qrtr_routes[r].used) {
					error = qrtr_send_as(b->from_node, b->confirm, b->type, b->from_port, qrtr_routes[r].node, b->to_port, b->data, b->length);
				}
			}
		} else {
			error = qrtr_send_as(b->from_node, b->confirm, b->type, b->from_port, b->to_node, b->to_port, b->data, b->length);
		}
		if (bridge_said++ < 200) {
			say("modem: bridge: DSP -> modem: type %u, node %u port %#x to node %u port %#x, %u bytes: %02x %02x %02x %02x %02x %02x %02x %02x: %d\n", b->type, b->from_node, b->from_port, b->to_node, b->to_port, b->length,
			    b->data[0], b->data[1], b->data[2], b->data[3], b->data[4], b->data[5], b->data[6], b->data[7], error);
		}
		__atomic_store_n(&bridge_out, bridge_out + 1, __ATOMIC_RELEASE);
		any = true;
	}
	return any;
}

/* a QMI client over QRTR: our one port, the reply of the request in flight */
#define QMI_PORT                0x4000u
#define QMI_PORT6               0x4003u         /* the second QMI client (the IPv6 data session: WDS clients are per QRTR port) */
static uint32_t qmi_src_port = QMI_PORT;       /* the port the next request goes out from */
#define IPA_SERVER_PORT         0x4001u         /* our side of the IPA QMI service (0x31, version 1, instance 1): the modem's driver is its client */
static void ipa_server_request(uint32_t from_node, uint32_t from_port, const uint8_t *q, uint32_t length);
static struct {
	bool have;
	uint16_t transaction, message, result, error;
	uint8_t type;
	uint32_t length;
	uint8_t data[IPCR_PACKET];
} qmi_reply;


/*
 * The AP's TFTP server over QRTR (service 4096, as Android's tftp_server / tqftpserv): the modem's file system client (MPSS and the SDX's application processor, rfs_tftp.c) reads
 * its read-only files ("/readonly/firmware/image/...", which is the modem partition's FAT) and writes its EFS through it. Read: from the FAT of the modem partition. Write: accepted and
 * thrown away (nothing of the phone's storage is written), said in the log. RFC 1350 with the options of RFC 2347/2348/2349 (blksize, tsize, timeout).
 */
#define TFTP_PORT               0x4002u
#define TFTP_SERVICE            4096u
static bool option_resume_all;          /* modem_resume=1: RESUME_TX after every datagram, asked or not */
static unsigned option_blk = 1024;     /* modem_blk=N: the largest TFTP block offered (what the device takes in one MHI element is learnt by trying) */
static bool option_tftp;                /* modem_tftp=1: the server announced and answering */
static struct {
	bool used, writing;
	uint32_t node, port, cluster, size, block, blksize, written, base;
	char name[80];
} tftp_sessions[8];
static unsigned tftp_logged, tftp_pkt_logged;

static void
tftp_reply(uint32_t node, uint32_t port, const uint8_t *p, uint32_t n)
{
	int e = qrtr_send(QRTR_DATA, TFTP_PORT, node, port, p, n);

	if (e != 0 || n > 600) {
		static unsigned logged;

		if (logged++ < 40) {
			say("modem: tftp: reply of %u bytes (opcode %u) to node %u port %u: %d\n", n, n >= 2 ? (uint32_t)(p[0] << 8 | p[1]) : 0, node, port, e);
		}
	}
}

static void
tftp_error(uint32_t node, uint32_t port, uint16_t code, const char *text)
{
	uint8_t p[64];
	size_t n = strlen(text);

	p[0] = 0; p[1] = 5; p[2] = (uint8_t)(code >> 8); p[3] = (uint8_t)code;
	memcpy(p + 4, text, n + 1);
	tftp_reply(node, port, p, (uint32_t)(4 + n + 1));
}

static void
tftp_data(unsigned s)
{
	static uint8_t p[4 + 4096];
	uint32_t offset = (tftp_sessions[s].block - 1) * tftp_sessions[s].blksize, n = 0;

	if (offset < tftp_sessions[s].size) {
		n = tftp_sessions[s].size - offset;
		n = n > tftp_sessions[s].blksize ? tftp_sessions[s].blksize : n;
		if (fat_read(tftp_sessions[s].cluster, tftp_sessions[s].base + tftp_sessions[s].size, tftp_sessions[s].base + offset, p + 4, n) != 0) {
			tftp_error(tftp_sessions[s].node, tftp_sessions[s].port, 2, "read failed");
			tftp_sessions[s].used = false;
			return;
		}
	}
	p[0] = 0; p[1] = 3; p[2] = (uint8_t)(tftp_sessions[s].block >> 8); p[3] = (uint8_t)tftp_sessions[s].block;
	tftp_reply(tftp_sessions[s].node, tftp_sessions[s].port, p, 4 + n);
}

static void
tftp_packet(uint32_t node, uint32_t port, const uint8_t *p, uint32_t length)
{
	unsigned s = 8;
	uint32_t opcode = length >= 2 ? (uint32_t)(p[0] << 8 | p[1]) : 0;

	for (unsigned i = 0; i < 8; i++) {
		if (tftp_sessions[i].used && tftp_sessions[i].node == node && tftp_sessions[i].port == port) {
			s = i;
		}
	}
	if (tftp_pkt_logged < 150 && opcode != 1 && opcode != 2) {
		tftp_pkt_logged++;
		say("modem: tftp: opcode %u, %u bytes, block %u, from node %u port %u\n", opcode, length, length >= 4 ? (uint32_t)(p[2] << 8 | p[3]) : 0, node, port);
	}
	if (opcode == 1 || opcode == 2) {
		/* a request: file name, mode, then option and value pairs, each ended by a zero */
		const char *name = (const char *)p + 2, *at, *end = (const char *)p + length;
		char opts[160];
		uint32_t blksize = 512, oack_n = 0, tsize_asked = 0, size = 0, cluster = 0, seek = 0, rsize = 0xffffffffu;
		bool oack = false, found = false, wsize_asked = false;
		static uint8_t o[96];
		char path[256];
		const char *rest = NULL;

		if (s == 8) {
			for (unsigned i = 0; i < 8 && s == 8; i++) {
				if (!tftp_sessions[i].used) {
					s = i;
				}
			}
			if (s == 8) {
				s = 0;                  /* (the oldest is overwritten) */
			}
		}
		opts[0] = 0;
		at = name + strnlen(name, (size_t)(end - name)) + 1;
		at += at < end ? strnlen(at, (size_t)(end - at)) + 1 : 0;       /* the mode */
		while (at < end) {
			const char *value = at + strnlen(at, (size_t)(end - at)) + 1;

			if (value >= end) {
				break;
			}
			if (strncasecmp(at, "blksize", 8) == 0) {
				blksize = (uint32_t)strtoul(value, NULL, 10);
				blksize = blksize < 8 ? 8 : blksize > option_blk ? option_blk : blksize;
				oack = true;
			} else if (strncasecmp(at, "tsize", 6) == 0) {
				tsize_asked = 1;
				oack = true;
			} else if (strncasecmp(at, "wsize", 6) == 0) {
				wsize_asked = true;
				oack = true;
			} else if (strncasecmp(at, "seek", 5) == 0) {
				seek = (uint32_t)strtoul(value, NULL, 10);
			} else if (strncasecmp(at, "rsize", 6) == 0) {
				rsize = (uint32_t)strtoul(value, NULL, 10);
			} else if (strncasecmp(at, "timeout", 8) == 0) {
				oack = true;
			}
			strlcat(opts, at, sizeof(opts));
			strlcat(opts, "=", sizeof(opts));
			strlcat(opts, value, sizeof(opts));
			strlcat(opts, " ", sizeof(opts));
			at = value + strnlen(value, (size_t)(end - value)) + 1;
		}
		strlcpy(path, name, sizeof(path));
		if (tftp_logged < 60) {
			tftp_logged++;
			say("modem: tftp: %s \"%s\" from node %u port %u, options: %s\n", opcode == 1 ? "read" : "WRITE", path, node, port, opts);
		}
		tftp_sessions[s].used = true;
		tftp_sessions[s].node = node;
		tftp_sessions[s].port = port;
		tftp_sessions[s].block = 0;
		tftp_sessions[s].blksize = blksize;
		tftp_sessions[s].writing = opcode == 2;
		tftp_sessions[s].written = 0;
		strlcpy(tftp_sessions[s].name, path, sizeof(tftp_sessions[s].name));
		if (opcode == 1) {
			if (strncmp(path, "/readonly/firmware/", 19) == 0) {
				rest = path + 19;
			} else if (strncmp(path, "/readonly/", 10) == 0) {
				rest = path + 10;
			}
			if (rest != NULL && fat_open(rest, &cluster, &size)) {
				found = true;
			}
			if (!found) {
				tftp_error(node, port, 1, "File not found");
				tftp_sessions[s].used = false;
				return;
			}
			tftp_sessions[s].cluster = cluster;
			seek = seek > size ? size : seek;
			size -= seek;
			size = size > rsize ? rsize : size;
			tftp_sessions[s].base = seek;
			tftp_sessions[s].size = size;
		}
		if (oack) {
			o[0] = 0; o[1] = 6;
			oack_n = 2;
			oack_n += (uint32_t)snprintf((char *)o + oack_n, sizeof(o) - oack_n, "blksize%c%u%c", 0, blksize, 0);
			if (wsize_asked) {
				oack_n += (uint32_t)snprintf((char *)o + oack_n, sizeof(o) - oack_n, "wsize%c1%c", 0, 0);       /* (one block, then its acknowledgement: the window the client asked for is not given) */
			}
			if (tsize_asked && opcode == 1) {
				oack_n += (uint32_t)snprintf((char *)o + oack_n, sizeof(o) - oack_n, "tsize%c%u%c", 0, size, 0);
			}
			tftp_reply(node, port, o, oack_n);
		} else if (opcode == 1) {
			tftp_sessions[s].block = 1;
			tftp_data(s);
		} else {
			o[0] = 0; o[1] = 4; o[2] = 0; o[3] = 0;
			tftp_reply(node, port, o, 4);
		}
		return;
	}
	if (s == 8) {
		if (opcode == 4 || opcode == 3) {
			tftp_error(node, port, 5, "Unknown transfer ID");
		}
		return;
	}
	if (opcode == 4 && !tftp_sessions[s].writing && length >= 4) {
		uint32_t acked = (uint32_t)(p[2] << 8 | p[3]);

		if (acked != tftp_sessions[s].block) {
			return;
		}
		if (acked != 0 && (acked - 1) * tftp_sessions[s].blksize + tftp_sessions[s].blksize > tftp_sessions[s].size) {
			tftp_sessions[s].used = false;      /* the last block (the short one) is acknowledged: done */
			return;
		}
		tftp_sessions[s].block = acked + 1;
		tftp_data(s);
	} else if (opcode == 3 && tftp_sessions[s].writing && length >= 4) {
		uint8_t a[4] = { 0, 4, p[2], p[3] };

		tftp_sessions[s].written += length - 4;
		tftp_reply(node, port, a, 4);
		if (length - 4 < tftp_sessions[s].blksize) {
			say("modem: tftp: \"%s\": %u bytes written to it (discarded)\n", tftp_sessions[s].name, tftp_sessions[s].written);
			tftp_sessions[s].used = false;
		}
	} else if (opcode == 5) {
		if (tftp_logged < 120) {
			tftp_logged++;
			say("modem: tftp: \"%s\": the client ended it (error %u)\n", tftp_sessions[s].name, length >= 4 ? (uint32_t)(p[2] << 8 | p[3]) : 0);
		}
		tftp_sessions[s].used = false;
	}
}

static unsigned qrtr_data_logged, qrtr_resumes;
static void
qrtr_packet(const uint8_t *d, uint32_t length, uint32_t dl_channel)
{
	struct qrtr_header h;
	uint32_t at;

	if (length >= sizeof(h) && d[0] == 1) {
		memcpy(&h, d, sizeof(h));
		at = sizeof(h);
	} else if (length >= 16 && d[0] == 3) {
		bzero(&h, sizeof(h));
		h.type = d[1];
		h.confirm = d[2] & 1;
		memcpy(&h.size, d + 4, 4);
		h.from_node = (uint32_t)(d[8] | d[9] << 8);
		h.from_port = (uint32_t)(d[10] | d[11] << 8);
		h.to_node = (uint32_t)(d[12] | d[13] << 8);
		h.to_port = (uint32_t)(d[14] | d[15] << 8);
		h.from_port = h.from_port == 0xfffe ? QRTR_CONTROL : h.from_port;
		h.to_port = h.to_port == 0xfffe ? QRTR_CONTROL : h.to_port;
		at = 16 + d[3];
	} else {
		say("modem: qrtr: %u bytes that are no datagram (they begin %#x)\n", length, length != 0 ? d[0] : 0);
		return;
	}
	if (h.size == 0 || at + h.size > length) {
		say("modem: qrtr: a datagram of %u bytes in a message of %u: dropped\n", h.size, length);
		return;
	}
	qrtr.packets_in++;
	{
		/* the link this node's packets come on is the one ours go back on: the transmit channel is the receive channel less one */
		bool known = false;

		for (unsigned r = 0; r < 4; r++) {
			if (qrtr_routes[r].used && qrtr_routes[r].node == h.from_node) {
				known = true;
			}
		}
		for (unsigned r = 0; r < 4 && !known; r++) {
			if (!qrtr_routes[r].used) {
				qrtr_routes[r].used = true;
				qrtr_routes[r].node = h.from_node;
				qrtr_routes[r].ul = dl_channel - 1;
				qrtr_routes[r].v2 = d[0] == 3;
				say("modem: qrtr: node %u is reached on channels %u/%u with version %u headers\n", h.from_node, dl_channel - 1, dl_channel, d[0] == 3 ? 2 : 1);
				if (option_bridge) {
					hakios_adsp_qrtr_announce();        /* (the DSP's services, for this node too: they came before it) */
				}
				known = true;
				if (option_tftp) {
					for (uint32_t instance = 0; instance < 11; instance++) {
						uint32_t serv[5] = { QRTR_NEW_SERVER, TFTP_SERVICE, instance < 5 ? 1u | instance << 8 : instance - 5, QRTR_US, TFTP_PORT };

						qrtr_send(QRTR_NEW_SERVER, QRTR_CONTROL, h.from_node, QRTR_CONTROL, serv, 20);
					}
					say("modem: qrtr: the TFTP service (4096, instances 0..4) announced to node %u\n", h.from_node);
				}
			}
		}
	}
	if (qrtr.node == 0) {
		qrtr_v2 = d[0] == 3;
		say("modem: qrtr: the modem's datagrams have %s headers\n", qrtr_v2 ? "version 2" : "version 1");
		qrtr.node = h.from_node;
	}
	if (option_bridge && (h.to_node == BRIDGE_DSP_NODE ||
	    (h.to_port == QRTR_CONTROL && (h.type == QRTR_NEW_SERVER || h.type == QRTR_DEL_SERVER || h.type == QRTR_DEL_CLIENT || h.type == QRTR_BYE)))) {
		static unsigned said;

		static unsigned said_data;

		if (h.type != QRTR_NEW_SERVER && h.type != QRTR_DEL_SERVER && h.type != QRTR_DEL_CLIENT ? said_data++ < 200 : said++ < 4) {
			const uint8_t *q = d + at;

			say("modem: bridge: modem -> DSP: type %u, node %u port %#x to node %u port %#x, %u bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", h.type, h.from_node, h.from_port, h.to_node, h.to_port, h.size,
			    q[0], h.size > 1 ? q[1] : 0, h.size > 2 ? q[2] : 0, h.size > 3 ? q[3] : 0, h.size > 4 ? q[4] : 0, h.size > 5 ? q[5] : 0, h.size > 6 ? q[6] : 0, h.size > 7 ? q[7] : 0, h.size > 8 ? q[8] : 0, h.size > 9 ? q[9] : 0, h.size > 10 ? q[10] : 0, h.size > 11 ? q[11] : 0);
		}
		if (h.type == QRTR_DATA && h.to_port == 9) {
			/* the modem's voice commands, whole: APR, a header of 32 bytes and what the opcode takes */
			char hex[3 * 96 + 1];
			uint32_t n = h.size < 96 ? h.size : 96;

			for (uint32_t i = 0; i < n; i++) {
				snprintf(hex + 3 * i, 4, "%02x ", d[at + i]);
			}
			say("modem: bridge: APR from the modem: %s\n", hex);
		}
		hakios_adsp_qrtr_from_modem(h.type, h.from_node, h.from_port, BRIDGE_DSP_NODE, h.to_port, h.confirm != 0, d + at, h.size);
		if (h.to_node == BRIDGE_DSP_NODE) {
			return;         /* (for the DSP alone; what is said to everybody is for us too) */
		}
	}
	if (h.type == QRTR_RESUME_TX) {
		return;
	}
	if (h.type == QRTR_DATA && qrtr_data_logged < 24) {
		say("modem: qrtr: data %u: from node %u port %u to port %u, %u bytes, flags byte %#x, confirm %u\n", qrtr_data_logged, h.from_node, h.from_port, h.to_port, h.size, d[2], h.confirm);
		qrtr_data_logged++;
	}
	if (h.confirm || (option_resume_all && h.type == QRTR_DATA)) {
		uint32_t resume[5] = { QRTR_RESUME_TX, h.to_node, h.to_port, 0, 0 };
		int e = qrtr_send(QRTR_RESUME_TX, h.to_port, h.from_node, h.from_port, resume, 20);

		if (h.from_node != 2) {
			/* the router that sends to us over this link is node 2 (which forwards node 3's packets): it counts them against us, and a resume addressed to node 3 may be forwarded, not used */
			qrtr_send(QRTR_RESUME_TX, h.to_port, 2, h.from_port, resume, 20);
		}

		qrtr_resumes++;
		if (qrtr_resumes <= 30) {
			say("modem: qrtr: RESUME_TX #%u sent to node %u port %u (from our port %#x): %d\n", qrtr_resumes, h.from_node, h.from_port, h.to_port, e);
		}
		if (e != 0) {
			say("modem: qrtr: RESUME_TX not sent (%d)\n", e);
		}
	}
	if (h.to_port == QRTR_CONTROL) {
		uint32_t w[5] = { 0 };

		memcpy(w, d + at, h.size < sizeof(w) ? h.size : sizeof(w));
		switch (w[0]) {
		case QRTR_HELLO: {
			uint32_t hello[5] = { QRTR_HELLO, w[1], w[2], 0, 0 };

			uint32_t lookup[5] = { QRTR_NEW_LOOKUP, 0, 0, 0, 0 };           /* every service, every instance */

			say("modem: qrtr: HELLO from node %u (payload %08x %08x)\n", h.from_node, w[1], w[2]);
			qrtr.hellos++;
			if (option_quiet == 0 || option_quiet >= 2) {
				say("modem: qrtr: ours to it: %s\n", qrtr_send(QRTR_HELLO, QRTR_CONTROL, h.from_node, QRTR_CONTROL, hello, 20) == 0 ? "sent" : "NOT SENT");
			}
			if (option_ipa && (option_quiet == 0 || option_quiet >= 3)) {
				uint32_t serv[5] = { QRTR_NEW_SERVER, 0x31, 0x101, QRTR_US, IPA_SERVER_PORT };

				say("modem: qrtr: our IPA service (0x31, instance 1, port %#x) announced to node %u: %s\n", IPA_SERVER_PORT, h.from_node,
				    qrtr_send(QRTR_NEW_SERVER, QRTR_CONTROL, h.from_node, QRTR_CONTROL, serv, 20) == 0 ? "sent" : "NOT SENT");
			}
			if (option_tftp) {
				for (uint32_t instance = 0; instance < 11; instance++) {
					uint32_t serv[5] = { QRTR_NEW_SERVER, TFTP_SERVICE, instance < 5 ? 1u | instance << 8 : instance - 5, QRTR_US, TFTP_PORT };

					qrtr_send(QRTR_NEW_SERVER, QRTR_CONTROL, h.from_node, QRTR_CONTROL, serv, 20);
				}
				say("modem: qrtr: the TFTP service (4096, port %#x) announced to node %u\n", TFTP_PORT, h.from_node);
			}
			if (option_quiet == 0 || option_quiet >= 3) {
				say("modem: qrtr: lookup of all its services: %s\n", qrtr_send(QRTR_NEW_LOOKUP, 0x4000, h.from_node, QRTR_CONTROL, lookup, 20) == 0 ? "sent" : "NOT SENT");
			}
			break;
		}
		case QRTR_NEW_SERVER:
			if (qrtr.servers < 64) {
				qrtr.server[qrtr.servers].service = w[1];
				qrtr.server[qrtr.servers].instance = w[2];
				qrtr.server[qrtr.servers].node = w[3];
				qrtr.server[qrtr.servers].port = w[4];
				qrtr.servers++;
			}
			say("modem: qrtr: a service: %u (%#x), version %u instance %u, at node %u port %u\n", w[1], w[1], w[2] & 0xff, w[2] >> 8, w[3], w[4]);
			break;
		case QRTR_DEL_SERVER:
			say("modem: qrtr: service %u at node %u port %u has gone\n", w[1], w[3], w[4]);
			break;
		case QRTR_NEW_LOOKUP: {
			/* the modem looks a service up: ours (the IPA's), then the empty NEW_SERVER that ends an answer */
			uint32_t serv[5] = { QRTR_NEW_SERVER, 0x31, 0x101, QRTR_US, IPA_SERVER_PORT }, end[5] = { QRTR_NEW_SERVER, 0, 0, 0, 0 };

			say("modem: qrtr: the modem looks for service %u (%#x), instance word %#x\n", w[1], w[1], w[2]);
			if (option_ipa && (w[1] == 0 || w[1] == 0x31)) {
				qrtr_send(QRTR_NEW_SERVER, QRTR_CONTROL, h.from_node, QRTR_CONTROL, serv, 20);
			}
			if (option_tftp && (w[1] == 0 || w[1] == TFTP_SERVICE)) {
				for (uint32_t instance = 0; instance < 11; instance++) {
					uint32_t t[5] = { QRTR_NEW_SERVER, TFTP_SERVICE, instance < 5 ? 1u | instance << 8 : instance - 5, QRTR_US, TFTP_PORT };

					qrtr_send(QRTR_NEW_SERVER, QRTR_CONTROL, h.from_node, QRTR_CONTROL, t, 20);
				}
			}
			qrtr_send(QRTR_NEW_SERVER, QRTR_CONTROL, h.from_node, QRTR_CONTROL, end, 20);
			break;
		}
		default:
			say("modem: qrtr: control command %u\n", w[0]);
			break;
		}
		return;
	}
	if (h.type == QRTR_DATA && h.to_port == BRIDGE_CLIENT_PORT) {
		const uint8_t *q = d + at;

		say("modem: bridge: the modem on \"the DSP is up\": %u bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", h.size, q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13]);
		return;
	}
	if (h.type == QRTR_DATA && h.to_port == TFTP_PORT) {
		tftp_packet(h.from_node, h.from_port, d + at, h.size);
		return;
	}
	if (h.type == QRTR_DATA && h.to_port == IPA_SERVER_PORT && h.size >= 7) {
		ipa_server_request(h.from_node, h.from_port, d + at, h.size);
		return;
	}
	if (h.type == QRTR_DATA && (h.to_port == QMI_PORT || h.to_port == QMI_PORT6) && h.size >= 7) {
		const uint8_t *q = d + at;
		uint32_t tl = q[5] | q[6] << 8;

		if (7 + tl > h.size) {
			tl = h.size - 7;
		}
		if (q[0] == 4) {
			static unsigned shown;

			if (shown++ < 40) {
				char hex[3 * 24 + 1];
				uint32_t m = tl < 24 ? tl : 24;

				for (uint32_t i = 0; i < m; i++) {
					snprintf(hex + 3 * i, 4, "%02x ", q[7 + i]);
				}
				say("modem: qmi: INDICATION %#06x from node %u port %u (%u bytes): %s\n", q[3] | q[4] << 8, h.from_node, h.from_port, tl, hex);
			}
			{
				uint32_t loc_node, loc_port;

				if (qmi_find(16, &loc_node, &loc_port) && h.from_port == loc_port) {
					/* the location service's: a position report is kept as it came, for hakios_location (kern.hakios.gps_report); nothing else of it is ours */
					if ((q[3] | q[4] << 8) == 0x0024) {
						gps_report.length = tl < sizeof(gps_report.data) ? tl : (uint32_t)sizeof(gps_report.data);
						memcpy(gps_report.data, q + 7, gps_report.length);
						__atomic_fetch_add(&gps_report.count, 1, __ATOMIC_RELEASE);
					} else if ((q[3] | q[4] << 8) == 0x0025) {
						gps_sats.length = tl < sizeof(gps_sats.data) ? tl : (uint32_t)sizeof(gps_sats.data);
						memcpy(gps_sats.data, q + 7, gps_sats.length);
						__atomic_fetch_add(&gps_sats.count, 1, __ATOMIC_RELEASE);
					} else if ((q[3] | q[4] << 8) != 0x0026) {
						static unsigned others;

						if (others++ < 40) {
							char hex[3 * 24 + 1];
							uint32_t m = tl < 24 ? tl : 24;

							for (uint32_t i = 0; i < m; i++) {
								snprintf(hex + 3 * i, 4, "%02x ", q[7 + i]);
							}
							printf("modem: gps: from the location service: %#06x (%u bytes): %s\n", q[3] | q[4] << 8, tl, hex);
						}
					}
					return;
				}
			}
			if ((q[3] | q[4] << 8) == 0x003a) {
				/* NAS operator name data: TLV 0x10 is the service provider name (display condition, length, name) */
				for (uint32_t t = 0; t + 3 <= tl;) {
					uint32_t n = q[7 + t + 1] | q[7 + t + 2] << 8;

					if (t + 3 + n > tl) {
						break;
					}
					if (q[7 + t] == 0x10 && n >= 2 && q[7 + t + 4] > 0 && 2 + q[7 + t + 4] <= n) {
						uint32_t len = q[7 + t + 4] < sizeof(cell.operator_name) - 1 ? q[7 + t + 4] : sizeof(cell.operator_name) - 1;

						memcpy(cell.operator_name, q + 7 + t + 5, len);
						cell.operator_name[len] = 0;
					}
					t += 3 + n;
				}
			}
			if ((q[3] | q[4] << 8) == 0x0001) {
				/* a WMS event report: TLV 0x11 is a new message (storage type, index) */
				for (uint32_t t = 0; t + 3 <= tl;) {
					uint32_t n = q[7 + t + 1] | q[7 + t + 2] << 8;

					if (t + 3 + n > tl) {
						break;
					}
					if (q[7 + t] == 0x10 && n >= 5) {
						/* TLV 0x10: a message stored (storage type, index): read with WMS Raw Read */
						cell_mt_storage = q[7 + t + 3];
						cell_mt_index = (uint32_t)(q[7 + t + 4] | q[7 + t + 5] << 8 | q[7 + t + 6] << 16 | (uint32_t)q[7 + t + 7] << 24);
						cell_mt_pending = true;
					} else if (q[7 + t] == 0x11 && n >= 8) {
						/* TLV 0x11: a message transferred, not stored (the route of this SIM): ack indicator (0: to be acknowledged), transaction id, format, length, the PDU */
						uint32_t len = q[7 + t + 3 + 6] | q[7 + t + 3 + 7] << 8;

						if (len <= sizeof(cell_route_pdu) && 8 + len <= n && !cell_route_pending) {
							cell_route_ack = q[7 + t + 3] == 0;
							cell_route_tid = (uint32_t)(q[7 + t + 4] | q[7 + t + 5] << 8 | q[7 + t + 6] << 16 | (uint32_t)q[7 + t + 7] << 24);
							cell_route_format = q[7 + t + 3 + 5];
							cell_route_len = len;
							memcpy(cell_route_pdu, q + 7 + t + 3 + 8, len);
							cell_route_pending = true;
						}
					}
					t += 3 + n;
				}
			}
			return;
		}
		qmi_reply.type = q[0];
		qmi_reply.transaction = (uint16_t)(q[1] | q[2] << 8);
		qmi_reply.message = (uint16_t)(q[3] | q[4] << 8);
		memcpy(qmi_reply.data, q + 7, tl);
		qmi_reply.length = tl;
		qmi_reply.result = 0xffff;
		qmi_reply.error = 0;
		for (uint32_t t = 0; t + 3 <= tl;) {
			uint32_t n = qmi_reply.data[t + 1] | qmi_reply.data[t + 2] << 8;

			if (qmi_reply.data[t] == 2 && n >= 4 && t + 3 + n <= tl) {
				qmi_reply.result = (uint16_t)(qmi_reply.data[t + 3] | qmi_reply.data[t + 4] << 8);
				qmi_reply.error = (uint16_t)(qmi_reply.data[t + 5] | qmi_reply.data[t + 6] << 8);
			}
			t += 3 + n;
		}
		qmi_reply.have = true;
		return;
	}
	say("modem: qrtr: a datagram of %u bytes from node %u port %u to port %u (type %u)\n", h.size, h.from_node, h.from_port, h.to_port, h.type);
}


/* ---- QMUX on QMI0 (channels 20/21): QMI as ModemManager's libqmi frames it ---------------------------------- */

/*
 * A frame: 0x01, a length of 16 bits (everything after the 0x01), flags (0: from the control point), the service, the client id; then the message: for the control service
 * (0) flags, a transaction of one byte, a message id and a length of 16 bits; for the others flags and a transaction of two bytes. TLVs follow: type, length (16 bits), value.
 */
static struct {
	bool have;
	uint8_t service, client;
	uint16_t message, transaction;
	uint8_t data[IPCR_PACKET];
	uint32_t length;               /* of the TLVs */
	uint16_t result, error;
} qmux_reply;
static uint16_t qmux_transaction;
static bool qrtr_on_qmi0;

static void
qmux_packet(const uint8_t *d, uint32_t length)
{
	uint32_t at = 6, tlvs;
	uint8_t service;

	if (length < 12 || d[0] != 1) {
		char hex[3 * 48 + 1];
		uint32_t n = length < 48 ? length : 48;

		for (uint32_t i = 0; i < n; i++) {
			snprintf(hex + 3 * i, 4, "%02x ", d[i]);
		}
		say("modem: qmux: %u bytes that are no QMUX frame: %s\n", length, hex);
		/* QRTR (a version 2 header: 3, type, flags, option length, size, nodes and ports of 16 bits) on this channel: treated as such */
		if (d[0] == 3 || d[0] == 1) {
			qrtr_on_qmi0 = true;
			qrtr_channel = 20;
			qrtr_packet(d, length, 21);
		}
		return;
	}
	service = d[4];
	if (service == 0) {
		qmux_reply.transaction = d[7];
		qmux_reply.message = (uint16_t)(d[8] | d[9] << 8);
		tlvs = d[10] | d[11] << 8;
		at = 12;
	} else {
		if (length < 13) {
			return;
		}
		qmux_reply.transaction = (uint16_t)(d[7] | d[8] << 8);
		qmux_reply.message = (uint16_t)(d[9] | d[10] << 8);
		tlvs = d[11] | d[12] << 8;
		at = 13;
	}
	if ((d[6] & 0x6) == 0 && service != 0) {
		say("modem: qmux: a frame of service %u that is no reply (flags %#x)\n", service, d[6]);
	}
	if (at + tlvs > length) {
		tlvs = length - at;
	}
	qmux_reply.service = service;
	qmux_reply.client = d[5];
	memcpy(qmux_reply.data, d + at, tlvs);
	qmux_reply.length = tlvs;
	qmux_reply.result = 0xffff;
	for (uint32_t t = 0; t + 3 <= tlvs;) {
		uint32_t n = qmux_reply.data[t + 1] | qmux_reply.data[t + 2] << 8;

		if (qmux_reply.data[t] == 2 && n >= 4 && t + 3 + n <= tlvs) {
			qmux_reply.result = (uint16_t)(qmux_reply.data[t + 3] | qmux_reply.data[t + 4] << 8);
			qmux_reply.error = (uint16_t)(qmux_reply.data[t + 5] | qmux_reply.data[t + 6] << 8);
		}
		t += 3 + n;
	}
	qmux_reply.have = true;
}

/* One request and its reply (waited for, up to `ms`, the channels serviced meanwhile). 0, or an errno; the reply is in qmux_reply. */
static int
qmux_request(uint8_t service, uint8_t client, uint16_t message, const uint8_t *tlvs, uint32_t tlv_length, uint32_t ms)
{
	uint8_t f[IPCR_PACKET / 2];
	uint32_t header = service == 0 ? 6 : 7, total = 1 + 5 + header + tlv_length;
	uint16_t transaction = ++qmux_transaction;
	uint64_t until;
	int error;

	if (total > sizeof(f)) {
		return EMSGSIZE;
	}
	f[0] = 1;
	f[1] = (uint8_t)(total - 1);
	f[2] = (uint8_t)((total - 1) >> 8);
	f[3] = 0;
	f[4] = service;
	f[5] = client;
	f[6] = 0;
	if (service == 0) {
		f[7] = (uint8_t)transaction;
		f[8] = (uint8_t)message;
		f[9] = (uint8_t)(message >> 8);
		f[10] = (uint8_t)tlv_length;
		f[11] = (uint8_t)(tlv_length >> 8);
		memcpy(f + 12, tlvs, tlv_length);
	} else {
		f[7] = (uint8_t)transaction;
		f[8] = (uint8_t)(transaction >> 8);
		f[9] = (uint8_t)message;
		f[10] = (uint8_t)(message >> 8);
		f[11] = (uint8_t)tlv_length;
		f[12] = (uint8_t)(tlv_length >> 8);
		memcpy(f + 13, tlvs, tlv_length);
	}
	qmux_reply.have = false;
	if ((error = hakios_mhi_send(&mhi, 20, f, total)) != 0) {
		return error;
	}
	for (until = mhi_op_nanoseconds(NULL) + (uint64_t)ms * 1000000ull; mhi_op_nanoseconds(NULL) < until;) {
		hakios_mhi_interrupt(&mhi, -1);
		while (sahara_in.tail != sahara_in.head) {
			uint32_t at = sahara_in.tail % SAHARA_QUEUE;

			__asm__ volatile ("dmb ish" ::: "memory");
			if (sahara_in.channel[at] == 21) {
				qmux_packet(sahara_in.data[at], sahara_in.length[at]);
			} else if (sahara_in.channel[at] == 33) {
				qrtr_packet(sahara_in.data[at], sahara_in.length[at], 33);
			}
			sahara_in.tail++;
		}
		if (qmux_reply.have && qmux_reply.service == service && qmux_reply.message == message) {
			return 0;
		}
		sleep_us(500);
	}
	return ETIMEDOUT;
}

static void
qmux_probe(void)
{
	int error = qmux_request(0, 0, 0x0021, NULL, 0, 4000);   /* CTL: get version info */

	if (error != 0) {
		say("modem: qmux: no answer to CTL get version info (%d)\n", error);
		return;
	}
	say("modem: qmux: CTL get version info answered: result %u error %u, %u bytes of TLVs\n", qmux_reply.result, qmux_reply.error, qmux_reply.length);
	for (uint32_t t = 0; t + 3 <= qmux_reply.length;) {
		uint32_t n = qmux_reply.data[t + 1] | qmux_reply.data[t + 2] << 8;

		if (qmux_reply.data[t] == 1 && n >= 1) {
			uint32_t count = qmux_reply.data[t + 3];

			for (uint32_t i = 0; i < count && 4 + i * 5 + 5 <= n + 1; i++) {
				const uint8_t *e = qmux_reply.data + t + 4 + i * 5;

				say("modem: qmux: service %u: version %u.%u\n", e[0], e[1] | e[2] << 8, e[3] | e[4] << 8);
			}
		}
		t += 3 + n;
	}
}



/*
 * DIAG (channels 4/5): HDLC-framed requests and logs (Qualcomm's diag protocol). The modem's F3 messages (extended message packets, 0x79) carry their format string and source file;
 * enabled by "set all message masks" they say what the modem's processors do, and are kept in a ring of the latest lines, printed when the services stop answering.
 */
static bool has_text(const char *haystack, const char *needle);
#define DIAG_RING               160
#define DIAG_LINE               300
static struct {
	bool on;
	uint8_t frame[4096];
	uint32_t frame_length;
	bool escaped;
	char ring[DIAG_RING][DIAG_LINE];
	uint32_t head, printed, frames, printed_lines;
} diag_state;

/* the lines that say nothing new (the card's power-up trace, ADC readings, the MCFG file walk, the TFTP client's own statistics) */
static bool
diag_noise(const char *l)
{
	static const char *const words[] = { "UIM_PUP_MSG", "VAdcLog", "ps_clat", "ps_logging_helper", "mcfg_utils", "mcfg_remote_discover", "tftp_connection", "RRQ stats", "RRQ time", "WRQ stats", "WRQ time", "TFTP_STAT", "wms.c:" };

	for (unsigned i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		if (has_text(l, words[i])) {
			return true;
		}
	}
	return false;
}

static uint16_t
diag_crc(const uint8_t *d, uint32_t n)
{
	uint16_t crc = 0xffff;

	while (n-- > 0) {
		crc ^= *d++;
		for (unsigned b = 0; b < 8; b++) {
			crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
		}
	}
	return (uint16_t)~crc;
}

static int
diag_request(const uint8_t *payload, uint32_t n)
{
	uint8_t out[96];
	uint32_t o = 0;
	uint16_t crc = diag_crc(payload, n);
	uint8_t tail[2] = { (uint8_t)crc, (uint8_t)(crc >> 8) };

	for (uint32_t i = 0; i < n + 2; i++) {
		uint8_t b = i < n ? payload[i] : tail[i - n];

		if (b == 0x7e || b == 0x7d) {
			out[o++] = 0x7d;
			out[o++] = b ^ 0x20;
		} else {
			out[o++] = b;
		}
	}
	out[o++] = 0x7e;
	return hakios_mhi_send(&mhi, 4, out, o);
}

static void
diag_message(const uint8_t *f, uint32_t n)
{
	/* extended message: 0x79, timestamp type, argument count, drop count, 8 bytes of timestamp, line, subsystem, mask, the arguments, the format string, the file name */
	char *line = diag_state.ring[diag_state.head % DIAG_RING];
	uint32_t args, at;
	const char *fmt, *file;
	char text[48];

	if (n < 22 || f[0] != 0x79) {
		return;
	}
	args = f[2];
	at = 20 + 4 * args;
	if (at >= n) {
		return;
	}
	fmt = (const char *)f + at;
	file = fmt + strnlen(fmt, n - at) + 1;
	text[0] = 0;
	for (uint32_t i = 0; i < args && i < 4 && 20 + 4 * i + 4 <= n; i++) {
		uint32_t v;
		char one[12];

		memcpy(&v, f + 20 + 4 * i, 4);
		snprintf(one, sizeof(one), " %x", v);
		strlcat(text, one, sizeof(text));
	}
	snprintf(line, DIAG_LINE, "ss%u %s:%u |%.*s|%s", f[16] | f[17] << 8, (const char *)(file < (const char *)f + n ? file : ""), f[14] | f[15] << 8, (int)strnlen(fmt, 170), fmt, text);
	if (diag_state.printed_lines < 9000 && !diag_noise(line)) {
		say("modem: f3: %s\n", line);
		diag_state.printed_lines++;
		diag_state.printed = diag_state.head + 1;
	}
	diag_state.head++;
}

static void
diag_packet(const uint8_t *d, uint32_t length)
{
	for (uint32_t i = 0; i < length; i++) {
		uint8_t b = d[i];

		if (b == 0x7e) {
			if (diag_state.frame_length > 3) {
				diag_state.frames++;
				diag_message(diag_state.frame, diag_state.frame_length - 2);
				if (diag_state.frame[0] != 0x79 && diag_state.frames < 12) {
					say("modem: diag: a packet of %u bytes begins %02x %02x %02x %02x\n", diag_state.frame_length, diag_state.frame[0], diag_state.frame[1], diag_state.frame[2], diag_state.frame[3]);
				}
			}
			diag_state.frame_length = 0;
			diag_state.escaped = false;
		} else if (b == 0x7d) {
			diag_state.escaped = true;
		} else {
			if (diag_state.escaped) {
				b ^= 0x20;
				diag_state.escaped = false;
			}
			if (diag_state.frame_length < sizeof(diag_state.frame)) {
				diag_state.frame[diag_state.frame_length++] = b;
			}
		}
	}
}

/* the newest `count` lines not printed yet */
static void
diag_flush(uint32_t count)
{
	uint32_t from = diag_state.printed;

	if (diag_state.head - from > count) {
		from = diag_state.head - count;
	}
	if (diag_state.head - from > DIAG_RING) {
		from = diag_state.head - DIAG_RING;
	}
	say("modem: f3: %u messages so far (%u frames), the newest %u:\n", diag_state.head, diag_state.frames, diag_state.head - from);
	for (; from < diag_state.head; from++) {
		const char *l = diag_state.ring[from % DIAG_RING];

		if (!diag_noise(l)) {
			say("modem: f3: %s\n", l);
		}
	}
	diag_state.printed = diag_state.head;
}

static void
diag_enable(void)
{
	static const uint8_t version[1] = { 0x1c };
	static const uint8_t all_masks[8] = { 0x7d, 0x05, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff };

	say("modem: diag: version request %d, all message masks %d\n", diag_request(version, 1), diag_request(all_masks, 8));
	diag_state.on = true;
}

/*
 * The modem's EFS channels (10/11, "EFS"): its application processor's remote file system (remotefs_sahara.c, the cause of the failure reason "ssr:AP:remotefs_sahara.c:850" seen when
 * nobody answers) talks Sahara to the host: HELLO from the device, then either image reads (the host serves the EFS partitions) or memory debug (the host reads the device's EFS image
 * to store it: Qualcomm's ks -m). Here everything it says is logged, and memory debug is carried through (the bytes are counted and thrown away: nothing is written to the phone).
 */
static struct {
	uint32_t state, want, got, chunk_total, entry, entries, table_length, regions;
	bool is64;
	uint8_t table[1024];
	uint64_t base[16], length[16], done, at;
	char name[16][21];
	unsigned logged;
	uint8_t *dump;
	uint64_t dump_length;
} efs;
#define EFS_DUMP_MAX (8u << 20)
enum { EFS_IDLE, EFS_TABLE, EFS_REGION };

static void
efs_send(const void *data, size_t length)
{
	if (hakios_mhi_send(&mhi, 10, data, length) != 0) {
		say("modem: efs: sending %zu bytes failed\n", length);
	}
}

static void
efs_read(uint64_t address, uint64_t length)
{
	uint32_t w[8] = { 0 };

	if (efs.is64) {
		w[0] = SAHARA_64BIT_MEM_READ; w[1] = 0x20; w[2] = (uint32_t)address; w[3] = (uint32_t)(address >> 32); w[4] = (uint32_t)length; w[5] = (uint32_t)(length >> 32);
		efs_send(w, 0x20);
	} else {
		w[0] = SAHARA_MEM_READ; w[1] = 0x10; w[2] = (uint32_t)address; w[3] = (uint32_t)length;
		efs_send(w, 0x10);
	}
}

/* the next piece of the region being read, or the next region, or the end of it all */

static bool
has_text(const char *haystack, const char *needle)
{
	size_t n = strlen(needle);

	for (; *haystack != 0; haystack++) {
		if (strncasecmp(haystack, needle, n) == 0) {
			return true;
		}
	}
	return false;
}

/* the dump's text: what looks like a failure message (ASSERT, error, fatal, a source file and line...) */
static void
efs_scan(void)
{
	unsigned shown = 0, generic = 0;

	say("modem: efs: scanning %llu bytes of the modem's memory for its failure text; it begins %08x %08x %08x %08x %08x %08x %08x %08x\n", efs.dump_length,
	    *(uint32_t *)efs.dump, *(uint32_t *)(efs.dump + 4), *(uint32_t *)(efs.dump + 8), *(uint32_t *)(efs.dump + 12), *(uint32_t *)(efs.dump + 16), *(uint32_t *)(efs.dump + 20), *(uint32_t *)(efs.dump + 24), *(uint32_t *)(efs.dump + 28));
	for (uint64_t i = 0; i + 16 < efs.dump_length && shown < 140;) {
		uint64_t j = i;
		char line[160];
		bool keyword;

		while (j < efs.dump_length && efs.dump[j] >= 0x20 && efs.dump[j] < 0x7f) {
			j++;
		}
		if (j - i >= 14) {
			size_t n = (size_t)(j - i < sizeof(line) - 1 ? j - i : sizeof(line) - 1);

			memcpy(line, efs.dump + i, n);
			line[n] = 0;
			keyword = has_text(line, "assert") || has_text(line, "fatal") || has_text(line, "err_") || has_text(line, "watchdog") || has_text(line, "wdog") ||
			    has_text(line, "timeout") || has_text(line, "failed") || has_text(line, ".c:") || has_text(line, "stuck") || has_text(line, "hang");
			if (keyword || (j - i >= 24 && generic < 60)) {
				say("modem: efs: dump @%#llx: %s\n", i, line);
				shown++;
				generic += keyword ? 0 : 1;
			}
		}
		i = j + 1;
	}
}

static void
efs_next(void)
{
	while (efs.entry < efs.entries) {
		uint64_t left = efs.length[efs.entry] - efs.done;

		if (left == 0) {
			efs.entry++;
			efs.done = 0;
			continue;
		}
		efs.chunk_total = left > 0x1000 ? 0x1000 : (uint32_t)left;
		efs.want = efs.chunk_total;
		efs.got = 0;
		efs.state = EFS_REGION;
		efs_read(efs.base[efs.entry] + efs.done, efs.chunk_total);
		return;
	}
	{
		uint32_t reset[2] = { SAHARA_RESET, 8 };

		say("modem: efs: memory debug done: %u regions read, thrown away\n", efs.regions);
		efs_scan();
		efs.state = EFS_IDLE;
		efs_send(reset, 8);
	}
}

static void
efs_packet(const uint8_t *p, uint32_t length)
{
	const uint32_t *w = (const uint32_t *)p;

	if (efs.state == EFS_TABLE || efs.state == EFS_REGION) {
		uint32_t take = length < efs.want - efs.got ? length : efs.want - efs.got;

		if (efs.state == EFS_REGION) {
			if (efs.dump == NULL) {
				efs.dump = (uint8_t *)kalloc_data(EFS_DUMP_MAX, Z_WAITOK | Z_ZERO);
			}
			if (efs.dump != NULL && efs.dump_length + take <= EFS_DUMP_MAX) {
				memcpy(efs.dump + efs.dump_length, p, take);
				efs.dump_length += take;
			}
		}

		if (efs.state == EFS_TABLE) {
			memcpy(efs.table + efs.got, p, take < sizeof(efs.table) - efs.got ? take : sizeof(efs.table) - efs.got);
		}
		efs.got += take;
		if (efs.got < efs.want) {
			return;
		}
		if (efs.state == EFS_TABLE) {
			uint32_t entry_size = efs.is64 ? 64 : 52;

			efs.entries = efs.table_length / entry_size;
			efs.entries = efs.entries > 16 ? 16 : efs.entries;
			for (uint32_t i = 0; i < efs.entries; i++) {
				const uint8_t *e = efs.table + i * entry_size;

				if (efs.is64) {
					memcpy(&efs.base[i], e + 8, 8);
					memcpy(&efs.length[i], e + 16, 8);
					memcpy(efs.name[i], e + 24, 20);
				} else {
					uint32_t b, l;

					memcpy(&b, e + 4, 4);
					memcpy(&l, e + 8, 4);
					efs.base[i] = b;
					efs.length[i] = l;
					memcpy(efs.name[i], e + 12, 20);
				}
				efs.name[i][20] = 0;
				say("modem: efs: region %u: \"%s\" at %#llx, %llu bytes\n", i, efs.name[i], efs.base[i], efs.length[i]);
			}
			efs.entry = 0;
			efs.done = 0;
			efs.regions = efs.entries;
		} else {
			efs.done += efs.chunk_total;
		}
		efs_next();
		return;
	}
	if (length < 8) {
		return;
	}
	if (efs.logged < 60) {
		efs.logged++;
		say("modem: efs: command %u, %u bytes: %08x %08x %08x %08x %08x %08x\n", w[0], length, w[2], w[3], length >= 20 ? w[4] : 0, length >= 24 ? w[5] : 0, length >= 28 ? w[6] : 0, length >= 32 ? w[7] : 0);
	}
	switch (w[0]) {
	case SAHARA_HELLO: {
		uint32_t reply[12] = { SAHARA_HELLO_RESP, 0x30, 2, 1, 0, w[5], 0, 0, 0, 0, 0, 0 };

		efs_send(reply, 0x30);
		break;
	}
	case SAHARA_MEM_DEBUG:
	case SAHARA_64BIT_MEM_DEBUG: {
		uint64_t address, table_length;

		efs.is64 = w[0] == SAHARA_64BIT_MEM_DEBUG;
		address = efs.is64 ? ((uint64_t)w[3] << 32 | w[2]) : w[2];
		table_length = efs.is64 ? ((uint64_t)w[5] << 32 | w[4]) : w[3];
		efs.table_length = table_length > sizeof(efs.table) ? (uint32_t)sizeof(efs.table) : (uint32_t)table_length;
		efs.want = efs.table_length;
		efs.got = 0;
		efs.state = EFS_TABLE;
		efs_read(address, efs.table_length);
		break;
	}
	case SAHARA_RESET_RESP:
		say("modem: efs: reset answered\n");
		break;
	default:
		break;
	}
}

/* a packet of the modem's on whatever channel: QRTR's on 21 and 33 (parsed), the loopback's echo on 1, the others logged */
static unsigned loopback_echoes;
static void
mission_dispatch(uint32_t channel, const uint8_t *d, uint32_t length)
{
	if (channel == 21) {
		qrtr_packet(d, length, channel);
	} else if (channel == 11) {
		efs_packet(d, length);
	} else if (channel == 5) {
		diag_packet(d, length);
	} else if (channel == 1) {
		loopback_echoes++;
		say("modem: mission: LOOPBACK echo: %u bytes: %.*s\n", length, length < 40 ? (int)length : 40, (const char *)d);
	} else {
		char hex[3 * 40 + 1];
		uint32_t n = length < 40 ? length : 40;

		for (uint32_t i = 0; i < n; i++) {
			snprintf(hex + 3 * i, 4, "%02x ", d[i]);
		}
		say("modem: mission: %u bytes on channel %u: %s\n", length, channel, hex);
	}
}

static void data_poll(void);

static uint8_t *sfr_buffer;              /* the failure reason buffer handed to the modem (MHI_CMD_SFR_CFG): read after a crash */
#define SFR_LEN 0x1000
static bool mission_link_down;          /* the pump found the link down (reads all ones) */
static bool mission_phase_two = true;   /* the QMI survey (on the first link, in the 2.5 s the modem lives) */

/* the channels serviced and what came dispatched, for up to `ms` or until `*stop` */
static void
mission_pump(uint32_t ms, const volatile bool *stop)
{
	uint64_t until = mhi_op_nanoseconds(NULL) + (uint64_t)ms * 1000000ull;

	do {
		static bool link_up = true;
		static uint32_t last_pins = ~0u;
		uint32_t pins = (instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u) | (instantnoodle_gpio_level(PIN_MDM2AP_ERRFATAL) ? 2u : 0u);
		bool alive = rd(pcie.bar) != 0xffffffffu;

		if (pins != last_pins || alive != link_up) {
			say("modem: monitor: link %s, MDM2AP status %u, error fatal %u, EE %d\n", alive ? "up" : "DOWN", pins & 1, pins >> 1 & 1, alive ? (int)rd(pcie.bar + pcie.bhi + BHI_EXECENV) : -1);
			last_pins = pins;
			link_up = alive;
		}
		if (!alive) {
			mission_link_down = true;
			cell_started = false;   /* (kern.hakios.cellular_up, at once: hakios_commcenter ends iOS's call on it; "mission: over" comes some 40 s later) */
			return;
		}
		hakios_mhi_interrupt(&mhi, -1);
		while (sahara_in.tail != sahara_in.head) {
			uint32_t at = sahara_in.tail % SAHARA_QUEUE;

			__asm__ volatile ("dmb ish" ::: "memory");
			mission_dispatch(sahara_in.channel[at], sahara_in.data[at], sahara_in.length[at]);
			sahara_in.tail++;
		}
		data_poll();
		if (stop != NULL && *stop) {
			return;
		}
		sleep_us(500);
	} while (mhi_op_nanoseconds(NULL) < until);
}

static bool
qmi_find(uint32_t service, uint32_t *node, uint32_t *port)
{
	for (unsigned i = 0; i < qrtr.servers; i++) {
		if (qrtr.server[i].service == service) {
			*node = qrtr.server[i].node;
			*port = qrtr.server[i].port;
			return true;
		}
	}
	return false;
}

/* One QMI request to a service (found by its number among the modem's) and its reply, waited for up to `ms`; 0, or an errno. The reply is in qmi_reply. */
static uint16_t qmi_transaction;
static int qmi_call_at(uint32_t node, uint32_t port, uint16_t message, const uint8_t *tlvs, uint32_t tlv_length, uint32_t ms);
static int
qmi_call(uint32_t service, uint16_t message, const uint8_t *tlvs, uint32_t tlv_length, uint32_t ms)
{
	uint32_t node, port;

	if (!qmi_find(service, &node, &port)) {
		return ENOENT;
	}
	return qmi_call_at(node, port, message, tlvs, tlv_length, ms);
}

static int
qmi_call_at(uint32_t node, uint32_t port, uint16_t message, const uint8_t *tlvs, uint32_t tlv_length, uint32_t ms)
{
	uint8_t q[IPCR_PACKET / 2];
	uint16_t transaction = ++qmi_transaction;

	if (7 + tlv_length > sizeof(q)) {
		return EMSGSIZE;
	}
	q[0] = 0;                               /* a request */
	q[1] = (uint8_t)transaction;
	q[2] = (uint8_t)(transaction >> 8);
	q[3] = (uint8_t)message;
	q[4] = (uint8_t)(message >> 8);
	q[5] = (uint8_t)tlv_length;
	q[6] = (uint8_t)(tlv_length >> 8);
	memcpy(q + 7, tlvs, tlv_length);
	qmi_reply.have = false;
	if (qrtr_send(QRTR_DATA, qmi_src_port, node, port, q, 7 + tlv_length) != 0) {
		return EIO;
	}
	for (uint64_t until = mhi_op_nanoseconds(NULL) + (uint64_t)ms * 1000000ull; mhi_op_nanoseconds(NULL) < until;) {
		mission_pump(5, NULL);
		if (qmi_reply.have && qmi_reply.transaction == transaction && qmi_reply.type == 2) {
			return 0;
		}
	}
	return ETIMEDOUT;
}

static void
qmi_hex(const char *what, const uint8_t *d, uint32_t n)
{
	char hex[3 * 64 + 1];
	uint32_t m = n < 64 ? n : 64;

	for (uint32_t i = 0; i < m; i++) {
		snprintf(hex + 3 * i, 4, "%02x ", d[i]);
	}
	say("modem: qmi: %s (%u bytes): %s%s\n", what, n, hex, n > m ? "..." : "");
}

/* a few read-only questions of the services, to see what each says (nothing personal is printed: no serial numbers, no card or subscriber identity) */
static void
qmi_survey(void)
{
	static const struct { uint32_t service; uint16_t message; const char *what; } asks[] = {
		{ 2, 0x002d, "DMS: operating mode" },
		{ 11, 0x002f, "UIM: card status" },
		{ 3, 0x0024, "NAS: serving system" },
		{ 3, 0x0020, "NAS: signal strength" },
		{ 5, 0x0034, "WMS: SMSC address" },
	};

	for (unsigned i = 0; i < sizeof(asks) / sizeof(asks[0]); i++) {
		int e = qmi_call(asks[i].service, asks[i].message, NULL, 0, 3000);

		if (e != 0) {
			say("modem: qmi: %s: no reply (%d)\n", asks[i].what, e);
			continue;
		}
		say("modem: qmi: %s: result %u error %u\n", asks[i].what, qmi_reply.result, qmi_reply.error);
		qmi_hex("TLVs", qmi_reply.data, qmi_reply.length);
	}
}


/* ---- the IPA's QMI handshake: the application processor's side (Android's ipa_qmi_service.c; messages as Linux's ipa_qmi_msg.h numbers them) ---- */

enum { IPA_QMI_INDICATION_REGISTER = 0x20, IPA_QMI_INIT_DRIVER = 0x21, IPA_QMI_INIT_COMPLETE = 0x22, IPA_QMI_INSTALL_FILTER_RULE = 0x23, IPA_QMI_FILTER_INSTALLED_NOTIF = 0x24,
	IPA_QMI_CONFIG = 0x27, IPA_QMI_DRIVER_INIT_COMPLETE = 0x35 };
static struct { uint32_t node, port; bool registered, init_answered, complete_sent; unsigned requests; } ipa;

static void
tlv_put(uint8_t *b, uint32_t *n, uint8_t type, const void *value, uint16_t length)
{
	b[(*n)++] = type;
	b[(*n)++] = (uint8_t)length;
	b[(*n)++] = (uint8_t)(length >> 8);
	memcpy(b + *n, value, length);
	*n += length;
}

static void
ipa_send_to_modem(uint8_t type, uint16_t transaction, uint16_t message, const uint8_t *tlvs, uint32_t tlv_length, uint32_t node, uint32_t port)
{
	uint8_t q[512];

	q[0] = type;
	q[1] = (uint8_t)transaction;
	q[2] = (uint8_t)(transaction >> 8);
	q[3] = (uint8_t)message;
	q[4] = (uint8_t)(message >> 8);
	q[5] = (uint8_t)tlv_length;
	q[6] = (uint8_t)(tlv_length >> 8);
	memcpy(q + 7, tlvs, tlv_length);
	qrtr_send(QRTR_DATA, IPA_SERVER_PORT, node, port, q, 7 + tlv_length);
}

/* INIT_COMPLETE (the master driver is initialised) as an indication to the modem's client, once it has registered and our request has been answered */
static void
ipa_maybe_complete(void)
{
	uint8_t t[16];
	uint32_t n = 0;
	uint16_t status[2] = { 0, 0 };

	if (!ipa.registered || !ipa.init_answered || ipa.complete_sent) {
		return;
	}
	tlv_put(t, &n, 0x01, status, sizeof(status));
	ipa_send_to_modem(4, 0, IPA_QMI_INIT_COMPLETE, t, n, ipa.node, ipa.port);
	ipa.complete_sent = true;
	say("modem: ipa: INIT_COMPLETE indication sent to node %u port %u\n", ipa.node, ipa.port);
}

/* a request of the modem's driver to our server port: logged, and answered as Android answers it */
static void
ipa_server_request(uint32_t from_node, uint32_t from_port, const uint8_t *q, uint32_t length)
{
	uint16_t transaction = (uint16_t)(q[1] | q[2] << 8), message = (uint16_t)(q[3] | q[4] << 8);
	uint32_t tl = q[5] | q[6] << 8;
	uint8_t t[16];
	uint32_t n = 0;
	uint16_t ok[2] = { 0, 0 };

	if (7 + tl > length) {
		tl = length - 7;
	}
	ipa.requests++;
	if (ipa.requests < 40) {
		say("modem: ipa: the modem's request %#x (type %u, transaction %u, %u bytes of TLVs) from node %u port %u\n", message, q[0], transaction, tl, from_node, from_port);
	}
	if (q[0] != 0) {
		return;
	}
	tlv_put(t, &n, 0x02, ok, sizeof(ok));
	switch (message) {
	case IPA_QMI_INDICATION_REGISTER:
		ipa.node = from_node;
		ipa.port = from_port;
		ipa.registered = true;
		ipa_send_to_modem(2, transaction, message, t, n, from_node, from_port);
		ipa_maybe_complete();
		break;
	case IPA_QMI_INSTALL_FILTER_RULE:
	case IPA_QMI_FILTER_INSTALLED_NOTIF:
	case IPA_QMI_CONFIG:
	case IPA_QMI_DRIVER_INIT_COMPLETE:
		ipa_send_to_modem(2, transaction, message, t, n, from_node, from_port);
		break;
	default:
		say("modem: ipa: a request this does not know (%#x)\n", message);
		break;
	}
}

/* INIT_MODEM_DRIVER_REQ to the modem's IPA service (0x31, instance 2): the IPA 4.5 memory layout (Android's ipa_4_5_mem_part: header, routing and filter tables, the modem's own area, the statistics blocks) */
static void
ipa_init_driver(void)
{
	uint8_t t[320];
	uint32_t n = 0, node = 0, port = 0;
	const uint32_t platform = 3;                           /* MSM Android */
	const uint32_t hdr[2] = { 0x688, 0x688 + 0x240 - 1 }, v4rt[2] = { 0x508, 7 }, v6rt[2] = { 0x608, 7 }, mem[2] = { 0x2fe8, 0x800 }, proc[2] = { 0xad0, 0xad0 + 0xb20 - 1 };
	const uint32_t v4hrt[2] = { 0x488, 7 }, v6hrt[2] = { 0x588, 7 };
	const uint32_t v4flt = 0x308, v6flt = 0x408, v4hflt = 0x288, v6hflt = 0x388, qbase = 0x2510, qsize = 0x30, dbase = 0x2fc0, dsize = 0x20;
	int e;

	for (unsigned i = 0; i < qrtr.servers; i++) {
		if (qrtr.server[i].service == 0x31 && qrtr.server[i].instance == 0x201) {
			node = qrtr.server[i].node;
			port = qrtr.server[i].port;
		}
	}
	if (node == 0) {
		say("modem: ipa: the modem's IPA service (0x31, instance 2) is not announced\n");
		return;
	}
	tlv_put(t, &n, 0x10, &platform, 4);
	tlv_put(t, &n, 0x11, hdr, 8);
	tlv_put(t, &n, 0x12, v4rt, 8);
	tlv_put(t, &n, 0x13, v6rt, 8);
	tlv_put(t, &n, 0x14, &v4flt, 4);
	tlv_put(t, &n, 0x15, &v6flt, 4);
	tlv_put(t, &n, 0x16, mem, 8);
	tlv_put(t, &n, 0x19, proc, 8);
	tlv_put(t, &n, 0x1b, v4hrt, 8);
	tlv_put(t, &n, 0x1c, v6hrt, 8);
	tlv_put(t, &n, 0x1d, &v4hflt, 4);
	tlv_put(t, &n, 0x1e, &v6hflt, 4);
	tlv_put(t, &n, 0x1f, &qbase, 4);
	tlv_put(t, &n, 0x20, &qsize, 4);
	tlv_put(t, &n, 0x21, &dbase, 4);
	tlv_put(t, &n, 0x22, &dsize, 4);
	{
		uint8_t q[7 + sizeof(t)];
		uint16_t transaction = ++qmi_transaction;

		q[0] = 0;
		q[1] = (uint8_t)transaction;
		q[2] = (uint8_t)(transaction >> 8);
		q[3] = IPA_QMI_INIT_DRIVER;
		q[4] = 0;
		q[5] = (uint8_t)n;
		q[6] = (uint8_t)(n >> 8);
		memcpy(q + 7, t, n);
		qmi_reply.have = false;
		e = qrtr_send(QRTR_DATA, QMI_PORT, node, port, q, 7 + n);
		say("modem: ipa: INIT_MODEM_DRIVER_REQ sent to node %u port %u: %d\n", node, port, e);
		for (uint64_t until = mhi_op_nanoseconds(NULL) + 2000000000ull; mhi_op_nanoseconds(NULL) < until;) {
			mission_pump(5, NULL);
			if (qmi_reply.have && qmi_reply.transaction == transaction && qmi_reply.type == 2) {
				break;
			}
		}
		if (qmi_reply.have && qmi_reply.transaction == transaction) {
			say("modem: ipa: INIT_MODEM_DRIVER answered: result %u error %u\n", qmi_reply.result, qmi_reply.error);
			qmi_hex("INIT_DRIVER reply TLVs", qmi_reply.data, qmi_reply.length);
			ipa.init_answered = qmi_reply.result == 0;
			ipa_maybe_complete();
		} else {
			say("modem: ipa: INIT_MODEM_DRIVER: no answer\n");
		}
	}
}


/* ---- the radio's bring-up over QMI: DMS online, UIM card and PIN, NAS registration, WMS ------------------------------------------------- */

static const uint8_t *
qmi_tlv(uint8_t type, uint32_t *length)
{
	for (uint32_t t = 0; t + 3 <= qmi_reply.length;) {
		uint32_t n = qmi_reply.data[t + 1] | qmi_reply.data[t + 2] << 8;

		if (t + 3 + n > qmi_reply.length) {
			return NULL;
		}
		if (qmi_reply.data[t] == type) {
			*length = n;
			return qmi_reply.data + t + 3;
		}
		t += 3 + n;
	}
	return NULL;
}

static struct {
	bool known;
	uint8_t card_state, upin_state, app_type, app_state, pin1_state, pin1_retries, puk1_retries, aid_len, aid[16], slot;
	uint16_t gw_index;
} sim;

/* UIM Get Card Status: what is printed is states and counters only (no identity of the card or of its subscriber) */
static bool
uim_card_status(void)
{
	uint32_t n;
	const uint8_t *c;
	int e = qmi_call(11, 0x002f, NULL, 0, 3000);

	if (e != 0 || qmi_reply.result != 0 || (c = qmi_tlv(0x10, &n)) == NULL || n < 9) {
		say("modem: sim: card status: %s (result %u error %u)\n", e != 0 ? "no reply" : "no card status", qmi_reply.result, qmi_reply.error);
		return false;
	}
	memset(&sim, 0, sizeof(sim));
	sim.gw_index = (uint16_t)(c[0] | c[1] << 8);
	{
		uint32_t at = 8;
		uint8_t cards = c[at++];

		if (cards == 0 || at + 6 > n) {
			say("modem: sim: no slot in the card status\n");
			return true;
		}
		sim.card_state = c[at];
		sim.upin_state = c[at + 1];
		sim.slot = 1;
		at += 5;
		{
			uint8_t apps = c[at++];

			for (unsigned a = 0; a < apps && at + 7 <= n; a++) {
				/* type, state, 4 bytes of personalisation, AID length, AID, then universal PIN, PIN1 state, PIN1 retries, PUK1 retries, PIN2 state, retries, PUK2 retries */
				uint8_t type = c[at], state = c[at + 1], aid_len = c[at + 6];

				if (at + 7 + aid_len + 7 > n) {
					break;
				}
				if (sim.app_type == 0 || type == 2) {      /* a USIM is preferred */
					sim.app_type = type;
					sim.app_state = state;
					sim.aid_len = aid_len < sizeof(sim.aid) ? aid_len : sizeof(sim.aid);
					memcpy(sim.aid, c + at + 7, sim.aid_len);
					sim.pin1_state = c[at + 7 + aid_len + 1];
					sim.pin1_retries = c[at + 7 + aid_len + 2];
					sim.puk1_retries = c[at + 7 + aid_len + 3];
				}
				at += 7 + aid_len + 7;
			}
		}
	}
	sim.known = true;
	{
		static uint32_t last_sim;
		uint32_t now = sim.card_state | sim.app_state << 8 | sim.pin1_state << 16 | sim.pin1_retries << 24;

		if (now == last_sim) {
			return true;
		}
		last_sim = now;
	}
	say("modem: sim: primary GW index %#x; slot 1: card state %u (1 present), application type %u (2 USIM) state %u (2 PIN1 required, 7 ready), PIN1 state %u (1 enabled, not verified; 3 disabled), retries %u, PUK retries %u\n",
	    sim.gw_index, sim.card_state, sim.app_type, sim.app_state, sim.pin1_state, sim.pin1_retries, sim.puk1_retries);
	return true;
}

static int
nas_registration(uint8_t *registration, uint8_t *radio)
{
	uint32_t n;
	const uint8_t *c;
	int e = qmi_call(3, 0x0024, NULL, 0, 3000);

	if (e != 0 || qmi_reply.result != 0 || (c = qmi_tlv(0x01, &n)) == NULL || n < 5) {
		return e != 0 ? e : EIO;
	}
	*registration = c[0];
	*radio = n >= 6 ? c[5] : 0;
	{
		static uint32_t last_nas = ~0u;
		uint32_t now = c[0] | c[1] << 8 | c[2] << 16 | *radio << 24;

		if (now != last_nas) {
			last_nas = now;
			say("modem: nas: registration state %u (1 registered), CS %u, PS %u (1 attached), radio interfaces %u, first %u\n", c[0], c[1], c[2], c[4], *radio);
		}
	}
	return 0;
}


/*
 * WMS Raw Send of SMS-SUBMIT messages (3GPP, the card's own SMSC): `text` is UTF-8. In the GSM 7-bit default alphabet (with its escape table) when every character is in it, else in UCS-2
 * (UTF-16: an emoji is a surrogate pair); more than one message's worth (160 septets, 70 UCS-2 units) goes as concatenated parts (a user data header 05 00 03 REF TOTAL SEQ; 153 / 67 per part).
 * Only started with modem_sms=1 (the owner's number, for tests), by `hakios_cell sms NUMBER TEXT` / the sysctl modem_sms_send, or by hakios_smsd for what Messages wants sent.
 */
static int
gsm7_of(uint32_t cp, uint8_t out[2])
{
	static const struct { uint32_t cp; uint8_t g; } basic[] = { { 0x40, 0x00 }, { 0xa3, 0x01 }, { 0x24, 0x02 }, { 0xa5, 0x03 }, { 0xe8, 0x04 }, { 0xe9, 0x05 }, { 0xf9, 0x06 }, { 0xec, 0x07 },
		{ 0xf2, 0x08 }, { 0xc7, 0x09 }, { 0x0a, 0x0a }, { 0xd8, 0x0b }, { 0xf8, 0x0c }, { 0x0d, 0x0d }, { 0xc5, 0x0e }, { 0xe5, 0x0f }, { 0x5f, 0x11 }, { 0xc6, 0x1c }, { 0xe6, 0x1d },
		{ 0xdf, 0x1e }, { 0xc9, 0x1f }, { 0xa4, 0x24 }, { 0xa1, 0x40 }, { 0xc4, 0x5b }, { 0xd6, 0x5c }, { 0xd1, 0x5d }, { 0xdc, 0x5e }, { 0xa7, 0x5f }, { 0xbf, 0x60 }, { 0xe4, 0x7b },
		{ 0xf6, 0x7c }, { 0xf1, 0x7d }, { 0xfc, 0x7e }, { 0xe0, 0x7f } };
	static const struct { uint32_t cp; uint8_t g; } escaped[] = { { 0x0c, 0x0a }, { 0x5e, 0x14 }, { 0x7b, 0x28 }, { 0x7d, 0x29 }, { 0x5c, 0x2f }, { 0x5b, 0x3c }, { 0x7e, 0x3d },
		{ 0x5d, 0x3e }, { 0x7c, 0x40 }, { 0x20ac, 0x65 } };

	for (unsigned i = 0; i < sizeof(basic) / sizeof(basic[0]); i++) {
		if (basic[i].cp == cp) {
			out[0] = basic[i].g;
			return 1;
		}
	}
	for (unsigned i = 0; i < sizeof(escaped) / sizeof(escaped[0]); i++) {
		if (escaped[i].cp == cp) {
			out[0] = 0x1b;
			out[1] = escaped[i].g;
			return 2;
		}
	}
	if (cp >= 0x20 && cp < 0x7f && cp != 0x60) {
		out[0] = (uint8_t)cp;           /* printable ASCII is the same septet (the ones that are not are in the tables above) */
		return 1;
	}
	return 0;
}

/* septets packed into bytes from bit `at` of `out` (zeroed by the caller) */
static void
pack7(uint8_t *out, uint32_t at, const uint8_t *sept, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++, at += 7) {
		uint32_t v = (uint32_t)(sept[i] & 0x7f) << (at & 7);

		out[at >> 3] |= (uint8_t)v;
		if ((at & 7) > 1) {
			out[(at >> 3) + 1] |= (uint8_t)(v >> 8);
		}
	}
}

static int
wms_send_pdu(const uint8_t *pdu, uint32_t n)
{
	uint8_t raw[3 + 200], t[220];
	uint32_t k = 0;
	int e;

	raw[0] = 6;                            /* format: 3GPP point to point */
	raw[1] = (uint8_t)n;
	raw[2] = (uint8_t)(n >> 8);
	memcpy(raw + 3, pdu, n);
	tlv_put(t, &k, 0x01, raw, (uint16_t)(3 + n));
	e = qmi_call(5, 0x0020, t, k, 30000);
	say("modem: sms: WMS raw send of %u bytes: %s (result %u error %u)\n", n, e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	if (e == 0 && qmi_reply.result == 0) {
		uint32_t m;
		const uint8_t *c = qmi_tlv(0x01, &m);

		say("modem: sms: SENT, message id %u\n", c != NULL && m >= 2 ? c[0] | c[1] << 8 : 0xffff);
		return 0;
	}
	return e != 0 ? e : EIO;
}

static int
wms_send_text(const char *to, const char *text)
{
	static uint8_t reference;
	/* static: only the modem's thread sends, and its stack (16 KB, with the QMI calls and QRTR buffers below this) cannot hold 8 KB more (it did not: the kernel hung) */
	static uint32_t cp[512], units[1024];
	static uint8_t sept[1536];
	uint32_t ncp = 0, nunits = 0, ns = 0, digits = (uint32_t)strlen(to);
	bool gsm = true;
	const uint8_t *u = (const uint8_t *)text;
	uint32_t total, per, parts, done = 0;
	int result = 0;

	while (*u != 0 && ncp < 512) {
		uint32_t c = *u++, more = 0;

		if (c >= 0xf0) { c &= 7; more = 3; } else if (c >= 0xe0) { c &= 15; more = 2; } else if (c >= 0xc0) { c &= 31; more = 1; } else if (c >= 0x80) { c = '?'; }
		while (more-- > 0 && (*u & 0xc0) == 0x80) {
			c = c << 6 | (*u++ & 0x3f);
		}
		cp[ncp++] = c;
	}
	for (uint32_t i = 0; i < ncp; i++) {
		uint8_t g[2];
		int w = gsm ? gsm7_of(cp[i], g) : 0;

		if (w == 0) {
			gsm = false;
		} else if (ns + (uint32_t)w <= sizeof(sept)) {
			sept[ns++] = g[0];
			if (w == 2) {
				sept[ns++] = g[1];
			}
		}
		if (cp[i] >= 0x10000) {
			units[nunits++] = 0xd800 + ((cp[i] - 0x10000) >> 10);
			units[nunits++] = 0xdc00 + ((cp[i] - 0x10000) & 0x3ff);
		} else {
			units[nunits++] = cp[i];
		}
	}
	total = gsm ? ns : nunits;
	say("modem: sms: encoding: %u characters (%u code points), %s: %u %s, a %u-digit number\n", (uint32_t)strlen(text), ncp, gsm ? "GSM 7-bit" : "UCS-2", total, gsm ? "septets" : "units", digits);
	if (total == 0 || digits == 0 || digits > 20) {
		say("modem: sms: nothing to send (total %u, digits %u)\n", total, digits);
		return EINVAL;
	}
	per = total <= (gsm ? 160u : 70u) ? total : (gsm ? 153u : 67u);
	parts = total <= (gsm ? 160u : 70u) ? 1 : (total + per - 1) / per;
	reference++;
	say("modem: sms: %u part(s) of up to %u, reference %u\n", parts, per, reference);
	for (uint32_t part = 0; part < parts; part++) {
		uint32_t from = done, count = total - done < per ? total - done : per, n = 0;
		uint8_t pdu[200], ud[160];
		bool multi = parts > 1;

		if (multi && count == per && ((gsm && sept[from + count - 1] == 0x1b) || (!gsm && units[from + count - 1] >= 0xd800 && units[from + count - 1] < 0xdc00))) {
			count--;                       /* not between the halves of one character */
		}
		done += count;
		pdu[n++] = 0x00;                       /* no SMSC in the PDU: the card's own is used */
		pdu[n++] = multi ? 0x41 : 0x01;        /* SMS-SUBMIT, relative validity absent, a user data header in front when in parts */
		pdu[n++] = 0x00;                       /* message reference (the modem sets it) */
		pdu[n++] = (uint8_t)digits;
		pdu[n++] = 0x91;                       /* international, ISDN numbering */
		for (uint32_t i = 0; i < digits; i += 2) {
			uint8_t lo = (uint8_t)(to[i] - '0'), hi = i + 1 < digits ? (uint8_t)(to[i + 1] - '0') : 0x0f;

			pdu[n++] = (uint8_t)(hi << 4 | lo);
		}
		pdu[n++] = 0x00;                       /* PID */
		pdu[n++] = gsm ? 0x00 : 0x08;          /* DCS: GSM 7-bit default alphabet, or UCS-2 */
		bzero(ud, sizeof(ud));
		if (gsm) {
			uint32_t bytes;

			if (multi) {
				static const uint8_t header[3] = { 0x05, 0x00, 0x03 };

				memcpy(ud, header, 3);
				ud[3] = reference; ud[4] = (uint8_t)parts; ud[5] = (uint8_t)(part + 1);
				pack7(ud, 6 * 8 + 1, sept + from, count);   /* (one fill bit: the text starts at a septet boundary, 7 septets in) */
				pdu[n++] = (uint8_t)(7 + count);
				bytes = (6 * 8 + 1 + count * 7 + 7) / 8;
			} else {
				pack7(ud, 0, sept + from, count);
				pdu[n++] = (uint8_t)count;
				bytes = (count * 7 + 7) / 8;
			}
			memcpy(pdu + n, ud, bytes);
			n += bytes;
		} else {
			uint32_t bytes = 0;

			if (multi) {
				static const uint8_t header[3] = { 0x05, 0x00, 0x03 };

				memcpy(pdu + n + 1, header, 3);
				pdu[n + 4] = reference; pdu[n + 5] = (uint8_t)parts; pdu[n + 6] = (uint8_t)(part + 1);
				bytes = 6;
			}
			pdu[n] = (uint8_t)(bytes + 2 * count);
			n++;
			if (multi) {
				n += 6;
			}
			for (uint32_t i = 0; i < count; i++) {
				pdu[n++] = (uint8_t)(units[from + i] >> 8);
				pdu[n++] = (uint8_t)units[from + i];
			}
		}
		say("modem: sms: part %u/%u: %u %s, PDU of %u bytes\n", part + 1, parts, count, gsm ? "septets" : "units", n);
		if ((result = wms_send_pdu(pdu, n)) != 0) {
			say("modem: sms: part %u/%u not sent: %d\n", part + 1, parts, result);
			return result;
		}
	}
	return result;
}

/*
 * ---- mobile data: WDA (the data format), WDS (the session) and the modem's hardware IP channel IP_HW0 (100: to the modem, 101: from it) ----
 *
 * Android runs this channel as the netdev rmnet_mhi0 (the device tree's mhi_rmnet@0: mru 0x8000) with the rmnet driver on top: raw IP packets inside QMAP frames (a 4-byte header: pad length, mux id, length in
 * big-endian, then the packet), the session bound to a mux id. The AP's IPA is not needed for this channel (it carries the "MHIP" offload channels 105..108 instead), so the host CPU can drive it as an
 * ordinary pair of MHI channels. Steps, each logged: WDA Set Data Format (link layer raw IP, QMAP both ways, the endpoint = the MHI data port) -> WDS Bind Mux Data Port -> Set IP Family -> Start Network
 * Interface (retried while the packet domain is not attached) -> Get Current Settings (address, gateway, DNS, MTU). modem_data=2 also sends a DNS query through the session and logs the answer.
 */
#define QMI_WDS                 1
#define QMI_WDA                 0x1a
static unsigned option_data;            /* modem_data=1: the session after registration; 2: and a DNS query through it as a test */
static unsigned option_data_ep = 0x30004;   /* modem_data_ep=HEX: endpoint type (high half, 3 = PCIe) and interface id (low half); other ids are tried when the modem refuses it */
static unsigned option_data6 = 2;       /* modem_data6: 0 no IPv6 session, 1 a second WDS client after the IPv4 call, 2 (default) the IPv6 call FIRST (the carrier profile's PDP type is IPv6: the modem makes IPv4 by 464XLAT on that PDN, and an IPv6 call after the IPv4 one fails, reason 0xec) */
static unsigned option_data_qmap = 1;   /* modem_data_qmap=0: raw IP packets with no QMAP header */
static unsigned option_en1 = 1;         /* modem_en1=0: no network interface for iOS (the session and the DNS test only) */
static char option_apn[64];             /* modem_apn=NAME: the access point name of the session (else the modem's default profile) */
static struct {
	volatile bool up;
	uint8_t mux;
	uint32_t handle, address, gateway, mask, dns1, dns2, mtu;
	uint64_t to_modem, from_modem, dropped_out, dropped_in, commands;
	void (*receive)(const uint8_t *packet, uint32_t length);        /* en1, when it exists */
	uint16_t dns_id;
	bool dns_answered;
	unsigned said;
	bool format_done;
	uint32_t ep_type, ep_iface;             /* the data port the modem accepted (WDA) */
} data;

static uint16_t
data_checksum(const uint8_t *d, uint32_t n)
{
	uint32_t sum = 0;

	for (uint32_t i = 0; i + 1 < n; i += 2) {
		sum += (uint32_t)d[i] << 8 | d[i + 1];
	}
	if (n & 1) {
		sum += (uint32_t)d[n - 1] << 8;
	}
	while (sum >> 16) {
		sum = (sum & 0xffff) + (sum >> 16);
	}
	return (uint16_t)~sum;
}

/* One IPv4 packet to the modem (QMAP-framed unless modem_data_qmap=0). */
static bool
data_send_mux(const uint8_t *packet, uint32_t length, uint8_t mux)
{
	uint8_t f[2048 + 4];
	uint32_t n = 0;

	if (length == 0 || length > 2048) {
		data.dropped_out++;
		return false;
	}
	if (option_data_qmap) {
		f[0] = 0;
		f[1] = mux;
		f[2] = (uint8_t)(length >> 8);
		f[3] = (uint8_t)length;
		n = 4;
	}
	memcpy(f + n, packet, length);
	n += length;
	if (hakios_mhi_send(&mhi, DATA_UL, f, n) != 0) {
		data.dropped_out++;
		return false;
	}
	data.to_modem++;
	return true;
}

static bool
data_send(const uint8_t *packet, uint32_t length)
{
	if (!data.up) {
		data.dropped_out++;
		return false;
	}
	return data_send_mux(packet, length, data.mux);
}

/* the DNS query for example.com (A), from `src` to `dst`, as one IPv4 packet in `out`; its length */
static uint32_t
data_dns_query(uint8_t *out, uint32_t src, uint32_t dst, uint16_t id)
{
	static const uint8_t question[] = { 7, 'e', 'x', 'a', 'm', 'p', 'l', 'e', 3, 'c', 'o', 'm', 0, 0, 1, 0, 1 };
	uint32_t udp_length = 8 + 12 + sizeof(question), total = 20 + udp_length;
	uint16_t sum;

	memset(out, 0, total);
	out[0] = 0x45;
	out[2] = (uint8_t)(total >> 8); out[3] = (uint8_t)total;
	out[4] = (uint8_t)(id >> 8); out[5] = (uint8_t)id;
	out[8] = 64;
	out[9] = 17;
	for (int i = 0; i < 4; i++) {
		out[12 + i] = (uint8_t)(src >> (24 - 8 * i));
		out[16 + i] = (uint8_t)(dst >> (24 - 8 * i));
	}
	sum = data_checksum(out, 20);
	out[10] = (uint8_t)(sum >> 8); out[11] = (uint8_t)sum;
	out[20] = 0xc3; out[21] = 0x51;                                 /* source port 50001 */
	out[22] = 0; out[23] = 53;
	out[24] = (uint8_t)(udp_length >> 8); out[25] = (uint8_t)udp_length;
	out[28] = (uint8_t)(id >> 8); out[29] = (uint8_t)id;            /* DNS: id, recursion desired, one question */
	out[30] = 0x01; out[31] = 0x00;
	out[33] = 1;
	memcpy(out + 40, question, sizeof(question));
	return total;
}

/* A packet from the modem: logged for the first few, the DNS test's answer recognised; the rest goes to en1. */
static void
data_receive(const uint8_t *p, uint32_t n)
{
	data.from_modem++;
	if (data.said < 12) {
		char hex[3 * 32 + 1];
		uint32_t m = n < 32 ? n : 32;

		for (uint32_t i = 0; i < m; i++) {
			snprintf(hex + 3 * i, 4, "%02x ", p[i]);
		}
		data.said++;
		say("modem: data: %u bytes from the modem: %s\n", n, hex);
	}
	if (n >= 48 && (p[0] >> 4) == 4 && p[9] == 17 && p[20] == 0 && p[21] == 53 && p[28] == (uint8_t)(data.dns_id >> 8) && p[29] == (uint8_t)data.dns_id) {
		uint32_t answers = p[34] << 8 | p[35], at = 40;

		/* skip the question (one name, type, class), then the first answer: name (a pointer), type, class, ttl, rdlength, rdata */
		while (at < n && p[at] != 0) {
			at += p[at] + 1;
		}
		at += 5;
		data.dns_answered = true;
		if (answers > 0 && at + 16 <= n) {
			at += 2;
			say("modem: data: DNS ANSWER for example.com: %u answer(s), flags %02x%02x, first: type %u, %u.%u.%u.%u\n", answers, p[30], p[31], p[at] << 8 | p[at + 1], p[at + 10], p[at + 11], p[at + 12], p[at + 13]);
		} else {
			say("modem: data: DNS ANSWER with %u answer(s), flags %02x%02x (%u bytes)\n", answers, p[30], p[31], n);
		}
	}
	if (data.receive != NULL) {
		data.receive(p, n);
	}
}

/* what the modem sent on channel 101: unframed (QMAP) into packets */
static void
data_poll(void)
{
	while (data_in.tail != data_in.head) {
		uint32_t at = data_in.tail % DATA_QUEUE, length = data_in.length[at];
		const uint8_t *d = data_in.data[at];

		__asm__ volatile ("dmb ish" ::: "memory");
		if (!option_data_qmap) {
			data_receive(d, length);
		} else {
			for (uint32_t o = 0; o + 4 <= length;) {
				uint32_t size = d[o + 2] << 8 | d[o + 3], pad = d[o] & 0x3f;

				if (o + 4 + size > length) {
					data.dropped_in++;
					break;
				}
				if (d[o] & 0x80) {
					if (data.commands++ < 6) {
						say("modem: data: a QMAP command (%02x %02x, %u bytes) from the modem\n", d[o], d[o + 1], size);
					}
				} else if (size > pad) {
					data_receive(d + o + 4, size - pad);
				}
				o += 4 + size;
			}
		}
		data_in.tail++;
	}
}

/*
 * ---- en1: the session's packets as a network interface iOS will use (the redwood port's idea, hackios-xnu-kernel hakios.cell.c) ----
 *
 * iOS's own cellular interface (pdp_ip0) is made by the baseband stack that is not here. What is here is the kernel's Ethernet-looking interface (drivers/hakios.net.c, unit 1) and configd's
 * IPConfiguration, which `net_test dhcp en1` asks to run DHCP on it (userland/src/net_test.m: the service it makes needs no preferences). So the session's IP packets are shown as Ethernet frames
 * between two made-up stations, the interface's and "the router's", and this code answers what the stack asks of the router: ARP for any address (everything is behind the router) and DHCP with
 * the session's settings. Packets to the router go to the modem (data_send), packets from it come from the router (data.receive). The session is IPv4 (the carrier's own address is the modem's
 * CLAT one, 192.0.0.x, translated in the modem); IPv6 is not carried. The mask and router given to DHCP are made up (a /24 around the address, the router at .1 or .2): any router will do.
 */
static const uint8_t cl_mac[6] = { 0x02, 0x68, 0x61, 0x6b, 0x69, 0x01 };
static const uint8_t cl_router_mac[6] = { 0x02, 0x68, 0x61, 0x6b, 0x69, 0xfe };
#define CL_UNIT                 1
static LCK_GRP_DECLARE(cl_lock_group, "instantnoodle-cell");
static LCK_MTX_DECLARE(cl_lock, &cl_lock_group);
static struct {
	bool up;
	uint32_t address, mask, router, dns1, dns2, mtu;
} cl_session;
static struct {
	uint64_t arp, dhcp, to_modem, from_modem, dropped_out, dropped_in, not_ip4;
} cl_count;
static bool cl_attached;

static inline uint32_t
cl_get32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

static inline void
cl_put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

static inline void
cl_put16(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}

static bool
cl_arp_answer(const uint8_t *frame, size_t length)
{
	const uint8_t *arp = frame + 14;
	uint8_t reply[42];

	/* Ethernet/IPv4, a request, not for our own address */
	if (length < 42 || cl_get32(arp) != 0x00010800 || arp[4] != 6 || arp[5] != 4 || arp[7] != 1 || cl_get32(arp + 24) == cl_session.address) {
		return false;
	}
	memcpy(reply, frame + 6, 6);
	memcpy(reply + 6, cl_router_mac, 6);
	cl_put16(reply + 12, 0x0806);
	cl_put32(reply + 14, 0x00010800);
	reply[18] = 6; reply[19] = 4;
	cl_put16(reply + 20, 2);
	memcpy(reply + 22, cl_router_mac, 6);
	memcpy(reply + 28, arp + 24, 4);
	memcpy(reply + 32, arp + 8, 10);
	cl_count.arp++;
	hakios_net_input_unit(CL_UNIT, reply, sizeof(reply));
	return true;
}

#define CL_DHCP_DISCOVER        1
#define CL_DHCP_OFFER           2
#define CL_DHCP_REQUEST         3
#define CL_DHCP_ACK             5
#define CL_DHCP_NAK             6

/* Is this a DHCP request (UDP 68 -> 67)? `udp` is set to its UDP header. */
static bool
cl_dhcp_request(const uint8_t *frame, size_t length, const uint8_t **udp)
{
	const uint8_t *ip = frame + 14;
	size_t ihl;

	if (length < 14 + 20 + 8 + 240 || (ip[0] >> 4) != 4 || ip[9] != 17) {
		return false;
	}
	ihl = 4 * (ip[0] & 0xf);
	if (ihl < 20 || 14 + ihl + 8 + 240 > length) {
		return false;
	}
	*udp = ip + ihl;
	return ((*udp)[2] << 8 | (*udp)[3]) == 67;
}

/* The answer to a DHCP request: an offer for a discover, an acknowledgement for a request, with the session's settings. */
static bool
cl_dhcp_answer(const uint8_t *frame, size_t length, const uint8_t *udp)
{
	const uint8_t *request = udp + 8, *end = frame + length, *option;
	uint8_t reply[14 + 20 + 8 + 240 + 64], *ip = reply + 14, *rudp = ip + 20, *bootp = rudp + 8, *o;
	uint32_t type = 0, wanted = 0, kind;
	size_t size;

	if (request[0] != 1 || request[1] != 1 || request[2] != 6 || cl_get32(request + 236) != 0x63825363) {
		return false;
	}
	for (option = request + 240; option + 2 <= end && *option != 255; option += option[0] == 0 ? 1 : 2 + option[1]) {
		if (option[0] == 0) {
			continue;
		}
		if (option + 2 + option[1] > end) {
			break;
		}
		if (option[0] == 53 && option[1] == 1) {
			type = option[2];
		} else if (option[0] == 50 && option[1] == 4) {
			wanted = cl_get32(option + 2);
		}
	}
	if (type != CL_DHCP_DISCOVER && type != CL_DHCP_REQUEST) {
		return false;
	}
	kind = type == CL_DHCP_DISCOVER ? CL_DHCP_OFFER : wanted != 0 && wanted != cl_session.address ? CL_DHCP_NAK : CL_DHCP_ACK;
	bzero(reply, sizeof(reply));
	bootp[0] = 2; bootp[1] = 1; bootp[2] = 6;
	memcpy(bootp + 4, request + 4, 4);
	memcpy(bootp + 10, request + 10, 2);
	if (kind != CL_DHCP_NAK) {
		cl_put32(bootp + 16, cl_session.address);
		cl_put32(bootp + 20, cl_session.router);
	}
	memcpy(bootp + 28, request + 28, 16);
	cl_put32(bootp + 236, 0x63825363);
	o = bootp + 240;
	*o++ = 53; *o++ = 1; *o++ = (uint8_t)kind;
	*o++ = 54; *o++ = 4; cl_put32(o, cl_session.router); o += 4;
	if (kind != CL_DHCP_NAK) {
		*o++ = 51; *o++ = 4; cl_put32(o, 86400); o += 4;
		*o++ = 1; *o++ = 4; cl_put32(o, cl_session.mask); o += 4;
		*o++ = 3; *o++ = 4; cl_put32(o, cl_session.router); o += 4;
		if (cl_session.dns1 != 0 || cl_session.dns2 != 0) {
			*o++ = 6; *o++ = (uint8_t)(4 * ((cl_session.dns1 != 0) + (cl_session.dns2 != 0)));
			if (cl_session.dns1 != 0) { cl_put32(o, cl_session.dns1); o += 4; }
			if (cl_session.dns2 != 0) { cl_put32(o, cl_session.dns2); o += 4; }
		}
		if (cl_session.mtu >= 576 && cl_session.mtu <= 1500) {
			*o++ = 26; *o++ = 2; cl_put16(o, cl_session.mtu); o += 2;
		}
	}
	*o++ = 255;
	size = (size_t)(o - bootp);
	if (size < 300) {
		size = 300;
	}
	cl_put16(rudp, 67); cl_put16(rudp + 2, 68); cl_put16(rudp + 4, (uint32_t)(8 + size)); cl_put16(rudp + 6, 0);
	ip[0] = 0x45; ip[1] = 0;
	cl_put16(ip + 2, (uint32_t)(20 + 8 + size));
	ip[8] = 64; ip[9] = 17;
	cl_put32(ip + 12, cl_session.router);
	cl_put32(ip + 16, 0xffffffff);
	cl_put16(ip + 10, data_checksum(ip, 20));
	memcpy(reply, request + 28, 6);
	memcpy(reply + 6, cl_router_mac, 6);
	cl_put16(reply + 12, 0x0800);
	cl_count.dhcp++;
	say("modem: en1: DHCP %s answered with %s: %u.%u.%u.%u, router %u.%u.%u.%u, MTU %u\n", type == CL_DHCP_DISCOVER ? "discover" : "request",
	    kind == CL_DHCP_OFFER ? "an offer" : kind == CL_DHCP_ACK ? "an acknowledgement" : "a refusal", cl_session.address >> 24, cl_session.address >> 16 & 0xff,
	    cl_session.address >> 8 & 0xff, cl_session.address & 0xff, cl_session.router >> 24, cl_session.router >> 16 & 0xff, cl_session.router >> 8 & 0xff, cl_session.router & 0xff, cl_session.mtu);
	hakios_net_input_unit(CL_UNIT, reply, 14 + 20 + 8 + size);
	return true;
}

/*
 * ---- IPv6 on en1 ----
 * The modem's IPv6 session (a second WDS client: data_bringup6) gives a global address, a prefix and DNS servers; the packets are the same Ethernet frames of ours, with the type 0x86dd.
 * What a router on a link does is done here: Router Advertisements (a prefix to autoconfigure from, no on-link flag: everything goes to the router; the MTU; the DNS servers, RFC 8106),
 * answers to Router Solicitations and to Neighbor Solicitations (for the router: its link-local address fe80::1, and whatever else is asked), and silence for the rest of the link-local
 * traffic (multicast listener reports, Duplicate Address Detection, mDNS...), which is not the modem's business. Everything else goes to the modem as QMAP mux 2.
 */
static const uint8_t cl6_router_ll[16] = { 0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
static struct {
	volatile bool up;
	uint8_t address[16], gateway[16], dns[2][16];
	uint32_t prefix_length, mtu, handle;
	uint64_t next_ra;
	uint64_t ra_sent, rs_seen, ns_seen, ns_answered, ns_dad, mld_dropped, multicast_dropped, to_modem, from_modem, dropped_out, dropped_in;
	unsigned logged;
} cl6;
#define CL6_MUX 2

static void
cl6_hex(char *out, size_t size, const uint8_t *a)
{
	snprintf(out, size, "%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15]);
}

static bool
cl6_zero(const uint8_t *a)
{
	for (int i = 0; i < 16; i++) {
		if (a[i] != 0) {
			return false;
		}
	}
	return true;
}

/* the ICMPv6 checksum over the pseudo header and the message */
static uint16_t
cl6_checksum(const uint8_t *src, const uint8_t *dst, const uint8_t *icmp, uint32_t length)
{
	uint32_t sum = length + 58;

	for (int i = 0; i < 16; i += 2) {
		sum += (uint32_t)src[i] << 8 | src[i + 1];
		sum += (uint32_t)dst[i] << 8 | dst[i + 1];
	}
	for (uint32_t i = 0; i + 1 < length; i += 2) {
		sum += (uint32_t)icmp[i] << 8 | icmp[i + 1];
	}
	if (length & 1) {
		sum += (uint32_t)icmp[length - 1] << 8;
	}
	while (sum >> 16) {
		sum = (sum & 0xffff) + (sum >> 16);
	}
	return (uint16_t)~sum;
}

/* an ICMPv6 message from `src` to `dst` as a frame to the stack (hop limit 255, as neighbour discovery wants it) */
static void
cl6_send_icmp(const uint8_t *src, const uint8_t *dst, const uint8_t *dst_mac, uint8_t *icmp, uint32_t length)
{
	uint8_t frame[14 + 40 + 192];
	uint16_t sum;

	if (length > 192) {
		return;
	}
	memcpy(frame, dst_mac, 6);
	memcpy(frame + 6, cl_router_mac, 6);
	cl_put16(frame + 12, 0x86dd);
	bzero(frame + 14, 40);
	frame[14] = 0x60;
	cl_put16(frame + 18, length);
	frame[20] = 58;
	frame[21] = 255;
	memcpy(frame + 22, src, 16);
	memcpy(frame + 38, dst, 16);
	icmp[2] = icmp[3] = 0;
	sum = cl6_checksum(src, dst, icmp, length);
	icmp[2] = (uint8_t)(sum >> 8);
	icmp[3] = (uint8_t)sum;
	memcpy(frame + 54, icmp, length);
	hakios_net_input_unit(CL_UNIT, frame, 54 + length);
}

/* a Router Advertisement, to the all-nodes address or to whoever solicited it */
static void
cl6_send_ra(const uint8_t *dst, const uint8_t *dst_mac, const char *why)
{
	uint8_t icmp[160];
	uint32_t n = 0, mtu = cl6.mtu >= 1280 && cl6.mtu <= 1500 ? cl6.mtu : 1280, dns = (cl6_zero(cl6.dns[0]) ? 0 : 1) + (cl6_zero(cl6.dns[1]) ? 0 : 1);
	char text[48];

	if (!cl6.up) {
		return;
	}
	bzero(icmp, sizeof(icmp));
	icmp[n++] = 134;                        /* Router Advertisement */
	icmp[n++] = 0;
	n += 2;                                 /* checksum */
	icmp[n++] = 64;                         /* hop limit */
	icmp[n++] = 0;                          /* no managed / other configuration: SLAAC and the RDNSS below */
	cl_put16(icmp + n, 1800);               /* router lifetime */
	n += 2;
	n += 8;                                 /* reachable and retransmit time: unspecified */
	icmp[n++] = 1; icmp[n++] = 1;           /* source link-layer address */
	memcpy(icmp + n, cl_router_mac, 6);
	n += 6;
	icmp[n++] = 5; icmp[n++] = 1;           /* MTU */
	n += 2;
	cl_put32(icmp + n, mtu);
	n += 4;
	icmp[n++] = 3; icmp[n++] = 4;           /* prefix information: /64 of the session's address, autonomous, not on-link */
	icmp[n++] = 64;
	icmp[n++] = 0x40;
	cl_put32(icmp + n, 86400);
	n += 4;
	cl_put32(icmp + n, 14400);
	n += 4;
	n += 4;
	memcpy(icmp + n, cl6.address, 8);       /* (the low 64 bits stay zero) */
	n += 16;
	if (dns > 0) {
		icmp[n++] = 25; icmp[n++] = (uint8_t)(1 + 2 * dns);
		n += 2;
		cl_put32(icmp + n, 1800);
		n += 4;
		for (int d = 0; d < 2; d++) {
			if (!cl6_zero(cl6.dns[d])) {
				memcpy(icmp + n, cl6.dns[d], 16);
				n += 16;
			}
		}
	}
	cl6_send_icmp(cl6_router_ll, dst, dst_mac, icmp, n);
	cl6.ra_sent++;
	cl6_hex(text, sizeof(text), cl6.address);
	if (cl6.logged++ < 40) {
		say("modem: en1: IPv6: router advertisement #%llu (%s): prefix %.19s::/64, MTU %u, %u DNS server(s), %u bytes\n", cl6.ra_sent, why, text, mtu, dns, n);
	}
}

/* a frame of the stack with an IPv6 packet: neighbour and router discovery answered, local multicast dropped, the rest to the modem. true: handled (sent or deliberately not) */
static bool
cl6_transmit(const uint8_t *frame, size_t length)
{
	const uint8_t *ip = frame + 14, *icmp = ip + 40;
	static const uint8_t all_nodes[16] = { 0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	static const uint8_t all_nodes_mac[6] = { 0x33, 0x33, 0, 0, 0, 1 };
	uint32_t payload;

	if (length < 14 + 40 || (ip[0] >> 4) != 6) {
		cl6.dropped_out++;
		return false;
	}
	payload = (uint32_t)ip[4] << 8 | ip[5];
	if (ip[6] == 58 && payload >= 8 && length >= 14 + 40 + 8) {
		switch (icmp[0]) {
		case 133:                       /* Router Solicitation: an advertisement to the asker (or to everybody if it has no address yet) */
			cl6.rs_seen++;
			if (cl6.logged++ < 40) {
				say("modem: en1: IPv6: router solicitation #%llu\n", cl6.rs_seen);
			}
			if (cl6_zero(ip + 8)) {
				cl6_send_ra(all_nodes, all_nodes_mac, "answer to a solicitation");
			} else {
				cl6_send_ra(ip + 8, frame + 6, "answer to a solicitation");
			}
			return true;
		case 135:                       /* Neighbor Solicitation */
			cl6.ns_seen++;
			if (payload >= 24 && length >= 14 + 40 + 24) {
				if (cl6_zero(ip + 8)) {
					cl6.ns_dad++;   /* Duplicate Address Detection of the stack's own address: nobody else has it */
					return true;
				}
				{
					uint8_t na[32];

					bzero(na, sizeof(na));
					na[0] = 136;
					na[4] = 0xe0;   /* router, solicited, override */
					memcpy(na + 8, icmp + 8, 16);
					na[24] = 2; na[25] = 1;
					memcpy(na + 26, cl_router_mac, 6);
					cl6_send_icmp(icmp + 8, ip + 8, frame + 6, na, 32);
					cl6.ns_answered++;
					if (cl6.logged++ < 40) {
						char text[48];

						cl6_hex(text, sizeof(text), icmp + 8);
						say("modem: en1: IPv6: neighbor solicitation for %s answered (#%llu)\n", text, cl6.ns_answered);
					}
				}
			}
			return true;
		case 130: case 131: case 132: case 143:    /* multicast listener discovery */
			cl6.mld_dropped++;
			return true;
		case 134: case 136: case 137:   /* nobody but a router answers or redirects here */
			return true;
		default:
			break;
		}
	}
	if (ip[24] == 0xff || (ip[24] == 0xfe && (ip[25] & 0xc0) == 0x80)) {
		cl6.multicast_dropped++;        /* multicast and link-local unicast stay on the link: there is nobody else on it */
		return true;
	}
	if (!cl6.up || length - 14 > 2048) {
		cl6.dropped_out++;
		return false;
	}
	if (!data_send_mux(ip, (uint32_t)(length - 14), CL6_MUX)) {
		cl6.dropped_out++;
		return false;
	}
	cl6.to_modem++;
	if (cl6.to_modem <= 5) {
		char text[48];

		cl6_hex(text, sizeof(text), ip + 24);
		say("modem: en1: IPv6: packet #%llu to the modem: %u bytes, next header %u, to %s\n", cl6.to_modem, (uint32_t)(length - 14), ip[6], text);
	}
	return true;
}

/* an IPv6 packet from the modem to the stack */
static void
cl6_receive(const uint8_t *packet, uint32_t length)
{
	uint8_t frame[HAKIOS_NET_FRAME_MAX];

	if (!cl6.up || length < 40 || length + 14 > sizeof(frame)) {
		cl6.dropped_in++;
		return;
	}
	memcpy(frame, cl_mac, 6);
	memcpy(frame + 6, cl_router_mac, 6);
	cl_put16(frame + 12, 0x86dd);
	memcpy(frame + 14, packet, length);
	cl6.from_modem++;
	if (cl6.from_modem <= 5) {
		char text[48];

		cl6_hex(text, sizeof(text), packet + 8);
		say("modem: en1: IPv6: packet #%llu from the modem: %u bytes, next header %u, from %s\n", cl6.from_modem, length, packet[6], text);
	}
	hakios_net_input_unit(CL_UNIT, frame, 14 + length);
}

static void
cl6_down(void)
{
	cl6.up = false;
	bzero(cl6.address, sizeof(cl6.address));
	cl6.handle = 0;
}

/* A frame from the stack: answered here, or its IP packet to the modem. */
static bool
cl_transmit(mbuf_t packet, size_t length)
{
	uint8_t frame[HAKIOS_NET_FRAME_MAX];
	const uint8_t *udp;
	uint32_t type;
	bool ok;

	if (length < 14 || length > sizeof(frame) || mbuf_copydata(packet, 0, length, frame) != 0) {
		return false;
	}
	type = (uint32_t)frame[12] << 8 | frame[13];
	lck_mtx_lock(&cl_lock);
	if (type == 0x0806) {
		ok = cl_arp_answer(frame, length);
	} else if (type == 0x0800 && cl_dhcp_request(frame, length, &udp)) {
		ok = cl_session.up && cl_dhcp_answer(frame, length, udp);
	} else if (type == 0x0800) {
		ok = cl_session.up && data_send(frame + 14, (uint32_t)(length - 14));
		if (ok) {
			cl_count.to_modem++;
		} else {
			cl_count.dropped_out++;
		}
	} else if (type == 0x86dd) {
		ok = cl6_transmit(frame, length);
	} else {
		cl_count.not_ip4++;
		ok = false;
	}
	lck_mtx_unlock(&cl_lock);
	return ok;
}

/* A packet from the modem: an Ethernet frame from the router made around it. */
static void
cl_receive(const uint8_t *packet, uint32_t length)
{
	uint8_t frame[HAKIOS_NET_FRAME_MAX];
	uint32_t version = length > 0 ? packet[0] >> 4 : 0;

	if (version == 6) {
		cl6_receive(packet, length);
		return;
	}
	if (length < 20 || length + 14 > sizeof(frame) || version != 4) {
		cl_count.dropped_in++;
		return;
	}
	memcpy(frame, cl_mac, 6);
	memcpy(frame + 6, cl_router_mac, 6);
	cl_put16(frame + 12, 0x0800);
	memcpy(frame + 14, packet, length);
	cl_count.from_modem++;
	hakios_net_input_unit(CL_UNIT, frame, 14 + length);
}

static struct hakios_net_link cl_link = {
	.name = "the modem (mobile data)",
	.unit = CL_UNIT,
	.address = { 0x02, 0x68, 0x61, 0x6b, 0x69, 0x01 },
	.wireless = false,
	.up = false,
	.transmit = cl_transmit,
	.poll = NULL,
};

/* The session is up: en1 made (once) and its link raised, with the session's settings for DHCP. */
static void
cl_session_up(uint32_t address, uint32_t gateway, uint32_t mask, uint32_t dns1, uint32_t dns2, uint32_t mtu)
{
	(void)gateway;
	(void)mask;
	lck_mtx_lock(&cl_lock);
	cl_session.address = address;
	cl_session.mask = 0xffffff00;
	cl_session.router = (address & 0xffffff00) | 1;
	if (cl_session.router == address) {
		cl_session.router = (address & 0xffffff00) | 2;
	}
	cl_session.dns1 = dns1;
	cl_session.dns2 = dns2;
	cl_session.mtu = mtu;
	cl_session.up = address != 0;
	lck_mtx_unlock(&cl_lock);
	if (!cl_attached) {
		cl_attached = hakios_net_attach(&cl_link);
	}
	data.receive = cl_receive;
	if (cl_attached) {
		hakios_net_link_state_unit(CL_UNIT, cl_session.up);
	}
	say("modem: en1: %s, link %s\n", cl_attached ? "made" : "NOT made", cl_session.up ? "up" : "down");
}

static uint32_t data_le32(const uint8_t *v);
static void data_put32(uint8_t *b, uint32_t *k, uint8_t type, uint32_t v);

/* every TLV of the last QMI reply: type, length and the first bytes (the IPv6 settings' numbers are not certain: this is what shows them) */
static void
qmi_tlv_list(const char *what)
{
	for (uint32_t t = 0; t + 3 <= qmi_reply.length;) {
		uint32_t n = qmi_reply.data[t + 1] | qmi_reply.data[t + 2] << 8;
		char hex[3 * 24 + 1];
		uint32_t m = n < 24 ? n : 24;

		if (t + 3 + n > qmi_reply.length) {
			break;
		}
		for (uint32_t i = 0; i < m; i++) {
			snprintf(hex + 3 * i, 4, "%02x ", qmi_reply.data[t + 3 + i]);
		}
		say("modem: qmi: %s: TLV %#04x, %u bytes: %s%s\n", what, qmi_reply.data[t], n, hex, n > m ? "..." : "");
		t += 3 + n;
	}
}

/* the IPv6 session: a second WDS client (our QRTR port QMI_PORT6) bound to the QMAP mux 2 of the same data port, family 6, started, its settings read; then en1 gets its router */
static void
data_bringup6(void)
{
	uint8_t t[100];
	uint32_t k, n;
	const uint8_t *c;
	int e;
	char text[48];

	if (!option_data6 || !data.format_done || !option_en1) {
		say("modem: data6: not tried (modem_data6 %u, data format %s, en1 %u)\n", option_data6, data.format_done ? "set" : "NOT set", option_en1);
		return;
	}
	qmi_src_port = QMI_PORT6;
	{
		uint8_t ep[8] = { (uint8_t)data.ep_type, (uint8_t)(data.ep_type >> 8), (uint8_t)(data.ep_type >> 16), (uint8_t)(data.ep_type >> 24), (uint8_t)data.ep_iface, (uint8_t)(data.ep_iface >> 8),
			(uint8_t)(data.ep_iface >> 16), (uint8_t)(data.ep_iface >> 24) };
		static const uint8_t mux = CL6_MUX, family = 6, preference = 6;

		k = 0;
		tlv_put(t, &k, 0x10, ep, 8);
		tlv_put(t, &k, 0x11, &mux, 1);
		e = qmi_call(QMI_WDS, 0x00a2, t, k, 5000);
		say("modem: data6: WDS bind mux data port (mux %u) from port %#x: %s (result %u error %u)\n", mux, qmi_src_port, e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
		k = 0;
		tlv_put(t, &k, 0x01, &family, 1);
		e = qmi_call(QMI_WDS, 0x004d, t, k, 5000);
		say("modem: data6: WDS set IP family (IPv6): %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
		for (unsigned attempt = 0; attempt < 6 && cl6.handle == 0; attempt++) {
			k = 0;
			if (option_apn[0] != 0) {
				tlv_put(t, &k, 0x14, option_apn, (uint16_t)strlen(option_apn));
			}
			tlv_put(t, &k, 0x19, &preference, 1);
			e = qmi_call(QMI_WDS, 0x0020, t, k, 45000);
			say("modem: data6: WDS start network interface (try %u): %s (result %u error %u)\n", attempt + 1, e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
			if (e == 0) {
				qmi_tlv_list("IPv6 start network reply");
			}
			if (e == 0 && qmi_reply.result == 0 && (c = qmi_tlv(0x01, &n)) != NULL && n >= 4) {
				cl6.handle = data_le32(c);
				break;
			}
			mission_pump(3000, NULL);
		}
	}
	if (cl6.handle == 0) {
		say("modem: data6: no IPv6 session\n");
		qmi_src_port = QMI_PORT;
		return;
	}
	k = 0;
	data_put32(t, &k, 0x10, 0xa310);
	e = qmi_call(QMI_WDS, 0x002d, t, k, 5000);
	say("modem: data6: WDS get current settings: %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	if (e != 0 || qmi_reply.result != 0) {
		qmi_src_port = QMI_PORT;
		return;
	}
	qmi_tlv_list("IPv6 settings");
	qmi_src_port = QMI_PORT;
	if ((c = qmi_tlv(0x25, &n)) != NULL && n >= 17) {
		memcpy(cl6.address, c, 16);
		cl6.prefix_length = c[16];
	}
	if ((c = qmi_tlv(0x26, &n)) != NULL && n >= 16) {
		memcpy(cl6.gateway, c, 16);
	}
	if ((c = qmi_tlv(0x27, &n)) != NULL && n >= 16) {
		memcpy(cl6.dns[0], c, 16);
	}
	if ((c = qmi_tlv(0x28, &n)) != NULL && n >= 16) {
		memcpy(cl6.dns[1], c, 16);
	}
	if ((c = qmi_tlv(0x29, &n)) != NULL && n >= 4) {
		cl6.mtu = data_le32(c);
	}
	cl6_hex(text, sizeof(text), cl6.address);
	say("modem: data6: address %s/%u, MTU %u, DNS servers %s %s\n", text, cl6.prefix_length, cl6.mtu, cl6_zero(cl6.dns[0]) ? "none" : "yes", cl6_zero(cl6.dns[1]) ? "none" : "yes");
	if (cl6_zero(cl6.address)) {
		say("modem: data6: the session has no IPv6 address (TLV 0x25 missing: see the list above for where it is)\n");
		return;
	}
	if (!cl_attached) {
		cl_attached = hakios_net_attach(&cl_link);
		say("modem: en1: made (by the IPv6 session): %s\n", cl_attached ? "yes" : "NO");
	}
	data.receive = cl_receive;
	cl6.up = true;
	cl6.next_ra = 0;
	if (cl_attached) {
		hakios_net_link_state_unit(CL_UNIT, true);
	}
	say("modem: data6: THE IPv6 SESSION IS UP; router advertisements follow\n");
}

/* every few seconds a Router Advertisement (soon after the session, then every 20 s) */
static void
cl6_tick(uint64_t now)
{
	static const uint8_t all_nodes[16] = { 0xff, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	static const uint8_t all_nodes_mac[6] = { 0x33, 0x33, 0, 0, 0, 1 };

	if (!cl6.up || now < cl6.next_ra) {
		return;
	}
	cl6_send_ra(all_nodes, all_nodes_mac, "periodic");
	cl6.next_ra = now + (cl6.ra_sent < 4 ? 3000000000ull : 20000000000ull);
}

/* The modem went away (or is about to be started again): no session, en1's link down; the queue and the counters of the old one forgotten. */
static void
data_down(void)
{
	data.up = false;
	data.format_done = false;
	data.handle = 0;
	data.address = data.gateway = data.mask = data.dns1 = data.dns2 = data.mtu = 0;
	cl6_down();
	data.dns_answered = false;
	data_in.head = data_in.tail = 0;
	lck_mtx_lock(&cl_lock);
	cl_session.up = false;
	lck_mtx_unlock(&cl_lock);
	if (cl_attached) {
		hakios_net_link_state_unit(CL_UNIT, false);
	}
}

static uint32_t
data_le32(const uint8_t *v)
{
	return (uint32_t)(v[0] | v[1] << 8 | v[2] << 16 | (uint32_t)v[3] << 24);
}

static void
data_put32(uint8_t *b, uint32_t *k, uint8_t type, uint32_t v)
{
	uint8_t w[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };

	tlv_put(b, k, type, w, 4);
}

static void data_bringup6(void);
static bool
data_bringup(void)
{
	static const uint32_t ifaces[] = { 4, 0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12 };
	uint32_t node, port, ep_type = option_data_ep >> 16, ep_iface = option_data_ep & 0xffff;
	bool format = false;
	uint8_t t[120];
	uint32_t k, n;
	const uint8_t *c;
	int e;

	if (!qmi_find(QMI_WDA, &node, &port) || !qmi_find(QMI_WDS, &node, &port)) {
		say("modem: data: the modem announces no WDA or WDS service\n");
		return false;
	}
	data.mux = 1;
	data.handle = 0;
	/* 1. the data format of the MHI port; the interface id the modem wants is not known: the configured one first, then the others */
	for (unsigned i = 0; i <= sizeof(ifaces) / sizeof(ifaces[0]) && !format; i++) {
		uint32_t iface = i == 0 ? ep_iface : ifaces[i - 1];
		uint8_t ep[8] = { (uint8_t)ep_type, (uint8_t)(ep_type >> 8), (uint8_t)(ep_type >> 16), (uint8_t)(ep_type >> 24), (uint8_t)iface, (uint8_t)(iface >> 8), (uint8_t)(iface >> 16), (uint8_t)(iface >> 24) };

		if (i > 0 && iface == ep_iface) {
			continue;
		}
		k = 0;
		data_put32(t, &k, 0x11, 2);                              /* link layer: raw IP */
		data_put32(t, &k, 0x12, option_data_qmap ? 5 : 0);       /* uplink aggregation: QMAP */
		data_put32(t, &k, 0x13, option_data_qmap ? 5 : 0);       /* downlink aggregation: QMAP */
		if (option_data_qmap) {
			data_put32(t, &k, 0x15, 4);                      /* at most 4 datagrams ... */
			data_put32(t, &k, 0x16, 0x1c00);                 /* ... or 7168 bytes per downlink transfer (the buffers are 8 KB) */
		}
		tlv_put(t, &k, 0x17, ep, 8);
		e = qmi_call(QMI_WDA, 0x0020, t, k, 5000);
		say("modem: data: WDA set data format on endpoint type %u interface %u: %s (result %u error %u)\n", ep_type, iface, e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
		if (e == 0 && qmi_reply.result == 0) {
			format = true;
			qmi_hex("WDA reply TLVs", qmi_reply.data, qmi_reply.length);
			ep_iface = iface;
		}
	}
	if (!format) {
		return false;
	}
	data.format_done = true;
	data.ep_type = ep_type;
	data.ep_iface = ep_iface;
	if (option_data6 == 2) {
		data_bringup6();
		if (cl6.up) {
			say("modem: data: the IPv6 session is up: no IPv4 call (the carrier's PDN is IPv6-only: IPv4 is iOS's own 464XLAT, CLAT46, over the NAT64 prefix the network announces)\n");
			return true;
		}
	}
	/* 2. the client bound to the mux id of that port */
	{
		uint8_t ep[8] = { (uint8_t)ep_type, (uint8_t)(ep_type >> 8), (uint8_t)(ep_type >> 16), (uint8_t)(ep_type >> 24), (uint8_t)ep_iface, (uint8_t)(ep_iface >> 8), (uint8_t)(ep_iface >> 16), (uint8_t)(ep_iface >> 24) };

		k = 0;
		tlv_put(t, &k, 0x10, ep, 8);
		tlv_put(t, &k, 0x11, &data.mux, 1);
		e = qmi_call(QMI_WDS, 0x00a2, t, k, 5000);
		say("modem: data: WDS bind mux data port (mux %u): %s (result %u error %u)\n", data.mux, e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	}
	{
		static const uint8_t family = 4;

		k = 0;
		tlv_put(t, &k, 0x01, &family, 1);
		e = qmi_call(QMI_WDS, 0x004d, t, k, 5000);
		say("modem: data: WDS set IP family (IPv4): %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	}
	/* 3. the session: the packet domain may not be attached yet, so a refusal is retried */
	for (unsigned attempt = 0; attempt < 8; attempt++) {
		static const uint8_t preference = 4;

		k = 0;
		if (option_apn[0] != 0) {
			tlv_put(t, &k, 0x14, option_apn, (uint16_t)strlen(option_apn));
		}
		tlv_put(t, &k, 0x19, &preference, 1);
		e = qmi_call(QMI_WDS, 0x0020, t, k, 45000);
		say("modem: data: WDS start network interface (try %u): %s (result %u error %u)\n", attempt + 1, e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
		if (e == 0 && qmi_reply.result == 0 && (c = qmi_tlv(0x01, &n)) != NULL && n >= 4) {
			data.handle = data_le32(c);
			break;
		}
		if (e == 0) {
			qmi_hex("start network reply TLVs", qmi_reply.data, qmi_reply.length);
		}
		mission_pump(3000, NULL);
	}
	if (data.handle == 0) {
		return false;
	}
	/* 4. what the network gave */
	k = 0;
	data_put32(t, &k, 0x10, 0xa310);
	e = qmi_call(QMI_WDS, 0x002d, t, k, 5000);
	say("modem: data: WDS get current settings: %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	if (e != 0 || qmi_reply.result != 0) {
		return false;
	}
	qmi_hex("settings TLVs", qmi_reply.data, qmi_reply.length);
	if ((c = qmi_tlv(0x1e, &n)) != NULL && n >= 4) { data.address = data_le32(c); }
	if ((c = qmi_tlv(0x20, &n)) != NULL && n >= 4) { data.gateway = data_le32(c); }
	if ((c = qmi_tlv(0x21, &n)) != NULL && n >= 4) { data.mask = data_le32(c); }
	if ((c = qmi_tlv(0x15, &n)) != NULL && n >= 4) { data.dns1 = data_le32(c); }
	if ((c = qmi_tlv(0x16, &n)) != NULL && n >= 4) { data.dns2 = data_le32(c); }
	if ((c = qmi_tlv(0x29, &n)) != NULL && n >= 4) { data.mtu = data_le32(c); }
	say("modem: data: THE SESSION IS UP: address %u.%u.%u.%u, gateway %u.%u.%u.%u, mask %u.%u.%u.%u, DNS %u.%u.%u.%u %u.%u.%u.%u, MTU %u\n",
	    data.address >> 24, data.address >> 16 & 0xff, data.address >> 8 & 0xff, data.address & 0xff, data.gateway >> 24, data.gateway >> 16 & 0xff, data.gateway >> 8 & 0xff, data.gateway & 0xff,
	    data.mask >> 24, data.mask >> 16 & 0xff, data.mask >> 8 & 0xff, data.mask & 0xff, data.dns1 >> 24, data.dns1 >> 16 & 0xff, data.dns1 >> 8 & 0xff, data.dns1 & 0xff,
	    data.dns2 >> 24, data.dns2 >> 16 & 0xff, data.dns2 >> 8 & 0xff, data.dns2 & 0xff, data.mtu);
	data.up = data.address != 0;
	if (data.up && option_en1) {
		cl_session_up(data.address, data.gateway, data.mask, data.dns1, data.dns2, data.mtu);
	}
	if (data.up && option_data >= 2) {
		/* the test: a DNS query to each server, three tries, the answer looked for while the channels are serviced */
		for (unsigned attempt = 0; attempt < 3 && !data.dns_answered; attempt++) {
			uint8_t q[80];
			uint32_t servers[2] = { data.dns1, data.dns2 };

			for (unsigned s = 0; s < 2 && !data.dns_answered; s++) {
				uint32_t len;

				if (servers[s] == 0) {
					continue;
				}
				data.dns_id = (uint16_t)(0x4800 + attempt * 2 + s);
				len = data_dns_query(q, data.address, servers[s], data.dns_id);
				say("modem: data: DNS query to %u.%u.%u.%u: %s\n", servers[s] >> 24, servers[s] >> 16 & 0xff, servers[s] >> 8 & 0xff, servers[s] & 0xff, data_send(q, len) ? "sent" : "NOT SENT");
				mission_pump(2000, &data.dns_answered);
			}
		}
		say("modem: data: DNS test: %s; %llu packets to the modem, %llu from it\n", data.dns_answered ? "PASSED" : "no answer", data.to_modem, data.from_modem);
	}
	return data.up;
}

/* when the services stop answering: is it the channel (loopback), the modem's other services, or all */
static void
modem_diagnose(void)
{
	static const char ping[] = "HackiOS loopback test";
	unsigned before = loopback_echoes;
	int e = hakios_mhi_send(&mhi, 0, ping, sizeof(ping));

	mission_pump(500, NULL);
	say("modem: diag: %u resume_tx sent, %u datagrams out; loopback ping %d, echoes %u; %u datagrams in so far, %u events serviced, MHI state %#x, EE %#x\n", qrtr_resumes, qrtr.packets_out, e, loopback_echoes - before, qrtr.packets_in, mhi.serviced, mhi.state, mhi.ee);
	for (unsigned c = 20; c <= 21; c++) {
		uint32_t state, hrp, hwp, elements;
		uint64_t crp, cwp;

		hakios_mhi_channel_state(&mhi, c, &state, &crp, &cwp, &hrp, &hwp, &elements);
		say("modem: diag: channel %u: state %u, device rp %#llx wp %#llx, host rp %u wp %u of %u\n", c, state, crp, cwp, hrp, hwp, elements);
	}
	say("modem: diag: MHICTRL %#x, MHISTATUS %#x, EE %u\n", rd(pcie.bar + 0x38), rd(pcie.bar + MHI_STATUS), rd(pcie.bar + pcie.bhi + BHI_EXECENV));
	if (diag_state.on) {
		diag_flush(DIAG_RING);
	}
	e = qmi_call(2, 0x002d, NULL, 0, 2000);
	say("modem: diag: DMS operating mode: %s\n", e == 0 ? "answered" : "no reply");
}

static void
modem_bringup(void)
{
	snprintf(cell.state, sizeof(cell.state), "starting");
	uint8_t registration = 0, radio = 0;
	uint32_t n;
	const uint8_t *c;
	int e;

	e = qmi_call(2, 0x002d, NULL, 0, 3000);
	c = e == 0 ? qmi_tlv(0x01, &n) : NULL;
	say("modem: bringup: DMS operating mode: %d (3 offline, 0 online)\n", c != NULL ? c[0] : -1);
	if (c != NULL && c[0] != 0) {
		/* the radio on: DMS Set Operating Mode, online */
		static const uint8_t tlv[4] = { 0x01, 0x01, 0x00, 0x00 };

		e = qmi_call(2, 0x002e, tlv, sizeof(tlv), 5000);
		say("modem: bringup: DMS set online: %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	}
	bool modes_set = option_modes == 0;
	for (unsigned round = 0; round < 150; round++) {
		bool pin_sent = false;

		if (rd(pcie.bar) == 0xffffffffu) {
			say("modem: bringup: THE LINK IS DOWN (the modem went away)\n");
			return;
		}
		{
			const uint8_t *m;
			uint32_t mn;

			if (qmi_call(2, 0x002d, NULL, 0, 2000) == 0 && (m = qmi_tlv(0x01, &mn)) != NULL && m[0] != 0) {
				static const uint8_t tlv[4] = { 0x01, 0x01, 0x00, 0x00 };
				int r = qmi_call(2, 0x002e, tlv, sizeof(tlv), 5000);

				say("modem: bringup: DMS mode %u, set online: %s (result %u error %u)\n", m[0], r == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
			}
		}
		if (!modes_set) {
			/*
			 * modem_modes=MASK: the radio technologies the modem may use, NAS Set System Selection Preference (0x0033), TLV 0x11 the
			 * mode preference (1 CDMA, 2 HDR, 4 GSM, 8 UMTS, 0x10 LTE, 0x20 TD-SCDMA, 0x40 NR5G), TLV 0x17 until the next power
			 * cycle only (1). Sent here, once the modem answers, until it takes it. spartan, 19 November: on Jio the modem stayed
			 * "searching" on its 5G (standalone) cell and never tried LTE; modem_modes=0x1c keeps it to GSM, UMTS and LTE.
			 */
			const uint8_t tlv[9] = { 0x11, 0x02, 0x00, (uint8_t)option_modes, (uint8_t)(option_modes >> 8), 0x17, 0x01, 0x00, 0x01 };
			int r = qmi_call(3, 0x0033, tlv, sizeof(tlv), 3000);

			modes_set = r == 0 && qmi_reply.result == 0;
			if (modes_set || round % 10 == 0) {
				say("modem: bringup: NAS mode preference %#x (until the next power cycle): %s (result %u error %u)\n", option_modes,
				    r == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
			}
		}
		if (round == 8 || round == 30) {
			/*
			 * spartan, 19 November: on LTE the network refused the modem (a reject cause of 3 in the serving system's
			 * report). Who the modem says it is -- its IMEI, of which only the length and whether its check digit holds are
			 * said, never the number -- and NAS Get System Info's every TLV (its reject domain and cause are in there).
			 */
			const uint8_t *imei;
			uint32_t n = 0;

			if (qmi_call(2, 0x0025, NULL, 0, 3000) == 0 && (imei = qmi_tlv(0x11, &n)) != NULL && n > 0) {
				unsigned sum = 0, digits = 0;
				bool numeric = true;

				for (uint32_t i = 0; i < n; i++) {
					unsigned d = (unsigned)(imei[n - 1 - i] - '0');

					if (d > 9) {
						numeric = false;
						break;
					}
					sum += i % 2 == 1 ? (2 * d > 9 ? 2 * d - 9 : 2 * d) : d;
					digits++;
				}
				say("modem: identity: the modem has an IMEI of %u characters, %s\n", n, !numeric ? "NOT ALL DIGITS" :
				    digits == 15 && sum % 10 == 0 ? "15 digits, its check digit holds" : "WHOSE CHECK DIGIT DOES NOT HOLD");
			} else {
				say("modem: identity: NO IMEI (DMS 0x0025: result %u error %u)\n", qmi_reply.result, qmi_reply.error);
			}
			if (qmi_call(3, 0x004d, NULL, 0, 3000) == 0) {
				qmi_tlv_list("NAS system info");
			}
		}
		if (!uim_card_status()) {
			modem_diagnose();
		} else if (sim.known) {
			if (sim.app_state == 2 && sim.pin1_state == 1 && sim.pin1_retries >= 3 && !pin_sent) {
				/* PIN1 verify with the owner's PIN: only when it is required and every try is left */
				uint8_t t[16];
				uint32_t k = 0;
				static const uint8_t session[2] = { 0, 0 };
				const uint8_t pin_tlv[6] = { 1, 4, '0', '0', '0', '0' };

				tlv_put(t, &k, 0x01, session, 2);
				tlv_put(t, &k, 0x02, pin_tlv, 6);
				e = qmi_call(11, 0x0026, t, k, 5000);
				say("modem: sim: PIN1 verified: %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
				pin_sent = true;
			} else if (sim.app_state == 2 && sim.pin1_retries < 3) {
				say("modem: sim: the PIN is required but only %u tries are left: NOT sent\n", sim.pin1_retries);
			}
		}
		if (nas_registration(&registration, &radio) == 0 && registration == 1) {
			say("modem: bringup: THE MODEM IS REGISTERED ON A NETWORK\n");
			cell_started = true;
			modem_generation++;
			cell.registered = 1;
			if (option_data) {
				data_bringup();
				if (option_data6 == 1) {
					data_bringup6();
				}
			}
			if (option_sms_self) {
				/* a message to the SIM's own number (DMS Get MSISDN), to see the receive path without anybody's help; the number is never printed */
				char digits[24];
				uint32_t d = 0;
				bool have = false;

				if (qmi_call(2, 0x0024, NULL, 0, 3000) == 0 && qmi_reply.result == 0 && (c = qmi_tlv(0x01, &n)) != NULL && n >= 6 && n < 24) {
					for (uint32_t i = 0; i < n; i++) {
						if (c[i] >= '0' && c[i] <= '9') {
							digits[d++] = (char)c[i];
						}
					}
					have = d >= 6;
				}
				if (!have) {
					/* the card's EF MSISDN (6F40, under the USIM's directory 3F00/7FFF), record 1: alpha identifier, then length, type of number, 10 digit bytes (swapped BCD), two more */
					uint8_t t[24], session[2] = { 0, 0 }, record[4] = { 1, 0, 0, 0 };
					uint32_t k = 0;

					tlv_put(t, &k, 0x01, session, 2);
					{
						uint8_t f[7] = { 0x40, 0x6f, 4, 0x00, 0x3f, 0xff, 0x7f };       /* file id, then the path: 3F00 7FFF as two little-endian words */

						tlv_put(t, &k, 0x02, f, sizeof(f));
					}
					tlv_put(t, &k, 0x03, record, 4);
					if (qmi_call(11, 0x0021, t, k, 5000) == 0 && qmi_reply.result == 0 && (c = qmi_tlv(0x11, &n)) != NULL && n >= 3 + 14) {
						uint32_t len = c[0] | c[1] << 8, base = len >= 14 ? len - 14 : 0;
						const uint8_t *r = c + 2;

						if (len + 2 <= n && r[base] >= 2 && r[base] <= 11) {
							uint32_t digits_bytes = r[base] - 1;

							d = 0;
							for (uint32_t i = 0; i < digits_bytes && i < 10; i++) {
								uint8_t lo = r[base + 2 + i] & 0xf, hi = r[base + 2 + i] >> 4;

								if (lo < 10) { digits[d++] = (char)('0' + lo); }
								if (hi < 10) { digits[d++] = (char)('0' + hi); }
							}
							digits[d] = 0;
							have = d >= 6;
						}
					}
					{
						uint32_t ff = 0, m = 0;
						const uint8_t *c2 = qmi_reply.result == 0 ? qmi_tlv(0x11, &m) : NULL;

						for (uint32_t i = 2; c2 != NULL && i < m; i++) {
							ff += c2[i] == 0xff;
						}
						say("modem: sms: EF MSISDN: TLV 0x11 of %u bytes, %u of them 0xff (an empty record is all 0xff)\n", m, ff);
					}
					say("modem: sms: EF MSISDN read: %s (result %u error %u)\n", have ? "a number" : "nothing usable", qmi_reply.result, qmi_reply.error);
				}
				if (!have) {
					/* modem_sms_num=DIGITS: the phone's own number, given by its owner for the test (the SIM stores none) */
					uint64_t given = 0;

					if (PE_parse_boot_argn("modem_sms_num", &given, sizeof(given)) && given >= 100000) {
						snprintf(digits, sizeof(digits), "%llu", given);
						d = (uint32_t)strlen(digits);
						have = true;
					}
				}
				if (have) {
					digits[d] = 0;
					say("modem: sms: own number has %u digits; self-test message sent: %d\n", d, wms_send_text(digits, "HackiOS self-test: this message came from the OnePlus 8 Pro to itself, line 1"));
				} else {
					say("modem: sms: the SIM has no readable own number (DMS Get MSISDN %u/%u)\n", qmi_reply.result, qmi_reply.error);
				}
			}
			if (option_sms) {
				/* one test message to the owner's number (given for these tests); an error answer is retried a few times, a missing answer is not (it may have gone) */
				for (unsigned attempt = 0; attempt < 6; attempt++) {
					int r = wms_send_text("33669141092", "HackiOS test: this SMS was sent by the OnePlus 8 Pro modem, driven from the iOS kernel.");

					if (r == 0 || r == ETIMEDOUT) {
						break;
					}
					mission_pump(3000, NULL);
				}
				mission_pump(20000, NULL);
			}
			return;
		}
		mission_pump(2000, NULL);
	}
	say("modem: bringup: not registered after the rounds\n");
}


/* ---- the service the rest of the system sees: sysctls kern.hakios.modem_*, and what keeps them true ------------------------------------------------ */


static int
cell_state_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[200];

	snprintf(text, sizeof(text), "%s", cell.state);
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}

static int
cell_bars_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int v = cell.bars;

	return SYSCTL_OUT(req, &v, sizeof(v));
}

static int
cell_sms_status_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	return SYSCTL_OUT(req, cell.sms_status, strlen(cell.sms_status) + 1);
}

/* the messages received, as lines "N<TAB>NUMBER: TEXT" (a new line in a text is the byte 0x1f), the last eight at most */
static int
cell_sms_ring_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[8 * 260 + 1];
	size_t n = 0;
	uint32_t count = cell.sms_received;

	text[0] = 0;
	for (uint32_t i = count > 8 ? count - 8 : 0; i < count; i++) {
		const char *m = cell.sms_ring[i % 8];

		n += (size_t)snprintf(text + n, sizeof(text) - n, "%u\t", i);
		for (; *m != 0 && n + 2 < sizeof(text); m++) {
			text[n++] = *m == '\n' || *m == '\r' ? 0x1f : *m;
		}
		text[n++] = '\n';
		text[n] = 0;
	}
	return SYSCTL_OUT(req, text, n + 1);
}

static int
cell_sms_last_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[300];

	snprintf(text, sizeof(text), "%u %s", cell.sms_received, cell.sms_last);
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}

/* writing "33612345678 text of the message" (digits of an international number, no plus, a space, then the text) queues one SMS */
/* kern.hakios.modem_qmi: write "SERVICE MSGID PORT TLVBYTES" (service decimal, message id and bytes in hex, PORT 4 or 6: which of our two QMI clients), read the answer once the modem's thread has made the call */
static int
cell_qmi_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[sizeof(cell.qmi_request)];
	int error;

	if (req->newptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, cell.qmi_answer, strlen(cell.qmi_answer) + 1);
	}
	if (req->newlen == 0 || req->newlen >= sizeof(text)) {
		return EINVAL;
	}
	bzero(text, sizeof(text));
	if ((error = SYSCTL_IN(req, text, req->newlen)) != 0) {
		return error;
	}
	if (cell.qmi_pending) {
		return EBUSY;
	}
	strlcpy(cell.qmi_request, text, sizeof(cell.qmi_request));
	snprintf(cell.qmi_answer, sizeof(cell.qmi_answer), "queued");
	cell.qmi_pending = true;
	printf("modem: qmi: a hand-made request is queued: %s\n", cell.qmi_request);
	return 0;
}

static int
cell_sms_send_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[sizeof(cell.sms_request)];
	int error;

	if (req->newptr == USER_ADDR_NULL) {
		return SYSCTL_OUT(req, "", 1);
	}
	if (req->newlen == 0 || req->newlen >= sizeof(text)) {
		return EINVAL;
	}
	bzero(text, sizeof(text));
	if ((error = SYSCTL_IN(req, text, req->newlen)) != 0) {
		return error;
	}
	if (cell.sms_pending) {
		return EBUSY;
	}
	strlcpy(cell.sms_request, text, sizeof(cell.sms_request));
	cell.sms_pending = true;
	snprintf(cell.sms_status, sizeof(cell.sms_status), "queued");
	printf("modem: sms: a request of %zu bytes from user space is queued for the modem's thread\n", strlen(cell.sms_request));
	return 0;
}

/* the names the status bar hook (userland src/inject_sb.c) reads, as on the redwood: up, registered, bars, dbm (negative while there is a reading), the network's name */
static int
cell_int_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, int which, struct sysctl_req *req)
{
	int v = which == 0 ? (cell_started ? 1 : 0) : which == 1 ? cell.registered : which == 2 ? cell.bars : which == 4 ? (data.up ? 1 : 0) | (cl6.up ? 2 : 0) : cell.dbm;

	return SYSCTL_OUT(req, &v, sizeof(v));
}

static int
cell_operator_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	char text[sizeof(cell.operator_name)];

	strlcpy(text, cell.operator_name, sizeof(text));
	return SYSCTL_OUT(req, text, strlen(text) + 1);
}

SYSCTL_DECL(_kern_hakios);
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_up, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_int_sysctl, "I", "1 once the modem is brought up");
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_registered, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 1, cell_int_sysctl, "I", "1 while registered on a network");
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_bars, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 2, cell_int_sysctl, "I", "signal bars 0..4");
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_data, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 4, cell_int_sysctl, "I", "mobile data: 1 the IPv4 session is up, 2 the IPv6 one");
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_dbm, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 3, cell_int_sysctl, "I", "signal strength in dBm");
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_operator, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_operator_sysctl, "A", "the network's name");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_state, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_state_sysctl, "A", "the cellular modem: off, starting, registered ...");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_bars, CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_bars_sysctl, "I", "signal bars 0..4");
/* kern.hakios.gps_report: { count, length, the TLVs } of the modem's latest position report */
static int
gps_report_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	return SYSCTL_OUT(req, &gps_report, sizeof(gps_report));
}
static int
gps_sats_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	return SYSCTL_OUT(req, &gps_sats, sizeof(gps_sats));
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, gps_sats, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, 0, 0, gps_sats_sysctl, "S", "the modem's latest list of satellites");
SYSCTL_PROC(_kern_hakios, OID_AUTO, gps_report, CTLTYPE_OPAQUE | CTLFLAG_RD | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, 0, 0, gps_report_sysctl, "S", "the modem's latest position report");

/* kern.hakios.modem_restart: write 1: a test of the recovery: the mission ends as if the link had gone, and the modem is started again from scratch (modem_thread()) */
static int
cell_restart_sysctl(__unused struct sysctl_oid *oid, __unused void *arg1, __unused int arg2, struct sysctl_req *req)
{
	int value = 0, error = sysctl_handle_int(oid, &value, 0, req);

	if (error == 0 && req->newptr != USER_ADDR_NULL && value == 1) {
		say("modem: restart asked for by hand: the mission ends as if the link had gone\n");
		mission_link_down = true;
		cell_started = false;
	}
	return error;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_restart, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_LOCKED, 0, 0, cell_restart_sysctl, "I", "write 1: start the modem again from scratch, a test");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_qmi, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, 0, 0, cell_qmi_sysctl, "A", "a QMI request by hand: SERVICE MSGID PORT HEXBYTES; read for the answer");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_sms_send, CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, 0, 0, cell_sms_send_sysctl, "A", "write NUMBER TEXT to send an SMS");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_sms_status, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_sms_status_sysctl, "A", "the last SMS request: queued, sent, failed");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_sms_ring, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_sms_ring_sysctl, "A", "the messages received: N, a tab, NUMBER: TEXT, one a line");
SYSCTL_PROC(_kern_hakios, OID_AUTO, modem_sms_last, CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_LOCKED, 0, 0, cell_sms_last_sysctl, "A", "the count and the latest message received");

static const char gsm7_basic[128] =
    "@\xa3$\xa5\xe8\xe9\xf9\xec\xf2\xc7\n\xd8\xf8\r\xc5\xe5" "\x94_\x96\x93\x99\x95\x96\x9a\x92\x9b\x9c\x9d\x9e\x9f\x9f\x9f" " !\"#\xa4%&'()*+,-./0123456789:;<=>?"
    "\xa1" "ABCDEFGHIJKLMNOPQRSTUVWXYZ\xc4\xd6\xd1\xdc\xa7\xbf" "abcdefghijklmnopqrstuvwxyz\xe4\xf6\xf1\xfc\xe0";

/* an SMS-DELIVER (3GPP TS 23.040) as text: the sender and the message (GSM 7-bit default alphabet, the common accented letters as UTF-8, or UCS-2 with non-ASCII shown as '?') */
static void
sms_decode(const uint8_t *pdu, uint32_t n)
{
	uint32_t at = 0;
	char from[24] = "", text[200];
	uint32_t oa_len, digits;
	uint8_t first, dcs;
	uint32_t udl, out = 0;

	if (n < 12) {
		return;
	}
	at += 1 + pdu[0];                       /* the SMSC */
	if (at + 3 >= n) {
		return;
	}
	first = pdu[at++];
	if ((first & 3) != 0) {
		snprintf(cell.sms_last, sizeof(cell.sms_last), "(not a deliver message, type %u)", first & 3);
		strlcpy(cell.sms_ring[cell.sms_received % 8], cell.sms_last, sizeof(cell.sms_ring[0]));
		cell.sms_received++;
		return;
	}
	oa_len = pdu[at++];
	digits = (oa_len + 1) / 2;
	{
		uint8_t type = pdu[at++];
		uint32_t f = 0;

		if ((type & 0x70) == 0x50) {            /* alphanumeric sender */
			uint32_t septets = oa_len * 4 / 7, acc = 0, bits = 0;

			for (uint32_t i = 0; i < digits && f < sizeof(from) - 1; i++) {
				acc |= (uint32_t)pdu[at + i] << bits;
				bits += 8;
				while (bits >= 7 && f < septets && f < sizeof(from) - 1) {
					from[f++] = (char)(acc & 0x7f) < 0x20 ? '?' : (char)(acc & 0x7f);
					acc >>= 7;
					bits -= 7;
				}
			}
		} else {
			if ((type & 0x70) == 0x10) {
				from[f++] = '+';
			}
			for (uint32_t i = 0; i < oa_len && f < sizeof(from) - 1; i++) {
				uint8_t d = (pdu[at + i / 2] >> ((i & 1) * 4)) & 0xf;

				from[f++] = d < 10 ? (char)('0' + d) : '?';
			}
		}
		from[f] = 0;
	}
	at += digits;
	at++;                                   /* PID */
	dcs = pdu[at++];
	at += 7;                                /* the timestamp */
	if (at >= n) {
		return;
	}
	udl = pdu[at++];
	if ((dcs & 0x0c) == 0x08) {             /* UCS-2 */
		/* UCS-2 (UTF-16 in practice: an emoji is a surrogate pair) as UTF-8 */
		for (uint32_t i = 0; i + 1 < udl && at + i + 1 < n && out + 5 < sizeof(text); i += 2) {
			uint32_t c = (uint32_t)(pdu[at + i] << 8 | pdu[at + i + 1]);

			if (c >= 0xd800 && c < 0xdc00 && i + 3 < udl && at + i + 3 < n) {
				uint32_t low = (uint32_t)(pdu[at + i + 2] << 8 | pdu[at + i + 3]);

				if (low >= 0xdc00 && low < 0xe000) {
					c = 0x10000 + ((c - 0xd800) << 10) + (low - 0xdc00);
					i += 2;
				}
			}
			if (c < 0x20 && c != 10) {
				text[out++] = '?';
			} else if (c < 0x80) {
				text[out++] = (char)c;
			} else if (c < 0x800) {
				text[out++] = (char)(0xc0 | c >> 6);
				text[out++] = (char)(0x80 | (c & 0x3f));
			} else if (c < 0x10000) {
				text[out++] = (char)(0xe0 | c >> 12);
				text[out++] = (char)(0x80 | (c >> 6 & 0x3f));
				text[out++] = (char)(0x80 | (c & 0x3f));
			} else {
				text[out++] = (char)(0xf0 | c >> 18);
				text[out++] = (char)(0x80 | (c >> 12 & 0x3f));
				text[out++] = (char)(0x80 | (c >> 6 & 0x3f));
				text[out++] = (char)(0x80 | (c & 0x3f));
			}
		}
	} else {
		uint32_t acc = 0, bits = 0, got = 0, i = 0;

		while (got < udl && at + i < n && out < sizeof(text) - 1) {
			if (bits < 7) {
				acc |= (uint32_t)pdu[at + i++] << bits;
				bits += 8;
			}
			{
				uint8_t c = (uint8_t)(acc & 0x7f);

				/* the accented letters of the default alphabet as UTF-8 (è é ù ì ò Ç É ä ö ñ ü à), the rest of the odd ones (the escape table too) as '?' */
				static const struct { uint8_t gsm, second; } accents[] = { { 0x04, 0xa8 }, { 0x05, 0xa9 }, { 0x06, 0xb9 }, { 0x07, 0xac }, { 0x08, 0xb2 }, { 0x09, 0x87 }, { 0x1f, 0x89 },
					{ 0x7b, 0xa4 }, { 0x7c, 0xb6 }, { 0x7d, 0xb1 }, { 0x7e, 0xbc }, { 0x7f, 0xa0 } };
				bool accent = false;

				for (unsigned a = 0; a < sizeof(accents) / sizeof(accents[0]) && out + 3 < sizeof(text); a++) {
					if (accents[a].gsm == c) {
						text[out++] = (char)0xc3;
						text[out++] = (char)accents[a].second;
						accent = true;
					}
				}
				if (!accent && c == 0x11) {
					text[out++] = '_';              /* (the second row of the alphabet: only the underscore is shown) */
					accent = true;
				}
				if (!accent) {
					text[out++] = (c < 0x20 && c != 10 && c != 13) || c >= 0x7f ? '?' : (char)gsm7_basic[c] < 0x7f && (char)gsm7_basic[c] >= 0x20 ? gsm7_basic[c] : (c == 10 ? '\n' : '?');
				}
				acc >>= 7;
				bits -= 7;
				got++;
			}
		}
	}
	text[out] = 0;
	snprintf(cell.sms_last, sizeof(cell.sms_last), "%s: %s", from, text);
	strlcpy(cell.sms_ring[cell.sms_received % 8], cell.sms_last, sizeof(cell.sms_ring[0]));
	cell.sms_received++;
	say("modem: sms: RECEIVED from %s (%u characters)\n", from, out);
}

/* the signal strength (NAS 0x20: TLV 1 = signed dBm and the radio interface) as bars */
static void
cell_poll(void)
{
	const uint8_t *c;
	uint32_t n;
	uint8_t registration = 0, radio = 0;

	if (qmi_call(3, 0x0020, NULL, 0, 2000) == 0 && qmi_reply.result == 0 && (c = qmi_tlv(0x01, &n)) != NULL && n >= 2) {
		int dbm = (int8_t)c[0];

		cell.dbm = dbm;
		cell.bars = dbm >= -70 ? 4 : dbm >= -85 ? 3 : dbm >= -100 ? 2 : dbm >= -110 ? 1 : 0;
	}
	if (nas_registration(&registration, &radio) == 0) {
		char was[sizeof(cell.state)];

		strlcpy(was, cell.state, sizeof(was));
		cell.registered = registration == 1;
		snprintf(cell.state, sizeof(cell.state), "%s radio %u, %d dBm, %d bars, %u message(s) received", registration == 1 ? "registered" : "not registered", radio, cell.dbm, cell.bars, cell.sms_received);
		if (strcmp(was, cell.state) != 0) {
			say("modem: cell: %s\n", cell.state);
		}
	}
}

static void
cell_tick(void)
{
	uint64_t now = mhi_op_nanoseconds(NULL);

	cl6_tick(now);

	if (!cell.event_report_on) {
		static const uint8_t report[4] = { 0x10, 0x01, 0x00, 0x01 };           /* WMS Set Event Report: TLV 0x10, report new messages */

		if (qmi_call(5, 0x0001, report, sizeof(report), 3000) == 0 && qmi_reply.result == 0) {
			cell.event_report_on = true;
			say("modem: sms: new messages are reported\n");
		}
	}
	if (cell.qmi_pending) {
		char *p = cell.qmi_request, *end;
		uint32_t service = (uint32_t)strtoul(p, &end, 10), message, port, n = 0;
		uint8_t tlv[300];
		int e;

		p = end;
		message = (uint32_t)strtoul(p, &end, 16);
		p = end;
		port = (uint32_t)strtoul(p, &end, 10);
		p = end;
		for (int high = -1; *p != 0 && n < sizeof(tlv); p++) {
			int v = *p >= '0' && *p <= '9' ? *p - '0' : *p >= 'a' && *p <= 'f' ? *p - 'a' + 10 : *p >= 'A' && *p <= 'F' ? *p - 'A' + 10 : -1;

			if (v < 0) {
				continue;
			}
			if (high < 0) {
				high = v;
			} else {
				tlv[n++] = (uint8_t)(high << 4 | v);
				high = -1;
			}
		}
		qmi_src_port = port == 6 ? QMI_PORT6 : QMI_PORT;
		say("modem: qmi: hand-made request: service %u, message %#x, client port %#x, %u TLV bytes\n", service, message, qmi_src_port, n);
		e = qmi_call(service, (uint16_t)message, tlv, n, 45000);
		qmi_src_port = QMI_PORT;
		if (e != 0) {
			snprintf(cell.qmi_answer, sizeof(cell.qmi_answer), "no answer (%d)", e);
		} else {
			uint32_t at = 0;

			snprintf(cell.qmi_answer, sizeof(cell.qmi_answer), "result %u error %u;", qmi_reply.result, qmi_reply.error);
			at = (uint32_t)strlen(cell.qmi_answer);
			for (uint32_t t = 0; t + 3 <= qmi_reply.length && at + 40 < sizeof(cell.qmi_answer);) {
				uint32_t len = qmi_reply.data[t + 1] | qmi_reply.data[t + 2] << 8;

				if (t + 3 + len > qmi_reply.length) {
					break;
				}
				at += (uint32_t)snprintf(cell.qmi_answer + at, sizeof(cell.qmi_answer) - at, " [%02x/%u:", qmi_reply.data[t], len);
				for (uint32_t i = 0; i < len && i < 160 && at + 4 < sizeof(cell.qmi_answer); i++) {
					at += (uint32_t)snprintf(cell.qmi_answer + at, sizeof(cell.qmi_answer) - at, "%02x", qmi_reply.data[t + 3 + i]);
				}
				at += (uint32_t)snprintf(cell.qmi_answer + at, sizeof(cell.qmi_answer) - at, "]");
				t += 3 + len;
			}
		}
		say("modem: qmi: hand-made request answered: %s\n", cell.qmi_answer);
		cell.qmi_pending = false;
	}
	if (cell.sms_pending) {
		char to[24], *text = cell.sms_request;
		size_t k = 0;

		say("modem: sms: the modem's thread takes the request (%zu bytes)\n", strlen(cell.sms_request));
		while (*text != 0 && *text != ' ' && k < sizeof(to) - 1) {
			to[k++] = *text++;
		}
		to[k] = 0;
		if (*text == ' ') {
			text++;
		}
		if (k > 0 && *text != 0) {
			int r = wms_send_text(to, text);

			uint32_t idn = 0;
			const uint8_t *idp = r == 0 ? qmi_tlv(0x01, &idn) : NULL;

			snprintf(cell.sms_status, sizeof(cell.sms_status), r == 0 ? "sent (the modem accepted it, id %u)" : "failed (%d)", r == 0 ? (unsigned)(idp != NULL && idn >= 2 ? idp[0] | idp[1] << 8 : 0) : (unsigned)r);
		} else {
			snprintf(cell.sms_status, sizeof(cell.sms_status), "failed (write: NUMBER TEXT)");
		}
		cell.sms_pending = false;
	}
	if (cell_route_pending) {
		uint8_t t[16], ack[6];
		uint32_t k = 0;

		if (cell_route_format == 6) {
			/* a transferred message has no SMSC field in front (it starts at the first octet): one empty one is put there for the decoder */
			uint8_t pdu[sizeof(cell_route_pdu) + 1];

			pdu[0] = 0;
			memcpy(pdu + 1, cell_route_pdu, cell_route_len);
			sms_decode(pdu, cell_route_len + 1);
		} else {
			say("modem: sms: a message of format %u (not 3GPP) is not decoded\n", cell_route_format);
		}
		if (cell_route_ack) {
			/* WMS Send Ack: the transaction id, protocol 1 (3GPP), success */
			ack[0] = (uint8_t)cell_route_tid; ack[1] = (uint8_t)(cell_route_tid >> 8); ack[2] = (uint8_t)(cell_route_tid >> 16); ack[3] = (uint8_t)(cell_route_tid >> 24);
			ack[4] = 1;
			ack[5] = 1;
			tlv_put(t, &k, 0x01, ack, 6);
			say("modem: sms: acknowledgement: %s (result %u error %u)\n", qmi_call(5, 0x0037, t, k, 3000) == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
		}
		cell_route_pending = false;
	}
	if (cell_mt_pending) {
		uint8_t t[8];
		uint32_t k = 0, m;
		const uint8_t *c;
		uint8_t req[5] = { cell_mt_storage, (uint8_t)cell_mt_index, (uint8_t)(cell_mt_index >> 8), (uint8_t)(cell_mt_index >> 16), (uint8_t)(cell_mt_index >> 24) };

		cell_mt_pending = false;
		tlv_put(t, &k, 0x01, req, 5);
		if (qmi_call(5, 0x0022, t, k, 3000) == 0 && qmi_reply.result == 0 && (c = qmi_tlv(0x01, &m)) != NULL && m > 4) {
			uint32_t len = c[1] | c[2] << 8;

			if (len + 3 <= m) {
				sms_decode(c + 3, len);
			}
		}
	}
	if (now >= cell.next_poll) {
		cell_poll();
		cell.next_poll = now + 5000000000ull;
	}
}

/* The mission: the channels for it started, and QRTR listened to for `seconds`. */
static void
mission_run(uint32_t seconds)
{
	uint64_t until;

	if (hakios_mhi_start_mission(&mhi) != 0) {
		say("modem: mission: the channels did not start\n");
		return;
	}
	say("modem: mission: IPCR (channels 32/33) and QMI0 (20/21) started\n");
	if (sfr_buffer == NULL) {
		uint64_t sfr_iova;

		sfr_buffer = dma_alloc(SFR_LEN, &sfr_iova);
		if (sfr_buffer != NULL) {
			say("modem: mission: failure reason buffer (%#llx) handed to the modem: %d\n", sfr_iova, hakios_mhi_sfr_config(&mhi, sfr_iova, SFR_LEN));
		}
	}
	/* the device kept in M0: the host's answer to its M1 (idle) is M2 unless somebody holds the wake doorbell, and the modem's reset came 2.5 s after the channels, as an idle M1 would */
	say("modem: mission: wake vote: %d (state %#x)\n", hakios_mhi_wake_get(&mhi), mhi.state);
	{
		static const char ping[] = "HackiOS loopback test";
		int e = hakios_mhi_send(&mhi, 0, ping, sizeof(ping));

		say("modem: mission: loopback ping on channel 0: %d\n", e);
		for (unsigned ms = 0; ms < 300 && loopback_echoes == 0; ms++) {
			hakios_mhi_interrupt(&mhi, -1);
			while (sahara_in.tail != sahara_in.head) {
				uint32_t at = sahara_in.tail % SAHARA_QUEUE;

				__asm__ volatile ("dmb ish" ::: "memory");
				mission_dispatch(sahara_in.channel[at], sahara_in.data[at], sahara_in.length[at]);
				sahara_in.tail++;
			}
			sleep_us(1000);
		}
		say("modem: mission: loopback: %s\n", loopback_echoes != 0 ? "THE MODEM ECHOED IT" : "no echo in 1.5 s");
	}
	if (option_chans >> 1 & 1) {
		uint32_t masks = 1;

		/* modem_diag=0: the DIAG channel stays open but the modem is not asked for its messages (with all masks set it streams them for ever: MHI traffic and parsing on both sides) */
		PE_parse_boot_argn("modem_diag", &masks, sizeof(masks));
		if (masks != 0) {
			diag_enable();
		} else {
			say("modem: diag: messages not asked for (modem_diag=0)\n");
		}
	}
	say("modem: mission: QRTR is listened to for %u s\n", seconds);
	if (option_ipa) {
		/* the IPA service of the modem (0x31, instance 2) announced: the handshake, within the 2.5 s */
		for (unsigned waited = 0; waited < 1200; waited += 10) {
			bool there = false;

			for (unsigned i = 0; i < qrtr.servers; i++) {
				there = there || (qrtr.server[i].service == 0x31 && qrtr.server[i].instance == 0x201);
			}
			if (there) {
				break;
			}
			mission_pump(10, NULL);
		}
		ipa_init_driver();
		mission_pump(2000, NULL);
	}
	/* as soon as the DMS service has been announced (the modem may not live long: redwood's does not, once online): its operating mode, and low power so that it stays up; then the survey */
	for (unsigned waited = 0; waited < 1200; waited += 20) {
		uint32_t node, port;

		mission_pump(20, NULL);
		if (qmi_find(2, &node, &port) && qmi_find(11, &node, &port)) {
			break;
		}
	}
	say("modem: mission: %u services announced so far\n", qrtr.servers);
	if (!mission_phase_two) {
		/* the first link: the modem's own PCIe re-initialisation follows by itself (2.58 s after the channels start): waited for */
		for (uint64_t until = mhi_op_nanoseconds(NULL) + 15000000000ull; !mission_link_down && mhi_op_nanoseconds(NULL) < until;) {
			mission_pump(20, NULL);
		}
		return;
	}
	{
		static const uint8_t lpm[4] = { 0x01, 0x01, 0x00, 0x01 };           /* TLV 1: operating mode 1 (low power) */
		static const struct { unsigned ul; bool v2; } tries[2] = { { 20, true }, { 20, false } };
		int e = ETIMEDOUT;

		for (unsigned t = 0; t < 2 && e != 0; t++) {
			qrtr_force.on = true;
			qrtr_force.ul = tries[t].ul;
			qrtr_force.v2 = tries[t].v2;
			e = qmi_call(2, 0x002d, NULL, 0, 600);
			say("modem: qmi: DMS operating mode over channel %u with version %u headers: %s\n", tries[t].ul, tries[t].v2 ? 2 : 1, e == 0 ? "ANSWERED" : "no reply");
		}
		qrtr_force.on = false;
		if (e == 0) {
			qmi_hex("DMS operating mode", qmi_reply.data, qmi_reply.length);
		}
		e = qmi_call(2, 0x002e, lpm, sizeof(lpm), 3000);
		say("modem: qmi: DMS set operating mode low power: %s (result %u error %u)\n", e == 0 ? "answered" : "NO REPLY", qmi_reply.result, qmi_reply.error);
	}
	if (option_bringup) {
		modem_bringup();
	} else {
		qmi_survey();
	}
	unsigned idle = 0;
	bool busy;

	for (until = mhi_op_nanoseconds(NULL) + (uint64_t)seconds * 1000000000ull; mhi_op_nanoseconds(NULL) < until;) {
		static uint64_t hello_at;

		/* until a service has been announced: a lookup of every service by every route this could be (channel 20 or 32, header version 1 or 2), every 3 s */
		if (qrtr.servers == 0 && mhi_op_nanoseconds(NULL) >= hello_at) {
			static unsigned round;
			static const struct { unsigned channel; bool v2; } routes[4] = { { 20, true }, { 20, false }, { 20, true }, { 20, false } };
			uint32_t lookup[5] = { QRTR_NEW_LOOKUP, 0, 0, 0, 0 }, hello[5] = { QRTR_HELLO, 0, 0, 0, 0 };
			unsigned saved_channel = qrtr_channel;
			bool saved_v2 = qrtr_v2;
			int e1, e2;

			qrtr_channel = routes[round % 4].channel;
			qrtr_v2 = routes[round % 4].v2;
			e1 = qrtr_send(QRTR_HELLO, QRTR_CONTROL, qrtr.node != 0 ? qrtr.node : 0xffffffffu, QRTR_CONTROL, hello, 20);
			e2 = qrtr_send(QRTR_NEW_LOOKUP, 0x4000, qrtr.node != 0 ? qrtr.node : 2, QRTR_CONTROL, lookup, 20);   /* (from a client port, as Linux's lookups are) */
			say("modem: qrtr: HELLO and a lookup of everything on channel %u with a version %u header: %d %d\n", qrtr_channel, qrtr_v2 ? 2 : 1, e1, e2);
			qrtr_channel = saved_channel;
			qrtr_v2 = saved_v2;
			round++;
			hello_at = mhi_op_nanoseconds(NULL) + 3000000000ull;
		}
		if (rd(pcie.bar) == 0xffffffffu || mission_link_down) {
			say("modem: mission: the link is down\n");
			break;
		}
		hakios_mhi_interrupt(&mhi, -1);
		busy = sahara_in.tail != sahara_in.head || data_in.tail != data_in.head;
		while (sahara_in.tail != sahara_in.head) {
			uint32_t at = sahara_in.tail % SAHARA_QUEUE;

			__asm__ volatile ("dmb ish" ::: "memory");
			mission_dispatch(sahara_in.channel[at], sahara_in.data[at], sahara_in.length[at]);
			sahara_in.tail++;
		}
		data_poll();
		if (option_service && cell_started) {
			cell_tick();
		}
		/*
		 * How long until the next look: 500 us while the modem or the data path has something (the pickup of what the modem sends waits on this loop; what goes
		 * to it does not), 2 ms after 100 ms of nothing and 10 ms after two seconds. At 500 us for ever this was 2000 wake-ups a second with the phone idle and
		 * its screen off, a large part of kernel_task's half a core (2026-11-02).
		 */
		if (option_bridge && qrtr_bridge_drain()) {
			busy = true;
		}
		if (option_sat && sat_poll()) {
			busy = true;
		}
		idle = busy ? 0 : idle + 1;
		sleep_us(idle < 200 ? 500 : idle < 1000 ? 2000 : 10000);
	}
	for (unsigned c = 0; c < 0; c++) {
		static const unsigned numbers[6] = { 2, 3, 32, 33, 20, 21 };
		const uint8_t *ctx = mhi.channel_contexts + (size_t)numbers[c] * 44;
		uint64_t rp, wp;

		memcpy(&rp, ctx + 28, 8);
		memcpy(&wp, ctx + 36, 8);
		say("modem: mission: channel %u: cfg %#x type %u erindex %u, context rp %#llx wp %#llx (ring at %#llx, host rp %u wp %u)\n", numbers[c], *(const uint32_t *)ctx,
		    *(const uint32_t *)(ctx + 4), *(const uint32_t *)(ctx + 8), rp, wp, (unsigned long long)mhi.channels[c].ring.address, mhi.channels[c].ring.rp, mhi.channels[c].ring.wp);
	}
	for (unsigned e = 0; e < 3; e++) {
		const uint8_t *ctx = mhi.event_contexts + (size_t)e * 44;
		uint64_t rp, wp;

		memcpy(&rp, ctx + 28, 8);
		memcpy(&wp, ctx + 36, 8);
		say("modem: mission: event ring %u: context rp %#llx wp %#llx (ring at %#llx, host rp %u wp %u)\n", e, rp, wp, (unsigned long long)mhi.events[e].address, mhi.events[e].rp,
		    mhi.events[e].wp);
	}
	say("modem: mission: events serviced %llu\n", (unsigned long long)mhi.serviced);
	say("modem: mission: over: %u datagrams in, %u out, %u hello(s), %u service(s) announced, MHISTATUS %#x, EE %u\n", qrtr.packets_in, qrtr.packets_out, qrtr.hellos, qrtr.servers,
	    rd(pcie.bar + MHI_STATUS), rd(pcie.bar + pcie.bhi + BHI_EXECENV));
	smmu_faults();
}

/* The modem's SBL started by MHI and served by Sahara, until it has the images, goes away, or `seconds` pass. */
static void
sahara_run(uint32_t seconds)
{
	uint32_t cluster, size;
	uint8_t *image;
	struct hakios_mhi_config cfg;
	uint64_t until;

	if (!fat_open("image/sdx55m/sbl1.mbn", &cluster, &size)) {
		return;
	}
	image = (uint8_t *)kalloc_data(size, Z_WAITOK | Z_ZERO);
	if (image == NULL || fat_read(cluster, size, 0, image, size) != 0) {
		say("modem: sahara: sbl1.mbn did not get into memory\n");
		return;
	}
	if (!smmu_on()) {
		return;
	}
	msi_enable();
	memset(&cfg, 0, sizeof(cfg));
	memset(&mhi, 0, sizeof(mhi));
	cfg.max_channels = 300;
	cfg.timeout_ms = 3000;
	cfg.buffer_length = 0x2000;       /* (Sahara's reads go out in pieces of what fits; QRTR's datagrams are up to 8 KB) */
	/*
	 * The device tree's channels (qcom,mhi@0: mhi_chan@N; the node names are DECIMAL and are the channel numbers: an earlier version of this took them for hex and started 16/17, 20/21, 32/33...,
	 * which are other channels), by name: UL number, event ring of the UL, event ring of the DL, elements. modem_chans bit i selects row i.
	 */
	cfg.channel_count = 0;
	cfg.channels[cfg.channel_count++] = (struct hakios_mhi_channel_config){ .number = 2, .elements = 32, .event_ring = 1, .inbound = false, .ee_mask = 1u << 1 };
	cfg.channels[cfg.channel_count++] = (struct hakios_mhi_channel_config){ .number = 3, .elements = 32, .event_ring = 1, .inbound = true, .ee_mask = 1u << 1 };
	{
		static const struct { uint32_t ul, ul_ring, dl_ring, elements, burst; } pairs[] = {
			{ 0, 2, 2, 16, 0 },     /* 0  LOOPBACK */
			{ 4, 1, 3, 16, 0 },     /* 1  DIAG */
			{ 8, 1, 1, 16, 0 },     /* 2  QDSS */
			{ 10, 1, 1, 16, 0 },    /* 3  EFS */
			{ 14, 1, 2, 16, 0 },    /* 4  QMI0 */
			{ 16, 3, 3, 16, 0 },    /* 5  QMI1 */
			{ 18, 1, 1, 16, 0 },    /* 6  IP_CTRL */
			{ 20, 2, 2, 64, 0 },    /* 7  IPCR (QRTR) */
			{ 22, 2, 2, 16, 0 },    /* 8  TF */
			{ 26, 3, 3, 16, 0 },    /* 9  DCI */
			{ 32, 3, 3, 16, 0 },    /* 10 DUN */
			{ 100, 7, 8, 64, 3 },   /* 11 IP_HW0 (the hardware channels: event rings 7 and 8, doorbell mode 3) */
		};

		for (unsigned i = 0; i < sizeof(pairs) / sizeof(pairs[0]) && cfg.channel_count + 2 <= HAKIOS_MHI_CHANNELS; i++) {
			if (!(option_chans >> i & 1)) {
				continue;
			}
			cfg.channels[cfg.channel_count++] = (struct hakios_mhi_channel_config){ .number = pairs[i].ul, .elements = pairs[i].elements, .event_ring = pairs[i].ul_ring, .inbound = false, .ee_mask = 1u << 2, .burst = pairs[i].burst };
			cfg.channels[cfg.channel_count++] = (struct hakios_mhi_channel_config){ .number = pairs[i].ul + 1, .elements = pairs[i].elements, .event_ring = pairs[i].dl_ring, .inbound = true, .ee_mask = 1u << 2, .burst = pairs[i].burst };
		}
	}
	/* the 17 event rings of the device tree: elements, MSI vector (mhi,msi), moderation (mhi,intmod), whether it is another client's (mhi,offload) */
	{
		static const struct { uint32_t elements, vector, intmod; bool offload; } rings[17] = {
			{ 32, 1, 0, false }, { 256, 2, 0, false }, { 256, 3, 0, false }, { 256, 4, 0, false }, { 512, 0, 5, true }, { 512, 0, 5, true }, { 64, 0, 0, false },
			{ 1024, 5, 5, false }, { 2048, 6, 5, false }, { 0, 0, 0, true }, { 1024, 7, 5, false }, { 0, 0, 0, true }, { 0, 0, 0, true }, { 0, 0, 0, true }, { 0, 0, 0, true },
			{ 1024, 8, 1, false }, { 1024, 9, 0, false },
		};

		cfg.event_count = 17;
		cfg.hw_event_rings = 10;                        /* rings 7..16 are "mhi,hw-ev" */
		for (unsigned e = 0; e < 17; e++) {
			cfg.events[e] = (struct hakios_mhi_event_config){ .elements = rings[e].elements, .vector = rings[e].vector, .moderation_ms = rings[e].intmod, .control = e == 0, .offload = rings[e].offload };
		}
	}
	cfg.image = image;
	cfg.image_length = size;
	cfg.sbl_length = size;
	cfg.segment_length = size;
	cfg.iova_start = DMA_IOVA;
	cfg.iova_stop = DMA_IOVA + DMA_SIZE - 1;
	cfg.no_bhie = true;
	say("modem: step 4 (writes): MHI up through the boot ROM and SBL, and Sahara served on its channels 2 and 3 from %s\n", fw_partition);
	if (hakios_mhi_init(&mhi, &mhi_ops, NULL, &cfg) != 0 || hakios_mhi_power_up(&mhi) != 0) {
		say("modem: mhi: the modem did not come up through SBL (EE %u, state %#x, MHISTATUS %#x)\n", mhi.ee, mhi.state, rd(pcie.bar + 0x48));
		smmu_faults();
		return;
	}
	for (until = mhi_op_nanoseconds(NULL) + (uint64_t)seconds * 1000000000ull; mhi_op_nanoseconds(NULL) < until && !sahara.done && !sahara.finished;) {
		if (rd(pcie.bar) == 0xffffffffu) {
			say("modem: sahara: the link is down (the modem has restarted?)\n");
			break;
		}
		hakios_mhi_interrupt(&mhi, -1);
		while (sahara_in.tail != sahara_in.head) {
			uint32_t at = sahara_in.tail % SAHARA_QUEUE;

			__asm__ volatile ("dmb ish" ::: "memory");
			if (sahara_in.channel[at] == 3) {
				sahara_packet(sahara_in.data[at], sahara_in.length[at]);
			}
			sahara_in.tail++;
		}
		sleep_us(500);
	}
	if (sahara.finished) {
		/* the modem has its images: what it does now, for `watch` seconds: its environment and state, whether the link stays */
		uint32_t last_ee = ~0u, last_status = ~0u;
		bool link = true;

		for (until = mhi_op_nanoseconds(NULL) + (uint64_t)sahara_watch * 1000000000ull; mhi_op_nanoseconds(NULL) < until;) {
			uint32_t first = rd(pcie.bar);

			if (first == 0xffffffffu) {
				if (link) {
					say("modem: after: THE LINK WENT DOWN (BAR 0 reads all ones); MDM2AP status %u, error %u\n",
					    instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u, instantnoodle_gpio_level(PIN_MDM2AP_ERRFATAL) ? 1u : 0u);
				}
				link = false;
			} else {
				uint32_t ee = rd(pcie.bar + pcie.bhi + BHI_EXECENV), status = rd(pcie.bar + MHI_STATUS);

				if (!link || ee != last_ee || status != last_status) {
					say("modem: after: EE %u, MHISTATUS %#x, ctrl %#x, MDM2AP status %u, error %u\n", ee, status, rd(pcie.bar + 0x38),
					    instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u, instantnoodle_gpio_level(PIN_MDM2AP_ERRFATAL) ? 1u : 0u);
				}
				link = true;
				last_ee = ee;
				last_status = status;
				hakios_mhi_interrupt(&mhi, -1);
			}
			sleep_us(20000);
		}
	}
	if (sahara.finished && last_step >= 5 && option_quiet >= 3) {
		/* the plain route: the right channels (the device tree's own numbers) started on the first link, no link cycle; HELLOs answered, services looked up, the QMI survey */
		mission_phase_two = true;
		mission_link_down = false;
		mission_run(sahara_qrtr_seconds);
	} else if (sahara.finished && last_step >= 5 && option_quiet != 0) {
		/* the bisection: channels started on the first link, no cycle, no queries; what the modem does is watched */
		mission_phase_two = false;
		hakios_mhi_start_mission(&mhi);
		say("modem: mission: quiet mode %u: the channels are started, the modem is watched for 12 s\n", option_quiet);
		mission_pump(12000, NULL);
	} else if (sahara.finished && last_step >= 5) {
		/*
		 * The modem says it has booted (MDM2AP status high, EE AMSS): Android resets the link at once ("msm_pcie_disable" / "msm_pcie_enable", PERST about 15 ms) and brings MHI
		 * up afresh with the modem already in its mission. Without that the modem resets itself 2.58 s after the mission's channels are started (seen on every run).
		 */
		/*
		 * Android (drivers/bus/mhi/controllers/mhi_qcom.c, mhi_status_cb, MHI_CB_EE_MISSION_MODE): "we need to force a suspend so device can switch to mission mode pcie phy settings":
		 * MHI to M3, the endpoint to D3hot, the link suspended (PERST, PHY, clocks) and resumed at once, MHI back to M0 -- all as soon as the device reports its mission, before any channel of the
		 * mission is started. Without it the modem resets itself 2.57 s later.
		 */
		bool up = false;
		int e;

		for (unsigned ms = 0; ms < 10000 && !mhi.mission; ms++) {
			hakios_mhi_interrupt(&mhi, -1);
			sleep_us(1000);
		}
		say("modem: mission: the device reports its mission: %s (EE %u, MDM2AP status %u)\n", mhi.mission ? "yes" : "NO", rd(pcie.bar + pcie.bhi + BHI_EXECENV),
		    instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u);
		e = hakios_mhi_suspend(&mhi);
		say("modem: mission: MHI to M3: %d (state %#x, MHISTATUS %#x)\n", e, mhi.state, rd(pcie.bar + MHI_STATUS));
		{
			/* the endpoint to D3hot: its power management capability's control register */
			uint32_t cap = rd(pcie.conf + 0x34) & 0xff;

			for (unsigned guard = 0; cap != 0 && guard < 48; guard++) {
				uint32_t head = rd(pcie.conf + (cap & ~3u));
				unsigned id = head >> (8 * (cap & 3)) & 0xff, next = head >> (8 * (cap & 3) + 8) & 0xff;

				if (id == 0x01) {
					set_bits(pcie.conf + cap + 4, 3, 3);
					say("modem: mission: endpoint to D3hot (PM control %#x)\n", rd(pcie.conf + cap + 4));
					break;
				}
				cap = next;
			}
		}
		{
			/* Android's msm_pcie_pm_suspend: PME_Turn_Off is sent (ELBI system control bit 4) and the controller's PM status waited for L23 ready (bit 5), up to 100 ms, before PERST */
			uint32_t sts = 0;

			for (unsigned round = 0; round < 15 && !(sts & (1u << 5)); round++) {
				wr(pcie.dbi + 0xf24, 1u << 4);
				for (unsigned ms = 0; ms < 100; ms++) {
					sts = rd(pcie.parf + 0x24);
					if (sts & (1u << 5)) {
						break;
					}
					sleep_us(1000);
				}
				if (!(sts & (1u << 5))) {
					say("modem: mission: PME_Turn_Off %u: PM status %#x, no L23 ready yet\n", round + 1, sts);
				}
			}
			say("modem: mission: PME_Turn_Off sent: PM status %#x (%s)\n", sts, (sts & (1u << 5)) ? "L23 READY" : "no L23 ready in 100 ms");
		}
		pcie.up = false;
		for (unsigned attempt = 0; attempt < 6 && !up; attempt++) {
			pcie_off();
			up = pcie_on();
			if (!up) {
				say("modem: mission: resume attempt %u: not trained (MDM2AP status %u)\n", attempt + 1, instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u);
				sleep_us(300000);
			}
		}
		say("modem: mission: link resumed: %s (MDM2AP status %u)\n", up ? "UP" : "NOT TRAINED", instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u);
		if (up) {
			msi_enable();
			say("modem: mission: after the resume: EE %u, MHICTRL %#x, MHISTATUS %#x\n", rd(pcie.bar + rd(pcie.bar + 0x28) + BHI_EXECENV), rd(pcie.bar + 0x38), rd(pcie.bar + MHI_STATUS));
			e = hakios_mhi_resume(&mhi);
			say("modem: mission: MHI back to M0: %d (state %#x, MHISTATUS %#x)\n", e, mhi.state, rd(pcie.bar + MHI_STATUS));
			if (e == 0) {
				uint64_t sfr_iova;

				sfr_buffer = dma_alloc(SFR_LEN, &sfr_iova);
				if (sfr_buffer != NULL) {
					e = hakios_mhi_sfr_config(&mhi, sfr_iova, SFR_LEN);
					say("modem: mission: failure reason buffer (%#llx) handed to the modem: %d\n", sfr_iova, e);
				}
				e = 0;
				mission_phase_two = true;
				mission_link_down = false;
				mission_run(sahara_qrtr_seconds);
			}
		}
	}
	if (sfr_buffer != NULL) {
		char text[161];
		uint32_t n = 0;

		dma_sync(sfr_buffer, SFR_LEN);
		for (uint32_t i = 0; i < SFR_LEN && n < 160; i++) {
			if (sfr_buffer[i] >= 0x20 && sfr_buffer[i] < 0x7f) {
				text[n++] = (char)sfr_buffer[i];
			} else if (sfr_buffer[i] == 0 && n > 0 && text[n - 1] != ' ') {
				text[n++] = ' ';
			}
		}
		text[n] = 0;
		say("modem: sfr: the modem's failure reason buffer reads: \"%s\"\n", n != 0 ? text : "(empty)");
	}
	say("modem: sahara: over: %u images served, %u bytes (and %u x 4 GB), finished %d, stopped on an unknown image %u; MHISTATUS %#x, EE %u\n", sahara.served_images,
	    sahara.bytes_lo, sahara.bytes_hi, sahara.finished, sahara.unknown, rd(pcie.bar + 0x48), rd(pcie.bar + pcie.bhi + BHI_EXECENV));
	smmu_faults();
}

/* ---- the sequence ---------------------------------------------------------------------------------- */

static uint32_t last_step, sahara_seconds = 60;
static bool keep;

static void
run_steps(void)
{
	state_read();
	if (last_step < 2) {
		return;
	}
	if (last_step >= 3) {
		say("modem: step 3/4 first reads the firmware: the storage is waited for\n");
	}
	lck_mtx_lock(&modem_pcie_lock);
	say("modem: step 2 (writes): PCIe 2 up, the modem's MHI registers read, then %s\n", keep ? "left up (modem_keep=1)" : "taken down again");
	if (!map_gcc() || !map_pcie()) {
		lck_mtx_unlock(&modem_pcie_lock);
		return;
	}
	/* (spartan: the firmware is on the root, which may not be mounted yet: waited for before the modem is told anything) */
	bool mounted = last_step < 3 || fat_mount();
	if (last_step >= 3 && !mounted) {
		lck_mtx_unlock(&modem_pcie_lock);
		return;
	}
	/* the application processor's two lines to the modem as Android's esoc driver sets them when it powers the modem on: AP2MDM status (the processor is up) high, error low */
	instantnoodle_gpio_output(PIN_AP2MDM_ERRFATAL, false, INSTANTNOODLE_PULL_NONE, 2);
	instantnoodle_gpio_output(PIN_AP2MDM_STATUS, true, INSTANTNOODLE_PULL_NONE, 2);
	if (pcie_on()) {
		say("modem: pcie: PCIe 2 is up with the modem behind it; MDM2AP status %u, error %u\n", instantnoodle_gpio_level(PIN_MDM2AP_STATUS) ? 1u : 0u,
		    instantnoodle_gpio_level(PIN_MDM2AP_ERRFATAL) ? 1u : 0u);
		mhi_read();
		if (last_step == 3 && pcie.bhi != 0) {
			say("modem: step 3 (writes): the modem's stream in the SMMU, sbl1.mbn from %s to the boot ROM through BHI\n", fw_partition);
			bhi_load("image/sdx55m/sbl1.mbn");
		} else if (last_step >= 4 && pcie.bhi != 0) {
			sahara_run(sahara_seconds);
		}
	} else {
		say("modem: pcie: PCIe 2 did not come up: no modem to talk to (see above)\n");
	}
	if (!keep) {
		smmu_off();
		pcie_off();
		instantnoodle_gpio_input(PIN_AP2MDM_STATUS, INSTANTNOODLE_PULL_DOWN);
		instantnoodle_gpio_input(PIN_AP2MDM_ERRFATAL, INSTANTNOODLE_PULL_NONE);
		say("modem: pcie: PCIe 2 is down, the modem held in reset\n");
	}
	lck_mtx_unlock(&modem_pcie_lock);
}

/*
 * The service survives the system's sleep by starting the modem again: the PCIe controller does not keep its link through it (the pump finds
 * the link down, ends, and puts the modem in reset as it always does), and a modem that is booted from scratch is back in about 30 s. What
 * the run left in the statics is cleared first.
 */
static void
modem_reset_state(void)
{
	bzero(&qrtr, sizeof(qrtr));
	bzero(qrtr_routes, sizeof(qrtr_routes));
	bzero(&qrtr_force, sizeof(qrtr_force));
	qrtr_channel = 32;
	qrtr_v2 = false;
	bzero(&sahara, sizeof(sahara));
	sahara_in.head = sahara_in.tail = 0;
	bzero(tftp_sessions, sizeof(tftp_sessions));
	efs.state = EFS_IDLE;
	diag_state.frame_length = 0;
	diag_state.escaped = false;
	diag_state.on = false;
	mission_link_down = false;
	mission_phase_two = true;
	cell_started = false;
	data_down();
	cell.event_report_on = false;
	cell.registered = 0;
	cell.bars = 0;
	cell.next_poll = 0;
	cell_mt_pending = false;
	cell_route_pending = false;
	qmi_reply.have = false;
}

static void
modem_thread(void *arg, wait_result_t wr_unused)
{
	(void)arg;
	(void)wr_unused;
	for (unsigned round = 0;; round++) {
		run_steps();
		if (!option_service || round >= 200) {
			break;
		}
		snprintf(cell.state, sizeof(cell.state), "restarting (the link went away, round %u)", round + 1);
		cell.registered = 0;
		cell_started = false;
		sat_modem_gone();
		delay_for_interval(5000000, NSEC_PER_USEC);      /* 5 s (and the system's sleep, if it comes now, gets through first) */
		modem_reset_state();
	}
	thread_terminate(current_thread());
}

/* Called while the system starts (drivers/hakios.input.c). */
void
hakios_spartan_modem_init(void)
{
	uint32_t n = 0, k = 0;
	thread_t thread;


	if (!PE_parse_boot_argn("modem", &n, sizeof(n)) || n == 0) {
		return;
	}
	if (!instantnoodle_board()) {
		printf("modem: modem=N is for the OnePlus 8 Pro's X55 modem, and this is not that board\n");
		return;
	}
	last_step = n > 5 ? 5 : n;
	PE_parse_boot_argn("modem_qrtr", &sahara_qrtr_seconds, sizeof(sahara_qrtr_seconds));
	PE_parse_boot_argn("modem_secs", &sahara_seconds, sizeof(sahara_seconds));
	PE_parse_boot_argn("modem_watch", &sahara_watch, sizeof(sahara_watch));
	{
		/* (the OnePlus 8's modem_fw=a chose a firmware slot; spartan's firmware is one tree on the root) */
		k = 0;
		if (PE_parse_boot_argn("modem_chans", &k, sizeof(k))) {
			option_chans = k;
		}
		k = 0;
		PE_parse_boot_argn("modem_bringup", &k, sizeof(k));
		option_bringup = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_resume", &k, sizeof(k));
		option_resume_all = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_blk", &k, sizeof(k));
		option_blk = k >= 8 && k <= 4096 ? k : option_blk;
		k = 0;
		PE_parse_boot_argn("modem_efs", &k, sizeof(k));
		option_efs = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_service", &k, sizeof(k));
		option_service = k == 1;
		PE_parse_boot_argn("modem_modes", &option_modes, sizeof(option_modes));
		k = 0;
		PE_parse_boot_argn("modem_sms", &k, sizeof(k));
		option_sms = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_sms_self", &k, sizeof(k));
		option_sms_self = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_tftp", &k, sizeof(k));
		{
			uint32_t bridge_on = 0;

			PE_parse_boot_argn("voice_bridge", &bridge_on, sizeof(bridge_on));
			option_bridge = bridge_on != 0;
			{
				uint32_t sat_on = 0;

				PE_parse_boot_argn("voice_sat", &sat_on, sizeof(sat_on));
				option_sat = sat_on != 0;
			}
		}
		option_tftp = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_data", &k, sizeof(k));
		option_data = k;
		if (PE_parse_boot_argn("modem_data_ep", &k, sizeof(k))) {
			option_data_ep = k;
		}
		k = 2;
		PE_parse_boot_argn("modem_data6", &k, sizeof(k));
		option_data6 = k;
		k = 1;
		PE_parse_boot_argn("modem_data_qmap", &k, sizeof(k));
		option_data_qmap = k != 0;
		PE_parse_boot_argn("modem_apn", option_apn, sizeof(option_apn));
		k = 1;
		PE_parse_boot_argn("modem_en1", &k, sizeof(k));
		option_en1 = k != 0;
		k = 0;
		PE_parse_boot_argn("modem_ipa", &k, sizeof(k));
		option_ipa = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_hw", &k, sizeof(k));
		option_hw = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_lanes", &k, sizeof(k));
		option_lanes = k;
		k = 0;
		PE_parse_boot_argn("modem_quiet", &k, sizeof(k));
		option_quiet = k;
		k = 0;
		PE_parse_boot_argn("modem_cycle", &k, sizeof(k));
		option_full_cycle = k == 1;
		k = 0;
		PE_parse_boot_argn("modem_nc", &k, sizeof(k));
		option_nc = k == 1;
	}
	{
		char ids[96];

		if (PE_parse_boot_argn("modem_ids", ids, sizeof(ids))) {
			for (char *p = ids; *p != 0 && sahara_mapped < SAHARA_MAP_MAX;) {
				char *eq = strchr(p, '='), *comma = strchr(p, ',');

				if (eq == NULL || (comma != NULL && comma < eq)) {
					break;
				}
				if (comma != NULL) {
					*comma = 0;
				}
				*eq = 0;
				sahara_files[sahara_mapped].id = (uint32_t)strtoul(p, NULL, 10);
				strlcpy(sahara_files[sahara_mapped].file, eq + 1, sizeof(sahara_files[0].file));
				for (unsigned i = 0; i < sahara_mapped; i++) {
					if (sahara_files[i].id == sahara_files[sahara_mapped].id) {
						sahara_files[i] = sahara_files[sahara_mapped];     /* (replaced: the table's row is overwritten and the new one dropped) */
						sahara_mapped--;
						break;
					}
				}
				sahara_mapped++;
				p = comma != NULL ? comma + 1 : p + strlen(p);
			}
		}
	}
	PE_parse_boot_argn("modem_keep", &k, sizeof(k));
	keep = k == 1;
	k = 0;
	PE_parse_boot_argn("modem_probe", &k, sizeof(k));
	option_probe = k == 1;
	if (kernel_thread_start(modem_thread, NULL, &thread) != KERN_SUCCESS) {
		printf("modem: no thread\n");
		return;
	}
	{
		extern void thread_set_thread_name(thread_t thread, const char *name);

		thread_set_thread_name(thread, "modem");	/* (to be told apart in a sample of kernel_task's threads) */
	}
	thread_deallocate(thread);
	printf("modem: steps up to %u, on a thread beside the system's start\n", last_step);
}
