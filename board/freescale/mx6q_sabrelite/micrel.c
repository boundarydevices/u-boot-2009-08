#include <config.h>
#include <common.h>
#include <miiphy.h>
#include <micrel.h>

/* Indicates what features are supported by the interface. */
#define SUPPORTED_10baseT_Half		(1 << 0)
#define SUPPORTED_10baseT_Full		(1 << 1)
#define SUPPORTED_100baseT_Half		(1 << 2)
#define SUPPORTED_100baseT_Full		(1 << 3)
#define SUPPORTED_1000baseT_Half	(1 << 4)
#define SUPPORTED_1000baseT_Full	(1 << 5)
#define SUPPORTED_Autoneg		(1 << 6)
#define SUPPORTED_TP			(1 << 7)
#define SUPPORTED_AUI			(1 << 8)
#define SUPPORTED_MII			(1 << 9)

#define PHY_BASIC_FEATURES	(SUPPORTED_10baseT_Half | \
				SUPPORTED_10baseT_Full | \
				SUPPORTED_100baseT_Half | \
				SUPPORTED_100baseT_Full | \
				SUPPORTED_Autoneg | \
				SUPPORTED_TP | \
				SUPPORTED_MII)

#define PHY_GBIT_FEATURES	(PHY_BASIC_FEATURES | \
				SUPPORTED_1000baseT_Half | \
				SUPPORTED_1000baseT_Full)

/* 1000BASE-T Control register */
#define ADVERTISE_1000FULL	0x0200  /* Advertise 1000BASE-T full duplex */
#define ADVERTISE_1000HALF	0x0100  /* Advertise 1000BASE-T half duplex */
#define MII_CTRL1000		0x09

/* ksz9021 PHY Registers */
#define MII_KSZ9021_EXTENDED_CTRL	0x0b
#define MII_KSZ9021_EXTENDED_DATAW	0x0c
#define MII_KSZ9021_EXTENDED_DATAR	0x0d
#define MII_KSZ9021_PHY_CTL		0x1f
#define MIIM_KSZ9021_PHYCTL_1000	(1 << 6)
#define MIIM_KSZ9021_PHYCTL_100		(1 << 5)
#define MIIM_KSZ9021_PHYCTL_10		(1 << 4)
#define MIIM_KSZ9021_PHYCTL_DUPLEX	(1 << 3)

#define CTRL1000_PREFER_MASTER		(1 << 10)
#define CTRL1000_CONFIG_MASTER		(1 << 11)
#define CTRL1000_MANUAL_CONFIG		(1 << 12)

static int phy_write(char *devname, unsigned char addr, unsigned char reg,
		     unsigned short value)
{
	int ret = miiphy_write(devname, addr, reg, value);
	if (ret)
		printf("Error writing to %s PHY addr=%02x reg=%02x\n", devname,
		       addr, reg);

	return ret;
}

static int phy_read(char *devname, unsigned char addr, unsigned char reg)
{
	unsigned short value = 0;
	int ret = miiphy_read(devname, addr, reg, &value);
	if (ret) {
		printf("Error reading from %s PHY addr=%02x reg=%02x\n", devname,
		       addr, reg);
		return ret;
	}
	return value;
}


int ksz9021_phy_extended_write(char *phydev, int phy_addr, int regnum, u16 val)
{
	/* extended registers */
	phy_write(phydev, phy_addr, MII_KSZ9021_EXTENDED_CTRL, regnum | 0x8000);
	return phy_write(phydev, phy_addr, MII_KSZ9021_EXTENDED_DATAW, val);
}

int ksz9021_phy_extended_read(char *phydev, int phy_addr, int regnum)
{
	/* extended registers */
	phy_write(phydev, phy_addr, MII_KSZ9021_EXTENDED_CTRL, regnum);
	return phy_read(phydev, phy_addr, MII_KSZ9021_EXTENDED_DATAR);
}

/* Micrel ksz9021 */
int ksz9021_config(char *phydev, int phy_addr)
{
	unsigned ctrl1000 = 0;
	const unsigned master = CTRL1000_PREFER_MASTER |
			CTRL1000_CONFIG_MASTER | CTRL1000_MANUAL_CONFIG;
	unsigned features = PHY_GBIT_FEATURES;

	if (getenv("disable_giga"))
		features &= ~(SUPPORTED_1000baseT_Half |
				SUPPORTED_1000baseT_Full);
	/* force master mode for 1000BaseT due to chip errata */
	if (features & SUPPORTED_1000baseT_Half)
		ctrl1000 |= ADVERTISE_1000HALF | master;
	if (features & SUPPORTED_1000baseT_Full)
		ctrl1000 |= ADVERTISE_1000FULL | master;
	phy_write(phydev, phy_addr, MII_CTRL1000, ctrl1000);
	return 0;
}
/* ************************************************************/

unsigned short ksz9031eng_por_cmds[] = {
	0, 0x2100,
	0xffff, 600,	/* wait 600 ms */
	/* 0x1c - maybe AFED */
	0x1c07, 0x0140,
	0x1c05, 0x0300,
	0x1c04, 0x00ff,
	0x1c1d, 0x67ff,
	0x1c1e, 0x6fff,
	0x1c22, 0x2604,
	0x1c23, 0x5173,
	/* 0 - unknown */
	0x800b, 0xc93e,
	/* 1 - unknown */
	0x0176, 0xe830,
	0x0104, 0xf4c9,
	0x0107, 0xf4c6,
	0x010a, 0xf435,
	0x010d, 0xf435,
	0x0110, 0xf435,
	0x0113, 0xf4c6,
	0x0116, 0xf445,
	0x0119, 0xf445,
	0x011c, 0x14e7,
	0x0126, 0x44c6,
	0x0127, 0x14c6,
	0x0128, 0x14c6,
	0x0129, 0x14c6,
	0x012a, 0x14c6,
	0x012b, 0x14c6,
	0x012c, 0x14c6,
	0x012d, 0x44c6,
	0x012e, 0x14c6,
	0x012f, 0x14c6,
	0x0130, 0x14c6,
	0x0131, 0x14c6,
	0x0132, 0x14c6,
	0x0133, 0x14c6,
	0x011f, 0x14e9,
	0x0124, 0x02e9,
	0x0153, 0x14c6,
	0x0156, 0x14c6,
	0x0159, 0x14c6,
	0x01a9, 0xa004,
	0x01c9, 0x00ff,
	0x01d1, 0x5555,
	/* 3 - EEE_WOL */
	0x0309, 0x005f,
	0x030c, 0x005b,
	0x030e, 0x002e,
	0x030f, 0x007a,
	0x0, 0x0
};

unsigned short ksz9031_por_cmds[] = {
	0x0205, 0x0,		/* RXDn pad skew */
	0x0206, 0x0,		/* TXDn pad skew */
	0x0208, 0x03ff,		/* TXC/RXC pad skew */
	0x0, 0x1140,
	0x0, 0x0
};

int dbg_phy_write(char *phydev, int phy_addr,
		unsigned char reg, unsigned short val)
{
	printf("w %x %04x\n", reg, val);
	return phy_write(phydev, phy_addr, reg, val);
}

int ksz9031_send_phy_cmds(char *phydev, int phy_addr, unsigned short* p)
{
	for (;;) {
		unsigned reg = *p++;
		unsigned val = *p++;
		if (reg == 0 && val == 0)
			break;
		if (reg < 32) {
			dbg_phy_write(phydev, phy_addr, reg, val);
		} else {
			unsigned dev_addr = (reg >> 8) & 0x7f;
			if (reg == 0xffff) {
				udelay(val * 1000);
				continue;
			}
			dbg_phy_write(phydev, phy_addr, 0x0d, dev_addr);
			dbg_phy_write(phydev, phy_addr, 0x0e, reg & 0xff);
			dbg_phy_write(phydev, phy_addr, 0x0d, dev_addr | 0x8000);
			dbg_phy_write(phydev, phy_addr, 0x0e, val);
		}
	}
	return 0;
}

/* Micrel ksz9031 */
int ksz9031_config(char *phydev, int phy_addr)
{
	unsigned ctrl1000 = 0;
	const unsigned master = CTRL1000_PREFER_MASTER |
			CTRL1000_CONFIG_MASTER | CTRL1000_MANUAL_CONFIG;
	unsigned features = PHY_GBIT_FEATURES;
	unsigned id2 = phy_read(phydev, phy_addr, PHY_PHYIDR2);

	if ((id2 & 0xf) == 0)
		ksz9031_send_phy_cmds(phydev, phy_addr, ksz9031eng_por_cmds);
	ksz9031_send_phy_cmds(phydev, phy_addr, ksz9031_por_cmds);
	if (getenv("disable_giga"))
		features &= ~(SUPPORTED_1000baseT_Half |
				SUPPORTED_1000baseT_Full);
	/* force master mode for 1000BaseT due to chip errata */
	if (features & SUPPORTED_1000baseT_Half)
		ctrl1000 |= ADVERTISE_1000HALF | master;
	if (features & SUPPORTED_1000baseT_Full)
		ctrl1000 |= ADVERTISE_1000FULL | master;
	phy_write(phydev, phy_addr, MII_CTRL1000, ctrl1000);
	return 0;
}
