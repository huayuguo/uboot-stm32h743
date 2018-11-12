// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2010
 * Vipin Kumar, ST Micoelectronics, vipin.kumar@st.com.
 */

/*
 * Designware ethernet IP driver for U-Boot
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <errno.h>
#include <miiphy.h>
#include <malloc.h>
#include <pci.h>
#include <reset.h>
#include <linux/compiler.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <power/regulator.h>
#include "designware.h"
#define CONFIG_PHY_ADDR 0
static int dw_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{

	debug("%s ..........reg=%x....\n", __func__,reg);
#ifdef CONFIG_DM_ETH
	struct dw_eth_dev *priv = dev_get_priv((struct udevice *)bus->priv);
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
#else
	struct eth_mac_regs *mac_p = bus->priv;
#endif
	ulong start;
	u16 miiaddr;
	int timeout = 35;
	u32 readdate =0;
	miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) |
		  ((reg << MIIREGSHIFT) & MII_REGMSK);
#ifdef CONFIG_STM32H7
	writel((0X40D  | (reg << 16)), 0x40028200);

#else
	writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &mac_p->miiaddr);
#endif
	debug("%s miiaddr= 0x%x \n", __func__,miiaddr | MII_CLKRANGE_150_250M | MII_BUSY);
	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (!(readl(0x40028200) & MII_BUSY)){
			readdate = readl(0x40028204);
			debug("%s readdate =0x%x \n", __func__,readdate);
			return readdate;
			}
		udelay(10);
	};

	return -ETIMEDOUT;
}

static int dw_mdio_write(struct mii_dev *bus, int addr, int devad, int reg,
			u16 val)
{

	debug("%s ..............\n", __func__);
#ifdef CONFIG_DM_ETH
	struct dw_eth_dev *priv = dev_get_priv((struct udevice *)bus->priv);
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
#else
	struct eth_mac_regs *mac_p = bus->priv;
#endif
	ulong start;
	u16 miiaddr;
	int ret = -ETIMEDOUT, timeout = CONFIG_MDIO_TIMEOUT;

	writel(val, 0x40028204);
	miiaddr = ((addr << MIIADDRSHIFT) & MII_ADDRMSK) |
		  ((reg << MIIREGSHIFT) & MII_REGMSK) | MII_WRITE;

#ifdef CONFIG_STM32H7
	writel((0X405  | (reg << 16)), 0x40028200);

#else
	writel(miiaddr | MII_CLKRANGE_150_250M | MII_BUSY, &mac_p->miiaddr);
#endif

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (!(readl(0x40028200) & MII_BUSY)) {
			ret = 0;
			break;
		}
		udelay(10);
	};

	return ret;
}

#if defined(CONFIG_DM_ETH) && defined(CONFIG_DM_GPIO)
static int dw_mdio_reset(struct mii_dev *bus)
{
	struct udevice *dev = bus->priv;
	struct dw_eth_dev *priv = dev_get_priv(dev);
	struct dw_eth_pdata *pdata = dev_get_platdata(dev);
	int ret;

	debug("%s ..............\n", __func__);
	if (!dm_gpio_is_valid(&priv->reset_gpio))
		return 0;

	/* reset the phy */
	ret = dm_gpio_set_value(&priv->reset_gpio, 0);
	if (ret)
		return ret;

	udelay(pdata->reset_delays[0]);

	ret = dm_gpio_set_value(&priv->reset_gpio, 1);
	if (ret)
		return ret;

	udelay(pdata->reset_delays[1]);

	ret = dm_gpio_set_value(&priv->reset_gpio, 0);
	if (ret)
		return ret;

	udelay(pdata->reset_delays[2]);

	return 0;
}
#endif

static int dw_mdio_init(const char *name, void *priv)
{
	struct mii_dev *bus = mdio_alloc();

	debug("%s ..............\n", __func__);
	if (!bus) {
		debug("Failed to allocate MDIO bus\n");
		return -ENOMEM;
	}

	bus->read = dw_mdio_read;
	bus->write = dw_mdio_write;
	snprintf(bus->name, sizeof(bus->name), "%s", name);
#if defined(CONFIG_DM_ETH) && defined(CONFIG_DM_GPIO)
	bus->reset = dw_mdio_reset;
#endif

	bus->priv = priv;

	return mdio_register(bus);
}

static void tx_descs_init(struct dw_eth_dev *priv)
{
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	struct dmamacdescr *desc_table_p = &priv->tx_mac_descrtable[0];
	char *txbuffs = &priv->txbuffs[0];
	struct dmamacdescr *desc_p;
	u32 idx;

	debug("%s ..............\n", __func__);
	for (idx = 0; idx < CONFIG_TX_DESCR_NUM; idx++) {
		desc_p = &desc_table_p[idx];
		desc_p->dmamac_addr = (ulong)&txbuffs[idx * CONFIG_ETH_BUFSIZE];
		desc_p->dmamac_next = (ulong)&desc_table_p[idx + 1];

#if defined(CONFIG_DW_ALTDESCRIPTOR)
		desc_p->txrx_status &= ~(DESC_TXSTS_TXINT | DESC_TXSTS_TXLAST |
				DESC_TXSTS_TXFIRST | DESC_TXSTS_TXCRCDIS |
				DESC_TXSTS_TXCHECKINSCTRL |
				DESC_TXSTS_TXRINGEND | DESC_TXSTS_TXPADDIS);

		desc_p->txrx_status |= DESC_TXSTS_TXCHAIN;
		desc_p->dmamac_cntl = 0;
		desc_p->txrx_status &= ~(DESC_TXSTS_MSK | DESC_TXSTS_OWNBYDMA);
#else
		desc_p->dmamac_cntl = DESC_TXCTRL_TXCHAIN;
		desc_p->txrx_status = 0;
#endif
	}

	/* Correcting the last pointer of the chain */
	desc_p->dmamac_next = (ulong)&desc_table_p[0];

	/* Flush all Tx buffer descriptors at once */
	flush_dcache_range((ulong)priv->tx_mac_descrtable,
			   (ulong)priv->tx_mac_descrtable +
			   sizeof(priv->tx_mac_descrtable));

	writel((ulong)&desc_table_p[0],0X40029114);
	priv->tx_currdescnum = 0;
}

static void rx_descs_init(struct dw_eth_dev *priv)
{
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	struct dmamacdescr *desc_table_p = &priv->rx_mac_descrtable[0];
	char *rxbuffs = &priv->rxbuffs[0];
	struct dmamacdescr *desc_p;
	u32 idx;

	debug("%s ..............\n", __func__);
	/* Before passing buffers to GMAC we need to make sure zeros
	 * written there right after "priv" structure allocation were
	 * flushed into RAM.
	 * Otherwise there's a chance to get some of them flushed in RAM when
	 * GMAC is already pushing data to RAM via DMA. This way incoming from
	 * GMAC data will be corrupted. */
	flush_dcache_range((ulong)rxbuffs, (ulong)rxbuffs + RX_TOTAL_BUFSIZE);

	for (idx = 0; idx < CONFIG_RX_DESCR_NUM; idx++) {
		desc_p = &desc_table_p[idx];
		desc_p->dmamac_addr = (ulong)&rxbuffs[idx * CONFIG_ETH_BUFSIZE];
		desc_p->dmamac_next = (ulong)&desc_table_p[idx + 1];

		desc_p->dmamac_cntl =
			(MAC_MAX_FRAME_SZ & DESC_RXCTRL_SIZE1MASK) |
				      DESC_RXCTRL_RXCHAIN;

		desc_p->txrx_status = DESC_RXSTS_OWNBYDMA;
	}

	/* Correcting the last pointer of the chain */
	desc_p->dmamac_next = (ulong)&desc_table_p[0];

	/* Flush all Rx buffer descriptors at once */
	flush_dcache_range((ulong)priv->rx_mac_descrtable,
			   (ulong)priv->rx_mac_descrtable +
			   sizeof(priv->rx_mac_descrtable));

	writel((ulong)&desc_table_p[0], 0X4002911c);
	priv->rx_currdescnum = 0;
}

static int _dw_write_hwaddr(struct dw_eth_dev *priv, u8 *mac_id)
{
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	u32 macid_lo, macid_hi;

	debug("%s ..............\n", __func__);
	macid_lo = mac_id[0] + (mac_id[1] << 8) + (mac_id[2] << 16) +
		   (mac_id[3] << 24);
	macid_hi = mac_id[4] + (mac_id[5] << 8);

	writel(macid_hi, &mac_p->macaddr0hi);
	writel(macid_lo, &mac_p->macaddr0lo);

	return 0;
}

static int dw_adjust_link(struct dw_eth_dev *priv, struct eth_mac_regs *mac_p,
			  struct phy_device *phydev)
{

	debug("%s ..............\n", __func__);
	u32 conf = readl(&mac_p->conf) | 1 << 10;

	if (!phydev->link) {
		printf("%s: No link.\n", phydev->dev->name);
		return 0;
	}

	if (phydev->speed != 1000)
		conf |= MII_PORTSELECT;
	else
		conf &= ~MII_PORTSELECT;

	if (phydev->speed == 100)
		conf |= FES_100;

	if (phydev->duplex)
		conf |= FULLDPLXMODE;

	writel(conf, &mac_p->conf);

	printf("Speed: %d, %s duplex%s\n", phydev->speed,
	       (phydev->duplex) ? "full" : "half",
	       (phydev->port == PORT_FIBRE) ? ", fiber mode" : "");

	return 0;
}

static void _dw_eth_halt(struct dw_eth_dev *priv)
{
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;

	debug("%s ..............\n", __func__);
	writel(readl(&mac_p->conf) & ~(RXENABLE | TXENABLE), &mac_p->conf);
	writel(readl(&dma_p->opmode) & ~(RXSTART | TXSTART), &dma_p->opmode);

	phy_shutdown(priv->phydev);
}

int designware_eth_init(struct dw_eth_dev *priv, u8 *enetaddr)
{
	struct eth_mac_regs *mac_p = priv->mac_regs_p;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	unsigned int start;
	int ret;

	debug("%s ..............\n", __func__);
	writel(readl(&dma_p->busmode) | DMAMAC_SRST, &dma_p->busmode);

	/*
	 * When a MII PHY is used, we must set the PS bit for the DMA
	 * reset to succeed.
	 */
#ifdef CONFIG_STM32H7


#else
	if (priv->phydev->interface == PHY_INTERFACE_MODE_MII)
		writel(readl(&mac_p->conf) | MII_PORTSELECT, &mac_p->conf);
	else
		writel(readl(&mac_p->conf) & ~MII_PORTSELECT, &mac_p->conf);
#endif

	start = get_timer(0);
	while (readl(&dma_p->busmode) & DMAMAC_SRST) {
		if (get_timer(start) >= CONFIG_MACRESET_TIMEOUT) {
			printf("DMA reset timeout\n");
			return -ETIMEDOUT;
		}

		mdelay(100);
	};

	/*
	 * Soft reset above clears HW address registers.
	 * So we have to set it here once again.
	 */
	_dw_write_hwaddr(priv, enetaddr);

	rx_descs_init(priv);
	tx_descs_init(priv);

#ifdef CONFIG_STM32H7
	writel(3 << 12,0X40029000); // 011: The priority ratio is 4:1  ETH_DMAMR.PR
	writel(0X1010000 | 1  ,0X40029004); //FB: Fixed Burst Length ETH_DMASBMR.FB
	writel(1 << 16 | 4 << 18,0X40029100);//DMA_PBL ETH_DMACCR.PBLX8
	writel(8 << 16,0X40029104);//Transmit Programmable Burst Length ETH_DMACTXCR.TXPBL[5:0]

#else
	writel(FIXEDBURST | PRIORXTX_41 | DMA_PBL, &dma_p->busmode);
#endif


#ifdef CONFIG_STM32H7

	writel(readl(0X40028d00) | 1 << 0 | 1 << 1,0X40028d00);// 刷新发送 FIFO?????
	writel(readl(0X40029104) | 8 << 16 ,0X40029104);
	writel(readl(0X40029108) | 1 << 0 | 1 << 31,0X40029108);

#else
#ifndef CONFIG_DW_MAC_FORCE_THRESHOLD_MODE
	writel(readl(&dma_p->opmode) | FLUSHTXFIFO | STOREFORWARD,
	       &dma_p->opmode);
#else
	writel(readl(&dma_p->opmode) | FLUSHTXFIFO,
	       &dma_p->opmode);
#endif

	writel(readl(&dma_p->opmode) | RXSTART | TXSTART, &dma_p->opmode);

#ifdef CONFIG_DW_AXI_BURST_LEN
	writel((CONFIG_DW_AXI_BURST_LEN & 0x1FF >> 1), &dma_p->axibus);
#endif

#endif
	/* Start up the PHY */
	ret = phy_startup(priv->phydev);
	if (ret) {
		printf("Could not initialize PHY %s\n",
		       priv->phydev->dev->name);
		return ret;
	}

	ret = dw_adjust_link(priv, mac_p, priv->phydev);
	if (ret)
		return ret;

	return 0;
}

int designware_eth_enable(struct dw_eth_dev *priv)
{
	struct eth_mac_regs *mac_p = priv->mac_regs_p;

	debug("%s ..............\n", __func__);
	if (!priv->phydev->link)
		return -EIO;

	writel(readl(&mac_p->conf) | RXENABLE | TXENABLE, &mac_p->conf);

	return 0;
}

#define ETH_ZLEN	60

static int _dw_eth_send(struct dw_eth_dev *priv, void *packet, int length)
{
	int i;
	struct eth_dma_regs *dma_p = priv->dma_regs_p;
	u32 desc_num = priv->tx_currdescnum;
	struct dmamacdescr *desc_p = &priv->tx_mac_descrtable[desc_num];
	ulong desc_start = (ulong)desc_p;
	ulong desc_end = desc_start +
		roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);
	ulong data_start = desc_p->dmamac_addr;
	ulong data_end = data_start + roundup(length, ARCH_DMA_MINALIGN);
	/*
	 * Strictly we only need to invalidate the "txrx_status" field
	 * for the following check, but on some platforms we cannot
	 * invalidate only 4 bytes, so we flush the entire descriptor,
	 * which is 16 bytes in total. This is safe because the
	 * individual descriptors in the array are each aligned to
	 * ARCH_DMA_MINALIGN and padded appropriately.
	 */

	debug("%s ..............\n", __func__);
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	debug("%s DMA Debug status register = %x\n", __func__,readl(0X4002900c));
	debug("%s MAC Debug status register = %x\n", __func__,readl(0X40028114));
	debug("%s DMA Tx descriptor register = %x\n", __func__,readl(0X40029114));
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s MDA Debug status register = %x\n", __func__,readl(0X4002900c));
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	debug("%s MAC Debug status register = %x\n", __func__,readl(0X40028114));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s DMA Tx descriptor register = %x\n", __func__,readl(0X40029114));
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));	
	debug("length =%x date= ",length);
	for(i=0;i<length;i+=4){
		debug("%x,",readl(packet+i));
		if((i/16)) debug("\n");
	}
	debug("\n date end \n");
	invalidate_dcache_range(desc_start, desc_end);

	/* Check if the descriptor is owned by CPU */
	if (desc_p->txrx_status & DESC_TXSTS_OWNBYDMA) {
		printf("CPU not owner of tx frame\n");
		return -EPERM;
	}

	length = max(length, ETH_ZLEN);

	memcpy((void *)data_start, packet, length);

	/* Flush data to be sent */
	flush_dcache_range(data_start, data_end);

#if defined(CONFIG_DW_ALTDESCRIPTOR)
	desc_p->txrx_status |= DESC_TXSTS_TXFIRST | DESC_TXSTS_TXLAST;
	desc_p->dmamac_cntl |= (length << DESC_TXCTRL_SIZE1SHFT) &
			       DESC_TXCTRL_SIZE1MASK;

	desc_p->txrx_status &= ~(DESC_TXSTS_MSK);
	desc_p->txrx_status |= DESC_TXSTS_OWNBYDMA;
#else
	desc_p->dmamac_cntl |= ((length << DESC_TXCTRL_SIZE1SHFT) &
			       DESC_TXCTRL_SIZE1MASK) | DESC_TXCTRL_TXLAST |
			       DESC_TXCTRL_TXFIRST;

	desc_p->txrx_status = DESC_TXSTS_OWNBYDMA;
#endif

	/* Flush modified buffer descriptor */
	flush_dcache_range(desc_start, desc_end);

	/* Test the wrap-around condition. */
	if (++desc_num >= CONFIG_TX_DESCR_NUM)
		desc_num = 0;

	priv->tx_currdescnum = desc_num;

	/* Start the transmission */  
#ifdef CONFIG_STM32H7
	writel( 1,0X4002912c);
	writel(readl(0X40029104) | 1 << 0,0X40029104);
#else
	writel(POLL_DATA, &dma_p->txpolldemand);
#endif
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	debug("%s DMA Debug status register = %x\n", __func__,readl(0X4002900c));
	debug("%s MAC Debug status register = %x\n", __func__,readl(0X40028114));
	debug("%s DMA Tx descriptor register = %x\n", __func__,readl(0X40029114));
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s MDA Debug status register = %x\n", __func__,readl(0X4002900c));
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	debug("%s MAC Debug status register = %x\n", __func__,readl(0X40028114));
	debug("%s Channel status register = %x\n", __func__,readl(0X40029160));
	debug("%s DMA Tx descriptor register = %x\n", __func__,readl(0X40029114));
	debug("%s DMA Tx being descriptor register = %x\n", __func__,readl(0X40029144));
	return 0;
}

static int _dw_eth_recv(struct dw_eth_dev *priv, uchar **packetp)
{
	u32 status, desc_num = priv->rx_currdescnum;
	struct dmamacdescr *desc_p = &priv->rx_mac_descrtable[desc_num];
	int length = -EAGAIN;
	ulong desc_start = (ulong)desc_p;
	ulong desc_end = desc_start +
		roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);
	ulong data_start = desc_p->dmamac_addr;
	ulong data_end;
	debug("%s ..............\n", __func__);
	/* Invalidate entire buffer descriptor */
	invalidate_dcache_range(desc_start, desc_end);

	status = desc_p->txrx_status;

	/* Check  if the owner is the CPU */
	if (!(status & DESC_RXSTS_OWNBYDMA)) {

		length = (status & DESC_RXSTS_FRMLENMSK) >>
			 DESC_RXSTS_FRMLENSHFT;

		/* Invalidate received data */
		data_end = data_start + roundup(length, ARCH_DMA_MINALIGN);
		invalidate_dcache_range(data_start, data_end);
		*packetp = (uchar *)(ulong)desc_p->dmamac_addr;
	}

	return length;
}

static int _dw_free_pkt(struct dw_eth_dev *priv)
{
			debug("%s ..............\n", __func__);
	u32 desc_num = priv->rx_currdescnum;
	struct dmamacdescr *desc_p = &priv->rx_mac_descrtable[desc_num];
	ulong desc_start = (ulong)desc_p;
	ulong desc_end = desc_start +
		roundup(sizeof(*desc_p), ARCH_DMA_MINALIGN);

	/*
	 * Make the current descriptor valid again and go to
	 * the next one
	 */
	desc_p->txrx_status |= DESC_RXSTS_OWNBYDMA;

	/* Flush only status field - others weren't changed */
	flush_dcache_range(desc_start, desc_end);

	/* Test the wrap-around condition. */
	if (++desc_num >= CONFIG_RX_DESCR_NUM)
		desc_num = 0;
	priv->rx_currdescnum = desc_num;

	return 0;
}

static int dw_phy_init(struct dw_eth_dev *priv, void *dev)
{
	struct phy_device *phydev;
	int mask = 0xffffffff, ret;
	debug("%s ..............\n", __func__);
#ifdef CONFIG_PHY_ADDR
	mask = 1 << CONFIG_PHY_ADDR;
#endif
	debug("%s ..............mask = %d \n", __func__,mask);
	phydev = phy_find_by_mask(priv->bus, mask, priv->interface);
	if (!phydev)
		return -ENODEV;

	phy_connect_dev(phydev, dev);

	phydev->supported &= PHY_GBIT_FEATURES;
	if (priv->max_speed) {
		ret = phy_set_supported(phydev, priv->max_speed);
		if (ret)
			return ret;
	}
	phydev->advertising = phydev->supported;

	priv->phydev = phydev;
	phy_config(phydev);

	return 0;
}

#ifndef CONFIG_DM_ETH
static int dw_eth_init(struct eth_device *dev, bd_t *bis)
{
	int ret;
	debug("%s ..............\n", __func__);

	ret = designware_eth_init(dev->priv, dev->enetaddr);
	if (!ret)
		ret = designware_eth_enable(dev->priv);

	return ret;
}

static int dw_eth_send(struct eth_device *dev, void *packet, int length)
{
		debug("%s ..............\n", __func__);
	return _dw_eth_send(dev->priv, packet, length);
}

static int dw_eth_recv(struct eth_device *dev)
{
	uchar *packet;
	int length;
	debug("%s ..............\n", __func__);
	length = _dw_eth_recv(dev->priv, &packet);
	if (length == -EAGAIN)
		return 0;
	net_process_received_packet(packet, length);

	_dw_free_pkt(dev->priv);

	return 0;
}

static void dw_eth_halt(struct eth_device *dev)
{
	return _dw_eth_halt(dev->priv);
}

static int dw_write_hwaddr(struct eth_device *dev)
{
		debug("%s ..............\n", __func__);
	return _dw_write_hwaddr(dev->priv, dev->enetaddr);
}

int designware_initialize(ulong base_addr, u32 interface)
{
	struct eth_device *dev;
	struct dw_eth_dev *priv;
	debug("%s ..............\n", __func__);
	dev = (struct eth_device *) malloc(sizeof(struct eth_device));
	if (!dev)
		return -ENOMEM;

	/*
	 * Since the priv structure contains the descriptors which need a strict
	 * buswidth alignment, memalign is used to allocate memory
	 */
	priv = (struct dw_eth_dev *) memalign(ARCH_DMA_MINALIGN,
					      sizeof(struct dw_eth_dev));
	if (!priv) {
		free(dev);
		return -ENOMEM;
	}

	if ((phys_addr_t)priv + sizeof(*priv) > (1ULL << 32)) {
		debug("designware: buffers are outside DMA memory\n");
		return -EINVAL;
	}

	memset(dev, 0, sizeof(struct eth_device));
	memset(priv, 0, sizeof(struct dw_eth_dev));

	sprintf(dev->name, "dwmac.%lx", base_addr);
	dev->iobase = (int)base_addr;
	dev->priv = priv;

	priv->dev = dev;
	priv->mac_regs_p = (struct eth_mac_regs *)base_addr;
	priv->dma_regs_p = (struct eth_dma_regs *)(base_addr +
			DW_DMA_BASE_OFFSET);

	dev->init = dw_eth_init;
	dev->send = dw_eth_send;
	dev->recv = dw_eth_recv;
	dev->halt = dw_eth_halt;
	dev->write_hwaddr = dw_write_hwaddr;

	eth_register(dev);

	priv->interface = interface;

	dw_mdio_init(dev->name, priv->mac_regs_p);
	priv->bus = miiphy_get_dev_by_name(dev->name);

	return dw_phy_init(priv, dev);
}
#endif

#ifdef CONFIG_DM_ETH
static int designware_eth_start(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);
	int ret;
	debug("%s ..............\n", __func__);
	ret = designware_eth_init(priv, pdata->enetaddr);
	if (ret)
		return ret;
	ret = designware_eth_enable(priv);
	if (ret)
		return ret;

	return 0;
}

int designware_eth_send(struct udevice *dev, void *packet, int length)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);
	debug("%s ..............\n", __func__);
	return _dw_eth_send(priv, packet, length);
}

int designware_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);
	debug("%s ..............\n", __func__);
	return _dw_eth_recv(priv, packetp);
}

int designware_eth_free_pkt(struct udevice *dev, uchar *packet, int length)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);
	debug("%s ..............\n", __func__);
	return _dw_free_pkt(priv);
}

void designware_eth_stop(struct udevice *dev)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);
	debug("%s ..............\n", __func__);
	return _dw_eth_halt(priv);
}

int designware_eth_write_hwaddr(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);
	debug("%s ..............\n", __func__);
	return _dw_write_hwaddr(priv, pdata->enetaddr);
}

static int designware_eth_bind(struct udevice *dev)
{
		debug("%s ..............\n", __func__);
#ifdef CONFIG_DM_PCI
	static int num_cards;
	char name[20];

	/* Create a unique device name for PCI type devices */
	if (device_is_on_pci_bus(dev)) {
		sprintf(name, "eth_designware#%u", num_cards++);
		device_set_name(dev, name);
	}
#endif

	return 0;
}

int designware_eth_probe(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_platdata(dev);
	struct dw_eth_dev *priv = dev_get_priv(dev);
	u32 iobase = pdata->iobase;
	ulong ioaddr;
	int ret;
	struct reset_ctl_bulk reset_bulk;
#ifdef CONFIG_CLK
	int i, err, clock_nb;

		debug("%s ..............\n", __func__);
	priv->clock_count = 0;
	clock_nb = dev_count_phandle_with_args(dev, "clocks", "#clock-cells");
	if (clock_nb > 0) {
		priv->clocks = devm_kcalloc(dev, clock_nb, sizeof(struct clk),
					    GFP_KERNEL);
		if (!priv->clocks)
			return -ENOMEM;

		for (i = 0; i < clock_nb; i++) {
			err = clk_get_by_index(dev, i, &priv->clocks[i]);
			if (err < 0)
				break;

			err = clk_enable(&priv->clocks[i]);
			if (err && err != -ENOSYS && err != -ENOTSUPP) {
				pr_err("failed to enable clock %d\n", i);
				clk_free(&priv->clocks[i]);
				goto clk_err;
			}
			priv->clock_count++;
		}
	} else if (clock_nb != -ENOENT) {
		pr_err("failed to get clock phandle(%d)\n", clock_nb);
		return clock_nb;
	}
#endif

#if defined(CONFIG_DM_REGULATOR)
	struct udevice *phy_supply;

	ret = device_get_supply_regulator(dev, "phy-supply",
					  &phy_supply);
	if (ret) {
		debug("%s: No phy supply\n", dev->name);
	} else {
		ret = regulator_set_enable(phy_supply, true);
		if (ret) {
			puts("Error enabling phy supply\n");
			return ret;
		}
	}
#endif

	ret = reset_get_bulk(dev, &reset_bulk);
	if (ret)
		dev_warn(dev, "Can't get reset: %d\n", ret);
	else{
		ret =  reset_deassert_bulk(&reset_bulk);
		debug("%s: reset_deassert_bulk ret =%d\n",__func__,ret);
	}
#ifdef CONFIG_DM_PCI
	/*
	 * If we are on PCI bus, either directly attached to a PCI root port,
	 * or via a PCI bridge, fill in platdata before we probe the hardware.
	 */
	if (device_is_on_pci_bus(dev)) {
		dm_pci_read_config32(dev, PCI_BASE_ADDRESS_0, &iobase);
		iobase &= PCI_BASE_ADDRESS_MEM_MASK;
		iobase = dm_pci_mem_to_phys(dev, iobase);

		pdata->iobase = iobase;
		pdata->phy_interface = PHY_INTERFACE_MODE_RMII;
	}
#endif

	debug("%s, iobase=%x, priv=%p\n", __func__, iobase, priv);
	pdata->phy_interface = PHY_INTERFACE_MODE_RMII;
	ioaddr = iobase;
	priv->mac_regs_p = (struct eth_mac_regs *)ioaddr;
	priv->dma_regs_p = (struct eth_dma_regs *)(ioaddr + DW_DMA_BASE_OFFSET);
	priv->interface = pdata->phy_interface;
	priv->max_speed = pdata->max_speed;

	dw_mdio_init(dev->name, dev);
	priv->bus = miiphy_get_dev_by_name(dev->name);

	ret = dw_phy_init(priv, dev);
	debug("%s, ret=%d\n", __func__, ret);

	return ret;

#ifdef CONFIG_CLK
clk_err:
	ret = clk_release_all(priv->clocks, priv->clock_count);
	if (ret)
		pr_err("failed to disable all clocks\n");

	return err;
#endif
}

static int designware_eth_remove(struct udevice *dev)
{
	struct dw_eth_dev *priv = dev_get_priv(dev);
		debug("%s ..............\n", __func__);
	free(priv->phydev);
	mdio_unregister(priv->bus);
	mdio_free(priv->bus);

#ifdef CONFIG_CLK
	return clk_release_all(priv->clocks, priv->clock_count);
#else
	return 0;
#endif
}

const struct eth_ops designware_eth_ops = {
	.start			= designware_eth_start,
	.send			= designware_eth_send,
	.recv			= designware_eth_recv,
	.free_pkt		= designware_eth_free_pkt,
	.stop			= designware_eth_stop,
	.write_hwaddr		= designware_eth_write_hwaddr,
};

int designware_eth_ofdata_to_platdata(struct udevice *dev)
{
	struct dw_eth_pdata *dw_pdata = dev_get_platdata(dev);
#ifdef CONFIG_DM_GPIO
	struct dw_eth_dev *priv = dev_get_priv(dev);
#endif
	struct eth_pdata *pdata = &dw_pdata->eth_pdata;
	const char *phy_mode;
#ifdef CONFIG_DM_GPIO
	int reset_flags = GPIOD_IS_OUT;
#endif
	int ret = 0;
	u32 reg = 0;

	// by xiaodong
	debug("%s ..............\n", __func__);
#ifdef CONFIG_STM32H7
	//writel(reg,0x58000404);
	reg = readl(0x58000404);
	debug("%s .....reg = %x.........\n", __func__,reg);
	setbits_le32(0x580244f4, BIT(1));
	reg = readl(0x58000404);
	debug("%s .....reg = %x.........\n", __func__,reg);
	setbits_le32(0x58000404, BIT(23));
	clrbits_le32(0x58000404, BIT(22));
	clrbits_le32(0x58000404, BIT(21));
	reg = readl(0x58000404);
	debug("%s .....reg = %x.........\n", __func__,reg);
	reg = readl(0x52004140);
	debug("%s .....sdcr1= %x.........\n", __func__,reg);
#endif
	pdata->iobase = dev_read_addr(dev);
	pdata->phy_interface = -1;
	phy_mode = dev_read_string(dev, "phy-mode");
	if (phy_mode)
		pdata->phy_interface = phy_get_interface_by_name(phy_mode);
	if (pdata->phy_interface == -1) {
		debug("%s: Invalid PHY interface '%s'\n", __func__, phy_mode);
		return -EINVAL;
	}

	pdata->max_speed = dev_read_u32_default(dev, "max-speed", 0);

#ifdef CONFIG_DM_GPIO
	if (dev_read_bool(dev, "snps,reset-active-low"))
		reset_flags |= GPIOD_ACTIVE_LOW;

	ret = gpio_request_by_name(dev, "snps,reset-gpio", 0,
		&priv->reset_gpio, reset_flags);
	if (ret == 0) {
		ret = dev_read_u32_array(dev, "snps,reset-delays-us",
					 dw_pdata->reset_delays, 3);
	} else if (ret == -ENOENT) {
		ret = 0;
	}
#endif

	return ret;
}

static const struct udevice_id designware_eth_ids[] = {
	{ .compatible = "allwinner,sun7i-a20-gmac" },
	{ .compatible = "altr,socfpga-stmmac" },
	{ .compatible = "amlogic,meson6-dwmac" },
	{ .compatible = "amlogic,meson-gx-dwmac" },
	{ .compatible = "st,stm32-dwmac" },
	{ }
};

U_BOOT_DRIVER(eth_designware) = {
	.name	= "eth_designware",
	.id	= UCLASS_ETH,
	.of_match = designware_eth_ids,
	.ofdata_to_platdata = designware_eth_ofdata_to_platdata,
	.bind	= designware_eth_bind,
	.probe	= designware_eth_probe,
	.remove	= designware_eth_remove,
	.ops	= &designware_eth_ops,
	.priv_auto_alloc_size = sizeof(struct dw_eth_dev),
	.platdata_auto_alloc_size = sizeof(struct dw_eth_pdata),
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};

static struct pci_device_id supported[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_QRK_EMAC) },
	{ }
};

U_BOOT_PCI_DEVICE(eth_designware, supported);
#endif