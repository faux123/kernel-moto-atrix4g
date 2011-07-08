/****************************************************************

Siano Mobile Silicon, Inc.
MDTV receiver kernel modules.
Copyright (C) 2006-2008, Uri Shkolnik

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/



#include <linux/kernel.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <asm/dma.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <mach/system.h>
#include <mach/pinmux.h>
#include <mach/dma.h>

#include "smsdbg_prn.h"
#include "smscoreapi.h"


#define TX_BUFFER_SIZE	0x200

// The GPIO pin define in file gpio-names.h under kernel/arch/arm/mach-tegra
#define MTV_SPI_INT	75	// TEGRA_GPIO_PJ3

/*
// for pinmux config using, moved from ../arch/arm/mach-tegra/board-mot-power.c
#define MTV_SPI_MISO	22
#define MTV_SPI_SCS	176
#define MTV_SPI_SCLK	203
#define MTV_SPI_MOSI	10
*/

int sms2186_irq_number = 0;
int sms2186_int_counter = 0;

static struct tegra_pingroup_config cmmb_spi[] = {
	{TEGRA_PINGROUP_LPW0, 
	TEGRA_MUX_SPI3,
	TEGRA_PUPD_NORMAL,
	TEGRA_TRI_NORMAL},	// MTV_SPI_MOSI

	{TEGRA_PINGROUP_LPW2, 
	TEGRA_MUX_SPI3,
	TEGRA_PUPD_NORMAL,
	TEGRA_TRI_NORMAL},	// MTV_SPI_MISO

	{TEGRA_PINGROUP_LM0, 
	TEGRA_MUX_SPI3,
	TEGRA_PUPD_NORMAL,
	//TEGRA_PUPD_PULL_DOWN,
	TEGRA_TRI_NORMAL},	// MTV_SPI_SCS

	{TEGRA_PINGROUP_LSC1, 
	TEGRA_MUX_SPI3,
	TEGRA_PUPD_NORMAL,
	TEGRA_TRI_NORMAL},	// MTV_SPI_SCLK
	
};


static struct spi_device * smsmdtv_dev;

/*! global bus data */
struct spiphy_dev_s {
	struct completion transfer_in_process;
	void (*interruptHandler) (void *);
	void *intr_context;
	struct spi_device* dev;	/*!< device model stuff */
	int irq;

	char *txpad;
	dma_addr_t txpad_phy_addr;

};

// SPI interrupt handler
static irqreturn_t spibus_interrupt(int irq, void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	sms2186_int_counter++;
	sms_info("***=== CPU received interrupt, No. %d ===***", sms2186_int_counter);
	if (spiphy_dev->interruptHandler)
		spiphy_dev->interruptHandler(spiphy_dev->intr_context);
	return IRQ_HANDLED;

}

void smsspibus_xfer(void *context, unsigned char *txbuf,
		    unsigned long txbuf_phy_addr, unsigned char *rxbuf,
		    unsigned long rxbuf_phy_addr, int len)
{
	struct spi_transfer	t; 
	struct spi_message	m;
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	int status = -1;

	unsigned long txdma;

	if( txbuf == 0 )
	{
		txbuf = (unsigned char *)spiphy_dev->txpad;
		txdma = spiphy_dev->txpad_phy_addr;
	}
	else
	{
		txdma = txbuf_phy_addr;
	}

	spi_message_init(&m);

	m.is_dma_mapped = 1;

	memset(&t, 0, sizeof(struct spi_transfer));

	t.tx_buf		= txbuf;
	t.tx_dma		= txdma;
	t.len			= len;
	t.rx_buf		= rxbuf;
	t.rx_dma		= rxbuf_phy_addr;
	t.delay_usecs		= 1;
	
	spi_message_add_tail(&t, &m);

	status = spi_sync(spiphy_dev->dev, &m);

	//sms_info("Xfer %d bytes.Tx buf %08x : Rx Buf %08x; Tx buf content:%x,%x,%x,%x",len,txbuf,rxbuf,txbuf[0],txbuf[1],txbuf[2],txbuf[3]);
	//if((u32)txbuf[0] != 0xff)
		//sms_info("Xfer %d bytes. txbuf[4] = 0x%x, txbuf[5] = 0x%x",len,txbuf[4],txbuf[5]);
}

void smschipreset(void *context)
{

}

	
static int __devinit smsmdtv_probe(struct spi_device *spi)
{
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	smsmdtv_dev = spi;

	return 0;
}

static struct spi_driver smsmdtv_driver = {
	.driver = 
	{
		.name = "smsmdtv",
		.bus  = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = smsmdtv_probe,
};

void *smsspiphy_init(void *context, void (*smsspi_interruptHandler) (void *),  void *intr_context)
{
	
	struct spiphy_dev_s *spiphy_dev;
	int ret;

	smsmdtv_dev = 0;

	spiphy_dev = kmalloc(sizeof(struct spiphy_dev_s), GFP_KERNEL);
	if(spiphy_dev == 0)
	{
		sms_err("malloc for spi_dev failed");
		goto err_malloc;
	}
	
	spiphy_dev->interruptHandler = smsspi_interruptHandler;		// smsspi_int_handler at smsspilog.c
	spiphy_dev->intr_context = intr_context;			// _spi_device_st

	ret = spi_register_driver(&smsmdtv_driver);
	if(ret < 0 || smsmdtv_dev == 0)
	{
		sms_info("Cann't get SPI device\n");
		goto err_register;
	}

	spiphy_dev->dev = smsmdtv_dev;

	(void)tegra_pinmux_config_table(cmmb_spi, 4);
	printk("$$$$$$$$$$$$ Pinmux config for CMMB spi when init SPI\n");

	ret = gpio_request(MTV_SPI_INT, NULL);
	if (ret < 0) {
		sms_err( "unable to get sms GPIO %d", ret);
		goto err_req;
	}
	gpio_direction_input(MTV_SPI_INT);

	spiphy_dev->irq = gpio_to_irq(MTV_SPI_INT);
	sms2186_irq_number = spiphy_dev->irq;
	printk("\tSPI_phy_dev->irq = %d\n", spiphy_dev->irq);
	set_irq_type(spiphy_dev->irq, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(spiphy_dev->irq, spibus_interrupt, IRQF_TRIGGER_FALLING|IRQF_DISABLED, "smsmdtv", spiphy_dev);
	disable_irq(sms2186_irq_number);

	if(ret < 0)
	{
		sms_err("Unable to request irq %d",ret);
		goto err_irq;
	}

	spiphy_dev->txpad = dma_alloc_coherent(NULL, TX_BUFFER_SIZE, &spiphy_dev->txpad_phy_addr, GFP_KERNEL | GFP_DMA);
	if (!spiphy_dev->txpad) {
		printk(KERN_INFO "%s dma_alloc_coherent(...) failed\n", __func__);
		ret = -ENOMEM;
		goto err_txpad;
	}

	memset(spiphy_dev->txpad, 0xFF, TX_BUFFER_SIZE);

	sms_info("exiting\n");	
	return spiphy_dev;

err_txpad:
	free_irq(spiphy_dev->irq, spiphy_dev);	
err_irq:
	gpio_free(MTV_SPI_INT);	
err_req:
	spi_unregister_driver(&smsmdtv_driver);
err_register:
	kfree(spiphy_dev);
err_malloc:
	return 0;
}

int smsspiphy_deinit(void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;

	spi_unregister_driver(&smsmdtv_driver);
	free_irq(spiphy_dev->irq, spiphy_dev);
	gpio_free(MTV_SPI_INT);
	dma_free_coherent(NULL, TX_BUFFER_SIZE, spiphy_dev->txpad, spiphy_dev->txpad_phy_addr);	
	kfree(spiphy_dev);
	return 0;
}

void smsspiphy_set_config(struct spiphy_dev_s *spiphy_dev, int clock_divider)
{

}

void prepareForFWDnl(void *context)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	smsspiphy_set_config(spiphy_dev, 2);
	msleep(100);
}

void fwDnlComplete(void *context, int App)
{
	struct spiphy_dev_s *spiphy_dev = (struct spiphy_dev_s *) context;
	smsspiphy_set_config(spiphy_dev, 1);
	msleep(100);
}

