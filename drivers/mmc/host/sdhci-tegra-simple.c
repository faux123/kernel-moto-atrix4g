/*
 * drivers/mmc/host/sdhci-tegra-simple.c
 *
 * Simplified SDHCI-compatible access driver for NVIDIA Tegra SoCs.   The
 * purpose is to be able to access an MMC/SD/SDIO device after the kernel has
 * panicked (no scheduling).
 *
 * Copyright (c) 2010, Motorola.
 *
 * Based on:
 *   drivers/mmc/host/sdhci-tegra.c
 *
 * SDHCI-compatible driver for NVIDIA Tegra SoCs
 *
 * Copyright (c) 2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#define NV_DEBUG 0

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include <mach/sdhci-simple.h>
#include <mach/pinmux.h>

#include <nvodm_sdio.h>
#include <nvodm_services.h>
#include <nvodm_query_discovery.h>
#include <nvodm_pmu.h>

#include "sdhci.h"


#define DRIVER_DESC "Simple Tegra SDHCI compliant driver"
#define DRIVER_NAME "tegra_sdhci_simple"

/* FIXME: steal this quirk until upstream adds another bitfield */
#define SDHCI_QUIRK_BROKEN_PIO SDHCI_QUIRK_RUNTIME_DISABLE


extern struct sdhci_host *sdhci_simple_alloc_host(int id);
extern int sdhci_simple_add_host(struct sdhci_host *host);


struct tegra_sdhci_simple
{
	struct platform_device		   *pdev;
	NvOdmServicesPmuHandle		    hPmu;
	NvU32				    VddAddress;
	NvOdmServicesPmuVddRailCapabilities VddRailCaps;
	struct clk			   *clk;
	const struct tegra_pingroup_config *pinmux;
	int				    nr_pins;
	unsigned long			    max_clk;
	bool				    clk_enable;
	void __iomem *			    ioaddr;
	unsigned int			    start_offset;
	unsigned int			    data_width;
};


static struct tegra_sdhci_simple tegra_sdhci_simple_host;


static void tegra_sdhci_set_clock(struct sdhci_host *sdhost,
	unsigned int clock)
{
	struct tegra_sdhci_simple *host = &tegra_sdhci_simple_host;

	if (clock) {
		clk_set_rate(host->clk, clock);
		sdhost->max_clk = clk_get_rate(host->clk);
		dev_dbg(&host->pdev->dev, "clock request: %uKHz. currently "
			"%uKHz\n", clock/1000, sdhost->max_clk/1000);
	}

	if (clock && !host->clk_enable) {
		clk_enable(host->clk);
		host->clk_enable = true;
	} else if (!clock && host->clk_enable) {
		clk_disable(host->clk);
		host->clk_enable = false;
	}
}

static struct sdhci_ops tegra_sdhci_simple_ops = {
	.set_clock	= tegra_sdhci_set_clock,
};

/*
 * sdhci_simple_host_init - initialize this SDHCI host interface
 *
 * @id: the controller interface index (i.e. the port)
 */
int sdhci_simple_host_init(int id)
{
	struct sdhci_host *sdhost;
	struct tegra_sdhci_simple *host;
	NvU32 SettlingTime = 0;
	int ret = 0;

	sdhost = sdhci_simple_alloc_host(id);
	host = &tegra_sdhci_simple_host;

	NvOdmServicesPmuSetVoltage(host->hPmu,
			           host->VddAddress,
				   host->VddRailCaps.requestMilliVolts,
				   &SettlingTime);
	if (SettlingTime)
	{
		udelay(SettlingTime);
	}

	if (host->pinmux && host->nr_pins)
		tegra_pinmux_config_tristate_table(host->pinmux,
			host->nr_pins, TEGRA_TRI_NORMAL);
	clk_set_rate(host->clk, host->max_clk);
	clk_enable(host->clk);
	host->max_clk = clk_get_rate(host->clk);
	host->clk_enable = true;

	sdhost->ioaddr = host->ioaddr;
	sdhost->data_width = host->data_width;
	sdhost->start_offset = host->start_offset;
	sdhost->ops = &tegra_sdhci_simple_ops;
	sdhost->quirks =
		SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		SDHCI_QUIRK_SINGLE_POWER_WRITE |
		SDHCI_QUIRK_ENABLE_INTERRUPT_AT_BLOCK_GAP |
		SDHCI_QUIRK_BROKEN_WRITE_PROTECT |
		SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		SDHCI_QUIRK_BROKEN_CTRL_HISPD |
		SDHCI_QUIRK_RUNTIME_DISABLE |
		SDHCI_QUIRK_BROKEN_DMA |
		SDHCI_QUIRK_BROKEN_PIO;
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	sdhost->quirks |= SDHCI_QUIRK_BROKEN_SPEC_VERSION |
		SDHCI_QUIRK_NO_64KB_ADMA;
	sdhost->version = SDHCI_SPEC_200;
#endif

	ret = sdhci_simple_add_host(sdhost);
	if (ret) {
		pr_err("%s: failed to add host: %d (ignoring)\n", __func__, ret);
	}

	return ret;
}

EXPORT_SYMBOL(sdhci_simple_host_init);

int __init tegra_sdhci_simple_probe(struct platform_device *pdev)
{
	struct tegra_sdhci_simple *host;
	struct tegra_sdhci_simple_platform_data *pdata = pdev->dev.platform_data;
	NvOdmPeripheralConnectivity *pConn;
	NvU32 NumOfGuids = 1;
	NvU64 guid;
	NvU32 searchVals[4];
	const NvOdmPeripheralSearch searchAttrs[] = {
		NvOdmPeripheralSearch_PeripheralClass,
		NvOdmPeripheralSearch_IoModule,
		NvOdmPeripheralSearch_Instance,
		NvOdmPeripheralSearch_Address,
	};
	int i, ret = -ENODEV;

	pr_info("%s", __func__);
	if (pdev->id == -1) {
		dev_err(&pdev->dev, "dynamic instance assignment not allowed\n");
		return -ENODEV;
	}

	host = &tegra_sdhci_simple_host;

	/*
	 * Emulate NvOdmSdioOpen() to avoid heap usage and power on request
	 */
	searchVals[0] =  NvOdmPeripheralClass_Other;
	searchVals[1] =  NvOdmIoModule_Sdio;
	searchVals[2] =  pdev->id;
	searchVals[3] =  0;

	NumOfGuids = NvOdmPeripheralEnumerate(searchAttrs,
					      searchVals,
					      4,
					      &guid,
					      NumOfGuids);

	pConn = (NvOdmPeripheralConnectivity*)NvOdmPeripheralGetGuid(guid);
	if (pConn == NULL)
		goto err_odm;

	for (i = 0; i < pConn->NumAddress; i++)
	{
		if (pConn->AddressList[i].Interface == NvOdmIoModule_Vdd)
		{
			host->VddAddress = pConn->AddressList[i].Address;
			NvOdmServicesPmuGetCapabilities(host->hPmu,
							host->VddAddress,
							&host->VddRailCaps);
		}
	}
	host->hPmu = NvOdmServicesPmuOpen();
	if (host->hPmu == NULL) {
		pr_err("%s: failed to open PMU\n", __func__);
		goto err_odm;
	}

	for (i = 0; i < pdata->num_resources; i++) {
		if (pdata->resource &&
		    pdata->resource[i].flags & IORESOURCE_MEM &&
		    pdata->resource[i].end - pdata->resource[i].start > 0)
			break;
	}

	if (i >= pdata->num_resources) {
		dev_err(&pdev->dev, "no memory I/O resource provided\n");
		ret = -ENODEV;
		goto err_odm;
	}

	host->ioaddr = ioremap(pdata->resource[i].start,
			       pdata->resource[i].end -
					pdata->resource[i].start + 1);

	host->clk = clk_get_sys(pdata->clk_dev_name, NULL);
	if (!host->clk) {
		dev_err(&pdev->dev, "unable to get clock %s\n", pdata->clk_dev_name);
		ret = -ENODEV;
		goto err_ioremap;
	}

	host->pdev = pdev;
	host->pinmux = pdata->sdhci_pdata->pinmux;
	host->nr_pins = pdata->sdhci_pdata->nr_pins;
	if (pdata->sdhci_pdata->max_clk)
		host->max_clk = min_t(unsigned int, 52000000, pdata->sdhci_pdata->max_clk);
	else {
		dev_info(&pdev->dev, "no max_clk specified, default to 52MHz\n");
		host->max_clk = 52000000;
	}
	host->data_width = pdata->sdhci_pdata->bus_width;
#ifdef CONFIG_EMBEDDED_MMC_START_OFFSET
	host->start_offset = pdata->sdhci_pdata->offset;
#endif

	dev_info(&pdev->dev, "probe complete\n");
	return  0;

err_ioremap:
	iounmap(host->ioaddr);
err_odm:
	if (host->hPmu)
		NvOdmServicesPmuClose(host->hPmu);
	dev_err(&pdev->dev, "probe failed\n");
	return ret;
}


static int tegra_sdhci_simple_remove(struct platform_device *pdev)
{
	struct sdhci_host *sdhost = platform_get_drvdata(pdev);

	if (tegra_sdhci_simple_host.clk_enable)
		clk_disable(tegra_sdhci_simple_host.clk);

	clk_put(tegra_sdhci_simple_host.clk);
	iounmap(sdhost->ioaddr);
	sdhost->ioaddr = NULL;

	if (tegra_sdhci_simple_host.hPmu)
		NvOdmServicesPmuClose(tegra_sdhci_simple_host.hPmu);

	return 0;
}

struct platform_driver tegra_sdhci_simple_driver = {
	.probe		= tegra_sdhci_simple_probe,
	.remove		= tegra_sdhci_simple_remove,
	.driver		= {
		.name	= "tegra-sdhci-simple",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_sdhci_simple_init(void)
{
	pr_info("%s", __func__);
	return platform_driver_register(&tegra_sdhci_simple_driver);
}

static void __exit tegra_sdhci_simple_exit(void)
{
	platform_driver_unregister(&tegra_sdhci_simple_driver);
}

module_init(tegra_sdhci_simple_init);
module_exit(tegra_sdhci_simple_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
