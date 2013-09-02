/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/clkdev.h>
#include <asm/mach-types.h>

#include <mach/msm_iomap.h>
#include <mach/clk.h>
#include <mach/rpm-regulator.h>
#include <mach/socinfo.h>

#include "clock-local.h"
#include "clock-rpm.h"
#include "clock-voter.h"
#include "clock-dss-8960.h"
#include "devices.h"
#include "clock-pll.h"

static struct clk_lookup msm_clocks_8064_r2[] = {
	CLK_LOOKUP("xo",		cxo_a_clk.c,	""),
	CLK_LOOKUP("xo",		pxo_a_clk.c,	""),
	CLK_LOOKUP("pwm_clk",		cxo_clk.c,	"0-0048"),
	CLK_LOOKUP("cxo",		cxo_clk.c,	"wcnss_wlan.0"),
	CLK_LOOKUP("cxo",		cxo_clk.c,	"pil_riva"),
	CLK_LOOKUP("xo",		pxo_clk.c,	"pil_qdsp6v4.0"),
	CLK_LOOKUP("xo",		cxo_clk.c,	"pil_qdsp6v4.1"),
	CLK_LOOKUP("xo",		cxo_clk.c,	"pil_qdsp6v4.2"),
	CLK_LOOKUP("xo",		cxo_clk.c,	"pil_gss"),
	CLK_LOOKUP("xo",		cxo_clk.c,	"BAM_RMNT"),
	CLK_LOOKUP("xo",		cxo_clk.c,	"msm_xo"),
	CLK_LOOKUP("pll2",		pll2_clk.c,	NULL),
	CLK_LOOKUP("pll8",		pll8_clk.c,	NULL),
	CLK_LOOKUP("pll4",		pll4_clk.c,	NULL),
	CLK_LOOKUP("measure",		measure_clk.c,	"debug"),

	CLK_LOOKUP("bus_clk",		afab_clk.c,	""),
	CLK_LOOKUP("bus_clk",		afab_a_clk.c,	""),
	CLK_LOOKUP("bus_clk",		cfpb_clk.c,	""),
	CLK_LOOKUP("bus_clk",		cfpb_a_clk.c,	""),
	CLK_LOOKUP("bus_clk",		dfab_clk.c,	""),
	CLK_LOOKUP("bus_clk",		dfab_a_clk.c,	""),
	CLK_LOOKUP("mem_clk",		ebi1_clk.c,	""),
	CLK_LOOKUP("mem_clk",		ebi1_a_clk.c,	""),
	CLK_LOOKUP("bus_clk",		mmfab_clk.c,	""),
	CLK_LOOKUP("bus_clk",		mmfab_a_clk.c,	""),
	CLK_LOOKUP("bus_clk",		mmfpb_clk.c,	""),
	CLK_LOOKUP("bus_clk",		mmfpb_a_clk.c,	""),
	CLK_LOOKUP("bus_clk",		sfab_clk.c,	""),
	CLK_LOOKUP("bus_clk",		sfab_a_clk.c,	""),
	CLK_LOOKUP("bus_clk",		sfpb_clk.c,	""),
	CLK_LOOKUP("bus_clk",		sfpb_a_clk.c,	""),

	CLK_LOOKUP("bus_clk",		afab_clk.c,		"msm_apps_fab"),
	CLK_LOOKUP("bus_a_clk",		afab_msmbus_a_clk.c,	"msm_apps_fab"),
	CLK_LOOKUP("bus_clk",		cfpb_clk.c,		"msm_cpss_fpb"),
	CLK_LOOKUP("bus_a_clk",		cfpb_a_clk.c,		"msm_cpss_fpb"),
	CLK_LOOKUP("bus_clk",		sfab_clk.c,		"msm_sys_fab"),
	CLK_LOOKUP("bus_a_clk",		sfab_msmbus_a_clk.c,	"msm_sys_fab"),
	CLK_LOOKUP("bus_clk",		sfpb_clk.c,		"msm_sys_fpb"),
	CLK_LOOKUP("bus_a_clk",		sfpb_a_clk.c,		"msm_sys_fpb"),
	CLK_LOOKUP("bus_clk",		mmfab_clk.c,		"msm_mm_fab"),
	CLK_LOOKUP("bus_a_clk",		mmfab_a_clk.c,		"msm_mm_fab"),
	CLK_LOOKUP("mem_clk",		ebi1_msmbus_clk.c,	"msm_bus"),
	CLK_LOOKUP("mem_a_clk",		ebi1_msmbus_a_clk.c,	"msm_bus"),
	CLK_LOOKUP("dfab_clk",		dfab_msmbus_clk.c,	"msm_bus"),
	CLK_LOOKUP("dfab_a_clk",	dfab_msmbus_a_clk.c,	"msm_bus"),
	CLK_LOOKUP("core_a_clk",	qdss_a_clk.c,		""),
	CLK_LOOKUP("core_clk",		qdss_clk.c,		"msm_etb.0"),
	CLK_LOOKUP("core_clk",		qdss_clk.c,		"msm_tpiu.0"),
	CLK_LOOKUP("core_clk",		qdss_clk.c,		"msm_funnel.0"),
	CLK_LOOKUP("core_clk",		qdss_clk.c,		"msm_stm.0"),
	CLK_LOOKUP("core_clk",		qdss_clk.c,		"msm_etm.0"),

	CLK_LOOKUP("ebi1_clk",		ebi1_clk.c,		""),
	CLK_LOOKUP("mmfpb_clk",		mmfpb_clk.c,		""),
	CLK_LOOKUP("mmfpb_a_clk",	mmfpb_a_clk.c,		"clock-8960"),
	CLK_LOOKUP("cfpb_a_clk",	cfpb_a_clk.c,		"clock-8960"),

	CLK_LOOKUP("core_clk",		gp0_clk.c,		""),
	CLK_LOOKUP("core_clk",		gp1_clk.c,		""),
	CLK_LOOKUP("core_clk",		gp2_clk.c,		""),
	CLK_LOOKUP("core_clk",		gsbi1_uart_clk.c, "msm_serial_hsl.1"),
	CLK_LOOKUP("core_clk",		gsbi2_uart_clk.c, "msm_serial_hsl.3"),
#ifdef CONFIG_QSC_MODEM
	CLK_LOOKUP("core_clk",		gsbi1_uart_clk.c,	"msm_serial_hs.1"),
	CLK_LOOKUP("iface_clk",		gsbi1_p_clk.c,		"msm_serial_hs.1"),
#endif
#ifdef CONFIG_SERIAL_IRDA
	CLK_LOOKUP("core_clk",		gsbi3_uart_clk.c,	"msm_serial_irda.2"),
#elif defined CONFIG_SERIAL_CIR
	CLK_LOOKUP("core_clk",		gsbi3_uart_clk.c,	"msm_serial_cir.2"),
#else
	CLK_LOOKUP("core_clk",		gsbi3_uart_clk.c,	""),
#endif
#ifdef CONFIG_GSBI4_UARTDM
	CLK_LOOKUP("core_clk",		gsbi4_uart_clk.c, "msm_serial_hs.1"),
#else
	CLK_LOOKUP("core_clk",		gsbi4_uart_clk.c,	""),
#endif
	CLK_LOOKUP("core_clk",		gsbi5_uart_clk.c,	""),
	CLK_LOOKUP("core_clk",		gsbi6_uart_clk.c,	"msm_serial_hs.0"),
#ifdef CONFIG_BT
	CLK_LOOKUP("core_clk",		gsbi6_uart_clk.c,	"msm_serial_hs_brcm.0"),
#endif 
	CLK_LOOKUP("core_clk",		gsbi7_uart_clk.c, "msm_serial_hsl.0"),
#ifndef CONFIG_QSC_MODEM
	CLK_LOOKUP("core_clk",		gsbi1_qup_clk.c,	"qup_i2c.0"),
#endif
	CLK_LOOKUP("core_clk",		gsbi2_qup_clk.c,	"qup_i2c.2"),
	CLK_LOOKUP("core_clk",		gsbi3_qup_clk.c,	"qup_i2c.3"),
	CLK_LOOKUP("core_clk",		gsbi4_qup_clk.c,	"qup_i2c.4"),
	CLK_LOOKUP("core_clk",		gsbi5_qup_clk.c,	"spi_qsd.0"),
#ifdef CONFIG_FPR_SPI
	CLK_LOOKUP("core_clk",		gsbi1_qup_clk.c,	"spi_qsd.1"),
#endif
	CLK_LOOKUP("core_clk",		gsbi5_qup_clk.c,	"qup_i2c.5"),
	CLK_LOOKUP("core_clk",		gsbi6_qup_clk.c,	""),
	CLK_LOOKUP("core_clk",		gsbi7_qup_clk.c,	"qup_i2c.7"),
	CLK_LOOKUP("core_clk",		pdm_clk.c,		""),
	CLK_LOOKUP("mem_clk",		pmem_clk.c,		"msm_sps"),
	CLK_LOOKUP("core_clk",          prng_clk.c,		"msm_rng.0"),
	CLK_LOOKUP("core_clk",		sdc1_clk.c,		"msm_sdcc.1"),
	CLK_LOOKUP("core_clk",		sdc2_clk.c,		"msm_sdcc.2"),
	CLK_LOOKUP("core_clk",		sdc3_clk.c,		"msm_sdcc.3"),
	CLK_LOOKUP("core_clk",		sdc4_clk.c,		"msm_sdcc.4"),
	CLK_LOOKUP("ref_clk",		tsif_ref_clk.c,		""),
	CLK_LOOKUP("core_clk",		tssc_clk.c,		""),
	CLK_LOOKUP("alt_core_clk",	usb_hs1_xcvr_clk.c,	"msm_otg"),
	CLK_LOOKUP("alt_core_clk",      usb_hs3_xcvr_clk.c,  "msm_ehci_host.0"),
	CLK_LOOKUP("alt_core_clk",      usb_hs4_xcvr_clk.c,  "msm_ehci_host.1"),
	CLK_LOOKUP("src_clk",		usb_fs1_src_clk.c,	""),
	CLK_LOOKUP("alt_core_clk",	usb_fs1_xcvr_clk.c,	""),
	CLK_LOOKUP("sys_clk",		usb_fs1_sys_clk.c,	""),
	CLK_LOOKUP("ref_clk",		sata_phy_ref_clk.c,	""),
	CLK_LOOKUP("cfg_clk",		sata_phy_cfg_clk.c,	""),
	CLK_LOOKUP("src_clk",		sata_src_clk.c,		""),
	CLK_LOOKUP("core_rxoob_clk",	sata_rxoob_clk.c,	""),
	CLK_LOOKUP("core_pmalive_clk",	sata_pmalive_clk.c,	""),
	CLK_LOOKUP("bus_clk",		sata_a_clk.c,		""),
	CLK_LOOKUP("iface_clk",		sata_p_clk.c,		""),
	CLK_LOOKUP("slave_iface_clk",	sfab_sata_s_p_clk.c,	""),
	CLK_LOOKUP("iface_clk",		ce3_p_clk.c,		"qce.0"),
	CLK_LOOKUP("iface_clk",		ce3_p_clk.c,		"qcrypto.0"),
	CLK_LOOKUP("core_clk",		ce3_core_clk.c,		"qce.0"),
	CLK_LOOKUP("core_clk",		ce3_core_clk.c,		"qcrypto.0"),
	CLK_LOOKUP("ce3_core_src_clk",	ce3_src_clk.c,		"qce.0"),
	CLK_LOOKUP("ce3_core_src_clk",	ce3_src_clk.c,		"qcrypto.0"),
	CLK_LOOKUP("dma_bam_pclk",	dma_bam_p_clk.c,	NULL),
	CLK_LOOKUP("iface_clk",		gsbi1_p_clk.c,	"msm_serial_hsl.1"),
	CLK_LOOKUP("iface_clk",		gsbi1_p_clk.c,	"qup_i2c.0"),
	CLK_LOOKUP("iface_clk",		gsbi2_p_clk.c,		"qup_i2c.2"),
	CLK_LOOKUP("iface_clk",		gsbi2_p_clk.c,	"msm_serial_hsl.3"),
#ifdef CONFIG_SERIAL_IRDA
	CLK_LOOKUP("iface_clk",		gsbi3_p_clk.c,		"msm_serial_irda.2"),
#endif
#ifdef CONFIG_SERIAL_CIR
	CLK_LOOKUP("iface_clk",		gsbi3_p_clk.c,		"msm_serial_cir.2"),
#endif
	CLK_LOOKUP("iface_clk",		gsbi3_p_clk.c,		"qup_i2c.3"),
#ifdef CONFIG_GSBI4_UARTDM
	CLK_LOOKUP("iface_clk",     	gsbi4_p_clk.c,  "msm_serial_hs.1"),
#endif
	CLK_LOOKUP("iface_clk",		gsbi4_p_clk.c,		"qup_i2c.4"),
	CLK_LOOKUP("iface_clk",		gsbi5_p_clk.c,		"spi_qsd.0"),
#ifdef CONFIG_FPR_SPI
	CLK_LOOKUP("iface_clk",		gsbi1_p_clk.c,		"spi_qsd.1"),
#endif
	CLK_LOOKUP("iface_clk",		gsbi5_p_clk.c,		"qup_i2c.5"),
	CLK_LOOKUP("iface_clk",		gsbi6_p_clk.c,		"msm_serial_hs.0"),
#ifdef CONFIG_BT
	CLK_LOOKUP("iface_clk",		gsbi6_p_clk.c,		"msm_serial_hs_brcm.0"),
#endif 
	CLK_LOOKUP("iface_clk",		gsbi7_p_clk.c,	"msm_serial_hsl.0"),
	CLK_LOOKUP("iface_clk",		gsbi7_p_clk.c,	"qup_i2c.7"),
#ifdef CONFIG_QSC_MODEM
	CLK_LOOKUP("core_clk",		gsbi1_uart_clk.c, 	"msm_serial_hs.1"),
	CLK_LOOKUP("iface_clk",		gsbi1_p_clk.c,		"msm_serial_hs.1"),
#endif
	CLK_LOOKUP("ref_clk",	tsif_ref_clk.c,	"msm_tspp.0"),
	CLK_LOOKUP("iface_clk",		tsif_p_clk.c,		"msm_tspp.0"),
	CLK_LOOKUP("tsif_pclk",        tsif_p_clk.c,           "msm_tsif.1"),
	CLK_LOOKUP("tsif_ref_clk",      tsif_ref_clk.c,         "msm_tsif.1"),
	CLK_LOOKUP("iface_clk",		usb_fs1_p_clk.c,	""),
	CLK_LOOKUP("iface_clk",		usb_hs1_p_clk.c,	"msm_otg"),
	CLK_LOOKUP("iface_clk",         usb_hs3_p_clk.c,     "msm_ehci_host.0"),
	CLK_LOOKUP("iface_clk",         usb_hs4_p_clk.c,     "msm_ehci_host.1"),
	CLK_LOOKUP("iface_clk",		sdc1_p_clk.c,		"msm_sdcc.1"),
	CLK_LOOKUP("iface_clk",		sdc2_p_clk.c,		"msm_sdcc.2"),
	CLK_LOOKUP("iface_clk",		sdc3_p_clk.c,		"msm_sdcc.3"),
	CLK_LOOKUP("iface_clk",		sdc4_p_clk.c,		"msm_sdcc.4"),
	CLK_LOOKUP("iface_clk",		pcie_p_clk.c,		"msm_pcie"),
	CLK_LOOKUP("ref_clk",		pcie_phy_ref_clk.c,	"msm_pcie"),
	CLK_LOOKUP("bus_clk",		pcie_a_clk.c,		"msm_pcie"),
	CLK_LOOKUP("core_clk",		adm0_clk.c,		"msm_dmov"),
	CLK_LOOKUP("iface_clk",		adm0_p_clk.c,		"msm_dmov"),
	CLK_LOOKUP("iface_clk",		pmic_arb0_p_clk.c,	""),
	CLK_LOOKUP("iface_clk",		pmic_arb1_p_clk.c,	""),
	CLK_LOOKUP("core_clk",		pmic_ssbi2_clk.c,	""),
	CLK_LOOKUP("mem_clk",		rpm_msg_ram_p_clk.c,	""),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-001a"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-0034"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-0020"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-0048"),
	CLK_LOOKUP("cam_clk",		cam1_clk.c,	"4-006c"),
	CLK_LOOKUP("csi_src_clk",	csi0_src_clk.c,		"msm_csiphy.0"),
	CLK_LOOKUP("csi_src_clk",	csi1_src_clk.c,		"msm_csiphy.1"),
	CLK_LOOKUP("csi_src_clk",	csi2_src_clk.c,		"msm_csiphy.2"),
	CLK_LOOKUP("csi_clk",		csi0_clk.c,		"msm_csiphy.0"),
	CLK_LOOKUP("csi_clk",		csi1_clk.c,		"msm_csiphy.1"),
	CLK_LOOKUP("csi_clk",		csi2_clk.c,		"msm_csiphy.2"),
	CLK_LOOKUP("csi_phy_clk",	csi0_phy_clk.c,		"msm_csiphy.0"),
	CLK_LOOKUP("csi_phy_clk",	csi1_phy_clk.c,		"msm_csiphy.1"),
	CLK_LOOKUP("csi_phy_clk",	csi2_phy_clk.c,		"msm_csiphy.2"),
	CLK_LOOKUP("csi_pix_clk",	csi_pix_clk.c,		"msm_ispif.0"),
	CLK_LOOKUP("csi_pix1_clk",	csi_pix1_clk.c,		"msm_ispif.0"),
	CLK_LOOKUP("csi_rdi_clk",	csi_rdi_clk.c,		"msm_ispif.0"),
	CLK_LOOKUP("csi_rdi1_clk",	csi_rdi1_clk.c,		"msm_ispif.0"),
	CLK_LOOKUP("csi_rdi2_clk",	csi_rdi2_clk.c,		"msm_ispif.0"),
	CLK_LOOKUP("csiphy_timer_src_clk",
			   csiphy_timer_src_clk.c, "msm_csiphy.0"),
	CLK_LOOKUP("csiphy_timer_src_clk",
			   csiphy_timer_src_clk.c, "msm_csiphy.1"),
	CLK_LOOKUP("csiphy_timer_src_clk",
			   csiphy_timer_src_clk.c, "msm_csiphy.2"),
	CLK_LOOKUP("csiphy_timer_clk",	csi0phy_timer_clk.c,	"msm_csiphy.0"),
	CLK_LOOKUP("csiphy_timer_clk",	csi1phy_timer_clk.c,	"msm_csiphy.1"),
	CLK_LOOKUP("csiphy_timer_clk",	csi2phy_timer_clk.c,	"msm_csiphy.2"),
	CLK_LOOKUP("byte_clk",	dsi1_byte_clk.c,	"mipi_dsi.1"),
	CLK_LOOKUP("byte_clk",	dsi2_byte_clk.c,	"mipi_dsi.2"),
	CLK_LOOKUP("esc_clk",	dsi1_esc_clk.c,		"mipi_dsi.1"),
	CLK_LOOKUP("esc_clk",	dsi2_esc_clk.c,		"mipi_dsi.2"),
	CLK_LOOKUP("rgb_clk",		rgb_tv_clk.c,		""),
	CLK_LOOKUP("npl_clk",		npl_tv_clk.c,		""),

	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"msm_camera_s5k3h2.0"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-0010"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"msm_camera_imx105.0"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"msm_camera_s5k4e5.0"),
	CLK_LOOKUP("cam_clk",		cam1_clk.c,	"msm_camera_mt9v113.0"),
	CLK_LOOKUP("cam_clk",		cam1_clk.c,	"msm_camera_s5k6aafx.0"),
	CLK_LOOKUP("cam_clk",		cam1_clk.c,	"msm_camera_s5k6a1gx.0"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-0036"),
	CLK_LOOKUP("cam_clk",		cam1_clk.c,	"msm_camera_ar0260.0"),
	CLK_LOOKUP("cam_clk",		cam0_clk.c,	"4-0048"),
	CLK_LOOKUP("cam_clk",		cam1_clk.c,	"2-0048"),

	CLK_LOOKUP("core_clk",		gfx3d_clk.c,	"kgsl-3d0.0"),
	CLK_LOOKUP("core_clk",		gfx3d_clk.c,	"footswitch-8x60.2"),
	CLK_LOOKUP("bus_clk",
			    gfx3d_axi_clk.c, "footswitch-8x60.2"),
	CLK_LOOKUP("iface_clk",         vcap_p_clk.c,           ""),
	CLK_LOOKUP("iface_clk",         vcap_p_clk.c,           "msm_vcap.0"),
	CLK_LOOKUP("iface_clk",         vcap_p_clk.c,	"footswitch-8x60.10"),
	CLK_LOOKUP("bus_clk",		vcap_axi_clk.c,	"footswitch-8x60.10"),
	CLK_LOOKUP("core_clk",          vcap_clk.c,             ""),
	CLK_LOOKUP("core_clk",          vcap_clk.c,             "msm_vcap.0"),
	CLK_LOOKUP("core_clk",          vcap_clk.c,	"footswitch-8x60.10"),
	CLK_LOOKUP("vcap_npl_clk",      vcap_npl_clk.c,         ""),
	CLK_LOOKUP("vcap_npl_clk",      vcap_npl_clk.c,         "msm_vcap.0"),
	CLK_LOOKUP("bus_clk",		ijpeg_axi_clk.c, "footswitch-8x60.3"),
	CLK_LOOKUP("mem_clk",		imem_axi_clk.c,	"msm_gemini.0"),
	CLK_LOOKUP("core_clk",          ijpeg_clk.c,    "msm_gemini.0"),
	CLK_LOOKUP("core_clk",		ijpeg_clk.c,	"footswitch-8x60.3"),
	CLK_LOOKUP("core_clk",		jpegd_clk.c,		""),
	CLK_LOOKUP("core_clk",		mdp_clk.c,		"mdp.0"),
	CLK_LOOKUP("core_clk",		mdp_clk.c,	 "footswitch-8x60.4"),
	CLK_LOOKUP("vsync_clk",	mdp_vsync_clk.c,	"mdp.0"),
	CLK_LOOKUP("vsync_clk",		mdp_vsync_clk.c, "footswitch-8x60.4"),
	CLK_LOOKUP("lut_clk",		lut_mdp_clk.c,		"mdp.0"),
	CLK_LOOKUP("lut_clk",		lut_mdp_clk.c,	"footswitch-8x60.4"),
	CLK_LOOKUP("core_clk",		rot_clk.c,	"msm_rotator.0"),
	CLK_LOOKUP("core_clk",		rot_clk.c,	"footswitch-8x60.6"),
	CLK_LOOKUP("tv_src_clk",	tv_src_clk.c,	"footswitch-8x60.4"),
	CLK_LOOKUP("src_clk",	tv_src_clk.c,		"dtv.0"),
	CLK_LOOKUP("div_clk",	tv_src_div_clk.c,	""),
	CLK_LOOKUP("core_clk",		vcodec_clk.c,		"msm_vidc.0"),
	CLK_LOOKUP("core_clk",		vcodec_clk.c,	"footswitch-8x60.7"),
	CLK_LOOKUP("mdp_clk",	mdp_tv_clk.c,		"dtv.0"),
	CLK_LOOKUP("tv_clk",		mdp_tv_clk.c,	"footswitch-8x60.4"),
	CLK_LOOKUP("hdmi_clk",		hdmi_tv_clk.c,		"dtv.0"),
	CLK_LOOKUP("core_clk",		hdmi_app_clk.c,		"hdmi_msm.1"),
	CLK_LOOKUP("vpe_clk",		vpe_clk.c,		"msm_vpe.0"),
	CLK_LOOKUP("core_clk",		vpe_clk.c,	"footswitch-8x60.9"),
	CLK_LOOKUP("vfe_clk",		vfe_clk.c,		"msm_vfe.0"),
	CLK_LOOKUP("core_clk",		vfe_clk.c,	"footswitch-8x60.8"),
	CLK_LOOKUP("csi_vfe_clk",	csi_vfe_clk.c,		"msm_vfe.0"),
	CLK_LOOKUP("bus_clk",		vfe_axi_clk.c,	"footswitch-8x60.8"),
	CLK_LOOKUP("bus_clk",		mdp_axi_clk.c,	"footswitch-8x60.4"),
	CLK_LOOKUP("bus_clk",		rot_axi_clk.c,	"footswitch-8x60.6"),
	CLK_LOOKUP("bus_clk",		vcodec_axi_clk.c,  "footswitch-8x60.7"),
	CLK_LOOKUP("bus_a_clk",        vcodec_axi_a_clk.c, "footswitch-8x60.7"),
	CLK_LOOKUP("bus_b_clk",        vcodec_axi_b_clk.c, "footswitch-8x60.7"),
	CLK_LOOKUP("bus_clk",		vpe_axi_clk.c,	"footswitch-8x60.9"),
	CLK_LOOKUP("arb_clk",		amp_p_clk.c,		"mipi_dsi.1"),
	CLK_LOOKUP("arb_clk",		amp_p_clk.c,		"mipi_dsi.2"),
	CLK_LOOKUP("csi_pclk",          csi_p_clk.c,            "msm_csiphy.0"),
	CLK_LOOKUP("csi_pclk",          csi_p_clk.c,            "msm_csiphy.1"),
	CLK_LOOKUP("csi_pclk",          csi_p_clk.c,            "msm_csiphy.2"),
	CLK_LOOKUP("master_iface_clk",	dsi1_m_p_clk.c,		"mipi_dsi.1"),
	CLK_LOOKUP("slave_iface_clk",	dsi1_s_p_clk.c,		"mipi_dsi.1"),
	CLK_LOOKUP("master_iface_clk",	dsi2_m_p_clk.c,		"mipi_dsi.2"),
	CLK_LOOKUP("slave_iface_clk",	dsi2_s_p_clk.c,		"mipi_dsi.2"),
	CLK_LOOKUP("iface_clk",		gfx3d_p_clk.c,	"kgsl-3d0.0"),
	CLK_LOOKUP("iface_clk",		gfx3d_p_clk.c,	"footswitch-8x60.2"),
	CLK_LOOKUP("master_iface_clk",	hdmi_m_p_clk.c,		"hdmi_msm.1"),
	CLK_LOOKUP("slave_iface_clk",	hdmi_s_p_clk.c,		"hdmi_msm.1"),
	CLK_LOOKUP("iface_clk",		ijpeg_p_clk.c,		"msm_gemini.0"),
	CLK_LOOKUP("iface_clk",		ijpeg_p_clk.c,	"footswitch-8x60.3"),
	CLK_LOOKUP("iface_clk",		jpegd_p_clk.c,		""),
	CLK_LOOKUP("mem_iface_clk",	imem_p_clk.c,	"kgsl-3d0.0"),
	CLK_LOOKUP("iface_clk",		mdp_p_clk.c,		"mdp.0"),
	CLK_LOOKUP("iface_clk",		mdp_p_clk.c,	"footswitch-8x60.4"),
	CLK_LOOKUP("iface_clk",		smmu_p_clk.c,		"msm_iommu"),
	CLK_LOOKUP("iface_clk",		rot_p_clk.c,	"msm_rotator.0"),
	CLK_LOOKUP("iface_clk",		rot_p_clk.c,	"footswitch-8x60.6"),
	CLK_LOOKUP("iface_clk",		vcodec_p_clk.c,		"msm_vidc.0"),
	CLK_LOOKUP("iface_clk",		vcodec_p_clk.c,	"footswitch-8x60.7"),
	CLK_LOOKUP("vfe_pclk",		vfe_p_clk.c,		"msm_vfe.0"),
	CLK_LOOKUP("iface_clk",		vfe_p_clk.c,	"footswitch-8x60.8"),
	CLK_LOOKUP("vpe_pclk",		vpe_p_clk.c,		"msm_vpe.0"),
	CLK_LOOKUP("iface_clk",		vpe_p_clk.c,	"footswitch-8x60.9"),

	CLK_LOOKUP("bit_clk",		mi2s_bit_clk.c,
			    "msm-dai-q6-mi2s"),
	CLK_LOOKUP("osr_clk",		mi2s_osr_clk.c,
			    "msm-dai-q6-mi2s"),
	CLK_LOOKUP("bit_clk",		codec_i2s_mic_bit_clk.c,
			   "msm-dai-q6.1"),
	CLK_LOOKUP("osr_clk",		codec_i2s_mic_osr_clk.c,
			   "msm-dai-q6.1"),
	CLK_LOOKUP("bit_clk",		spare_i2s_mic_bit_clk.c,
			   "msm-dai-q6.5"),
	CLK_LOOKUP("osr_clk",		spare_i2s_mic_osr_clk.c,
			   "msm-dai-q6.5"),
	CLK_LOOKUP("bit_clk",		codec_i2s_spkr_bit_clk.c,
			   "msm-dai-q6.16384"),
	CLK_LOOKUP("osr_clk",		codec_i2s_spkr_osr_clk.c,
			   "msm-dai-q6.16384"),
	CLK_LOOKUP("bit_clk",		spare_i2s_spkr_bit_clk.c,
			   "msm-dai-q6.4"),
	CLK_LOOKUP("osr_clk",		spare_i2s_spkr_osr_clk.c,
			   "msm-dai-q6.4"),
	CLK_LOOKUP("pcm_clk",		pcm_clk.c,		"msm-dai-q6.2"),
	CLK_LOOKUP("pcm_clk",		pcm_clk.c,		"msm-dai-q6.3"),
	CLK_LOOKUP("sps_slimbus_clk",	sps_slimbus_clk.c,	""),
	CLK_LOOKUP("core_clk",		audio_slimbus_clk.c, "msm_slim_ctrl.1"),
	CLK_LOOKUP("core_clk",		jpegd_axi_clk.c,	""),
	CLK_LOOKUP("core_clk",		vpe_axi_clk.c,		""),
	CLK_LOOKUP("core_clk",		mdp_axi_clk.c,		""),
	CLK_LOOKUP("core_clk",		vcap_axi_clk.c,		""),
	CLK_LOOKUP("core_clk",		rot_axi_clk.c,		""),
	CLK_LOOKUP("core_clk",		ijpeg_axi_clk.c,	""),
	CLK_LOOKUP("core_clk",		vfe_axi_clk.c,		""),
	CLK_LOOKUP("core_clk",		vcodec_axi_a_clk.c,	""),
	CLK_LOOKUP("core_clk",		vcodec_axi_b_clk.c,	""),
	CLK_LOOKUP("core_clk",		gfx3d_axi_clk.c,	""),

	CLK_LOOKUP("dfab_dsps_clk",	dfab_dsps_clk.c, NULL),
	CLK_LOOKUP("core_clk",		dfab_usb_hs_clk.c,	"msm_otg"),
	CLK_LOOKUP("core_clk",		dfab_usb_hs3_clk.c, "msm_ehci_host.0"),
	CLK_LOOKUP("core_clk",		dfab_usb_hs3_clk.c, "msm_ehci_host.1"),
	CLK_LOOKUP("bus_clk",		dfab_sdc1_clk.c, "msm_sdcc.1"),
	CLK_LOOKUP("bus_clk",		dfab_sdc2_clk.c, "msm_sdcc.2"),
	CLK_LOOKUP("bus_clk",		dfab_sdc3_clk.c, "msm_sdcc.3"),
	CLK_LOOKUP("bus_clk",		dfab_sdc4_clk.c, "msm_sdcc.4"),
	CLK_LOOKUP("dfab_clk",		dfab_sps_clk.c,	"msm_sps"),
	CLK_LOOKUP("bus_clk",		dfab_bam_dmux_clk.c,	"BAM_RMNT"),
	CLK_LOOKUP("bus_clk",		dfab_scm_clk.c,	"scm"),

	CLK_LOOKUP("alt_core_clk",    usb_hsic_xcvr_fs_clk.c,  "msm_hsic_host"),
	CLK_LOOKUP("phy_clk",	      usb_hsic_hsic_clk.c,     "msm_hsic_host"),
	CLK_LOOKUP("cal_clk",	      usb_hsic_hsio_cal_clk.c, "msm_hsic_host"),
	CLK_LOOKUP("core_clk",	      usb_hsic_system_clk.c,   "msm_hsic_host"),
	CLK_LOOKUP("iface_clk",	      usb_hsic_p_clk.c,        "msm_hsic_host"),

	CLK_LOOKUP("core_clk",		jpegd_axi_clk.c,	"msm_iommu.0"),
	CLK_LOOKUP("core_clk",		vpe_axi_clk.c,		"msm_iommu.1"),
	CLK_LOOKUP("core_clk",		mdp_axi_clk.c,		"msm_iommu.2"),
	CLK_LOOKUP("core_clk",		mdp_axi_clk.c,		"msm_iommu.3"),
	CLK_LOOKUP("core_clk",		rot_axi_clk.c,		"msm_iommu.4"),
	CLK_LOOKUP("core_clk",		ijpeg_axi_clk.c,	"msm_iommu.5"),
	CLK_LOOKUP("core_clk",		vfe_axi_clk.c,		"msm_iommu.6"),
	CLK_LOOKUP("core_clk",		vcodec_axi_a_clk.c,	"msm_iommu.7"),
	CLK_LOOKUP("core_clk",		vcodec_axi_b_clk.c,	"msm_iommu.8"),
	CLK_LOOKUP("core_clk",		gfx3d_axi_clk.c,	"msm_iommu.9"),
	CLK_LOOKUP("core_clk",		gfx3d_axi_clk.c,	"msm_iommu.10"),
	CLK_LOOKUP("core_clk",		vcap_axi_clk.c,		"msm_iommu.11"),

	CLK_LOOKUP("mdp_iommu_clk", mdp_axi_clk.c,	"msm_vidc.0"),
	CLK_LOOKUP("rot_iommu_clk",	rot_axi_clk.c,	"msm_vidc.0"),
	CLK_LOOKUP("vcodec_iommu0_clk", vcodec_axi_a_clk.c, "msm_vidc.0"),
	CLK_LOOKUP("vcodec_iommu1_clk", vcodec_axi_b_clk.c, "msm_vidc.0"),
	CLK_LOOKUP("smmu_iface_clk", smmu_p_clk.c,	"msm_vidc.0"),
	CLK_LOOKUP("core_clk",		vcodec_axi_clk.c,  "pil_vidc"),
	CLK_LOOKUP("smmu_iface_clk",	smmu_p_clk.c,  "pil_vidc"),

	CLK_LOOKUP("mem_clk",		ebi1_adm_clk.c, "msm_dmov"),
	CLK_LOOKUP("mem_clk",		ebi1_acpu_a_clk.c, ""),
	CLK_LOOKUP("bus_clk",		afab_acpu_a_clk.c, ""),

	CLK_LOOKUP("l2_mclk",		l2_m_clk,     ""),
	CLK_LOOKUP("krait0_mclk",	krait0_m_clk, ""),
	CLK_LOOKUP("krait1_mclk",	krait1_m_clk, ""),
	CLK_LOOKUP("krait2_mclk",	krait2_m_clk, ""),
	CLK_LOOKUP("krait3_mclk",	krait3_m_clk, ""),
};

struct clock_init_data apq8064_clock_init_data_r2 __initdata = {
	.table = msm_clocks_8064_r2,
	.size = ARRAY_SIZE(msm_clocks_8064_r2),
	.pre_init = msm8960_clock_pre_init,
	.post_init = msm8960_clock_post_init,
	.late_init = msm8960_clock_late_init,
};

#if 0
void clk_ignor_list_add(const char *dev_id, const char *con_id, struct clock_init_data *msm_clock_init_data)
{
	struct clk_lookup *p, *cl = NULL;
	int match, best = 0;
	int i;

	for (i = 0; i < msm_clock_init_data->size; i++) {
		p = &msm_clock_init_data->table[i];
		match = 0;
		if (p->dev_id) {
			if (!dev_id || strcmp(p->dev_id, dev_id))
				continue;
			match += 2;
		}
		if (p->con_id) {
			if (!con_id || strcmp(p->con_id, con_id))
				continue;
			match += 1;
		}

		if (match > best) {
			cl = p;
			if (match != 3)
				best = match;
			else
				break;
		}
	}

	if (cl)
		cl->clk->flags |= CLKFLAG_IGNORE;
}
#endif
