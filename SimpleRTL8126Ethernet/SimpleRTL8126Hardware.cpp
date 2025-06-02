/* SimpleRTL8126Hardware.cpp -- RTL8126 hardware initialzation methods.
*
* Copyright (c) 2020 Laura Müller <laura-mueller@uni-duesseldorf.de>
* All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation; either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* Driver for Realtek RTL8126 PCIe 2.5GB ethernet controllers.
*
* This driver is based on Realtek's r8126 Linux driver (9.003.04).
*/

#include "SimpleRTL8126Ethernet.hpp"

#pragma mark --- hardware initialization methods ---

bool SimpleRTL8126::initPCIConfigSpace(IOPCIDevice *provider)
{
    IOByteCount pmCapOffset;
    UInt32 pcieLinkCap;
    UInt16 pcieLinkCtl;
    UInt16 cmdReg;
    UInt16 pmCap;
    bool result = false;
    
    /* Get vendor and device info. */
    pciDeviceData.vendor = provider->configRead16(kIOPCIConfigVendorID);
    pciDeviceData.device = provider->configRead16(kIOPCIConfigDeviceID);
    pciDeviceData.subsystem_vendor = provider->configRead16(kIOPCIConfigSubSystemVendorID);
    pciDeviceData.subsystem_device = provider->configRead16(kIOPCIConfigSubSystemID);
    
    /* Setup power management. */
    if (provider->extendedFindPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset)) {
        pmCap = provider->extendedConfigRead16(pmCapOffset + kIOPCIPMCapability);
        DebugLog("PCI power management capabilities: 0x%x.\n", pmCap);
        
        if (pmCap & kPCIPMCPMESupportFromD3Cold) {
            wolCapable = true;
            DebugLog("PME# from D3 (cold) supported.\n");
        }
        pciPMCtrlOffset = pmCapOffset + kIOPCIPMControl;
    } else {
        IOLog("PCI power management unsupported.\n");
    }
    provider->enablePCIPowerManagement(kPCIPMCSPowerStateD0);
    
    /* Get PCIe link information. */
    if (provider->extendedFindPCICapability(kIOPCIPCIExpressCapability, &pcieCapOffset)) {
        pcieLinkCap = provider->configRead32(pcieCapOffset + kIOPCIELinkCapability);
        pcieLinkCtl = provider->configRead16(pcieCapOffset + kIOPCIELinkControl);
        DebugLog("PCIe link capabilities: 0x%08x, link control: 0x%04x.\n", pcieLinkCap, pcieLinkCtl);
        
        if (linuxData.configASPM == 0) {
            IOLog("Disable PCIe ASPM.\n");
            provider->setASPMState(this, 0);
        } else {
            IOLog("Warning: Enable PCIe ASPM.\n");
            provider->setASPMState(this, kIOPCIELinkCtlASPM | kIOPCIELinkCtlClkPM);
            linuxData.configASPM = 1;
        }
    }
    /* Enable the device. */
    cmdReg    = provider->configRead16(kIOPCIConfigCommand);
    cmdReg  &= ~kIOPCICommandIOSpace;
    cmdReg    |= (kIOPCICommandBusMaster | kIOPCICommandMemorySpace | kIOPCICommandMemWrInvalidate);
    provider->configWrite16(kIOPCIConfigCommand, cmdReg);
    //provider->configWrite8(kIOPCIConfigLatencyTimer, 0x40);
    
    baseMap = provider->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress2, kIOMapInhibitCache);
    
    if (!baseMap) {
        IOLog("region #2 not an MMIO resource, aborting.\n");
        goto done;
    }
    baseAddr = reinterpret_cast<volatile void *>(baseMap->getVirtualAddress());
    linuxData.mmio_addr = baseAddr;
    result = true;
    
done:
    return result;
}

IOReturn SimpleRTL8126::setPowerStateWakeAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    SimpleRTL8126 *ethCtlr = OSDynamicCast(SimpleRTL8126, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;
    
    if (ethCtlr && ethCtlr->pciPMCtrlOffset) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
        val16 |= kPCIPMCSPowerStateD0;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

IOReturn SimpleRTL8126::setPowerStateSleepAction(OSObject *owner, void *arg1, void *arg2, void *arg3, void *arg4)
{
    SimpleRTL8126 *ethCtlr = OSDynamicCast(SimpleRTL8126, owner);
    IOPCIDevice *dev;
    UInt16 val16;
    UInt8 offset;

    if (ethCtlr && ethCtlr->pciPMCtrlOffset) {
        dev = ethCtlr->pciDevice;
        offset = ethCtlr->pciPMCtrlOffset;
        
        val16 = dev->extendedConfigRead16(offset);
        
        val16 &= ~(kPCIPMCSPowerStateMask | kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);

        if (ethCtlr->wolActive)
            val16 |= (kPCIPMCSPMEStatus | kPCIPMCSPMEEnable | kPCIPMCSPowerStateD3);
        else
            val16 |= kPCIPMCSPowerStateD3;
        
        dev->extendedConfigWrite16(offset, val16);
    }
    return kIOReturnSuccess;
}

/*
 * These functions have to be rewritten after every update
 * of the underlying Linux sources.
 */

IOReturn SimpleRTL8126::identifyChip()
{
    struct rtl8126_private *tp = &linuxData;
    IOReturn result = kIOReturnSuccess;
    UInt32 reg, val32;
    UInt32 version;

    val32 = ReadReg32(TxConfig);
    reg = val32 & 0x7c800000;
    version = val32 & 0x00700000;

    switch (reg) {
            case 0x64800000:
                    if (version == 0x00000000) {
                            tp->mcfg = CFG_METHOD_1;
                    } else if (version == 0x100000) {
                            tp->mcfg = CFG_METHOD_2;
                    } else if (version == 0x200000) {
                            tp->mcfg = CFG_METHOD_3;
                    } else {
                            tp->mcfg = CFG_METHOD_3;
                            tp->HwIcVerUnknown = TRUE;
                    }

                    tp->efuse_ver = EFUSE_SUPPORT_V4;
                    break;
            default:
                    IOLog("unknown chip version (%x)\n",reg);
                    tp->mcfg = CFG_METHOD_DEFAULT;
                    tp->HwIcVerUnknown = TRUE;
                    tp->efuse_ver = EFUSE_NOT_SUPPORT;
                    result = kIOReturnError;
                    break;
            }
    return result;
}

bool SimpleRTL8126::initRTL8126()
{
    struct rtl8126_private *tp = &linuxData;
    UInt32 i;
    UInt8 macAddr[MAC_ADDR_LEN];
    bool result = false;
    
    /* Identify chip attached to board. */
    if(identifyChip()) {
        IOLog("Unsupported chip found. Aborting...\n");
        goto done;
    }
    
    /* Setup EEE support. */
    tp->eee_adv_t = eeeCap = (MDIO_EEE_100TX | MDIO_EEE_1000T);
    
    tp->phy_reset_enable = rtl8126_xmii_reset_enable;
    tp->phy_reset_pending = rtl8126_xmii_reset_pending;

    tp->max_jumbo_frame_size = rtl_chip_info[tp->chipset].jumbo_frame_sz;
    
    rtl8126_get_bios_setting(tp);
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            //tp->HwSuppDashVer = 3;
            break;
        default:
            tp->HwSuppDashVer = 0;
            break;
    }

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwPkgDet = rtl8126_mac_ocp_read(tp, 0xDC00);
            tp->HwPkgDet = (tp->HwPkgDet >> 3) & 0x07;
            break;
    }

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppNowIsOobVer = 1;
            break;
    }

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwPcieSNOffset = 0x16C;
            break;
    }

    #ifdef ENABLE_REALWOW_SUPPORT
            rtl8126_get_realwow_hw_version(dev);
    #endif //ENABLE_REALWOW_SUPPORT

    if (linuxData.configASPM) {
        switch (tp->mcfg) {
            case CFG_METHOD_1:
            case CFG_METHOD_2:
            case CFG_METHOD_3:
                    tp->org_pci_offset_99 = csiFun0ReadByte(0x99);
                    tp->org_pci_offset_99 &= ~(BIT_5|BIT_6);
                    break;
        }
        switch (tp->mcfg) {
            case CFG_METHOD_1:
            case CFG_METHOD_2:
            case CFG_METHOD_3:
                    tp->org_pci_offset_180 = csiFun0ReadByte(0x22c);
                    break;
        }
    }
    tp->org_pci_offset_80 = pciDevice->configRead8(0x80);
    tp->org_pci_offset_81 = pciDevice->configRead8(0x81);
    tp->use_timer_interrupt = true;
/*
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        default:
            tp->use_timer_interrrupt = true;
            break;
    }
*/

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_V3;
            break;
        default:
            tp->HwSuppMagicPktVer = WAKEUP_MAGIC_PACKET_NOT_SUPPORT;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppLinkChgWakeUpVer = 3;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppD0SpeedUpVer = 1;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppCheckPhyDisableModeVer = 3;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppGigaForceMode = true;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppTxNoCloseVer = 3;
            break;
    }
    if (tp->HwSuppTxNoCloseVer > 0)
        tp->EnableTxNoClose = true;

    switch (tp->mcfg) {
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->RequireLSOPatch = true;
            break;
    }

    switch (tp->mcfg) {
            case CFG_METHOD_1:
                    tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_1;
                    break;
            case CFG_METHOD_2:
                    tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_2;
                    break;
            case CFG_METHOD_3:
                    tp->sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_3;
                    break;
            }

    if (tp->HwIcVerUnknown) {
            tp->NotWrRamCodeToMicroP = true;
            tp->NotWrMcuPatchCode = true;
    }

    switch (tp->mcfg) {
    case CFG_METHOD_3:
        if ((rtl8126_mac_ocp_read(tp, 0xD442) & BIT_5) &&
            (mdio_direct_read_phy_ocp(tp, 0xD068) & BIT_1)
            ) {
                tp->RequirePhyMdiSwapPatch = true;
        }
        break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppNumTxQueues = 2;
            tp->HwSuppNumRxQueues = 4;
            break;
        default:
            tp->HwSuppNumTxQueues = 1;
            tp->HwSuppNumRxQueues = 1;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppRssVer = 5;
            tp->HwSuppIndirTblEntries = 128;
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->HwSuppPtpVer = 2;
            break;
    }

    //init interrupt
    switch (tp->mcfg) {
    case CFG_METHOD_1:
            tp->HwSuppIsrVer = 2;
            break;
    case CFG_METHOD_2:
    case CFG_METHOD_3:
            tp->HwSuppIsrVer = 3;
            break;
    default:
            tp->HwSuppIsrVer = 1;
            break;
    }
    
    switch (tp->mcfg) {
            case CFG_METHOD_1:
                    tp->HwSuppIntMitiVer = 4;
                    break;
            case CFG_METHOD_2:
            case CFG_METHOD_3:
                    tp->HwSuppIntMitiVer = 5;
                    break;
            }

    tp->NicCustLedValue = ReadReg16(CustomLED);

    tp->wol_opts = rtl8126_get_hw_wol(tp);
    tp->wol_enabled = (tp->wol_opts) ? WOL_ENABLED : WOL_DISABLED;

    /* Set wake on LAN support. */
    wolCapable = (tp->wol_enabled == WOL_ENABLED);

    //tp->eee_enabled = eee_enable;
    tp->eee_adv_t = MDIO_EEE_1000T | MDIO_EEE_100TX;
    
    exitOOB();
    rtl8126_hw_init(tp);
    rtl8126_nic_reset(tp);
    
    /* Get production from EEPROM */
    rtl8126_eeprom_type(tp);

    if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
            rtl8126_set_eeprom_sel_low(tp);

    for (i = 0; i < MAC_ADDR_LEN; i++)
            macAddr[i] = ReadReg8(MAC0 + i);

    if(tp->mcfg == CFG_METHOD_1 ||
        tp->mcfg == CFG_METHOD_2 ||
        tp->mcfg == CFG_METHOD_3) {
            *(UInt32*)&macAddr[0] = ReadReg32(BACKUP_ADDR0_8126);
            *(UInt16*)&macAddr[4] = ReadReg16(BACKUP_ADDR1_8126);
    }

    if (is_valid_ether_addr((UInt8 *) macAddr)) {
        rtl8126_rar_set(tp, macAddr);
    } else {
        IOLog("Using fallback MAC.\n");
        rtl8126_rar_set(tp, fallBackMacAddr.bytes);
    }
    for (i = 0; i < MAC_ADDR_LEN; i++) {
        currMacAddr.bytes[i] = ReadReg8(MAC0 + i);
        origMacAddr.bytes[i] = currMacAddr.bytes[i]; /* keep the original MAC address */
    }
    IOLog("%s: (Chipset %d), %2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x\n",
          rtl_chip_info[tp->chipset].name, tp->chipset,
          origMacAddr.bytes[0], origMacAddr.bytes[1],
          origMacAddr.bytes[2], origMacAddr.bytes[3],
          origMacAddr.bytes[4], origMacAddr.bytes[5]);
    
    tp->cp_cmd = (ReadReg16(CPlusCmd) | RxChkSum);
    
    intrMaskRxTx = (SYSErr | LinkChg | RxDescUnavail | TxOK | RxOK);
    intrMaskTimer = (SYSErr | LinkChg | RxDescUnavail | PCSTimeout | RxOK);
    intrMaskPoll = (SYSErr | LinkChg);
    intrMask = intrMaskRxTx;
    
    /* Get the RxConfig parameters. */
    rxConfigReg = rtl_chip_info[tp->chipset].RCR_Cfg;
    rxConfigMask = rtl_chip_info[tp->chipset].RxConfigMask;
  
    /* Reset the tally counter. */
    WriteReg32(CounterAddrHigh, (statPhyAddr >> 32));
    WriteReg32(CounterAddrLow, (statPhyAddr & 0x00000000ffffffff) | CounterReset);

    rtl8126_disable_rxdvgate(tp);
    
#ifdef DEBUG
    
    if (wolCapable)
        IOLog("Device is WoL capable.\n");
    
#endif
    
    result = true;
    
done:
    return result;
}

void SimpleRTL8126::enableRTL8126()
{
    struct rtl8126_private *tp = &linuxData;
    
    setLinkStatus(kIONetworkLinkValid);
    
    intrMask = intrMaskRxTx;
    clear_bit(__POLL_MODE, &stateFlags);
    
    exitOOB();
    rtl8126_hw_init(tp);
    rtl8126_nic_reset(tp);
    rtl8126_powerup_pll(tp);
    rtl8126_hw_ephy_config(tp);
    configPhyHardware();
    setupRTL8126();
    
    setPhyMedium();
}

void SimpleRTL8126::disableRTL8126()
{
    struct rtl8126_private *tp = &linuxData;
    
    /* Disable all interrupts by clearing the interrupt mask. */
    WriteReg32(IMR0_8126, 0);
    WriteReg16(IntrStatus, ReadReg16(IntrStatus));

    rtl8126_nic_reset(tp);
    hardwareD3Para();
    powerDownPLL();
    
    if (test_and_clear_bit(__LINK_UP, &stateFlags)) {
        setLinkStatus(kIONetworkLinkValid);
        IOLog("Link down on en%u\n", netif->getUnitNumber());
    }
}

/* Reset the NIC in case a tx deadlock or a pci error occurred. timerSource and txQueue
 * are stopped immediately but will be restarted by checkLinkStatus() when the link has
 * been reestablished.
 */

void SimpleRTL8126::restartRTL8126()
{
    /* Stop output thread and flush txQueue */
    netif->stopOutputThread();
    netif->flushOutputQueue();
    
    clear_bit(__LINK_UP, &stateFlags);
    setLinkStatus(kIONetworkLinkValid);
    
    /* Reset NIC and cleanup both descriptor rings. */
    rtl8126_nic_reset(&linuxData);
/*
    if (rxInterrupt(netif, kNumRxDesc, NULL, NULL))
        netif->flushInputQueue();
*/
    clearRxTxRings();

    /* Reinitialize NIC. */
    enableRTL8126();
}

void SimpleRTL8126::setupRTL8126()
{
    struct rtl8126_private *tp = &linuxData;
    UInt32 i;
    UInt16 mac_ocp_data;
    
    WriteReg32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));
    
    rtl8126_nic_reset(tp);
    
    WriteReg8(Cfg9346, ReadReg8(Cfg9346) | Cfg9346_Unlock);
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            WriteReg8(0xF1, ReadReg8(0xF1) & ~BIT_7);
            WriteReg8(Config2, ReadReg8(Config2) & ~BIT_7);
            WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
            break;
    }

    //clear io_rdy_l23
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            WriteReg8(Config3, ReadReg8(Config3) & ~BIT_1);
            break;
    }

    switch (tp->HwSuppIntMitiVer) {
            case 3:
            case 6:
                    //IntMITI_0-IntMITI_31
                    for (i=0xA00; i<0xB00; i+=4)
                            RTL_W32(tp, i, 0x0000);
                    break;
            case 4:
            case 5:
                    //IntMITI_0-IntMITI_15
                    for (i = 0xA00; i < 0xA80; i += 4)
                            RTL_W32(tp, i, 0x0000);

                    if (tp->HwSuppIntMitiVer == 5)
                            WriteReg8(INT_CFG0_8126, ReadReg8(INT_CFG0_8126) &
                                   ~(INT_CFG0_TIMEOUT0_BYPASS_8126 |
                                     INT_CFG0_MITIGATION_BYPASS_8126 |
                                     INT_CFG0_RDU_BYPASS_8126));
                    else
                            WriteReg8(INT_CFG0_8126, ReadReg8(INT_CFG0_8126) &
                                   ~(INT_CFG0_TIMEOUT0_BYPASS_8126 | INT_CFG0_MITIGATION_BYPASS_8126));

                    RTL_W16(tp, INT_CFG1_8126, 0x0000);
                    break;
            }

    //keep magic packet only
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xC0B6);
            mac_ocp_data &= BIT_0;
            rtl8126_mac_ocp_write(tp, 0xC0B6, mac_ocp_data);
            break;
    }
    /* Fill tally counter address. */
    WriteReg32(CounterAddrHigh, (statPhyAddr >> 32));
    WriteReg32(CounterAddrLow, (statPhyAddr & 0x00000000ffffffff));

    /* Setup the descriptor rings. */
    txTailPtr0 = txClosePtr0 = 0;
    txNextDescIndex = txDirtyDescIndex = 0;
    txNumFreeDesc = kNumTxDesc;
    rxNextDescIndex = 0;
    
    WriteReg32(TxDescStartAddrLow, (txPhyAddr & 0x00000000ffffffff));
    WriteReg32(TxDescStartAddrHigh, (txPhyAddr >> 32));
    WriteReg32(RxDescAddrLow, (rxPhyAddr & 0x00000000ffffffff));
    WriteReg32(RxDescAddrHigh, (rxPhyAddr >> 32));

    /* Set DMA burst size and Interframe Gap Time */
    WriteReg32(TxConfig, (TX_DMA_BURST_unlimited << TxDMAShift) |
            (InterFrameGap << TxInterFrameGapShift));

    if (tp->EnableTxNoClose)
            WriteReg32(TxConfig, (ReadReg32(TxConfig) | BIT_6));
    
    if (tp->mcfg == CFG_METHOD_1 ||
        tp->mcfg == CFG_METHOD_2 ||
        tp->mcfg == CFG_METHOD_3 ) {
        set_offset70F(tp, 0x27);
        setOffset79(0x40);

        WriteReg16(0x382, 0x221B);

        /* Disable RSS. */
        WriteReg8(RSS_CTRL_8126, 0x00);
        WriteReg16(Q_NUM_CTRL_8126, 0x0000);

        WriteReg8(Config1, ReadReg8(Config1) & ~0x10);

        rtl8126_mac_ocp_write(tp, 0xC140, 0xFFFF);
        rtl8126_mac_ocp_write(tp, 0xC142, 0xFFFF);

        /*
         * Disabling the new tx descriptor format seems to prevent
         * tx timeouts when using TSO.
         */
        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEB58);
        if (tp->mcfg == CFG_METHOD_2 || tp->mcfg == CFG_METHOD_3)
                mac_ocp_data &= ~(BIT_0 | BIT_1);
        mac_ocp_data |= (BIT_0);
        rtl8126_mac_ocp_write(tp, 0xEB58, mac_ocp_data);

        if (tp->HwSuppRxDescType == RX_DESC_RING_TYPE_4) {
                if (tp->InitRxDescType == RX_DESC_RING_TYPE_4)
                        WriteReg8(0xd8, ReadReg8(0xd8) |
                                EnableRxDescV4_0);
                else
                        WriteReg8(0xd8, ReadReg8(0xd8) &
                                ~EnableRxDescV4_0);
        }

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE614);
        mac_ocp_data &= ~(BIT_10 | BIT_9 | BIT_8);
        mac_ocp_data |= ((4 & 0x07) << 8);
        rtl8126_mac_ocp_write(tp, 0xE614, mac_ocp_data);

        
        //rtl8126_set_tx_q_num(tp, tp->HwSuppNumTxQueues);
        
        /* Set tx queue num to one. */
        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE63E);
        mac_ocp_data &= ~(BIT_5 | BIT_4);
        mac_ocp_data |= ((0x02 & 0x03) << 4);
        rtl8126_mac_ocp_write(tp, 0xE63E, mac_ocp_data);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xC0B4);
        mac_ocp_data |= (BIT_3 | BIT_2);
        rtl8126_mac_ocp_write(tp, 0xC0B4, mac_ocp_data);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEB6A);
        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
        mac_ocp_data |= (BIT_5 | BIT_4 | BIT_1 | BIT_0);
        rtl8126_mac_ocp_write(tp, 0xEB6A, mac_ocp_data);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEB50);
        mac_ocp_data &= ~(BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5);
        mac_ocp_data |= (BIT_6);
        rtl8126_mac_ocp_write(tp, 0xEB50, mac_ocp_data);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE056);
        mac_ocp_data &= ~(BIT_7 | BIT_6 | BIT_5 | BIT_4);
        //mac_ocp_data |= (BIT_4 | BIT_5);
        rtl8126_mac_ocp_write(tp, 0xE056, mac_ocp_data);

        WriteReg8(TDFNR, 0x10);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xE040);
        mac_ocp_data &= ~(BIT_12);
        rtl8126_mac_ocp_write(tp, 0xE040, mac_ocp_data);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEA1C);
        mac_ocp_data &= ~(BIT_1 | BIT_0);
        mac_ocp_data |= (BIT_0);
        rtl8126_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);

        rtl8126_mac_ocp_write(tp, 0xE0C0, 0x4000);
        
        rtl8126_set_mac_ocp_bit(tp, 0xE052, (BIT_6 | BIT_5));
        rtl8126_clear_mac_ocp_bit(tp, 0xE052, BIT_3 | BIT_7);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xD430);
        mac_ocp_data &= ~(BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
        mac_ocp_data |= 0x45F;
        rtl8126_mac_ocp_write(tp, 0xD430, mac_ocp_data);

        //rtl8126_mac_ocp_write(tp, 0xE0C0, 0x4F87);
        if (!tp->DASH)
                WriteReg8(0xD0, ReadReg8(0xD0) | BIT_6 | BIT_7);
        else
                WriteReg8(0xD0, ReadReg8(0xD0) & ~(BIT_6 | BIT_7));
        
        rtl8126_disable_eee_plus(tp);

        mac_ocp_data = rtl8126_mac_ocp_read(tp, 0xEA1C);
        mac_ocp_data &= ~(BIT_2);
        if (tp->mcfg == CFG_METHOD_2 || tp->mcfg == CFG_METHOD_3)
                mac_ocp_data &= ~(BIT_9 | BIT_8);
        rtl8126_mac_ocp_write(tp, 0xEA1C, mac_ocp_data);
        
        SetMcuAccessRegBit(tp, 0xEB54, BIT_0);
        udelay(1);
        ClearMcuAccessRegBit(tp, 0xEB54, BIT_0);
        WriteReg16(0x1880, RTL_R16(tp, 0x1880) & ~(BIT_4 | BIT_5));
    }
    //other hw parameters
    rtl8126_hw_clear_timer_int(tp);

    rtl8126_hw_clear_int_miti(tp);

    rtl8126_enable_exit_l1_mask(tp);

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_mac_ocp_write(tp, 0xE098, 0xC302);
            break;
    }

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            if (linuxData.configASPM) {
                initPCIOffset99();
            }
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            if (linuxData.configASPM) {
                rtl8126_init_pci_offset_180(tp);
            }
            break;
    }

    tp->cp_cmd &= ~(EnableBist | Macdbgo_oe | Force_halfdup |
                    Force_rxflow_en | Force_txflow_en | Cxpl_dbg_sel |
                    ASF | Macdbgo_sel);

    WriteReg16(CPlusCmd, tp->cp_cmd);

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3: {
            int timeout;
            for (timeout = 0; timeout < 10; timeout++) {
                if ((rtl8126_ocp_read(tp, 0x124, 1) & BIT_0)==0)
                    break;
                mdelay(1);
            }
        }
        break;
    }
    /*
     * Make sure that a packet fits into one buffer or there
     * will be trouble.
     */
    WriteReg16(RxMaxSize, rxBufferSize - 1);
    
    rtl8126_disable_rxdvgate(tp);

    /* Set receiver mode. */
    setMulticastMode(test_bit(__M_CAST, &stateFlags));

    #ifdef ENABLE_DASH_SUPPORT
            if (tp->DASH && !tp->dash_printer_enabled)
                    NICChkTypeEnableDashInterrupt(tp);
    #endif

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            if (linuxData.configASPM) {
                WriteReg8(Config5, ReadReg8(Config5) | BIT_0);
                WriteReg8(Config2, ReadReg8(Config2) | BIT_7);
            } else {
                WriteReg8(Config2, ReadReg8(Config2) & ~BIT_7);
                WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
            }
            break;
    }

    WriteReg8(Cfg9346, ReadReg8(Cfg9346) & ~Cfg9346_Unlock);
    
    /* Enable all known interrupts by setting the interrupt mask. */
    WriteReg32(IMR0_8126, intrMask);

    udelay(10);
}

void SimpleRTL8126::setPhyMedium()
{
    struct rtl8126_private *tp = netdev_priv(&linuxData);
    int auto_nego = 0;
    int giga_ctrl = 0;
    int ctrl_2500 = 0;
    
    if (speed != SPEED_5000 && speed != SPEED_2500 && (speed != SPEED_1000) &&
        (speed != SPEED_100) && (speed != SPEED_10)) {
        duplex = DUPLEX_FULL;
        autoneg = AUTONEG_ENABLE;
    }
    /* Enable or disable EEE support according to selected medium. */
    if ((linuxData.eee_adv_t != 0) && (autoneg == AUTONEG_ENABLE)) {
        rtl8126_enable_eee(tp);
        DebugLog("Enable EEE support.\n");
    } else {
        rtl8126_disable_eee(tp);
        DebugLog("Disable EEE support.\n");
    }
    //Disable Giga Lite
    ClearEthPhyOcpBit(tp, 0xA428, BIT_9);
    ClearEthPhyOcpBit(tp, 0xA5EA, BIT_0 | BIT_1 | BIT_2);

    giga_ctrl = rtl8126_mdio_read(tp, MII_CTRL1000);
    giga_ctrl &= ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
    
    ctrl_2500 = mdio_direct_read_phy_ocp(tp, 0xA5D4);
    ctrl_2500 &= ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL);
    
    auto_nego = rtl8126_mdio_read(tp, MII_ADVERTISE);
    auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                   ADVERTISE_100HALF | ADVERTISE_100FULL |
                   ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

    if (autoneg == AUTONEG_ENABLE) {
        /* The default medium has been selected. */
        auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);
        giga_ctrl |= (ADVERTISE_1000FULL | ADVERTISE_1000HALF);
        ctrl_2500 |= (RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL);
    } else if (speed == SPEED_5000) {
        ctrl_2500 |= RTK_ADVERTISE_5000FULL;
    } else if (speed == SPEED_2500) {
        ctrl_2500 |= RTK_ADVERTISE_2500FULL;
    } else if (speed == SPEED_1000) {
        if (duplex == DUPLEX_HALF) {
            giga_ctrl |= ADVERTISE_1000HALF;
        } else {
            giga_ctrl |= ADVERTISE_1000FULL;
        }
    } else if (speed == SPEED_100) {
        if (duplex == DUPLEX_HALF) {
            auto_nego |= ADVERTISE_100HALF;
        } else {
            auto_nego |=  ADVERTISE_100FULL;
        }
    } else { /* speed == SPEED_10 */
        if (duplex == DUPLEX_HALF) {
            auto_nego |= ADVERTISE_10HALF;
        } else {
            auto_nego |= ADVERTISE_10FULL;
        }
    }
    /* Set flow control support. */
    if (flowCtl == kFlowControlOn)
        auto_nego |= (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);

    tp->phy_auto_nego_reg = auto_nego;
    tp->phy_1000_ctrl_reg = giga_ctrl;

    tp->phy_2500_ctrl_reg = ctrl_2500;

    rtl8126_mdio_write(tp, 0x1f, 0x0000);
    rtl8126_mdio_write(tp, MII_ADVERTISE, auto_nego);
    rtl8126_mdio_write(tp, MII_CTRL1000, giga_ctrl);
    mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);
    rtl8126_phy_restart_nway(tp);
    mdelay(20);

    tp->autoneg = AUTONEG_ENABLE;
    tp->speed = speed;
    tp->duplex = duplex;
}

/* Set PCI configuration space offset 0x79 to setting. */

void SimpleRTL8126::setOffset79(UInt8 setting)
{
    UInt8 deviceControl;
    
    DebugLog("setOffset79() ===>\n");
    
    if (!(linuxData.hwoptimize & HW_PATCH_SOC_LAN)) {
        deviceControl = pciDevice->configRead8(0x79);
        deviceControl &= ~0x70;
        deviceControl |= setting;
        pciDevice->configWrite8(0x79, deviceControl);
    }
    
    DebugLog("setOffset79() <===\n");
}

UInt8 SimpleRTL8126::csiFun0ReadByte(UInt32 addr)
{
    struct rtl8126_private *tp = &linuxData;
    UInt8 retVal = 0;
    
    if (tp->mcfg == CFG_METHOD_DEFAULT) {
        retVal = pciDevice->configRead8(addr);
    } else {
        UInt32 tmpUlong;
        UInt8 shiftByte;
        
        shiftByte = addr & (0x3);
        tmpUlong = rtl8126_csi_other_fun_read(&linuxData, 0, addr);
        tmpUlong >>= (8 * shiftByte);
        retVal = (UInt8)tmpUlong;
    }
    udelay(20);

    return retVal;
}

void SimpleRTL8126::csiFun0WriteByte(UInt32 addr, UInt8 value)
{
    struct rtl8126_private *tp = &linuxData;

    if (tp->mcfg == CFG_METHOD_DEFAULT) {
        pciDevice->configWrite8(addr, value);
    } else {
        UInt32 tmpUlong;
        UInt16 regAlignAddr;
        UInt8 shiftByte;
        
        regAlignAddr = addr & ~(0x3);
        shiftByte = addr & (0x3);
        tmpUlong = rtl8126_csi_other_fun_read(&linuxData, 0, regAlignAddr);
        tmpUlong &= ~(0xFF << (8 * shiftByte));
        tmpUlong |= (value << (8 * shiftByte));
        rtl8126_csi_other_fun_write(&linuxData, 0, regAlignAddr, tmpUlong );
    }
    udelay(20);
}

void SimpleRTL8126::enablePCIOffset99()
{
    struct rtl8126_private *tp = &linuxData;
    u32 csi_tmp;
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            csiFun0WriteByte(0x99, linuxData.org_pci_offset_99);
            break;
    }
    
    switch (linuxData.mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            csi_tmp = rtl8126_mac_ocp_read(tp, 0xE032);
            csi_tmp &= ~(BIT_0 | BIT_1);
            
            if (!(tp->org_pci_offset_99 & (BIT_5 | BIT_6)))
                    csi_tmp |= BIT_1;
            
            if (!(tp->org_pci_offset_99 & BIT_2))
                    csi_tmp |= BIT_0;
            
            rtl8126_mac_ocp_write(tp, 0xE032, csi_tmp);
            break;
    }
}

void SimpleRTL8126::disablePCIOffset99()
{
    struct rtl8126_private *tp = &linuxData;

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_mac_ocp_write(tp, 0xE032,  rtl8126_mac_ocp_read(tp, 0xE032) & ~(BIT_0 | BIT_1));
            break;
    }

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            csiFun0WriteByte(0x99, 0x00);
            break;
    }
}

void SimpleRTL8126::initPCIOffset99()
{
    struct rtl8126_private *tp = &linuxData;
    u32 csi_tmp;

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_mac_ocp_write(tp, 0xCDD0, 0x9003);
            csi_tmp = rtl8126_mac_ocp_read(tp, 0xE034);
            csi_tmp |= (BIT_15 | BIT_14);
            rtl8126_mac_ocp_write(tp, 0xE034, csi_tmp);
            rtl8126_mac_ocp_write(tp, 0xCDD2, 0x889C);
            rtl8126_mac_ocp_write(tp, 0xCDD8, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDD4, 0x8C30);
            rtl8126_mac_ocp_write(tp, 0xCDDA, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDD6, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDDC, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDE8, 0x883E);
            rtl8126_mac_ocp_write(tp, 0xCDEA, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDEC, 0x889C);
            rtl8126_mac_ocp_write(tp, 0xCDEE, 0x9003);
            rtl8126_mac_ocp_write(tp, 0xCDF0, 0x8C09);
            rtl8126_mac_ocp_write(tp, 0xCDF2, 0x9003);
            csi_tmp = rtl8126_mac_ocp_read(tp, 0xE032);
            csi_tmp |= (BIT_14);
            rtl8126_mac_ocp_write(tp, 0xE032, csi_tmp);
            csi_tmp = rtl8126_mac_ocp_read(tp, 0xE0A2);
            csi_tmp |= (BIT_0);
            rtl8126_mac_ocp_write(tp, 0xE0A2, csi_tmp);
            break;
    }
    enablePCIOffset99();
}

void SimpleRTL8126::setPCI99_180ExitDriverPara()
{
    struct rtl8126_private *tp = &linuxData;
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
                rtl8126_issue_offset_99_event(tp);
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            disablePCIOffset99();
            break;
    }
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_disable_pci_offset_180(tp);
            break;
    }
}

void SimpleRTL8126::hardwareD3Para()
{
    struct rtl8126_private *tp = &linuxData;
    
    /* Set RxMaxSize register */
    WriteReg16(RxMaxSize, RX_BUF_SIZE);
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            WriteReg8(0xF1, ReadReg8(0xF1) & ~BIT_7);
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) | Cfg9346_Unlock);
            WriteReg8(Config2, ReadReg8(Config2) & ~BIT_7);
            WriteReg8(Config5, ReadReg8(Config5) & ~BIT_0);
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) & ~Cfg9346_Unlock);
            break;
    }
    rtl8126_disable_exit_l1_mask(tp);

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_mac_ocp_write(tp, 0xEA18, 0x0064);
            break;
    }
    setPCI99_180ExitDriverPara();

    /*disable ocp phy power saving*/
    if (tp->mcfg == CFG_METHOD_1 || tp->mcfg == CFG_METHOD_2 ||
        tp->mcfg == CFG_METHOD_3) {
            rtl8126_disable_ocp_phy_power_saving(tp);
    }
    rtl8126_disable_rxdvgate(tp);
}

UInt16 SimpleRTL8126::getEEEMode()
{
    struct rtl8126_private *tp = &linuxData;
    UInt16 eee = 0;
    UInt16 sup, adv, lpa, ena;

    if (eeeCap) {
        /* Get supported EEE. */
        sup = mdio_direct_read_phy_ocp(tp, 0xA5C4);
        DebugLog("EEE supported: %u\n", sup);

        /* Get advertisement EEE. */
        adv = mdio_direct_read_phy_ocp(tp, 0xA5D0);
        DebugLog("EEE advertised: %u\n", adv);

        /* Get LP advertisement EEE. */
        lpa = mdio_direct_read_phy_ocp(tp, 0xA5D2);
        DebugLog("EEE link partner: %u\n", lpa);

        ena = rtl8126_mac_ocp_read(tp, 0xE040);
        ena &= BIT_1 | BIT_0;
        DebugLog("EEE enabled: %u\n", ena);

        eee = (sup & adv & lpa);
    }
    return eee;
}
void SimpleRTL8126::exitOOB()
{
    struct rtl8126_private *tp = &linuxData;
    UInt16 data16;
    
    WriteReg32(RxConfig, ReadReg32(RxConfig) & ~(AcceptErr | AcceptRunt | AcceptBroadcast | AcceptMulticast | AcceptMyPhys |  AcceptAllPhys));
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            //rtl8126_dash2_disable_txrx(tp);
            break;
    }

    //Disable realwow  function
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_mac_ocp_write(tp, 0xC0BC, 0x00FF);
            break;
    }

    rtl8126_nic_reset(tp);

    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            rtl8126_disable_now_is_oob(tp);

            data16 = rtl8126_mac_ocp_read(tp, 0xE8DE) & ~BIT_14;
            rtl8126_mac_ocp_write(tp, 0xE8DE, data16);
            rtl8126_wait_ll_share_fifo_ready(tp);

            rtl8126_mac_ocp_write(tp, 0xC0AA, 0x07D0);
            rtl8126_mac_ocp_write(tp, 0xC0A6, 0x01B5);
            rtl8126_mac_ocp_write(tp, 0xC01E, 0x5555);

            rtl8126_wait_ll_share_fifo_ready(tp);
            break;
    }

    //wait ups resume (phy state 2)
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            if (rtl8126_is_ups_resume(tp)) {
                rtl8126_wait_phy_ups_resume(tp, 2);
                rtl8126_clear_ups_resume_bit(tp);
                rtl8126_clear_phy_ups_reg(tp);
            }
            break;
    };
    tp->phy_reg_anlpar = 0;
}

void SimpleRTL8126::powerDownPLL()
{
    struct rtl8126_private *tp = &linuxData;

    if (tp->wol_enabled == WOL_ENABLED || tp->DASH || tp->EnableKCPOffload) {
        int auto_nego;
        int giga_ctrl;
        u16 anlpar;

        rtl8126_set_hw_wol(tp, tp->wol_opts);

        if (tp->mcfg == CFG_METHOD_1 || tp->mcfg == CFG_METHOD_2 ||
            tp->mcfg == CFG_METHOD_3) {
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) | Cfg9346_Unlock);
            WriteReg8(Config2, ReadReg8(Config2) | PMSTS_En);
            WriteReg8(Cfg9346, ReadReg8(Cfg9346) & ~Cfg9346_Unlock);
        }

        rtl8126_mdio_write(tp, 0x1F, 0x0000);
        auto_nego = rtl8126_mdio_read(tp, MII_ADVERTISE);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                       | ADVERTISE_100HALF | ADVERTISE_100FULL);

        if (test_bit(__LINK_UP, &stateFlags))
            anlpar = tp->phy_reg_anlpar;
        else
            anlpar = rtl8126_mdio_read(tp, MII_LPA);

        if (anlpar & (LPA_10HALF | LPA_10FULL))
            auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL);
        else
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

        if (tp->DASH)
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);

            giga_ctrl = rtl8126_mdio_read(tp, MII_CTRL1000) & ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
            rtl8126_mdio_write(tp, MII_ADVERTISE, auto_nego);
            rtl8126_mdio_write(tp, MII_CTRL1000, giga_ctrl);
        
            if (tp->mcfg == CFG_METHOD_1 || tp->mcfg == CFG_METHOD_2 ||
                tp->mcfg == CFG_METHOD_3) {
                int ctrl_2500;

                ctrl_2500 = mdio_direct_read_phy_ocp(tp, 0xA5D4);
                ctrl_2500 &= ~(RTK_ADVERTISE_2500FULL | RTK_ADVERTISE_5000FULL);
                mdio_direct_write_phy_ocp(tp, 0xA5D4, ctrl_2500);
            }
            rtl8126_phy_restart_nway(tp);

            WriteReg32(RxConfig, ReadReg32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);

            return;
        }

        if (tp->DASH)
                return;

        rtl8126_phy_power_down(tp);

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            WriteReg8(PMCH, ReadReg8(PMCH) & ~BIT_7);
            break;
        }

        switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            WriteReg8(0xF2, ReadReg8(0xF2) & ~BIT_6);
            break;
        }
}

void SimpleRTL8126::configPhyHardware()
{
    struct rtl8126_private *tp = &linuxData;

    if (tp->resume_not_chg_speed) return;
    
    tp->phy_reset_enable(tp);
    
    if (HW_DASH_SUPPORT_TYPE_3(tp) && tp->HwPkgDet == 0x06) return;
    
    rtl8126_set_hw_phy_before_init_phy_mcu(tp);
    
    rtl8126_init_hw_phy_mcu(tp);
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
            configPhyHardware8126a1();
            break;
        case CFG_METHOD_2:
            configPhyHardware8126a2();
            break;
        case CFG_METHOD_3:
            configPhyHardware8126a3();
            break;
    }
    
    //legacy force mode(Chap 22)
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
        default:
            rtl8126_clear_eth_phy_ocp_bit(tp, 0xA5B4, BIT_15);
            break;
    }
    
    /*ocp phy power saving*/
    /*
     if (aspm) {
     if (tp->mcfg == CFG_METHOD_2 || tp->mcfg == CFG_METHOD_3)
     rtl8126_enable_ocp_phy_power_saving(dev);
     }
     */
    
    rtl8126_mdio_write(tp, 0x1F, 0x0000);
    
    if (HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp)) {
        if (tp->eee_enabled == 1)
            rtl8126_enable_eee(tp);
        else
            rtl8126_disable_eee(tp);
    }
}

void SimpleRTL8126::configPhyHardware8126a1()
{
    struct rtl8126_private *tp = &linuxData;

    rtl8126_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);


    if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
            rtl8126_enable_phy_aldps(tp);
}

void SimpleRTL8126::configPhyHardware8126a2()
{
    struct rtl8126_private *tp = &linuxData;
    
    rtl8126_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80BF);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0xED00);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80CD);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x1000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80D1);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0xC800);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80D4);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0xC800);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80E1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x10CC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x80E5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x4F0C);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8387);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x4700);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA80C,
                                          BIT_7 | BIT_6,
                                          BIT_7);


    rtl8126_clear_eth_phy_ocp_bit(tp, 0xAC90, BIT_4);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xAD2C, BIT_15);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8321);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xACF8, (BIT_3 | BIT_2));
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8183);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x5900);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xAD94, BIT_5);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xA654, BIT_11);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xB648, BIT_14);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x839E);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x2F00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83F2);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0800);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xADA0, BIT_1);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80F3);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x9900);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8126);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xC100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x893A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x8080);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8647);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xE600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x862C);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1200);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864A);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xE600);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80A0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xBCBC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x805E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0xBCBC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8056);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3077);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8058);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5A00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8098);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3077);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x809A);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5A00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8052);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3733);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8094);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x3733);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x807F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C75);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x803D);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x7C75);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8036);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8078);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8031);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3300);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8073);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3300);


    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xAE06,
                                          0xFC00,
                                          0x7C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89D1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0004);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FBD);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x0A00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FBE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0D09);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89CD);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0F0F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89CF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0F0F);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83A4);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83A6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83C0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83C2);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8414);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8416);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83F8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x83FA);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x6601);


    rtl8126_set_phy_mcu_patch_request(tp);

    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xBD96,
                                          0x1F00,
                                          0x1000);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xBF1C,
                                          0x0007,
                                          0x0007);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xBFBE, BIT_15);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xBF40,
                                          0x0380,
                                          0x0280);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xBF90,
                                          BIT_7,
                                          (BIT_6 | BIT_5));
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xBF90,
                                          BIT_4,
                                          BIT_3 | BIT_2);

    rtl8126_clear_phy_mcu_patch_request(tp);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x843B);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x2000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x843D);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x2000);


    rtl8126_clear_eth_phy_ocp_bit(tp, 0xB516, 0x7F);


    rtl8126_clear_eth_phy_ocp_bit(tp, 0xBF80, (BIT_5 | BIT_4));


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8188);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0044);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00A8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00D6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00EC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00F6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00BC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0058);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x002A);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8015);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0800);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFD);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFF);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x7F00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFB);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE9);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0002);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FEF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x00A5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FF1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0106);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0102);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE3);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0400);


    rtl8126_set_eth_phy_ocp_bit(tp, 0xA654, BIT_11);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0XA65A, (BIT_1 | BIT_0));

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xAC3A, 0x5851);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0XAC3C,
                                          BIT_15 | BIT_14 | BIT_12,
                                          BIT_13);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xAC42,
                                          BIT_9,
                                          BIT_8 | BIT_7 | BIT_6);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xAC3E, BIT_15 | BIT_14 | BIT_13);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xAC42, BIT_5 | BIT_4 | BIT_3);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xAC42,
                                          BIT_1,
                                          BIT_2 | BIT_0);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xAC1A, 0x00DB);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xADE4, 0x01B5);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xAD9C, BIT_11 | BIT_10);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814B);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814D);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814F);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0B00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8142);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8144);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8150);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8118);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811A);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811C);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0500);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x810F);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8111);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811D);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);

    rtl8126_set_eth_phy_ocp_bit(tp, 0xAC36, BIT_12);
    rtl8126_clear_eth_phy_ocp_bit(tp, 0xAD1C, BIT_8);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xADE8,
                                          0xFFC0,
                                          0x1400);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864B);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x9D00);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8F97);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x003F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3F02);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x023C);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3B0A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);


    rtl8126_set_eth_phy_ocp_bit(tp, 0xAD9C, BIT_5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8122);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82C8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80EF);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82A0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0006);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x001A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03D8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0023);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0054);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0322);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x00DD);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03AB);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03DC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0027);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F9);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0012);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0001);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8018);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xA438, BIT_13);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FE4);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0000);

    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB54C,
                                          0xFFC0,
                                          0x3700);


    if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
            rtl8126_enable_phy_aldps(tp);
}

void SimpleRTL8126::configPhyHardware8126a3()
{
    struct rtl8126_private *tp = &linuxData;

    rtl8126_set_eth_phy_ocp_bit(tp, 0xA442, BIT_11);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8183);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x5900);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xA654, BIT_11);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xB648, BIT_14);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xAD2C, BIT_15);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xAD94, BIT_5);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xADA0, BIT_1);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xAE06,
                                          BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10,
                                          BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8647);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0xE600);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8036);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8078);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x3000);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x89E9);
    rtl8126_set_eth_phy_ocp_bit(tp, 0xB87E, 0xFF00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFD);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFE);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0200);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8FFF);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0400);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8018);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x7700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8F9C);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0005);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x00ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0502);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0B00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0xD401);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8FA8);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xA438,
                                          0xFF00,
                                          0x2900);


    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814B);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814D);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x814F);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0B00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8142);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8144);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8150);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8118);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811A);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0700);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811C);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0500);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x810F);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8111);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x811D);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0100);


    rtl8126_set_eth_phy_ocp_bit(tp, 0xAD1C, BIT_8);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xADE8,
                                          BIT_15 | BIT_14 | BIT_13 | BIT_12 | BIT_11 | BIT_10 | BIT_9 | BIT_8 | BIT_7 | BIT_6,
                                          BIT_12 | BIT_10);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x864B);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x9D00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x862C);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x1200);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA436, 0x8566);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x003F);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3F02);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x023C);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x3B0A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x1C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xA438, 0x0000);


    rtl8126_set_eth_phy_ocp_bit(tp, 0xAD9C, BIT_5);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8122);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82C8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0049);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03B8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F7);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0021);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000B);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0009);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FF);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);

    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x80EF);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x0C00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x82A0);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03FE);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03ED);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0006);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x001A);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03D8);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0023);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0054);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0322);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x00DD);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03AB);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03DC);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0027);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x000E);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03E5);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F9);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0012);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x0001);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87E, 0x03F1);


    rtl8126_set_eth_phy_ocp_bit(tp, 0xA430, BIT_1 | BIT_0);


    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB54C,
                                          0xFFC0,
                                          0x3700);


    rtl8126_set_eth_phy_ocp_bit(tp, 0xB648, BIT_6);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x8082);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5D00);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x807C);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5000);
    rtl8126_mdio_direct_write_phy_ocp(tp, 0xB87C, 0x809D);
    rtl8126_clear_and_set_eth_phy_ocp_bit(tp,
                                          0xB87E,
                                          0xFF00,
                                          0x5000);


    if (aspm && HW_HAS_WRITE_PHY_MCU_RAM_CODE(tp))
            rtl8126_enable_phy_aldps(tp);
}
