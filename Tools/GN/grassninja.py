#python3
#
#  _____                   _   _ _       _       _    _           _   
# / ____|                 | \ | (_)     (_)     | |  | |         | |  
#| |  __ _ __ __ _ ___ ___|  \| |_ _ __  _  __ _| |__| | ___  ___| |_ 
#| | |_ | '__/ _` / __/ __| . ` | | '_ \| |/ _` |  __  |/ _ \/ __| __|
#| |__| | | | (_| \__ \__ \ |\  | | | | | | (_| | |  | | (_) \__ \ |_ 
# \_____|_|  \__,_|___/___/_| \_|_|_| |_| |\__,_|_|  |_|\___/|___/\__|
#                                      _/ |                           
#                                     |__/ 
# GrassNinja python driver
# HWTE Yan Li, 2020/10/10
# yan.li@apple.com
# DO NOT DISTRIBUTE THE CODE TO THE 3RD PARTY

# v0.2 - HWTE Grace Wang, 2020/12/21
# gracewang@apple.com

# v1.0 - HWTE Yan Li, 2020/12/25
# Reorganize the grassninja host class architecture
# Add more test required functions

# v1.1 - Add timeout settings on locust host and device before comms, HWTE Yan Li, 2021/06/07

import serial
import argparse
import time
import struct
import random
import threading
import pyftdi
from pyftdi.ftdi import Ftdi
from pyftdi.i2c import I2cController, I2cIOError
import pyftdi.gpio as gpio
from locust import locust


LAGUNA_I2C_7BIT_ADDR = 0x11
DEBUG_FLAG = True


class grassNinjaHost():
    def __init__(self, uart,i2cPortName): 
        if int(pyftdi.__version__[2:4]) >=53:  # Only after version 53 the set_latency_timer function is OK2USE 
            FTDIDev = Ftdi() 
            FTDIDev.open(vendor=1027,product=24592) 
            FTDIDev.set_latency_timer(4) # Set the RX latency to 4ms, default is 12ms; Improve b2p UART throughput
        self.debug = False
        self.i2c = i2c(i2cPortName, self.debug) # i2c bus on GN host
        self.b2pUart = uart # uart on GN host
        self.ioexp = tca9555(self.i2c) # i2c connection to io expander
        self.parrot = cd2e22(self.i2c) # i2c connection to parrot
        self.locustI2C = locustI2C(self.i2c) # Direct access to Locust host register by using i2c bus
        self.locustB2P = locust(self.b2pUart,debug=self.debug) # Access to locust host, slave, laguna and durant

    #++++++++++++++++++++ 
    # One time functions - Use the function only once after power on GrassNinja host
    #++++++++++++++++++++ 
    def resetAll(self):
        '''
        Initialize the GrassNinjaHost board, including Parrot, Locust host and IOExpander
        Use this function only once for the whole test
        '''
        print("====== Resetting everything ======")
        self.b2pUart.flush()
        self.ioexp.reset()
        self.parrot.reset()
        self.locustB2P.sendReset("HostLocust", reset_mode=0x01)
        self.locustI2C.reset() # Bug fix: Move locust host configuration below host reset; Otherwise TX settings can't be enabled
        self.locustB2P.sendPing("HostLocust")

    def closePorts(self):
        self.b2pUart.flush()
        self.b2pUart.close()
        self.i2c.i2cSlave.terminate()

    def setTimeouts(self):
        '''
        Setting mid-remote-packet and total-remote-packet timeouts to higher values
        Default values are ~10mS
        '''

        REG_CONFIRM_COMMS = 0x51
        REG_CONFIRM_COMMS_TIMEOUT = 0x59

        #Set GN timeout to max
        reg59_dev = self.locustB2P.readLocustReg(REG_CONFIRM_COMMS_TIMEOUT, target='host', n=1)[0]
        if reg59_dev >> 5 != 0x7:
            value_to_send = (reg59_dev & 0x1F | 0xE0)
            self.locustB2P.writeLocustReg(REG_CONFIRM_COMMS_TIMEOUT,value_to_send, target='host')

        #Set DUT timeout to max
        reg59_dev = self.locustB2P.readLocustReg(REG_CONFIRM_COMMS_TIMEOUT, target='device', n=1, addDum=True)[0]
        if reg59_dev >> 5 != 0x7:
            value_to_send = (reg59_dev & 0x1F | 0xE0)
            self.locustB2P.writeLocustReg(REG_CONFIRM_COMMS_TIMEOUT,value_to_send, target='device', addDum=True)

        hostCFG= self.locustB2P.readLocustReg(0x55, target='host', n=2) # 4:0, 10ms to 320ms, mid_remote_timeout
        devCFG = self.locustB2P.readLocustReg(0x55, target='device', n=2, addDum=True) # 4:0, 10ms to 320ms, mid_remote_timeout
        hostCFG5= hostCFG[0]
        devCFG5 = devCFG[0]
        hostCFG6= hostCFG[1]
        devCFG6 = devCFG[1]
        print('GN host: Default [MIDPACK_REMOTE_TIMEOUT], GN - %s, DUT - %s' %(hex(hostCFG5 & 0b11111), hex(devCFG5 & 0b11111)))
        print('GN host: Default [TOTAL_REMOTE_TIMEOUT], GN - %s, DUT - %s' %(hex(hostCFG6 & 0b11111), hex(devCFG6 & 0b11111)))

        hostcfg5 = hostCFG5 | 0b11100
        devcfg5 = devCFG5 | 0b11111
        hostcfg6 = hostCFG6 | 0b11100
        devcfg6 = devCFG6 | 0b11111



        self.locustB2P.writeLocustReg(0x55, [hostcfg5,hostcfg6], target='host') # 4:0, 10ms to 320ms, mid_remote_timeout
        self.locustB2P.writeLocustReg(0x55, [devcfg5,devcfg6], target='device', addDum=True) # 4:0, 10ms to 320ms, mid_remote_timeout
        self.locustB2P.writeLocustReg(REG_CONFIRM_COMMS, 0x01, target='device', addDum=True)
        self.locustB2P.writeLocustReg(REG_CONFIRM_COMMS, 0x0, target='device', addDum=True)

        hostCFG= self.locustB2P.readLocustReg(0x55, target='host', n=2) # 4:0, 10ms to 320ms, mid_remote_timeout
        devCFG = self.locustB2P.readLocustReg(0x55, target='device', n=2, addDum=True) # 4:0, 10ms to 320ms, mid_remote_timeout
        hostCFG5= hostCFG[0]
        devCFG5 = devCFG[0]
        hostCFG6= hostCFG[1]
        devCFG6 = devCFG[1]

        print('GN host: New [MIDPACK_REMOTE_TIMEOUT], GN - %s, DUT - %s' %(hex(hostCFG5 & 0b11111), hex(devCFG5 & 0b11111)))
        print('GN host: New [TOTAL_REMOTE_TIMEOUT], GN - %s, DUT - %s' %(hex(hostCFG6 & 0b11111), hex(devCFG6 & 0b11111)))

    #++++++++++++++++++++ 
    # Test functions
    #++++++++++++++++++++ 
    def enableLink(self,PPOn=True, delay=0.1, fast=False):
        '''
        try to setup buddy link with the following steps:
        1) cap detection
        2) enable powerpath of locust host and slave
        3) b2p ping locust host and slave
        4) report current locust state machine
        '''
        REG_COMMS_Mode = 0x52
        print("====== Establishing link between GN host and device ======")
        self.ioexp.locustVin(False)
        readback_reg52_host = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=REG_COMMS_Mode, rtype="reg8", destination = "HostLocust")
        value_to_send_host = (readback_reg52_host[0] & 0x7F) | 0x00
        self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_send_host, wtype="reg8", register=REG_COMMS_Mode, destination = "HostLocust")
        self.ioexp.locustVin(True)
        if fast:
            if PPOn: 
                self.enableCharging() 
                ppenable = 'enabled'
            else:
                self.enableCharging(on=False) 
                ppenable = 'disabled'
            self.locustB2P.sendReset("DeviceLocust", reset_mode=0x01) # reset has to be done before ping to setup linkstate; Otherwise b2p path to device doesn't work
            self.locustB2P.sendPing(destination="DeviceLocust")
#            self.locustB2P.getState("DeviceLocust")
            return
        else:
            capExist = self.locustI2C.capDetection()
#            if capExist: 
            if 1:  #Dev1 uses 0.1uF cap that is too small for cap detection, it will report a short and failed to continue
                self.locustI2C.idleMode() # GN host doesn't need to stay in 25Hz oscillating unless dead host detection is required
                if PPOn: 
                    self.enableCharging() 
                    ppenable = 'enabled'
                else:
                    self.enableCharging(on=False) 
                    ppenable = 'disabled'
                time.sleep(delay)
                self.locustB2P.sendReset("DeviceLocust", reset_mode=0x01) # reset has to be done before ping to setup linkstate; Otherwise b2p path to device doesn't work
                self.locustB2P.sendPing(destination="DeviceLocust")
#                self.locustB2P.getState("DeviceLocust")
                self.setTimeouts()
                print("GN host: cap detection successful, power path %s, link established" %ppenable)
            else: 
                raise RuntimeError("GN host: Error - cap detection failed, check funbus connection")

    def enableCharging(self,on=True):
        '''
        on: bool<True|False>
        Locust A0 bug: rdar://69454425 (Locust A0 ECO #8: Reset caused by turning off the pull up current when the Power Path is also enabled.)
        '''
        if on:
            self.locustB2P.enablePowerPath("HostLocust")
            self.locustB2P.pullup(False)
            self.locustB2P.getState()
        else:
            self.locustB2P.pullup(True)
            self.locustB2P.disablePowerPath("HostLocust")
            self.locustB2P.getState()

    def PMUReset(self):
        '''
        Pulse PMU reset
        Locust A0 bug -- after crabs reset, locust host has to be reset, and link again if a 2nd crabs reset is required.
        CAUTION: After PMUReset, gn.resetAll() and gn.enableLink() is required to establish connections to DUT
        CAUTION: WE ARE DOING CRABS RESETS, THIS SHOULD BE A LAST RESORT ONLY
        '''
        self.enableCharging(False)
        self.locustB2P.sendReset("HostLocust", reset_mode=0x00)

    # Buddy Test Packet uses the ping command and has 12 additional bytes of payload.
    # The first 8 bytes of the payload has the following word 0x00FF00AA5533CC01 and could be left shifted by upto 7.
    # Packet tx count is in the last 4 bytes.

    def sendAPTestPacket(self, data):
        #data should be a list
        payload = data
        op, status, resp = self.locustB2P.b2p_uart.SendRead(rcode=0x0, cmd=0x5E, payload=payload, timeout=0.1, verb=True)
        payload_tuple = struct.unpack('{}B'.format(len(resp)),resp)
        if list(payload_tuple) != data:
            print("ERROR: loopback data not equal!!!")
            return False
        print("loopback pass")
        return resp

    def checkLinkStatus(self,timeout=1):
        #check the GN link status
        if self.locustB2P.sendPing(destination="DeviceLocust") and self.b2pPingSoC(timeout=timeout) and self.locustI2C.waitUSBActive(timeout=timeout): 
            print("GN host: Link is healthy") 
            return True 
        else: 
            print("GN host: link is broken")
            return False
                
    #++++++++++++++++++++ 
    # KIS functions
    #++++++++++++++++++++ 
    def KIS2SoC(self):
        '''
        MiX connector to Parrot to SoC
        Use this function in pairs: first 'on', and once test done use 'off';
        Parrot is easy to cause hang on Mac, if the function is not used in pair
        on: <True|False>
        '''
        print("====== Enabling KIS from MiX connector to SOC ======")
        self.parrot2MiX()
        time.sleep(0.1)
        self.eUSBDurant(True)
        print("GN host: KIS from MiX connector to SoC is enabled")

    def hostLocust2DeviceLocustLoopbackTest(self, comms_mode = "DPM", num = 5000):
        test_len = num
        print("====== Enabling DPM between Loocust's and doing {} loopback commands with max checkerboard payload ======".format(test_len))
        if comms_mode == "DPM": self.setDPMCommsMode()
        else: self.setCommsMode(mode="ASK")
        P = 0 
        F = 0
        print("Running Loopback Test...")
        
        for i in range(test_len):
            if i == (.25 * test_len): print("25% complete...")
            elif i == (.5 * test_len): print("50% complete...")
            elif i == (.75 * test_len): print("75% complete...")
            elif i == (1 * (test_len - 1)): print("100% complete...")
            resp = self.locustB2P.sendLoopback(quiet = True, destination = "DeviceLocust", data = [0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55])            
            #print(resp)
            if resp is not False: P = P + 1
            else: F = F + 1 

        print("Pass: {}, Fail: {}, % Success: {}%".format(P, F, 100*(P/(P+F))))

    def KIS2Locust(self,on=True,forceDFU=False,fast=False):
        '''
        USB micro to Parrot to Locust host to Locust device to SoC
        Use this function in pairs: first 'on', and once test done use 'off';
        Parrot is easy to cause hang on Mac, if the function is not used in pair
        on: <True|False>
        forceDFU: <True|False>
        '''
        if on:
            print("====== Enabling KIS from USB micro to SOC, passthrough Locusts ======")
            if not fast: 
                self.parrot2Locust()
                self.setDPMCommsMode()
            else: 
                self.parrot2Locust()
                self.setCommsMode(mode="DPM")
            self.locustB2P.eUSBMode(True)
            time.sleep(0.2)
            self.eUSBDurant(on=True,forceDFU=forceDFU)
            print("GN host: KIS from USB micro to SoC, passthrough Locusts, is enabled")
        else:
            print("====== Disabling KIS from USB micro to SOC, passthrough Locusts ======")
            self.eUSBDurant(False)
#            self.locustB2P.eUSBMode(False)
#            time.sleep(0.3)
#            self.parrot.port(on=False)
            print("GN host: KIS from USB micro to SoC, passthrough Locusts, is disabled")

    def waitSOCReset(self):
        '''
        This function is used for avoid KIS port drop during SOC RESET;
        The RPC function needs to be called by station test SW,
        as soon as a ’sysmgr reset xxxx’ command is sent to the DUT.
        Once the RPC command finishes, the station can poll the KIS port again — should be the same KIS port.
        '''
        # First, disable USB connection after SoC reset
        self.locustB2P.eUSBMode(False)
        self.ioexp.parrotReset(0) # disable Parrot, disconnect USB

        # Second, wait firmware boots up, usually b2p response comes in 2s, here timeout is 5s
        self.b2pPingSoC(5)

        # Third, once firmware is up and running, enable USB path again
        self.ioexp.parrotReset(1) # enable Parrot, reconnect USB, this guarantees all dock channels are enabled
        self.locustB2P.eUSBMode(True)

        # Last, toggle USB Enable to enumerate USB device
        self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=0x00, wtype="reg16", register=0x0804, destination = "DeviceLocust")
        self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=0xd0, wtype="reg16", register=0x0804, destination = "DeviceLocust")
        print("GN host: Parrot and SoC resets completed")
        return 

    #++++++++++++++++++++ 
    # DFU functions
    #++++++++++++++++++++ 

    def forceAwake(self):
        '''
        Force PMU to enter awake
        '''
        print("====== Force SoC into Awake ======")
        self.forceLagunaPowerState(state="RESTART")
        time.sleep(0.3)
        print("GN host: Restart PMU to enter AWAKE mode")
        return True

    def forceDFU(self,on=True):
        '''
        Change force_dfu pin status by using 0x1868 (0x0804) register
        '''

        regVal = self.locustB2P.readLagunaReg(register=0x0804, n=1, addDum=True)[0]

        if on:
            regTarget = regVal | 0b00100000
        else:
            regTarget = regVal & 0b11011111

        self.locustB2P.writeLagunaReg(0x0804, regTarget, True)

        return True


    def forceDFUGPIO(self,on=True):
        '''
        Change forceDFU pin of Laguna to on or off, laguna gpio17
        on: bool <True|False>
        '''
        if on:
            self.locustB2P.writeLagunaReg(0x1825,0x41, addDum=False)
            print("GN host: Force DFU on")

        else:
            self.locustB2P.writeLagunaReg(0x1825,0x62, addDum=False)
            print("GN host: Force DFU off")


    def forceDFUMode(self, hold=False):
        '''
        Force DUT to enter DFU mode
        '''
        print("====== Force SoC into DFU mode ======")
        REG_DFU_CANCEL = 0x240b
        self.forceLagunaPowerState(state="RESTART_DFU")
        time.sleep(0.3)
        if hold: 
            self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=0x1, wtype="reg16", register=REG_DFU_CANCEL, destination = "DeviceLocust")
        print("GN host: Force DUT into DFU mode")
        return True

    def getDeviceInfo(self): # DFU mode is required
        print("====== Get SoC device information======")
        RSP = self.locustB2P.b2p_uart.SendRead(rcode=0x0,cmd=0x02,payload=[0],timeout=0.2)
        [stat, chipidLow, chipidHigh, boardidLow, boardidHigh, securityEpoch, prodStatus, securityMode, securityDomain, ecid, nonce1, nonce2, nonce3, nonce4, chipRev] = struct.unpack('>BBBBBBBBBQ4QB', RSP)
        chipid = (chipidHigh << 8 | chipidLow)
        boardid = (boardidHigh << 8 | boardidLow)
        temp = 0
        while ecid != 0:
            temp = (temp << 8) + (ecid & 0xFF)
            ecid = ecid >> 8
        ecid = temp
        print("GN host: ChipId = 0x%x, BoardID = 0x%x, Epoch = %d, Prod = %s, SecMode = %s, SecDom = %d, ECID = 0x%x, Nonce1 = 0x%x, Nonce2 = 0x%x, Nonce3 = 0x%x, Nonce4 = 0x%x, CRev = %d" % (chipid, boardid, securityEpoch, prodStatus, securityMode, securityDomain, ecid, nonce1, nonce2, nonce3, nonce4, chipRev))
        return ecid

    def b2pPingSoC(self, timeout=3): # b2p ping SoC in DFU mode
        print("====== Ping SoC ======")
        startTime = time.time()
        rsp = False
        while (time.time() - startTime <= timeout): 
            if (rsp == False):
                rsp = self.locustB2P.b2p_uart.SendRead(rcode=0x0,cmd=0x0,payload=[0,2],timeout=0.2) 
            else: 
                print("GN host:",list(rsp)) 
                print("GN host: SoC b2p responded within %fs" %(time.time()-startTime))
                return True
        else:    
            print("GN host: failed to get b2p response from SoC within",timeout,'s')
            return False

    def b2pSysmgrReset(self, timeout=1): # b698 b2p function to sysmgr reset

        print("====== b2p sysmgr reset system ======")
        rsp = self.locustB2P.b2p_uart.SendRead(rcode=0x0,cmd=0x0154,payload=[],timeout=0.2) 
        if (rsp == False):
            print("GN host: failed to get correct response",list(rsp)) 
        else: 
            rsp = list(rsp)
            print("GN host: sysmgr reset initiated successfully") 

        self.resetAll()
        self.enableLink()

        return

    def b2pEnableConsole(self, timeout=1): # b698 b2p function to enable console devices

        print("====== b2p enable conosle devices ======")
        rsp = self.locustB2P.b2p_uart.SendRead(rcode=0x0,cmd=0x0E02,payload=[],timeout=0.2)
        if (rsp == False):
            print("GN host: failed to enable conosle devices",list(rsp))
        else:
            rsp = list(rsp)
            print("GN host: ", rsp)
            print("GN host: enabled conosle devices")
        return

    def b2pFTVersion(self, timeout=0.3): # b698 b2p function to get ft version

        print("====== FW ft version ======")
        rsp = self.locustB2P.b2p_uart.SendRead(rcode=0x0,cmd=0x0104,payload=[],timeout=0.2) 
        if (rsp == False):
            print("GN host: failed to get correct response",list(rsp)) 
        else: 
            rsp = list(rsp)
            print("GN host: ft:ok %d %d %d" %(rsp[0], rsp[2], rsp[1])) 

        return

    def b2pShutdownUVP(self, timeout=3): # b698 b2p function to put SiP into shutdown uvp
        print("====== SiP UVP Shutdown ======")
        startTime = time.time()
        self.enableLink(PPOn=False,delay=0, fast=True) # this has to be used to prevent locust device from dropping data
        rsp = False
        while (time.time() - startTime <= timeout): 
            if (rsp == False):
                rsp = self.locustB2P.b2p_uart.SendRead(rcode=0x0,cmd=0x0E00,payload=[],timeout=0.2) 
            else: 
                print("GN host:",list(rsp)) 
                print("GN host: SoC b2p responded within %fs" %(time.time()-startTime))
                return
        else:    
            print("GN host: failed to get b2p response from SoC within",timeout,'s')

        return

    #++++++++++++++++++++ 
    # Laguna functions
    #++++++++++++++++++++ 
    def setLagunaPowerState(self, state):
        destination = "DeviceLocust"
        #REG_POWER_STATE = 0x2407
        REG_POWER_STATE = 0x0800
        #REG_SHDN_CMD_CFG = 0x2409
        REG_SHDN_CMD_CFG = 0x0801
        stateVal = {
            "AWAKE": 0x0,
            "SLP": 0x1, #works
            "RESTART": 0x2,
            "RESTART_DFU": 0x2,
            "OFF": 0x4,
            "ULP": 0x5,
#            "UVP": 0x5 #Do we need UVP
        }
        if stateVal[state] is None:
            print("GN host: power state invalid")
            return
        if stateVal[state] == 0x5: 
            self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=0x02, wtype="reg16", register=REG_SHDN_CMD_CFG, destination = destination)

        self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=stateVal[state], wtype="reg16", register=REG_POWER_STATE, destination = destination)
        return True
    
    def getLagunaPowerState(self):
        '''
        Read back Laguna's current power state
        '''
        stateVal = {
                0x0:"AWAKE",
                0x1:"SLP", #works
                0x4:"OFF",
                0x5:"Error",
                0x6:"Reset"
        }
        destination="DeviceLocust"
        REG_POWER_STATUS = 0x2406
        readback = self.locustB2P.readI2CRegs(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, read_len=0x1, register=REG_POWER_STATUS, rtype="reg16", destination = destination)
        CURState = readback[0] & 0b111
        PREState = (readback[0] >> 3) & 0b111
        print("GN host: previous power state", stateVal[PREState], ",current power state", stateVal[CURState])
        return CURState

    def forceLagunaPowerState(self, state):
        destination="DeviceLocust"
        #REG_POWER_STATE = 0x2406
        #REG_FROCE_DFU = 0x2407
        REG_FROCE_DFU = 0x0800
        state_val = {
            "AWAKE": 0x0,
            "SLP": 0x1,
            "RESTART": 0x2,
            "RESTART_DFU": 0x3,
            "OFF": 0x4,
            "SHUTDOWN": 0x5
        }
        if state_val[state] is None:
            print("GN host: power state invalid")
            return
        readback = self.locustB2P.readI2CRegs(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, read_len=0x1, register=REG_FROCE_DFU, rtype="reg16", destination = destination, addDum=True)
        if readback is None:
            print("GN host: Error, b2p not able to readback data, check funbus connection")
            return
        value_to_write = readback[0] & 0xF8 | state_val[state]
        self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=value_to_write, wtype="reg16", register=REG_FROCE_DFU, destination = destination, addDum=True)
        return True
    
    def hostReset(self,on=True):
        '''
        Toggle host reset pin of Durant
        on: bool<True|False>
        '''
        if on:
#            self.locustB2P.writeLagunaReg(0x24bb,0x00, addDum=False)
            self.locustB2P.writeLagunaReg(0x080c,0x00, addDum=True)
            print("GN host: host reset on")
        else:
#            self.locustB2P.writeLagunaReg(0x24bb,0x03, addDum=True)
            self.locustB2P.writeLagunaReg(0x080c,0x03, addDum=True)
            print("GN host: host reset off")
        return

    def enterLagunaTestMode(self):
        '''
        Enable test mode of laguna
        '''
        TEST_REG_EN = [0x3100, 0x3101, 0x3102, 0x3103]
        TEST_EN_STATUS = 0x3105
        data = [0x61,0x45,0x72,0x4f]
        for i in range(4): 
            self.locustB2P.writeLagunaReg(TEST_REG_EN[i], data[i])
        resp = self.locustB2P.readLagunaReg(TEST_EN_STATUS)
        if resp[0] == 1: 
            print("GN host: enter Laguna test mode succesfully, status =", resp)
        else:
            print("GN host: Error - enter Laguna test mode failed, status =", resp)

        return

    def exitLagunaTestMode(self):
        '''
        exit test mode of laguna
        '''
        TEST_REG_CLR = 0x3104
        TEST_EN_STATUS = 0x3105
        data = 0x00 
        self.locustB2P.writeLagunaReg(TEST_REG_CLR, data)
        resp = self.locustB2P.readLagunaReg(TEST_EN_STATUS)
        if resp[0] == 0: 
            print("GN host: exit Laguna test mode successfully, status =", resp)
        else:
            print("GN host: Error - exit Laguna test mode failed, status =", resp)

        return

    def insecureMode(self,on=True):
        '''
        Enable/disable insecure boot strap of Laguna
        on: bool <True|False>
        '''

        if on:
          self.enterLagunaTestMode()
          self.locustB2P.writeLagunaReg(0x183d, 0x0)
          self.locustB2P.writeLagunaReg(0x1826, 0x41)
          print("GN host: Insecure pin is now tied to high")
        else:
          self.locustB2P.writeLagunaReg(0x183d, 0x0)
          self.locustB2P.writeLagunaReg(0x1826, 0x40)
          print("GN host: Insecure pin is now tied to low")

        return

    def laguna7V5Passthru(self,on=True):
        '''
        Enable PP7V5 passthrough to SoC PP5V3
        on: bool <True|False>
        '''

        if on:
          self.enterLagunaTestMode()
          self.locustB2P.writeLagunaReg(0x2f1c, 0x01)
          print("GN host: PP7V5 to SoC 5V3 pass through enabled")
        else:
          self.locustB2P.writeLagunaReg(0x2f1c, 0x00)
          print("GN host: PP7V5 to SoC 5V3 pass through disabled")

        return

    def lagunaIsink(self, current=0x05, target='amuxout_ax'):
        '''
        current: enum<0x00|0x01|0x10|0x11>, greater than 0x11 means disable, 0x00-12.8mA, 0x01-6.4mA, 0x02-1.6mA, 0x03-0.8mA;
        target: enum<'buck0'|...|'buck5'|'ldo0'|...|'ldo3'|'ls_hpa'|'amuxout_ax'>
        CAUTION: When AMUXOUT pin is biased with 2.7V, there is 150uA leakage current to GND
        '''
        REG_CTRL = 0x7d00
        REG_CONF = 0X7d06
        targetList = {
            'buck0':0x00,
            'buck1':0x01,
            'buck2':0x02,
            'buck3':0x03,
            'buck4':0x04,
            'buck5':0x05,
            'ldo0':0x06,
            'ldo1':0x07,
            'ldo2':0x08,
            'ldo3':0x09,
            'ls_hpa':0x0a,
            'amuxout_ax':0x0b
        }
        if current < 0b100:
            reg_ctrl_val = (targetList[target]<<2) + 0b01
        else:
            reg_ctrl_val = 0x00

        # The order of the operation needs to be followed
        self.locustB2P.writeLagunaReg(REG_CTRL,0x00) # disable isink enable first
        self.locustB2P.writeLagunaReg(REG_CONF,current) #set sink current
        self.locustB2P.writeLagunaReg(REG_CTRL,reg_ctrl_val & 0xfe) # select target
        self.locustB2P.writeLagunaReg(REG_CTRL,reg_ctrl_val) # enable isink

        resp1 = self.locustB2P.readLagunaReg(REG_CTRL)
        resp2 = self.locustB2P.readLagunaReg(REG_CONF)
        print("GN host: Laguna Isink control register:",resp1[0],"configure register:",resp2[0])

    def ccadcRead(self): # not done yet
        REG_OPCFG = 0x7c0a
        REG_MEAS_STATUS = 0x7c18
        REG_COULOMB_COUNT = [0x7c14,0x7c15,0x7c16,0x7c17]
        REG_BATT_VOLTAGE = [0x7c10,0x7c11]
        REG_BATT_CURRENT = [0x7c12,0X7C13]
        self.locustB2P.writeLagunaReg(REG_OPCFG,0xf3)
        resp = self.locustB2P.readLagunaReg(REG_OPCFG)
        print(hex(resp[0]))
        return

    def lagunaAmuxSel(self, src='AMUX0', target ='ax'):
        '''
        Make sure Laguna is in test mode before setting the AMUX
        src: enum<'disabled'|'AMUX0/1/2/3/4/5/6/7'|'VDD_MAIN_SNS'|'VBAT_SNS_CHG'|'BUCK0/1/2/3/4/5_FB'|'LDO1/2/3_VOUT'|'LSC_VOUT0/1/2'> 
        target: enum<'ax'|'ay'>
        '''
        #CHANNEL TABLE 'Name':[acore_domain,reg_value]
        CHTable = {
                'disabled':[0,0x00,1],
                'AMUX0':[1,0x01,2], #200 RON 
                'AMUX1':[1,0x02,3], #200 RON 
                'AMUX2':[1,0x03,4], #200 RON
                'AMUX3':[1,0x04,5], #200 RON
                'AMUX4':[0,0x05,6], #200 RON
                'AMUX5':[0,0x02,7], #5k RON
                'AMUX6':[0,0x03,8], #5k RON
                'AMUX7':[0,0x04,9], #5k RON
                'VDD_MAIN_SNS':[0,0x01,10], 
                'VBAT_SNS_CHG':[0,0x02,11], 
                'NTC_BAT_P':[0,0x03,12], 
                'BUCK0_FB':[0,0x04,13], 
                'BUCK1_FB':[0,0x05,14], 
                'BUCK2_FB':[0,0x06,15], 
                'BUCK3_FB':[0,0x07,16], 
                'BUCK4_FB':[0,0x08,17], 
                'BUCK5_FB':[0,0x09,18], 
                'PMU_1V5':[0,0x0A,19], 
                'LDO1_VOUT':[0,0x0B,20], 
                'LDO2_VOUT':[0,0x0C,21], 
                'LDO3_VOUT':[0,0x0D,22], 
                'LSA_VOUT0':[0,0x0E,23], 
                'LSB_VOUT0':[0,0x0F,24], 
                'LSB_VOUT1':[0,0x10,25],
                'LSC_VOUT0':[0,0x11,26], 
                'LSC_VOUT1':[0,0x12,27],
                'LSC_VOUT2':[0,0x13,28],
                'LS_HPA_VOUT':[0,0x14,29], 
                'VSS_REF':[0,0x15,30], 
                'GPADC_LVMUX':[0,0x16,31], 
                'GPADC_HVMUX':[0,0x17,32], 
                'VSS':[0,0x18,33]
                }

        amuxSelect = CHTable[src][1]
        index = CHTable[src][2]

        if target in ['ax', 'ay']:
            if target == 'ax':
                gpio_cfg1_reg = 0x1822 #LAGUNA GPIO14 CFG1
                gpio_cfg2_reg = 0x1839 #LAGUNA GPIO14 CFG2
                if index in [1,2,3,4,5,6]: 
                    REG_SOURCE = 0x2f6c
                    amuxSelect = amuxSelect << 1
                if index in [7,8,9]:
                    REG_SOURCE = 0x7d12
                    amuxSelect = amuxSelect << 1
                if index in range(10,34,1):
                    REG_SOURCE0 = 0x7d12
                    amuxSelect0 = 0x05 << 1
                    REG_SOURCE = 0x7d10
                    amuxSelect = amuxSelect << 1
            else: # target == ay
                gpio_cfg1_reg = 0x1823 #LAGUNA GPIO15 CFG1
                gpio_cfg2_reg = 0x183A #LAGUNA GPIO15 CFG2
                if index in [1,2,3,4,5,6]: 
                    REG_SOURCE = 0x2f6d
                    amuxSelect = amuxSelect << 1
                if index in [7,8,9]:
                    REG_SOURCE = 0x7d12
                    amuxSelect = amuxSelect << 5
                if index in range(10,34,1):
                    REG_SOURCE0 = 0x7d12
                    amuxSelect0 = 0x05 << 5
                    REG_SOURCE = 0x7d11
                    amuxSelect = amuxSelect << 1

#            value = self.locustB2P.readLagunaReg(gpio_cfg2_reg)
#            newValue = value[0] & (0b11111100) | 0x00 #disable pull down
            newValue = 0x00 #disable pull down, select highest voltage range
            self.locustB2P.writeLagunaReg(gpio_cfg2_reg,newValue)
            newValue = 0xc0 # disable gpio
            self.locustB2P.writeLagunaReg(gpio_cfg1_reg,newValue) #disable gpio, configure as analog pin

            self.locustB2P.writeLagunaReg(0x7d12,0x00) #disable 5k MUX
            self.locustB2P.writeLagunaReg(0x2f6c,0x00) #disable 200OHM MUX X
            self.locustB2P.writeLagunaReg(0x2f6d,0x00) #disable 200OHM MUX Y

            if index <= 6:
                self.locustB2P.writeLagunaReg(REG_SOURCE,amuxSelect)
            else:
                if index > 9: 
                    self.locustB2P.writeLagunaReg(REG_SOURCE,amuxSelect) #configure 2nd stage AMUX
                    self.locustB2P.writeLagunaReg(REG_SOURCE0,amuxSelect0)  #configure 1st stage AMUX to select 2nd stage AMUX
                else:
                    self.locustB2P.writeLagunaReg(REG_SOURCE,amuxSelect)
        else:
            print("GN host: Error, target is not ax or ay")
        return


    def lagunaAmuxSelRTC(self,pin='ax',on=True):
        '''
        Enable or disable the 32k RTC clock on AMUXOUT_AX or AMUXOUT_AY pin
        on: bool <True|False>
        '''
        if on: 
            CFG1 = 0x60 # Output 32k RTC 
            CFG2 = 0x38 # 1.8V LSC, no pull, 10mA driving 
        else:
            CFG1 = 0x00
            CFG2 = 0xc0
        
        CFG = (CFG1 << 8) + CFG2 

        gpio = 14

        if pin == 'ax': gpio = 14
        if pin == 'ay': gpio = 15

        self.lagunaGpioConfig(gpio,CFG)

        return

    def lagunaGpioConfig(self,gpio=0,val=0x0000):
        '''
        configure laguna gpio
        gpio: enum <0..22>
        val: (configure1<<8) + configure2
        '''
        REG_GPIO_CFG1 = 0x1814
        REG_GPIO_CFG2 = 0x182b
        val1 = (val >> 8)&0xff
        val2 = val & 0xff
        self.locustB2P.writeLagunaReg(REG_GPIO_CFG1 + gpio,val1)
        self.locustB2P.writeLagunaReg(REG_GPIO_CFG2 + gpio,val2)

        return
    

    def lagunaGpioStatus(self):
        print("GN host: query gpio status")
        REG_GPIO_CFG1 = 0x1809
        val = self.locustB2P.readLagunaReg(REG_GPIO_CFG1,n=5)
        i = 0
        for num in list(val):
            print('GN host: gpio reading',i,':',hex(num))
            i += 1
        return

    def dumplagunaGpioConfig(self):
        ioTable = {
            14:"amouxout_ax",
            15:"amuxout_ay",
            16:"wdog",
            17:"force_dfu",
            18:"dfu_status",
            19:"dock_connect",
            20:"usb_en",
            21:"kis_dfu_select",
            22:"uvp_trig"
        }
        REG_GPIO_CFG1 = 0x1814
        REG_GPIO_CFG2 = 0x182b
        cfg1 = self.locustB2P.readLagunaReg(REG_GPIO_CFG1,n=23)
        cfg2 = self.locustB2P.readLagunaReg(REG_GPIO_CFG2,n=23)
        i=0
        print("GN host: query gpio configure1")
        for num in list(cfg1):
            print('GN host: gpio cfg1',i,':',hex((num>>6)&0b11), hex((num>>1)&0b11111), hex(num&0x1))
            i += 1
        i=0
        print("GN host: query gpio configure2")
        for num in list(cfg2):
            print('GN host: gpio cfg2',i,':',hex((num>>5)&0b11), hex((num>>3)&0b11), hex(num&0b11))
            i += 1
        return

    def lagunaGpadcGainOffset(self, channel=1):
        calTable = {
            (0,1): [1500.0/4095, 0.0],
            (2,2): [22500.0/4095, 0.0],
            (3,3): [50.0/4095, 0.0],  # mA
            (4,27):[1500.0/4095, 0.0],
            (28,28):[7500.0/4095, 0.0],
            (29,71):[5000.0/4095, 0.0],
            (72,74):[2500.0/4095, 0.0],
            (75,75):[5000.0/4095, 0.0],
            (76,76):[10000.0/4095, 0.0],
            (77,82):[5000.0/4095, 0.0]
        }

        for key in calTable:
            if (key[0]<=channel<=key[1]):
                [gain, offset] = calTable[key]
        return [gain,offset]

    def lagunaGpadc(self, channel=1, agent='a1'):
        if agent == 'a1':
            adcCFG = 0x7e00
            adcLSB = 0x7e01
            adcMSB = 0x7e02
            adcMode = 0x7e03
        else:
            adcCFG = 0x7e04
            adcLSB = 0x7e05
            adcMSB = 0x7e06
            adcMode = 0x7e07
    
        self.locustB2P.writeLagunaReg(adcCFG,channel) #configure manual agent
        time.sleep(0.01) # required, otherwise b2p fail
        dataLSB = self.locustB2P.readLagunaReg(adcLSB)[0] # read LSB data register
    
        while (dataLSB >> 7) == 1: # Loop until MSB is 0 (conversion valid)
            time.sleep(0.001)
            dataLSB = self.locustB2P.readlagunaReg(adcLSB)[0]
    
        dataMSB = self.locustB2P.readLagunaReg(adcMSB)[0]
        data = (dataMSB << 4)+ (dataLSB & 0x0F)
        [gain, offset] = self.lagunaGpadcGainOffset(channel)
        data = data * gain + offset
        print("GN host: gpadc channel %d, agent %s, result %f" %(channel, agent, data))
    
        return data


    #++++++++++++++++++++ 
    # Sub functions
    #++++++++++++++++++++ 

    def eUSB(self,on=False):
        '''
        Cut off the eUSB path completely, by disable parrot ports
        CAUTION: eUSB paths won't work after this command
        '''
        self.parrot.port(on=False)
        return

    def parrot2MiX(self):
        self.parrot.port(on=False)
        self.ioexp.parrotCross(0)
        self.ioexp.parrotReset(0)
        time.sleep(0.01)
        self.ioexp.parrotReset(1)
        self.parrot.port(on=True)
        print("GN host: eUSB to MiX connector path complete")
        return

    def parrot2Locust(self):
        self.parrot.port(on=False)
        self.ioexp.parrotCross(1)
        self.ioexp.parrotReset(0)
        time.sleep(0.01)
        self.ioexp.parrotReset(1)
        self.parrot.port(on=True)
        print("GN host: eUSB to Locust host path complete")
        return

    def eUSBDurant(self, on=False, forceDFU=False):

        if forceDFU:
            data = 0xf0
        else:
            data = 0xd0

        if on: 
#            self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=data, wtype="reg16", register=0x1868, destination = "DeviceLocust")
            self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=data, wtype="reg16", register=0x0804, destination = "DeviceLocust")
        else:
#            self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=0x00, wtype="reg16", register=0x1868, destination = "DeviceLocust")
            self.locustB2P.writeI2C(bus=0x01, slave_addr=LAGUNA_I2C_7BIT_ADDR, data=0x00, wtype="reg16", register=0x0804, destination = "DeviceLocust")
            self.parrot.softReset()

    def setCommsMode(self, mode="DPM"):
        REG_COMMS_Mode = 0x52
        REG_CONFIRM_COMMS = 0x51
        REG_CONFIRM_COMMS_TIMEOUT = 0x59

        mode_val = {
            "ASK": 0x00,
            "DPM": 0x80
        }
        if mode_val[mode] is None:
            print("GN host: Error - wrong input mode")
            return False

        self.locustB2P.b2p_uart.sequence = 0
        print("GN host: COMMS timeout check")
        readback_reg59_dev = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=REG_CONFIRM_COMMS_TIMEOUT, rtype="reg8", destination = "DeviceLocust")
        reg59_dev = readback_reg59_dev[0]
        if reg59_dev >> 5 != 0x7:
            value_to_send = (reg59_dev & 0x1F | 0xE0)
            self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_send, wtype="reg8", register=REG_CONFIRM_COMMS_TIMEOUT, destination = "DeviceLocust")

        print("GN host: Set COMMS mode bit")
        readback_reg52_dev = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=REG_COMMS_Mode, rtype="reg8", destination = "DeviceLocust")
        value_to_send_dev = (readback_reg52_dev[0] & 0x7F) | mode_val[mode]
        
        readback_reg52_host = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=REG_COMMS_Mode, rtype="reg8", destination = "HostLocust")
        value_to_send_host = (readback_reg52_host[0] & 0x7F) | mode_val[mode]

        NTries = 3
        PASS = False

        while ((NTries > 0) and (PASS == False)): 
            self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=(value_to_send_host & 0x7F), wtype="reg8", register=REG_COMMS_Mode, destination = "HostLocust")  # revert host back to ASK to talk to device
            self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_send_dev, wtype="reg8", register=REG_COMMS_Mode, destination = "DeviceLocust", get_resp=False) # no response 
            self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_send_host, wtype="reg8", register=REG_COMMS_Mode, destination = "HostLocust") 
            PASS = self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=0x1, wtype="reg8", register=REG_CONFIRM_COMMS, destination = "DeviceLocust", timeout=0.2)
            print("GN host: %d try to set confirm COMMS bit of device" %NTries)
            NTries -= 1

        self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=0x0, wtype="reg8", register=REG_CONFIRM_COMMS, destination = "DeviceLocust", timeout=0.2)
        print( "GN host: ASK/DPM bit written in Host and Device .")
        return True 

    
    def setDPMCommsMode(self, DPM_TX_THSEL=5, DPM_RX_THSEL=8, DPM_SEL_ROUT=2, DPM_SR=0, DPM_CFUN_SEL=2, destination="HostLocust"):
    # DPM_TX_THSEL = "800mV", DPM_RX_THSEL = "250mV", DPM_SEL_ROUT = "4 Ohm", DPM_SR = "100mV/nS", DPM_CFUN_SEL = "45pF to 85pF"
        DPM_TX_THSEL_values = {
        0x0 : "200mV" ,
        0x1 : "300mV" ,
        0x2 : "400mV" ,
        0x3 : "500mV" ,
        0x4 : "600mV" ,
        0x5 : "800mV" ,
        0x6 : "1000mV",
        0x7 : "1200mV"
        } 

        DPM_RX_THSEL_values = {
        0x0 : "25mV"   ,
        0x1 : "37.5mV" ,
        0x2 : "50mV"   ,
        0x3 : "62.5mV" ,
        0x4 : "75mV"   ,
        0x5 : "87.5mV" ,
        0x6 : "100mV"  ,
        0x7 : "112.5mV",
        0x8 : "125mV"  ,
        0x9 : "150mV"  ,
        0xA : "175mV"  ,
        0xB : "200mV"  ,
        0xC : "250mV"  ,
        0xD : "300mV"  ,
        0xE : "350mV"  ,
        0xF : "400mV"
        }

        DPM_SEL_ROUT_values = {
        0x0 : "3 Ohm"  ,
        0x1 : "4 Ohm"  ,
        0x2 : "5.8 Ohm",
        0x3 : "7.6 Ohm"
        }

        DPM_SR_values = {
        0x1 : "100mV/nS" ,
        0x0 : "200mV/nS" 
        }

        DPM_CFUN_SEL_values = {
        0x0 : "80pF to 135pF" ,
        0x1 : "130pF to 200pF",
        0x2 : "45pF to 85pF" ,
        0x3 : "10pF to 50pF"
        }

        try:
            print("GN host: set DPM_TX_THSEL = ", DPM_TX_THSEL_values[DPM_TX_THSEL])
            print("GN host: set DPM_RX_THSEL = ", DPM_RX_THSEL_values[DPM_RX_THSEL])
            print("GN host: set DPM_SEL_ROUT = ", DPM_SEL_ROUT_values[DPM_SEL_ROUT])
            print("GN host: set DPM_SR = ", DPM_SR_values[DPM_SR])
            print("GN host: set DPM_CFUN_SEL = ", DPM_CFUN_SEL_values[DPM_CFUN_SEL])
        except:
            raise RuntimeWarning("GN host: DPM parameters input wrong!!!!")

        rb_data = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=0x52, rtype="reg8", destination = destination)
        if DEBUG_FLAG:
            print("GN host: DPM_TX_THSEL, DPM_RX_THSEL", hex(rb_data[0])) # TX bit[2:0] -> 101 ->800mV, RX bit[6:3] -> 1000 -> 125mV

        value_to_write_RX_th = (rb_data[0] & 0x87) | (DPM_RX_THSEL << 3)  # set RX threshold to 175mV -> 0x48
        value_to_write_TX_th = (value_to_write_RX_th & 0xF8) | DPM_TX_THSEL

        self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_write_TX_th, wtype="reg8", register=0x52, destination = destination)
        rb_data = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=0x52, rtype="reg8", destination = destination)
        if rb_data[0] != value_to_write_TX_th:
            raise RuntimeError("GN host: DPM_TX_THSEL readback wrong")
            return False

        rb_data = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=0x57, rtype="reg8", destination = destination) 
        if DEBUG_FLAG:
            print("GN host: DPM_SEL_ROUT", hex(rb_data[0])) # bit[5:4] -> 0x10 -> 5.8 ohm
        value_to_write_rout = (rb_data[0] & 0xCF) | (DPM_SEL_ROUT << 4) 
        self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_write_rout, wtype="reg8", register=0x57, destination = destination)

        rb_data = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=0x58, rtype="reg8", destination = destination) 
        if DEBUG_FLAG:
            print("GN host: DPM_SR", hex(rb_data[0])) # bit[4] -> 0 ->200mV/ns
        value_to_write_sr = (rb_data[0] & 0xFF) | (DPM_SR << 4) 
        #Writing register 0x58 can disable PP to DUT
        self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_write_sr, wtype="reg8", register=0x58, destination = destination)

        rb_data = self.locustB2P.readI2CRegs(bus=0x00, slave_addr=0x33, read_len=0x1, register=0x5B, rtype="reg8", destination = destination) 
        if DEBUG_FLAG:
            print("GN host: DPM_CFUN_SEL", hex(rb_data[0])) # bit[3:2] ->0x10 -> 45pF to 85pF
        value_to_write_cfun = (rb_data[0] & 0xF3) | (DPM_SEL_ROUT << 2) 
        self.locustB2P.writeI2C(bus=0x00, slave_addr=0x33, data=value_to_write_cfun, wtype="reg8", register=0x5B, destination = destination)
        
        ret = self.setCommsMode(mode="DPM")

        return ret

    def resetDeviceLocust(self):
        '''
        b2p reset device Locust
        '''
        self.locustB2P.sendReset("DeviceLocust", reset_mode=0x01)
        self.locustB2P.sendPing(destination="DeviceLocust")
        self.locustB2P.enablePowerPath(destination="DeviceLocust")

    def disconnectUSB(self):
        '''
        USE THE FUNCTION WITH CAUTION:
        ONCE USB MUX IS OFF, ALL GRASSNINJA PORT WILL DISAPPEAR IMDEDIATELY
        THIS CAN'T BE RECOVERED UNTIL GRASSNINJA IS POWER CYCLED
        '''
        self.ioexp.usbMuxOE(False)

# Sub classes begins here
#=======================================================

CAP_DET_STATUS_values = {
    0 : "CAP DET Not Active",
    1 : "Open Detected",
    2 : "Short Detected",
    3 : "Connection Detected"
}

class locustI2C(): # i2c is not necessary as b2p uart i2c is more efficient 100k vs 1M
    def __init__(self,i2c):
        self.i2c = i2c
        self.addr = 0x33<<1

    def reset(self):
        self.masterCheck()
        self.setASKTXParameter(rout=8, timeout=30e-3, amplitude=2) # enable maximum ASK TX amplitude
        return

    def crabsResetTime(self,duration=100): #100ms by default
        val = int(duration/50) << 3
        self.i2c.write(self.addr,0x2c,val)
        return


    def waitUSBActive(self,timeout=5):
        '''
        timeout in seconds
        '''
        EUSBStatus = self.i2c.read(self.addr,0x42,1)
        startTime = time.time()
        currentDelay = 0
        while ((EUSBStatus & 0b10001000) != 0x88) : 
            EUSBStatus = self.i2c.read(self.addr,0x42,1)
            currentDelay = time.time() - startTime
            if (currentDelay > timeout):
                print('GN host: error, failed to establish KIS USB connection in %f s' %(currentDelay))
                return False
        print('GN host: KIS USB connection is established successfully in %f s' %(currentDelay))
        return True

    def checkFaultStatus(self):
        '''
        Check locust host fault registers
        '''
        faultRegs = self.i2c.read(self.addr,0x40,14)
        print("GN host: locust fault status", hex(faultRegs))
        return


    def masterCheck(self):
        deviceID = self.i2c.read(self.addr,0x00,2)
        deviceREV = self.i2c.read(self.addr,0x02,1)
        traceID = self.i2c.read(self.addr,0x03,4)
        ASK_TX_DRV_BW = self.i2c.read(self.addr,0x1a,1)
        ControlReg789AB = self.i2c.read(self.addr,0x55,5)
        LinkStatus = self.i2c.read(self.addr, 0x44,1)
        EUSBStatus = self.i2c.read(self.addr,0x42,1)
        Control7Reg = ControlReg789AB & 0xff
        Control8Reg = (ControlReg789AB >> 8) & 0xff
        Control9Reg = (ControlReg789AB >> 16) & 0xff
        Control11Reg = (ControlReg789AB >> 32) & 0xff
        Trim19Reg = self.i2c.read(self.addr,0x1A,1)
        val = self.i2c.read(self.addr,0x21,1)
        if ((val & 0x80)==0x80): 
            print("GN host: error, locust host not found")
        else:
            print("GN host: locust host found, deviceID %s, device revison %s, traceID %s, Trim19Reg %s" %(hex(deviceID), hex(deviceREV), hex(traceID), hex(Trim19Reg)))
            print("GN host: default ASK TX setting: 0x55 = %s,0x56 = %s, 0x57 = %s, 0x59 = %s" %(hex(Control7Reg), hex(Control8Reg), hex(Control9Reg),hex(Control11Reg)))

    def powerPath(self,switch='on'):
        if (switch == 'on'):
            self.i2c.write(self.addr,0x4E,0x80)
            print("GN host: Power path enabled")
        if (switch == 'off'):
            self.i2c.write(self.addr,0x4E,0x00)
            print("GN host: Power path disabled")
    
    def setASKTXParameter(self,rout=10,timeout=30e-3,amplitude=1):

        if rout<=5.3: ASK_SEL_ROUT = 0x5
        if 5.3<rout<=6.2: ASK_SEL_ROUT = 0x4
        if 6.2<rout<=7.3: ASK_SEL_ROUT = 0x7
        if 7.3<rout<=9: ASK_SEL_ROUT = 0x6
        if 9<rout<=11.7: ASK_SEL_ROUT = 0x1
        if 11.7<rout<=17: ASK_SEL_ROUT = 0x0
        if 17<rout<=30: ASK_SEL_ROUT = 0x3
        if rout>30: ASK_SEL_ROUT = 0x2

        B2PTIMEOUT = int(abs((timeout-10e-3)/10e-3)) & 0x1F
        Reg8 = (ASK_SEL_ROUT << 5) + B2PTIMEOUT
        self.i2c.write(self.addr,0x56,Reg8)
        self.i2c.write(self.addr,0x59,0xc4) # SS is causing a modulation on the waveform, disable SS could improve SI

        if amplitude < 1.25: ASK_TX_GAIN_TUNE = 0x2
        else: ASK_TX_GAIN_TUNE = 0x0

        Reg9 = self.i2c.read(self.addr,0x57,1)
        Reg9 = (Reg9 & 0xFC) | ASK_TX_GAIN_TUNE
        self.i2c.writeN(self.addr,0x55,0xa88160,N=3)
#        self.i2c.write(self.addr,0x55,0x60)
#        self.i2c.write(self.addr,0x56,0x81)
#        self.i2c.write(self.addr,0x57,0xa8)

        ControlReg789AB = self.i2c.read(self.addr,0x55,5)
        Reg7 = ControlReg789AB & 0xff
        Reg8 = (ControlReg789AB >> 8) & 0xff
        Reg9 = (ControlReg789AB >> 16) & 0xff
        Reg11 = (ControlReg789AB >> 32) & 0xff
#        Reg7 = self.i2c.read(self.addr,0x55,1)
#        Reg8 = self.i2c.read(self.addr,0x56,1)
#        Reg9 = self.i2c.read(self.addr,0x57,1)
#        Reg11 = self.i2c.read(self.addr,0x59,1)

        print("GN host: ASK TX setting change to 0x55 = %s, 0x56 = %s, 0x57 = %s" %(hex(Reg7),hex(Reg8), hex(Reg9)))
        print("GN host: default SS setting 0x59 update to %s (0xc4)" %(hex(Reg11)))
        return

    def idleMode(self,on=True):
        val = self.i2c.read(self.addr,0x4e,1)
        if on: val = val | 0x4
        else: val = val & 0b11111011
        self.i2c.write(self.addr,0x4e,val)
        print("GN host: idle mode is %d" %((val>>2)&0b1))
        return

    def dumpStatus(self):
        val = self.i2c.read(self.addr,0x40,14)
        print("GN host: status", hex(val))

    def powerStatus(self):
        val = self.i2c.read(self.addr,0x41,1)
        if (val != 0xf0): 
            print("GN host: Error, locust power connection failed -- status 0x%x" %val)
        else:
            print("GN host: Locust power good.")

    def capDetection(self):
        self.funcConfigureCapDet(CAPDET_OPEN='600', CAPDET_DETECT='5000')
        self.funcEnterCapDet()
        time.sleep(0.02)
        resp = self.funcReadCapDetStatus()
        self.funcExitCapDet()
        if resp == 3: return True
        else: return False

    def funcConfigureCapDet(self, CAPDET_OPEN, CAPDET_DETECT):
        '''
        CAPDET_OPEN: int<0-65535>,ms, cap open timing threshold
        CAPDET_DETECT: int<0-65535>,ms, cap found timing threshold
        '''
        # reset value: CAPDET_OPEN = '600us', CAPDET_DETECT = '5000us'
        CAPDET_OPEN_values = {}
        for i in range(256):
            CAPDET_OPEN_text = str(50 + i*50) #us
            CAPDET_OPEN_values.update({CAPDET_OPEN_text: i})
        CAPDET_OPEN_values.update({"None": None})

        CAPDET_DETECT_values = {}
        for i in range(256):
            CAPDET_DETECT_text = str(50 + i*50) #us
            CAPDET_DETECT_values.update({CAPDET_DETECT_text: i})
        CAPDET_DETECT_values.update({"None": None})

        # CAPOPENTIME_values = {}
        # DEVICE_DETECTED_values = {}
        # DEVICE_NOTDETECTED_values = {}

        try:
            capdet_open_tosend = CAPDET_OPEN_values[CAPDET_OPEN]
        except:
            raise RuntimeWarning("GN host: input CAPDET_OPEN time from 50us to 12800 us in 50us intervals (8-bits). POR default to 600us")
        try:
            capdet_detect_tosend = CAPDET_DETECT_values[CAPDET_DETECT]
        except:
            raise RuntimeWarning("Warning: input CAPDET_DETECT time from 50us to 12800 us in 50us intervals (8-bits). POR default to 5,000 us")

        if capdet_open_tosend is not None:
            self.i2c.write(self.addr,0x5F, capdet_open_tosend)
        if capdet_detect_tosend is not None:
            self.i2c.write(self.addr,0x60, capdet_detect_tosend)
        
        print("GN host: configure cap detection with open time", CAPDET_OPEN,"ms, detect time", CAPDET_DETECT,"ms")
        return

    def funcEnterCapDet(self):
        val = self.i2c.read(self.addr,0x4E,1)
        if (0x01 & val) == 0x01:
            print( "GN host: HOST_CAPDET_EN already enabled.")
        else:
            masked_value = (val & 0xFE) | 0x01
            self.i2c.write(self.addr,0x4E,masked_value)
            print("GN host: cap detection enabled")

    def funcExitCapDet(self):
        val = self.i2c.read(self.addr,0x4E,1)
        if (0x01 & val) == 0x00:
            print( "GN host: HOST_CAPDET_EN already disabled.")
        else:
            masked_value = (val & 0xFE) | 0x00
            self.i2c.write(self.addr,0x4E,masked_value)
            print("GN host: cap detection disabled")

    def funcReadCapDetStatus(self):
        val = self.i2c.read(self.addr,0x41,1)
        CAP_DET_STATUS = val & 0x03
        print("GN host:",CAP_DET_STATUS_values[CAP_DET_STATUS])
        return CAP_DET_STATUS


    
class cd2e22(): # Parrot functions
    def __init__(self,i2c):
        self.i2c = i2c
        self.addr = 0x42
    
    def reset(self):
        self.i2c.writeN(self.addr,0x82,0x0f0f88,N=3)
#        self.i2c.write(self.addr,0x82,0b10001000) # configure INT polarity
#        self.i2c.write(self.addr,0x83,0x0f) # configure INT enables
#        self.i2c.write(self.addr,0x84,0x0f) # configure INT enables
        revID = self.i2c.read(self.addr,0x80,1)
        gpio0 = self.i2c.read(self.addr,0x30,1)
        gpio1 = self.i2c.read(self.addr,0x70,1)
        intConf = self.i2c.read(self.addr,0x83,2)
        print("GN host: Parrot ID 0x%x, gpio0 0x%x, gpio1 0x%x, intConf 0x%x" %(revID,gpio0,gpio1,intConf))
        print("GN host: Parrot reset complete")
    
    def port(self,on=True):
        if on: self.i2c.write(self.addr,0x82,0x00)
        else: self.i2c.write(self.addr,0x82,0x20)
    
    def softReset(self):
        self.i2c.write(self.addr,0x82,0x80)

    def INTStatus(self):
        val = self.i2c.read(self.addr,0x93,1)
        val = (val<<8) + self.i2c.read(self.addr,0x94,1)
        if (val > 0):
            print("Interrupt 0x%x" %val)
        else:
            print("No eUSB interrupt found")

    def readReg(self,register):
        value = self.i2c.read(self.addr,register,1)
        return value

    def writeReg(self,register,data):
        self.i2c.write(self.addr,register,data)
        return

class tca9555():
    def __init__(self,i2c):
        self.i2c = i2c
        self.addr = 0x40

    def reset(self):
        self.i2c.write(self.addr,0x02,0b11001111) # enable all LEDs, reset Parrot
        self.i2c.write(self.addr,0x06,0x00)
        self.i2c.write(self.addr,0x03,0b00001000)
        self.i2c.write(self.addr,0x07,0b11100110) # reset Locust
#        time.sleep(0.5) # This delay seems useless
        self.i2c.write(self.addr,0x02,0b11010000) # disable all LEDs, enable Parrot
        self.i2c.write(self.addr,0x07,0b11100111) # enable Locust
        self.write(1,4,0) #enable USB MUX
        self.config(1,4,0)

    def readAll(self):
        val = self.i2c.read(self.addr,0x0,2)
#        val = self.i2c.read(self.addr,0x0,1)
        print("Port 0 %s" %(hex(val&0xff)))
#        val = self.i2c.read(self.addr,0x1,1)
        print("Port 1 %s" %(hex((val>>8)&0xff)))

    def read(self,port,io): 
        '''
        port: enum 0|1
        io: enum 0..7
        '''
        val = self.i2c.read(self.addr,port,1)
        val = (val >> io) & 0x01
        return val
    
    def write(self,port,io,val):
        '''
        port: enum 0|1
        io: enum 0..7
        '''
        reg = port + 2
        data = self.i2c.read(self.addr, reg, 1)
        if (val == 0):
            data = data & (0xff - (1<<io))
        else:
            data = data | (1<<io)

        self.i2c.write(self.addr,reg,data)
        return

    def config(self,port,io,config):
        '''
        port: enum 0|1
        io: enum 0..7
        config: enum 0|1, 0--output, 1--input
        '''
        reg = port + 6
        data = self.i2c.read(self.addr, reg, 1)
        if (config == 0): #output
            data = data & (0xff - (1<<io))
        else: #input
            data = data | (1<<io)

        self.i2c.write(self.addr,reg,data)
        return

    def parrotCross(self,value):
        '''
        value: enum<0|1>
        '''
        self.write(0,5,value)
        return

    def parrotReset(self,value):
        '''
        value: enum<0|1>
        '''
        self.write(0,4,value)
        return
    
    def usbMuxOE(self,on=True):
        '''
        on: bool <True|False>, True- outptut enable, False- output disable
        CAUTION: DON'T TOUCH THIS FUNCTION IF YOU DON'T KNOW WHAT IT DOES
        '''
        print("GN host: USB connection will be broken if on=False")
        if on:
            self.write(1,4,0)
        else:
            self.write(1,4,1)

    def locustVin(self,on=True):
        '''
        on: bool <True|False>, True- outptut enable, False- output disable
        '''
        if on:
            self.write(0,7,1)
        else:
            self.write(0,7,0)

    def powerCycle(self):
        '''
        This function is available for GN-B0 that with locust B0 silicon
        CAUTION: After executing the function, all the GN ports will be missing and enumerated again
        '''
        self.write(1,5,1)
        self.config(1,5,0)
        return

    def pp3V3(self,value):
        '''
        value: enum<0|1>, default==1 (on)
        ''' 
        self.write(0,6,value) 
        self.config(0,6,0)
        return
        

class i2c():
    def __init__(self,portname, debug=False):
        self.i2cSlave = I2cController()
        self.debug = debug
        try:
            self.i2cSlave.configure(portname,frequency=400000)
        except:
            raise RuntimeError("GN host: Error, can't open i2c port %s", portname)

    def read(self,addr,reg,N):
        '''
        read register from address
        addr: int8<0-255>, i2c slave 8bit address
        reg: int8<0-255>, register
        N: int8<0-255>, number of registers to read
        '''
        addr = addr >>1
        i2cDevice = self.i2cSlave.get_port(addr)
        data = i2cDevice.read_from(reg,N)
#        data = int.from_bytes(data,"big")
        data = int.from_bytes(data,"little")
        if self.debug:
            print("GN host i2c: Read from i2c addr 0x%x, reg 0x%x, data 0x%x" %(addr, reg, data))
        return data
    
    def write(self,addr,reg,data):
        '''
        write register with data
        addr: int8<0-255>, i2c slave 8bit address
        reg: int8<0-255>, register
        data: int8<0-255>, data to be written
        '''
        addr = addr >>1
        i2cDevice = self.i2cSlave.get_port(addr)
        i2cDevice.write_to(reg,bytes([data]))
        if self.debug: 
            print("GN host i2c: Write to i2c addr 0x%x, reg 0x%x, data 0x%x" %(addr, reg, data))

    def writeN(self,addr,reg,data,N):
        '''
        write register with data
        addr: int8<0-255>, i2c slave 8bit address
        reg: int8<0-255>, register
        data: int64, data to be written, little endian, for example 0xA8FFB9- 1st reg 0xB9, 2nd 0xFF, 3rd 0xA8
        N: int8<0-255>
        '''
        addr = addr >>1
        regVal = reg.to_bytes(1,'little')
        regVal = regVal + data.to_bytes(N,'little')
        i2cDevice = self.i2cSlave.get_port(addr)
        i2cDevice.write(regVal)
        if self.debug: 
            print("GN host i2c: Write to i2c addr 0x%x, reg 0x%x, data 0x%x" %(addr, reg, data))


def get_unprogrammed_sn():
    from pyftdi.ftdi import Ftdi
    i2cPortName = 'ftdi://ftdi:2232:/2'
    devices = Ftdi.list_devices(i2cPortName)
    for device in devices:
        if not device[0].sn:
            i2cPortName = 'ftdi://ftdi:2232:{:x}:{:x}/2'.format(device[0].bus, device[0].address)
            break
    return i2cPortName


if __name__ == '__main__':
    print('GrassNinjaHost test: P3')
    parser = argparse.ArgumentParser(description='GrassNinja Host Application, ver 2.0')
    parser.add_argument('-p', '--serialport', help='Serial port name.', type=str, default='/dev/cu.usbserial-gnhost0', required=True)
    parser.add_argument('-n', '--not_programmed', help='Use if serial# is not programmed.', action='store_true', default=False)
    parser.add_argument('-s', '--supply', help='Fun bus 4.5V supply: [on|off]', type=str, default='on', required=False)
    parser.add_argument('-f', '--fast', help='fast mode KIS reconnect', type=bool, default=False, required=False)

    args = parser.parse_args()
    funbus4p5 = args.supply
    fastMode = args.fast

    GNH_UART ={ 
            'portname': args.serialport,
            'baudrate': 905600, # DON'T TOUCH BAUDRATE!
            'timeout':0.2
            }
    FTDISerialNumber = GNH_UART['portname'].split('-')[1][:-1]
    if args.not_programmed:
        i2cPortName = get_unprogrammed_sn()
    else:
        i2cPortName = 'ftdi://ftdi:2232:'+FTDISerialNumber+'/2'

    b2pUart = serial.Serial(GNH_UART['portname'],baudrate=GNH_UART['baudrate'],timeout=GNH_UART['timeout']) 


    gnTest = grassNinjaHost(b2pUart,i2cPortName)

    timeBegin = time.time()

    if not fastMode: 
        gnTest.resetAll()
    if 'off' in funbus4p5: 
        gnTest.enableLink(PPOn=False,delay=0, fast=fastMode)
    else: 
        gnTest.enableLink(PPOn=True,delay=0, fast=fastMode)

    gnTest.locustI2C.masterCheck()
    gnTest.b2pPingSoC(timeout=5) # Added this in P3 --  it's found SoC firmware boot time can interfere with the GN script; Add this line to wait for firmware bootup
    gnTest.KIS2Locust(on=True,forceDFU=False, fast=fastMode)
    gnTest.locustI2C.waitUSBActive()

#    if 0 != gnTest.getLagunaPowerState(): # make sure PMU is awake
#        raise RuntimeError("GN host: PMU power mode is incorrect")

    print("\nScript complete in %fs" %(time.time()-timeBegin))
    # Close the FTDI ports
    gnTest.closePorts()
