#!/bin/sh
/usr/local/bin/python3 grassninja.py -p /dev/cu.usbserial-GNhostE0 -s on
sleep 1
/usr/local/bin/python3 gn_hostReset.py -p /dev/cu.usbserial-GNhostE0 -s on
sleep 1
/usr/local/bin/python3 gn_powerpath.py -p /dev/cu.usbserial-GNhostE0 -s off

# GrassNinjaHost test: P3
# ====== Resetting everything ======
# GN host: Parrot ID 0x1, gpio0 0x10, gpio1 0x10, intConf 0xf0f
# GN host: Parrot reset complete
# GN host - locustb2p: send reset HostLocust
# B2P response: Reset detected
# GN host: locust host found, deviceID 0x135, device revison 0xb0, traceID 0xa58ee853, Trim19Reg 0x8
# GN host: default ASK TX setting: 0x55 = 0x1,0x56 = 0xa2, 0x57 = 0x29, 0x59 = 0xc2
# GN host: ASK TX setting change to 0x55 = 0x60, 0x56 = 0x81, 0x57 = 0xa8
# GN host: default SS setting 0x59 update to 0xc4 (0xc4)
# GN host - locustb2p: send ping HostLocust
# GN host - locustb2p: b2p protocol version = 2, mode = 3
# ====== Establishing link between GN host and device ======
# GN host: configure cap detection with open time 600 ms, detect time 5000 ms
# GN host: cap detection enabled
# GN host: Connection Detected
# GN host: cap detection disabled
# GN host: idle mode is 1
# GN host - locustb2p: HostLocust powerpath enabled.
# GN host - locustb2p: HostLocust local pullup disabled.
# GN host: HostLocust current state HOST_ASK_PPON
# GN host - locustb2p: send reset DeviceLocust
# GN host - locustb2p: send ping DeviceLocust
# GN host - locustb2p: b2p protocol version = 2, mode = 3
# GN host: Default [MIDPACK_REMOTE_TIMEOUT], GN - 0x0, DUT - 0x1
# GN host: Default [TOTAL_REMOTE_TIMEOUT], GN - 0x1, DUT - 0x2
# GN host: New [MIDPACK_REMOTE_TIMEOUT], GN - 0x1c, DUT - 0x1f
# GN host: New [TOTAL_REMOTE_TIMEOUT], GN - 0x1d, DUT - 0x1f
# GN host: cap detection successful, power path enabled, link established
# GN host: locust host found, deviceID 0x135, device revison 0xb0, traceID 0xa58ee853, Trim19Reg 0x8
# GN host: default ASK TX setting: 0x55 = 0x7c,0x56 = 0x9d, 0x57 = 0xa8, 0x59 = 0xe4
# ====== Ping SoC ======
# GN host: [2, 3]
# GN host: SoC b2p responded within 1.005916s
# ====== Enabling KIS from USB micro to SOC, passthrough Locusts ======
# GN host: eUSB to Locust host path complete
# GN host: set DPM_TX_THSEL =  800mV
# GN host: set DPM_RX_THSEL =  125mV
# GN host: set DPM_SEL_ROUT =  5.8 Ohm
# GN host: set DPM_SR =  200mV/nS
# GN host: set DPM_CFUN_SEL =  45pF to 85pF
# GN host: DPM_TX_THSEL, DPM_RX_THSEL 0x45
# GN host: DPM_SEL_ROUT 0xa8
# GN host: DPM_SR 0x88
# GN host: DPM_CFUN_SEL 0x78
# GN host: COMMS timeout check
# GN host: Set COMMS mode bit
# GN host: 3 try to set confirm COMMS bit of device
# GN host: ASK/DPM bit written in Host and Device .
# GN host - locustb2p: enable both locust host and device eUSB mode
# GN host: KIS from USB micro to SoC, passthrough Locusts, is enabled
# GN host: KIS USB connection is established successfully in 0.218367 s

# Script complete in 3.224772s
# GrassNinjaHost test: P1
# GN host: host reset on
# GrassNinjaHost test: P1
# GN host - locustb2p: HostLocust local pullup enabled.
# GN host - locustb2p: HostLocust powerpath disabled.
# GN host: HostLocust current state HOST_DPM_LCLPU