<meta charset="utf-8">

GrassNinja scripts
==================

This project provides python scripts for accessing KIS resources via GrassNinja.

See https://confluence.sd.apple.com/display/SMT/GrassNinja+Host+Application.

Python environment
==================

Python3 3.7.2 or later.

Please run `pip3 install -r requirements.txt`

Download `libusb.tar` from [rdar://75560201](rdar://75560201 (B698 -- Grassninja testsuit)) (B698 -- Grassninja testsuit), 
and untar it to `/usr/local/lib`:

    $ tar -x --directory /usr/local/lib ~/Downloads/libusb.tar

Script usage
==================
gn_init.py -- Initialize Grassninja, this has to run as the first step

gn_link.py -- Pairs GN with a DUT with locust device, including cap detection and ping commands, this has to run as the second step


grassninja.py -- One script to enable the KIS portals for Diags communication; It includes the steps in gn_init.py and gn_link.py

gn_kis_fwdl.py -- One script to enable the KIS portal for FWDL and also prepare the DUT for DFU

gn_pmuReset.py --  Use crabs signal to reset the PMU on a DUT; It doesn't depend on the gn_init.py and gn_link.py

gn_powerCycle.py -- Power cycle GN module without plug out and in its USB cable

The following scripts need to run after gn_init.py and gn_link.py
==================
gn_amuxVref.py -- Bring PMU bandgap reference voltage to AMUXOUT_AY pin

gn_b2pGetDeviceInfo.py --  If the DUT is in DFU mode, this shows the SoC information like boardID, ECID etc.

gn_b2pPingSoC.py -- Ping SoC in either DFU mode or Diags mode; If the DUT hangs in Diags, the response will fail

gn_forceDFU.py -- Force SoC into DFU mode, by toggling the force_DFU and SoC_reset pins. Note -- Laguna firmware is probably still working in this mode

gn_forceDFUMode.py -- Force SoC into DFU mode, by changing the Laguna power state. Note -- there is no laguna firmware running in this mode

gn_hostReset.py -- Toggles SoC_Reset pin on or off

gn_loopback.py -- Loopback test from locust device

gn_powerpath.py -- Enable/disable the 4.5V funbus supply to a DUT; Disabling powerpath switches FUNBUS to 1.8V to continue the communication

gn_toggleUSB.py -- re-enumerate all KIS ports again for provenance application, usually after gn_kis_fwdl.py script

gn_FWBootTime.py -- script to check how much time firmware can boot up when funbus is powered on (no battery)
