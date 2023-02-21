#python3
# GrassNinja python driver
# HWTE Yan Li, 2020/10/10
# yan.li@apple.com
# DO NOT DISTRIBUTE THE CODE TO THE 3RD PART

import serial
import time
import argparse
from grassninja import grassNinjaHost


LAGUNA_I2C_7BIT_ADDR = 0x11
DEBUG_FLAG = True

if __name__ == '__main__': 
    print('GrassNinjaHost test: P1')
    parser = argparse.ArgumentParser(description='GrassNinja Host Application, ver 1.0')
    parser.add_argument('-p', '--serialport', help='Serial port name.', type=str, default=False, required=True)
    args = parser.parse_args()

    GNH_UART ={ 
            'portname': args.serialport,
            'baudrate': 905600, # DON'T TOUCH BAUDRATE!
            'timeout':0.2
            }
    FTDISerialNumber = GNH_UART['portname'].split('-')[1][:-1]
    i2cPortName = 'ftdi://ftdi:2232:'+FTDISerialNumber+'/2'

    b2pUart = serial.Serial(GNH_UART['portname'],baudrate=GNH_UART['baudrate'],timeout=GNH_UART['timeout']) 
    gnTest = grassNinjaHost(b2pUart,i2cPortName)

    gnTest.forceDFUMode(hold=True)
    gnTest.enterLagunaTestMode()

    gnTest.locustB2P.writeLagunaReg(0x24bd, 0x00) # disable charging
    gnTest.locustB2P.writeLagunaReg(0x7e1b, 0x00) # disable tdev, tdie monitor
    gnTest.locustB2P.writeLagunaReg(0x7d12, 0xA0) # enable amux_ay
    gnTest.locustB2P.writeLagunaReg(0x7d11, 0x2D) # enable gpadc_lv on AY
    gnTest.locustB2P.writeLagunaReg(0x1823, 0xc0) # disable amux gpio on AY
    gnTest.locustB2P.writeLagunaReg(0x183a, 0x00) # disable pulldown on AMUXOUT_AY
    gnTest.locustB2P.writeLagunaReg(0x7e74, 0x03) # set GPADC KEEP_SEL and KEEP_ADC_EN bit
    gnTest.locustB2P.writeLagunaReg(0x7e00, 0x01) # select channel 1

    print("GN host: %s is now on AMUXOUT_%s" %('GPADC_LVMUX', 'AY'))

    # Close the FTDI ports
    gnTest.closePorts()
