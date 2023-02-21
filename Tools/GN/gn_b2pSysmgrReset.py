#python3
# GrassNinja python driver
# HWTE Yan Li, 2020/10/10
# yan.li@apple.com
# DO NOT DISTRIBUTE THE CODE TO THE 3RD PART

import serial
import time
import argparse
from grassninja import grassNinjaHost


DEBUG_FLAG = True


if __name__ == '__main__': 
    print('GrassNinjaHost test: P1')
    parser = argparse.ArgumentParser(description='GrassNinja Host Application, ver 1.0')
    parser.add_argument('-p', '--serialport', help='Serial port name.', type=str, default=False, required=True)
    parser.add_argument('-n', '--not_programmed', help='Use if serial# is not programmed.', action='store_true', default=False)

    args = parser.parse_args()

    GNH_UART ={ 
            'portname': args.serialport,
            'baudrate': 905600, # DON'T TOUCH BAUDRATE!
            'timeout':0.2
            }
    FTDISerialNumber = GNH_UART['portname'].split('-')[1][:-1]
    if args.not_programmed:
        from grassninja import get_unprogrammed_sn
        i2cPortName = get_unprogrammed_sn()
    else:
        i2cPortName = 'ftdi://ftdi:2232:'+FTDISerialNumber+'/2'

    b2pUart = serial.Serial(GNH_UART['portname'],baudrate=GNH_UART['baudrate'],timeout=GNH_UART['timeout']) 

    gnTest = grassNinjaHost(b2pUart,i2cPortName)
    gnTest.b2pSysmgrReset()

    # Close the FTDI ports
    gnTest.closePorts()
