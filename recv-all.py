#!/usr/bin/env python

"""
Shows how the receive messages via polling.
"""

import can
from can.bus import BusState
import time
import can.interfaces.slcan as sl
from can.exceptions import (
    CanInterfaceNotImplementedError,
    CanInitializationError,
    CanOperationError,
    error_check,
)
from can import typechecking

import argparse


parser = argparse.ArgumentParser(description='Send Std and Ext via SLCAN')
parser.add_argument('tty', type=str,
                    help='/dev/tty.usbmodem12303')
parser.add_argument('--boudrate', type=int, default=115200, help='1000000')

args = parser.parse_args()


class slcanBus(can.interface.Bus):

    def open(self) -> None:
        self._write("O")
        buffer = self._read(0.1)

    def close(self) -> None:
        self._write("C")
        buffer = self._read(0.1)

    def set_bitrate(self, bitrate: int) -> None:
        """
        :param bitrate:
            Bitrate in bit/s

        :raise ValueError: if ``bitrate`` is not among the possible values
        """
        if bitrate in self._BITRATES:
            bitrate_code = self._BITRATES[bitrate]
        else:
            bitrates = ", ".join(str(k) for k in self._BITRATES.keys())
            raise ValueError(f"Invalid bitrate, choose one of {bitrates}.")

        self.close()
        self._write(bitrate_code)
        buffer = self._read(0.1)
        self.open()

    def set_bitrate_reg(self, btr: str) -> None:
        """
        :param btr:
            BTR register value to set custom can speed
        """
        self.close()
        self._write("s" + btr)
        buffer = self._read(0.1)
        self.open()

def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""

    with slcanBus(
            bustype="slcan", 
            channel=args.tty,
            ttyBoudrate=args.boudrate) as bus:
        # bus = can.interface.Bus(bustype='ixxat', channel=0, bitrate=250000)
        # bus = can.interface.Bus(bustype='vector', app_name='CANalyzer', channel=0, bitrate=250000)

        # set to read-only, only supported on some interfaces
        # bus.state = BusState.PASSIVE

        # try:
        #     while True:
        #         msg = bus.recv(1)
        #         if msg is not None:
        #             print(msg)

        # except KeyboardInterrupt:
        #     pass  # exit normally
        print("opaaa hui na vilica")
        while True:
            msg = bus.recv(5)
            if msg is not None:
                print(msg)
            else:
                print('empty message')
            pass
        pass


if __name__ == "__main__":
    receive_all()
