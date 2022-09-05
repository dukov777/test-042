import time
import can
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


# class slcanBus(sl.slcanBus):
#     pass

class slcanBus(sl.slcanBus):

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


bus = slcanBus(channel=args.tty,
               ttyBoudrate=args.boudrate, sleep_after_open=0)

msg = can.Message(arbitration_id=0x11223344, data=[1, 2, 3, 4, 5, 6, 7, 8], is_extended_id=True)
try:
    bus.send(msg)
    print(f"Message sent on {bus.channel_info}")
except can.CanError:
    print("Message NOT sent")

msg = can.Message(arbitration_id=0x001, data=[
                  0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8], is_extended_id=False)
try:
    bus.send(msg)
    print(f"Message sent on {bus.channel_info}")
except can.CanError:
    print("Message NOT sent")

time.sleep(0.1)
version = bus.get_version(1)
print(version)

time.sleep(0.1)
bus.set_bitrate(125000)

bus.shutdown()
