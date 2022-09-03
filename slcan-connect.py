import time
import can.interfaces.slcan as sl
from can.exceptions import (
    CanInterfaceNotImplementedError,
    CanInitializationError,
    CanOperationError,
    error_check,
)
from can import typechecking


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


bus = slcanBus(channel='/dev/tty.usbmodem12303',
                  ttyBoudrate=115200, sleep_after_open=0)

time.sleep(0.1)
version = bus.get_version(1)
print(version)

time.sleep(0.1)
bus.set_bitrate(1000000)

bus.shutdown()
