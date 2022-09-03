import serial
import argparse

parser = argparse.ArgumentParser(description='Connect to SLCAN port.')
parser.add_argument('tty', type=str, 
                    help='/dev/tty.usbmodem12303')
parser.add_argument('--boudrate', type=int, default=115200, help='115200')

args = parser.parse_args()

ser = serial.Serial(args.tty, args.boudrate, timeout=1)

slcan_command = b"t0012aabb\r"
ser.write(slcan_command)

print("sent = " + slcan_command.decode())
reply = ser.read(1)
print("received = " + str(int.from_bytes(reply, 'little')))

ser.close()
