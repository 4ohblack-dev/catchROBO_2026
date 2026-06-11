import serial
import struct
import time

Serial_port = "COM3"
Boudrate = 921600

HEADER = b'\xAA'
DATA_FORMAT = '<ff'
DATA_SIZE = struct.calcsize(DATA_FORMAT)
PACKET_SIZE = len(HEADER) + DATA_SIZE + 1

def calculateCRC(data:bytes) ->int:#引数はbytes 型、戻り値はint で、１バイトの整数を返す
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def main():
    try:
        ser=serial.Serial(Serial_port,Boudrate,timeout=0.1)
        time.sleep(2)
        dx,dy =1.5,-0.8
        while True:
            data_bytes = struct.pack(DATA_FORMAT,dx,dy)
            crc=calculateCRC(data_bytes)

            packet=HEADER + data_bytes + bytes([crc])
            ser.write(packet)

            if ser.in_waiting >= PACKET_SIZE:
                if ser.read(1==HEADER):
                    rx_data_bytes = ser.read(DATA_SIZE)
                    rx_crc=ser.read(1)[0]
                    if calculateCRC(rx_data_bytes)==rx_crc:
                        unpacked = struct.unpack(DATA_FORMAT,rx_data_bytes)
                        print(f"Echo Received -> dx: {unpacked[0]:.2f}, dy: {unpacked[1]:.2f}")
                    else:
                        print("CRC Error!")
            

            # テスト用に値を適当に変える
            dx = 0.5 if dx > 5.0 else dx + 0.1
            time.sleep(0.01) # 100Hz周期



    except KeyboardInterrupt:
        print("\nClosing port.")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()