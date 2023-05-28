from socket import timeout
import serial
import time
import re
import matplotlib.pyplot as plt


def main():
    timeout = 20
    data = str()
    red_samples = list()
    ir_samples = list()

    max30100 = serial.Serial(
        port="COM4",
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0,
    )

    timeout_start = time.time()
    while time.time() < timeout_start + timeout:
        char = max30100.read()
        if char:
            data += char.decode("utf-8")

    samples = data.split("\n\r")
    for sample in samples:
        result = re.search("Red:(.*) ", sample)
        if result:
            red_samples.append(int(result.group(1)))

        result = re.search("IR:(.*)", sample)
        if result:
            ir_samples.append(int(result.group(1)))

    plt.figure()
    plt.subplot(211)
    plt.title("Red samples")
    plt.plot(red_samples, '.')

    plt.subplot(212)
    plt.title("IR samples")
    plt.plot(ir_samples, '.')
    plt.show()
    return 0


if __name__ == "__main__":
    main()
