#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy
import scipy
import scipy.io
import scipy.io.wavfile
import scipy.signal
import scipy.fftpack
import struct
import sys
import wave

PACKET = struct.Struct('<bH')
def plot(name):
    if name.endswith('.wav'):
        data = scipy.io.wavfile.read(name)
        encoder = list(data[1])
        SAMPLE_RATE_HZ = data[0]
    else:
        SAMPLE_RATE_HZ = 40000

        all_data = open(name, 'rb').read()

        # Our data has a 0x5a header with 2 data bytes.  Read a section,
        # and look for an offset that has the header at the appropriate
        # intervals.
        for i in range(PACKET.size):
            # Take every size'th element starting at the given offset.
            maybe_all_header = all_data[slice(i, PACKET.size*20, PACKET.size)]
            if maybe_all_header == bytes([0x5a])* len(maybe_all_header):
                all_data = all_data[i:]
                break
        else:
            print("Could not find structure")

        data = [PACKET.unpack(all_data[i:i+PACKET.size])
                for i in range(0, len(all_data) - PACKET.size, PACKET.size)]
        N = len(data)
        encoder = [x[1] for x in data]

    # Remove any DC bias... we will assume that we aren't close to a
    # wraparound point.
    avg = sum(encoder) / len(encoder)
    encoder = [x - avg for x in encoder]

    # Print a value for sanity checking
    print(encoder[0], encoder[1])

    f, Pxx = scipy.signal.welch(encoder, fs = SAMPLE_RATE_HZ, nperseg = 100000)
    plt.loglog(f, numpy.sqrt(Pxx), label=name)


def main():
    for name in sys.argv[1:]:
        plot(name)

    plt.xlabel('Frequency Hz')
    plt.ylabel('PSD')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
