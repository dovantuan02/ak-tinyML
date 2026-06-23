#!/usr/bin/env python3
"""
PCM 16kHz 16-bit receiver over UART.

Reads raw little-endian PCM samples from serial (921600 baud),
plots waveform in real-time, and optionally saves to WAV file.

Usage:
    python3 pcm_receiver.py                          # default: /dev/ttyUSB0
    python3 pcm_receiver.py /dev/ttyACM0
    python3 pcm_receiver.py /dev/ttyUSB0 --save      # save to file.wav
    python3 pcm_receiver.py /dev/ttyUSB0 --out recording.wav
    python3 pcm_receiver.py --list                   # list available ports
"""

import argparse
import struct
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing pyserial. Install: pip install pyserial")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("Missing numpy. Install: pip install numpy")
    sys.exit(1)

# Match STM32 firmware
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2  # bytes (int16)
BAUDRATE = 921600


def write_wav(path: str, samples: np.ndarray, fs: int):
    """Write int16 PCM samples to WAV file."""
    import wave
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(samples.astype("<i2").tobytes())
    print(f"\nSaved {len(samples)} samples ({len(samples)/fs:.1f}s) to {path}")


class WavePlotter:
    """Real-time waveform plotter with scrolling."""

    def __init__(self, window_sec: float = 0.5):
        self.window = int(window_sec * SAMPLE_RATE)
        self.buf = np.zeros(self.window, dtype=np.int16)
        self.idx = 0
        self._fig = None
        self._ax = None
        self._line = None

    def push(self, sample: int):
        self.buf[self.idx] = sample
        self.idx = (self.idx + 1) % self.window
        # When wrapped, fill order: [oldest..newest]
        if self.buf[-1] != 0:  # buffer fully filled at least once
            pass

    def update(self):
        """Refresh plot with current buffer (rolling display)."""
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            return

        if self._fig is None:
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=(10, 3))
            self._ax.set_ylim(-32768, 32767)
            self._ax.set_xlabel("Sample")
            self._ax.set_ylabel("Amplitude")
            self._ax.set_title("PCM 16kHz — Real-time")
            self._ax.grid(True, alpha=0.3)
            # Show last window samples
            xs = np.arange(self.window)
            self._line, = self._ax.plot(xs, self.buf)
            self._fig.canvas.draw()
            self._fig.canvas.flush_events()
        else:
            # Re-order circular buffer for display: [idx..end] + [0..idx]
            ordered = np.concatenate((self.buf[self.idx:], self.buf[:self.idx]))
            self._line.set_ydata(ordered)
            self._ax.relim()
            self._ax.autoscale_view(scalex=False)
            self._fig.canvas.draw()
            self._fig.canvas.flush_events()

    def close(self):
        if self._fig:
            import matplotlib.pyplot as plt
            plt.ioff()
            plt.close(self._fig)


def find_stm32_port() -> str | None:
    """Auto-detect STM32 serial port."""
    for p in list_ports.comports():
        # Common STM32 VID/PID or description
        if p.vid == 0x0483:  # STMicroelectronics
            return p.device
        if "STM32" in (p.description or "").upper():
            return p.device
        if "STLink" in (p.description or ""):
            return p.device
    # Try common names
    for guess in ["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyS0"]:
        if Path(guess).exists():
            return guess
    return None


def main():
    parser = argparse.ArgumentParser(description="Receive 16kHz PCM from STM32 over UART")
    parser.add_argument("port", nargs="?", default=None, help="Serial port")
    parser.add_argument("-b", "--baud", type=int, default=BAUDRATE)
    parser.add_argument("-o", "--out", default=None, help="Save to WAV file")
    parser.add_argument("--save", action="store_true", help="Save to auto-named WAV")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--no-plot", action="store_true", help="Disable real-time plotting")
    parser.add_argument("--duration", type=float, default=0, help="Record N seconds then exit (0=unlimited)")
    args = parser.parse_args()

    if args.list:
        print("Available serial ports:")
        for p in list_ports.comports():
            print(f"  {p.device} — {p.description} (VID={p.vid:04X} PID={p.pid:04X})")
        return

    port = args.port or find_stm32_port()
    if not port:
        print("ERROR: No serial port specified and none auto-detected.")
        print("  Pass a port: python3 pcm_receiver.py /dev/ttyUSB0")
        print("  Or list ports: python3 pcm_receiver.py --list")
        sys.exit(1)

    # Connect
    print(f"Opening {port} @ {args.baud} baud ...")
    ser = serial.Serial(port, args.baud, timeout=1.0)
    time.sleep(0.5)  # let STM32 finish reset
    ser.reset_input_buffer()
    print(f"Connected. Receiving {SAMPLE_RATE} Hz PCM ...")

    # Setup
    frame_size = SAMPLE_WIDTH  # 2 bytes per sample
    buf = bytearray()
    samples = []
    plotter = WavePlotter() if not args.no_plot else None
    start = time.time()

    # WAV save path
    wav_path = args.out or ("recording.wav" if args.save else None)

    try:
        while True:
            chunk = ser.read(max(256, frame_size))
            if not chunk:
                continue
            buf.extend(chunk)

            # Parse complete frames
            while len(buf) >= frame_size:
                raw = buf[:frame_size]
                buf = buf[frame_size:]
                sample = struct.unpack("<h", raw)[0]  # little-endian int16
                # print("Rev sample {sample.size}\n")
                samples.append(sample)

                if plotter:
                    plotter.push(sample)

            # Update plot periodically
            if plotter and len(samples) % 256 == 0:
                plotter.update()

            # Check duration limit
            if args.duration > 0 and (time.time() - start) >= args.duration:
                print(f"\nReached {args.duration}s recording limit.")
                break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()
        if plotter:
            plotter.close()

    elapsed = time.time() - start
    n = len(samples)
    print(f"\nReceived {n} samples ({n/SAMPLE_RATE:.1f}s) in {elapsed:.1f}s real time")
    if n > 0 and n / SAMPLE_RATE > elapsed * 0.8:
        print("✓ Real-time streaming — no buffer underrun")
    else:
        print("⚠ Samples slower than real-time — check baud rate or USB latency")

    # Save WAV
    if wav_path and n > 0:
        arr = np.array(samples, dtype=np.int16)
        write_wav(wav_path, arr, SAMPLE_RATE)


if __name__ == "__main__":
    main()
