import serial
import struct


class Interface:
    """
    Interface class to interact with the servo controller board via serial communication.

    Attributes:
        dev (str): The device path for the serial connection.
        baud (int): The baud rate for the serial connection.
        ser (serial.Serial): The serial connection instance.
    """

    def __init__(self, dev='/dev/ttyACM0', baud=115200):
        """
        Initializes the Interface with the given device and baud rate.

        Args:
            dev (str): The device path for the serial connection (default is '/dev/ttyACM0').
            baud (int): The baud rate for the serial connection (default is 115200).
        """
        self.dev = dev
        self.baud = baud
        self.ser = None

    def open(self):
        """Opens the serial connection to the device."""
        self.ser = serial.Serial(self.dev, self.baud)

    def close(self):
        """Closes the serial connection."""

        # Detach the servos before closing
        self.detach_servos()

        self.ser.close()

    def __enter__(self):
        """Context manager entry method to open the connection."""
        self.open()

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit method to close the connection."""
        self.close()

    def send_command(self, opcode, *args):
        """
        Sends a command with an opcode and optional arguments.

        Args:
            opcode (int): The command opcode to be sent.
            *args: Additional arguments to be sent after the opcode.
        """
        self.ser.write(bytes([opcode]))
        for arg in args:
            self.ser.write(arg)

    def get_voltage(self):
        """
        Sends a command to read the voltage from the controller.

        Returns:
            float: The voltage value as a floating-point number.
        """
        self.send_command(0x01)
        response = self.ser.read(4)  # Float is 4 bytes
        return struct.unpack('f', response)[0]

    def get_current(self):
        """
        Sends a command to read the current from the controller.

        Returns:
            float: The current value as a floating-point number.
        """
        self.send_command(0x02)
        response = self.ser.read(4)  # Float is 4 bytes
        return struct.unpack('f', response)[0]

    def read_sensor(self, pin):
        """
        Sends a command to read a sensor value from the specified pin.

        Args:
            pin (int): The pin number to read from.

        Returns:
            float: The sensor value as a floating-point number.
        """
        self.send_command(0x03, bytes([pin]))
        response = self.ser.read(4)  # Float is 4 bytes
        return struct.unpack('f', response)[0]

    def set_led(self, pin, r=0, g=0, b=255):
        """
        Sends a command to set the LED color on the specified pin.

        Args:
            pin (int): The pin number for the LED.
            r (int): Red value (default 0).
            g (int): Green value (default 0).
            b (int): Blue value (default 255).

        Returns:
            bytes: The response from the controller.
        """
        self.send_command(0x04, bytes([pin, r, g, b]))
        return self.ser.read(1)

    def set_leds(self, pins, arr_rgb_tuples):
        """
        Sends a command to set the LED colors for multiple pins.

        Args:
            pins (list): List of pin numbers for the LEDs.
            arr_rgb_tuples (list): List of RGB tuples corresponding to each pin.

        Returns:
            bytes: The response from the controller.
        """
        comm = [len(pins)] + [elem for pin, rgb in zip(pins, arr_rgb_tuples) for elem in [pin] + list(rgb)]
        args = bytes(comm)
        self.send_command(0x05, args)
        return self.ser.read(1)

    def attach_servos(self):
        """
        Sends a command to attach all servos.

        Returns:
            bytes: The response from the controller.
        """
        self.send_command(0x06)
        return self.ser.read(1)

    def detach_servos(self):
        """
        Sends a command to detach all servos.

        Returns:
            bytes: The response from the controller.
        """
        self.send_command(0x07)
        return self.ser.read(1)

    def set_pulse(self, pin, pulse):
        """
        Sends a command to set the pulse width for a servo on the specified pin.

        Args:
            pin (int): The pin number of the servo.
            pulse (float): The pulse width to set (in microseconds).

        Returns:
            bytes: The response from the controller.
        """
        pulse_bytes = struct.pack('f', pulse)
        self.send_command(0x08, bytes([pin]), pulse_bytes)
        return self.ser.read(1)

    def set_pulses(self, pins, pulses):
        """
        Sends a command to set the pulse widths for multiple servos.

        Args:
            pins (list): List of pin numbers for the servos.
            pulses (list): List of pulse widths (in microseconds) for each servo.

        Returns:
            bytes: The response from the controller.
        """
        comm = [len(pins)] + [byte for pin, value in zip(pins, pulses) for byte in [pin] + list(struct.pack('f', value))]
        args = bytes(comm)
        self.send_command(0x09, args)
        return self.ser.read(1)

    def set_angle(self, pin, angle):
        """
        Sends a command to set the angle of a servo on the specified pin.

        Args:
            pin (int): The pin number of the servo.
            angle (float): The angle to set for the servo.

        Returns:
            bytes: The response from the controller.
        """
        pulse_bytes = struct.pack('f', angle)
        self.send_command(0x0A, bytes([pin]), pulse_bytes)
        return self.ser.read(1)

    def set_angles(self, pins, angles):
        """
        Sends a command to set the angles for multiple servos.

        Args:
            pins (list): List of pin numbers for the servos.
            angles (list): List of angles to set for each servo.

        Returns:
            bytes: The response from the controller.
        """
        comm = [len(pins)] + [byte for pin, value in zip(pins, angles) for byte in [pin] + list(struct.pack('f', value))]
        args = bytes(comm)
        self.send_command(0x0B, args)
        return self.ser.read(1)


if __name__ == '__main__':

    """
    Test the interface class by sending some commands. Attach the servos, set a servo (0) to home position and detach the servos.
    """

    import time

    controller = Interface()
    with controller:
        controller.attach_servos()
        time.sleep(1)

        angle = 0
        for i in range(18):

            print(f'Setting servo [{i}] to {angle}')
            controller.set_angle(i, angle)
            time.sleep(0.5)

        controller.detach_servos()
