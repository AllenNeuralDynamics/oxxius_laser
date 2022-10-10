"""Oxxius Laser Driver."""

import sys
from enum import Enum, IntEnum
from functools import cache
from time import perf_counter
from serial import Serial, EIGHTBITS, STOPBITS_ONE, PARITY_NONE, \
    SerialTimeoutException

# Define StrEnums if they don't yet exist.
if sys.version_info < (3, 11):
    class StrEnum(str, Enum):
        pass
else:
    from enum import StrEnum


class Cmd(StrEnum):
    LaserDriverControlMode = "ACC"  # Set laser mode: [Power=0, Current=1]
    ExternalPowerControl = "AM"  # Enable(1)/Disable(0) External power control
    LaserEmission = "L"  # Enable/Disable Laser Emission. Or DL?
    LaserCurrent = "CM"  # Set laser current ## [%] or C? C saves to memory
    LaserPower = "PM"  # Set laser power ###.# [mW] Or PM?
    FiveSecEmissionDelay = "CDRH"  # Enable/Disable 5-second CDRH delay
    FaultCodeReset = "RST"  # Clears all fault codes or resets the laser unit (0)
    TemperatureRegulationLoop = "T"  # Set Temperature Regulation Loop


class Query(StrEnum):
    FaultCode = "?F"  # Request fault code
    ExternalPowerControl = "?AM"  # Request external power control
    BasePlateTemperature = "?BT"
    FiveSecEmissionDelay = "?CDRH"  # Request 5-second CDRH Delay status
    LaserOperatingHours = "?HH"  # Request laser operating hours.
    LaserIdentification = "INF?"  # Request Laser type.
    LaserEmission = "?L"  # Request laser emission status.
    LaserPower = "?P"  # Request measured laser power.
    LaserPowerSetting = "?SP"  # Request desired laser power setpoint.
    MaximumLaserPower = "?MAXLP"  # Request maximum laser power.
    LaserCurrent = "?C"  # Request measured laser current
    LaserCurrentSetting = "?SC"  # Request desired laser current setpoint
    MaximumLaserCurrent = "?MAXLC"  # Request maximum laser current.
    InterlockStatus = "?INT"  # Request interlock status
    LaserVoltage = "?IV"  # Request measured laser current
    TemperatureRegulationLoopStatus = "?T"  # Request Temperature Regulation Loop status


# Requesting a FaultCode will return a 16-bit number who's bitfields
# represent which faults are active.
# Many fields (bits) can be asserted at once.
class FaultCodeField(IntEnum):
    NO_ALARM = 0,
    DIODE_CURRENT = 1,
    LASER_POWER = 2,
    POWER_SUPPLY = 3,
    DIODE_TEMPERATURE = 4,
    BASE_TEMPERATURE = 5,
    INTERLOCK = 7


# Laser State Representation
class OxxiusState(IntEnum):
    WARMUP = 0,
    STANDBY = 2,
    LASER_EMISSION_ACTIVE = 3,
    INTERNAL_ERROR = 4,
    FAULT = 5,
    SLEEP = 6


# Boolean command value that can also be compared like a boolean.
class BoolVal(StrEnum):
    OFF = "0"
    ON = "1"

    def __bool__(self):
        return self.value == "1"


OXXIUS_COM_SETUP = \
    {
        "baudrate": 9600,
        "bytesize": EIGHTBITS,
        "parity": PARITY_NONE,
        "stopbits": STOPBITS_ONE,
        "xonxoff": False,
        "timeout": 1
    }


class OxxiusLaser:
    REPLY_TERMINATION = b'\r\n'

    def __init__(self, port: str = "COM7"):
        self.ser = Serial(port, **OXXIUS_COM_SETUP)
        self.ser.reset_input_buffer()
        # Since we're likely connected over an RS232-to-usb-serial interface,
        # ask for some sort of reply to make sure we're not timing out.
        try:
            # Put the interface into a known state to simplify communication.
            self.reset_laser()
        except SerialTimeoutException:
            print(f"Connected to '{port}' but the device is not responding.")
            raise

    # Convenience functions
    def enable(self):
        """Enable emission."""
        self.set(Cmd.LaserEmission, BoolVal.ON)

    def disable(self):
        """disable emission."""
        self.set(Cmd.LaserEmission, BoolVal.OFF)

    # @property
    # @cache
    # def wavelength(self):
    #     """return the current wavelength."""
    #     return int(self.get(Query.LaserWavelength))

    @property
    def temperature(self):
        """Return the current temperature as measured from the base plate."""
        return self.get(Query.BasePlateTemperature)

    @property
    def state(self) -> OxxiusState:
        """Return the laser state as a OxxiusState Enum.
        Note: The "FAULT" state encompasses many cases. A list of
            all specific fault codes can be accessed with get_faults().
        """
        fault_code = int(self.get(Query.FaultCode))
        # All Fault Codes >=4 represent some sort of issue.
        # Fault Codes <4 relate to laser state.
        if fault_code > OxxiusState.FAULT.value:
            return OxxiusState.FAULT
        return OxxiusState(fault_code)

    @property
    def interlock_is_closed(self):
        """True if the key is turned and laser is armed; False otherwise."""
        return True if BoolVal(self.get(Query.InterlockStatus)) else False

    @property
    def laser_is_emitting(self):
        """True if the laser is emitting. False otherwise."""
        return True if BoolVal(self.get(Query.LaserEmission)) else False

    def disable_cdrh(self):
        """disable 5-second delay"""
        self.set(Cmd.FiveSecEmissionDelay, BoolVal.OFF)

    def set_external_power_control(self):
        """Configure the laser to be controlled by an external analog input.
        0 to max output power is linearlly mapped to an analog voltage of 0-5V
        where any present power is ignored (datasheet, pg67).
        """
        self.set(Cmd.ExternalPowerControl, BoolVal.ON)

    def get_faults(self):
        """return a list of faults or empty list if no faults are present."""
        faults = []
        try:
            fault_code = int(self.get(Query.FaultCode))
        except ValueError:
            return None
        # Skip first Enum (LASER_EMISSION_ACTIVE), which is not really a fault.
        fault_code_fields = iter(FaultCodeField)
        next(fault_code_fields)
        for index, field in enumerate(fault_code_fields):
            if bin(fault_code)[-1] == '1':
                faults.append(field)
            fault_code = fault_code >> 1
            return faults

    def reset_laser(self):
        """Resets the laser unit"""

        self.set(Cmd.FaultCodeReset, BoolVal.OFF)

    # Low level Interface. All commands and queries can be accessed
    # through the get/set interface.
    def get(self, setting: Query) -> str:
        """Request a setting from the device."""
        reply = self._send(setting.value)
        return reply.lstrip(f"?{setting}= ")

    def set(self, cmd: Cmd, value: str) -> str:
        return self._send(f"{cmd} {value}")

    def _send(self, msg: str, raise_timeout: bool = True) -> str:
        """send a message and return the reply.
        :param msg: the message to send in string format
        :param raise_timeout: bool to indicate if we should raise an exception
            if we timed out.
        :returns: the reply (without line formatting chars) in str format
            or emptystring if no reply. Raises a timeout exception if flagged
            to do so.
        """
        # Note: Timing out on a serial port read does not throw an exception,
        #   so we need to do this manually.

        # All outgoing commands are bookended with a '\r\n' at the beginning
        # and end of the message.
        self.ser.write(f"{msg}\r".encode('ascii'))
        start_time = perf_counter()
        # Read the first '\r\n'.
        reply = self.ser.read_until(OxxiusLaser.REPLY_TERMINATION)
        # Raise a timeout if we got no reply and have been flagged to do so.
        if not len(reply) and raise_timeout and \
                perf_counter() - start_time > self.ser.timeout:
            raise SerialTimeoutException
        return reply.rstrip(OxxiusLaser.REPLY_TERMINATION).decode('utf-8')


"""Class for 638 Laser"""


class L1Laser(OxxiusLaser):
    def _send(self, msg: str, raise_timeout: bool = True) -> str:
        """send a message and return the reply.
        :param msg: the message to send in string format
        :param raise_timeout: bool to indicate if we should raise an exception
            if we timed out.
        :returns: the reply (without line formatting chars) in str format
            or emptystring if no reply. Raises a timeout exception if flagged
            to do so.
        """
        # Note: Timing out on a serial port read does not throw an exception,
        #   so we need to do this manually.

        # All outgoing commands are bookended with a '\r\n' at end of the message.
        self.ser.write(f"L1 {msg}\r".encode('ascii'))
        start_time = perf_counter()
        # Read the first '\r\n'.
        reply = self.ser.read_until(OxxiusLaser.REPLY_TERMINATION)
        # Raise a timeout if we got no reply and have been flagged to do so.
        if not len(reply) and raise_timeout and \
                perf_counter() - start_time > self.ser.timeout:
            raise SerialTimeoutException
        return reply.rstrip(OxxiusLaser.REPLY_TERMINATION).decode('utf-8')


"""Class for 561 Laser"""


class L3Laser(OxxiusLaser):
    def _send(self, msg: str, raise_timeout: bool = True) -> str:
        """send a message and return the reply.
        :param msg: the message to send in string format
        :param raise_timeout: bool to indicate if we should raise an exception
            if we timed out.
        :returns: the reply (without line formatting chars) in str format
            or emptystring if no reply. Raises a timeout exception if flagged
            to do so.
        """
        # Note: Timing out on a serial port read does not throw an exception,
        #   so we need to do this manually.

        # All outgoing commands are bookended with a '\r\n' at end of the message.
        self.ser.write(f"L3 {msg}\r".encode('ascii'))
        start_time = perf_counter()
        # Read the first '\r\n'.
        reply = self.ser.read_until(OxxiusLaser.REPLY_TERMINATION)
        # Raise a timeout if we got no reply and have been flagged to do so.
        if not len(reply) and raise_timeout and \
                perf_counter() - start_time > self.ser.timeout:
            raise SerialTimeoutException
        return reply.rstrip(OxxiusLaser.REPLY_TERMINATION).decode('utf-8')


"""Class for 488 Laser"""


class L5Laser(OxxiusLaser):
    def _send(self, msg: str, raise_timeout: bool = True) -> str:
        """send a message and return the reply.
        :param msg: the message to send in string format
        :param raise_timeout: bool to indicate if we should raise an exception
            if we timed out.
        :returns: the reply (without line formatting chars) in str format
            or emptystring if no reply. Raises a timeout exception if flagged
            to do so.
        """
        # Note: Timing out on a serial port read does not throw an exception,
        #   so we need to do this manually.

        # All outgoing commands are bookended with a '\r\n' at end of the message.
        self.ser.write(f"L5 {msg}\r".encode('ascii'))
        start_time = perf_counter()
        # Read the first '\r\n'.
        reply = self.ser.read_until(OxxiusLaser.REPLY_TERMINATION)
        # Raise a timeout if we got no reply and have been flagged to do so.
        if not len(reply) and raise_timeout and \
                perf_counter() - start_time > self.ser.timeout:
            raise SerialTimeoutException
        return reply.rstrip(OxxiusLaser.REPLY_TERMINATION).decode('utf-8')


"""Class for 405 Laser"""


class L6Laser(OxxiusLaser):
    def _send(self, msg: str, raise_timeout: bool = True) -> str:
        """send a message and return the reply.
        :param msg: the message to send in string format
        :param raise_timeout: bool to indicate if we should raise an exception
            if we timed out.
        :returns: the reply (without line formatting chars) in str format
            or emptystring if no reply. Raises a timeout exception if flagged
            to do so.
        """
        # Note: Timing out on a serial port read does not throw an exception,
        #   so we need to do this manually.

        # All outgoing commands are bookended with a '\r\n' at end of the message.
        self.ser.write(f"L6 {msg}\r".encode('ascii'))
        start_time = perf_counter()
        # Read the first '\r\n'.
        reply = self.ser.read_until(OxxiusLaser.REPLY_TERMINATION)
        # Raise a timeout if we got no reply and have been flagged to do so.
        if not len(reply) and raise_timeout and \
                perf_counter() - start_time > self.ser.timeout:
            raise SerialTimeoutException
        return reply.rstrip(OxxiusLaser.REPLY_TERMINATION).decode('utf-8')
