import logging
import sys
from enum import Enum, IntEnum
from time import perf_counter

from serial import (EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial,
                    SerialTimeoutException)

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
    LaserCurrent = "CM"  # Set laser current ##.# [mA] or C? C saves to memory
    LaserPower = "P"  # Set laser power ###.# [mW] Or PM?
    FiveSecEmissionDelay = "CDRH"  # Enable/Disable 5-second CDRH delay
    FaultCodeReset = "RST"  # Clears all fault codes or resets the laser unit (0)
    TemperatureRegulationLoop = "T"  # Set Temperature Regulation Loop
    PercentageSplit = "IPA"  # Set % split between lasers
    DigitalModulation = "TTL"  # Sets the digital high-speed modulation


class Query(StrEnum):
    DigitalModulation = "?TTL"
    EmmissionKeyStatus = "?KEY"
    LaserType = "INF?"
    USBConfiguration = "?CDC"
    LaserDriverControlMode = "?ACC"  # Request laser control mode
    FaultCode = "?F"  # Request fault code
    ExternalPowerControl = "?AM"  # Request external power control
    BasePlateTemperature = "?BT"  # Request baseplate temp
    FiveSecEmissionDelay = "?CDRH"  # Request 5-second CDRH Delay status
    LaserOperatingHours = "?HH"  # Request laser operating hours.
    LaserIdentification = "?HID"  # Request Laser type.
    LaserEmission = "?L"  # Request laser emission status.
    LaserPower = "?P"  # Request measured laser power.
    LaserPowerSetting = "?SP"  # Request desired laser power setpoint.
    MaximumLaserPower = "?MAXLP"  # Request maximum laser power.
    LaserCurrent = "?C"  # Request measured laser current
    LaserCurrentSetting = "?SC"  # Request desired laser current setpoint
    MaximumLaserCurrent = "?MAXLC"  # Request maximum laser current.
    InterlockStatus = "?INT"  # Request interlock status
    LaserVoltage = "?IV"  # Request measured laser voltage
    TemperatureRegulationLoopStatus = "?T"  # Request Temperature Regulation Loop status
    PercentageSplitStatus = "?IPA"


class FaultCodeField(IntEnum):
    NO_ALARM = (0,)
    DIODE_CURRENT = (1,)
    LASER_POWER = (2,)
    POWER_SUPPLY = (3,)
    DIODE_TEMPERATURE = (4,)
    BASE_TEMPERATURE = (5,)
    INTERLOCK = 7


# Laser State Representation
class OxxiusState(IntEnum):
    WARMUP = (0,)
    STANDBY = (2,)
    LASER_EMISSION_ACTIVE = (3,)
    INTERNAL_ERROR = (4,)
    FAULT = (5,)
    SLEEP = 6


class OxxiusUSBConfiguration(IntEnum):
    STANDARD_USB = 0
    VIRTUAL_SERIAL_PORT = 1


# Boolean command value that can also be compared like a boolean.
class BoolVal(StrEnum):
    OFF = "0"
    ON = "1"


OXXIUS_COM_SETUP = {
    "baudrate": 9600,
    "bytesize": EIGHTBITS,
    "parity": PARITY_NONE,
    "stopbits": STOPBITS_ONE,
    "xonxoff": False,
    "timeout": 1,
}

REPLY_TERMINATION = b"\r\n"


class OxxiusController:

    def __init__(self, port: str | Serial):
        """
        Initialize the OxxiusController.

        :param port: Serial port name or Serial object.
        :type port: str or Serial
        :raises SerialTimeoutException: If the device does not respond.
        """
        self.ser = Serial(port, **OXXIUS_COM_SETUP) if type(port) != Serial else port
        self.ser.reset_input_buffer()
        try:
            self.get(Query.LaserCurrent)
        except SerialTimeoutException:
            print(f"Connected to '{self.ser.port}' but the device is not responding.")
            raise

    @property
    def temperature(self) -> str:
        """
        Get the base plate temperature.

        :return: Base plate temperature.
        :rtype: str
        """
        return self.get(Query.BasePlateTemperature)

    @property
    def faults(self) -> list[FaultCodeField]:
        """
        Get the list of current fault codes.

        :return: List of fault code fields.
        :rtype: list[FaultCodeField]
        """
        faults = []
        fault_code = int(self.get(Query.FaultCode))
        fault_code_fields = iter(FaultCodeField)
        next(fault_code_fields)
        for index, field in enumerate(fault_code_fields):
            if bin(fault_code)[-1] == "1":
                faults.append(field)
            fault_code = fault_code >> 1
            return faults

    @property
    def serial_number(self) -> str:
        """
        Get the laser serial number.

        :return: Serial number.
        :rtype: str
        """
        return self.get(Query.LaserIdentification)

    def get(self, prefix: str, msg: Query) -> str:
        """
        Send a query command to the device.

        :param prefix: Command prefix.
        :type prefix: str
        :param msg: Query message.
        :type msg: Query
        :return: Device reply.
        :rtype: str
        """
        reply = self._send(f"{prefix}{msg.value}")
        return reply

    def set(self, prefix: str, msg: Cmd, value: str | float | BoolVal) -> str:
        """
        Send a set command to the device.

        :param prefix: Command prefix.
        :type prefix: str
        :param msg: Command message.
        :type msg: Cmd
        :param value: Value to set.
        :type value: str or float or BoolVal
        :return: Device reply.
        :rtype: str
        """
        return self._send(f"{prefix}{msg} {value}")

    def _send(self, msg: str, raise_timeout: bool = True) -> str:
        """
        Send a raw message to the device and return the reply.

        :param msg: Message to send.
        :type msg: str
        :param raise_timeout: Whether to raise on timeout.
        :type raise_timeout: bool, optional
        :raises SerialTimeoutException: If no reply is received in time.
        :return: Device reply.
        :rtype: str
        """
        self.ser.write(f"{msg}\r".encode("ascii"))
        start_time = perf_counter()
        reply = self.ser.read_until(REPLY_TERMINATION)
        if (
            not len(reply)
            and raise_timeout
            and perf_counter() - start_time > self.ser.timeout
        ):
            raise SerialTimeoutException
        return reply.rstrip(REPLY_TERMINATION).decode("utf-8")


class LCX(OxxiusController):

    def __init__(self, port: str | Serial, prefix: str):
        """
        Initialize the LCX laser controller.

        :param port: Serial port name or Serial object.
        :type port: str or Serial
        :param prefix: Command prefix for this laser.
        :type prefix: str
        """
        super().__init__(port)
        self.prefix = prefix
        self.log = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    @property
    def emission_status(self) -> BoolVal:
        """
        Get the emission status.

        :return: Emission status.
        :rtype: BoolVal
        """
        return BoolVal(self.get(self.prefix, Query.LaserEmission))

    def enable(self) -> None:
        """
        Enable laser emission.
        """
        self.set(self.prefix, Cmd.LaserEmission, BoolVal.ON)

    def disable(self) -> None:
        """
        Disable laser emission.
        """
        self.set(self.prefix, Cmd.LaserEmission, BoolVal.OFF)

    @property
    def max_power(self) -> str:
        """
        Get the maximum laser power.

        :return: Maximum laser power.
        :rtype: str
        """
        return self.get(self.prefix, Query.MaximumLaserPower)

    @property
    def power(self) -> str:
        """
        Get the current laser power.

        :return: Current laser power.
        :rtype: str
        """
        return self.get(self.prefix, Query.LaserPower)

    @power.setter
    def power(self, value: float) -> None:
        """
        Set the laser power.

        :param value: Power value to set.
        :type value: float
        """
        self.set(self.prefix, Cmd.LaserPower, value)

    @property
    def power_setpoint(self) -> str:
        """
        Get the power setpoint.

        :return: Power setpoint.
        :rtype: str
        """
        return self.get(self.prefix, Query.LaserPowerSetting)

    @power_setpoint.setter
    def power_setpoint(self, value: float) -> None:
        """
        Set the power setpoint.

        :param value: Power setpoint value.
        :type value: float
        """
        if 0 > value > self.max_power:
            reason = (
                f"exceeds maximum power output {self.max_power}mW"
                if value > self.max_power
                else f"is below 0mW"
            )
            self.log.error(f"Cannot set laser to {value}ml because it {reason}")
        else:
            self.set(self.prefix, Cmd.LaserPower, value)


class LBX(OxxiusController):

    def __init__(self, port: str | Serial, prefix: str):
        """
        Initialize the LBX laser controller.

        :param port: Serial port name or Serial object.
        :type port: str or Serial
        :param prefix: Command prefix for this laser.
        :type prefix: str
        """
        super().__init__(port)
        self.prefix = prefix
        self.log = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        @property
        def cdrh(self) -> BoolVal:
            """
            Get the status of the 5-second CDRH emission delay.

            :return: CDRH emission delay status.
            :rtype: BoolVal
            """
            status = self.get(self.prefix, Query.FiveSecEmissionDelay)
            return BoolVal(status)

        @cdrh.setter
        def cdrh(self, status: BoolVal) -> None:
            """
            Set the status of the 5-second CDRH emission delay.

            :param status: Desired CDRH emission delay status.
            :type status: BoolVal
            """
            self.set(self.prefix, Cmd.FiveSecEmissionDelay, status)

        @property
        def constant_current(self) -> BoolVal:
            """
            Get the constant current mode status.

            :return: Constant current mode status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.LaserDriverControlMode))

        @constant_current.setter
        def constant_current(self, value: BoolVal) -> None:
            """
            Set the constant current mode status.

            :param value: Desired constant current mode status.
            :type value: BoolVal
            """
            if value == BoolVal.OFF and self.digital_modulation == BoolVal.ON:
                self.log.warning(
                    f"Putting Laser {self.prefix} in constant power mode and disabling digital modulation mode"
                )
            self.set(self.prefix, Cmd.LaserDriverControlMode, value)

        @property
        def digital_modulation(self) -> BoolVal:
            """
            Get the digital modulation mode status.

            :return: Digital modulation mode status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.DigitalModulation))

        @digital_modulation.setter
        def digital_modulation(self, value: BoolVal) -> None:
            """
            Set the digital modulation mode status.

            :param value: Desired digital modulation mode status.
            :type value: BoolVal
            """
            # Note if laser in constant power mode, digital modulation can't be turned on
            if self.constant_current == BoolVal.OFF:
                self.log.warning(
                    f"Laser {self.prefix} is in constant power mode and cannot be put in digital modulation mode"
                )
            else:
                self.set(self.prefix, Cmd.DigitalModulation, value)

        @property
        def external_control_mode(self) -> BoolVal:
            """
            Get the external power control mode status.

            :return: External power control mode status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.ExternalPowerControl))

        @external_control_mode.setter
        def external_control_mode(self, value: BoolVal) -> None:
            """
            Set the external power control mode status.

            :param value: Desired external power control mode status.
            :type value: BoolVal
            """
            self.set(self.prefix, Cmd.ExternalPowerControl, value)

        @property
        def emission_status(self) -> BoolVal:
            """
            Get the laser emission status.

            :return: Laser emission status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.LaserEmission))

        def enable(self) -> None:
            """
            Enable laser emission.
            """
            self.set(self.prefix, Cmd.LaserEmission, BoolVal.ON)

        def disable(self) -> None:
            """
            Disable laser emission.
            """
            self.set(self.prefix, Cmd.LaserEmission, BoolVal.OFF)

        @property
        def max_power(self) -> str:
            """
            Get the maximum laser power.

            :return: Maximum laser power.
            :rtype: str
            """
            return self.get(self.prefix, Query.MaximumLaserPower)

        @property
        def power(self) -> str:
            """
            Get the current laser power.

            :return: Current laser power.
            :rtype: str
            """
            return self.get(self.prefix, Query.LaserPower)

        @property
        def power_setpoint(self) -> str:
            """
            Get the power setpoint.

            :return: Power setpoint.
            :rtype: str
            """
            return self.get(self.prefix, Query.LaserPowerSetting)

        @power_setpoint.setter
        def power_setpoint(self, value: float) -> None:
            """
            Set the power setpoint.

            :param value: Desired power setpoint.
            :type value: float
            """
            if 0 > value > self.max_power:
                reason = (
                    f"exceeds maximum power output {self.max_power}mW"
                    if value > self.max_power
                    else f"is below 0mW"
                )
                self.log.error(f"Cannot set laser to {value}ml because it {reason}")
            else:
                if self.constant_current == BoolVal.ON:
                    self.log.warning(
                        "Laser is in constant current mode so changing power will not change intensity"
                    )
                self.set(self.prefix, Cmd.LaserPower, value)

        @property
        def max_current(self) -> str:
            """
            Get the maximum laser current.

            :return: Maximum laser current.
            :rtype: str
            """
            return self.get(self.prefix, Query.MaximumLaserCurrent)

        @property
        def current(self) -> str:
            """
            Get the current laser current.

            :return: Current laser current.
            :rtype: str
            """
            return self.get(self.prefix, Query.LaserPower)

        @property
        def current_setpoint(self) -> str:
            """
            Get the current setpoint for laser current.

            :return: Current setpoint.
            :rtype: str
            """
            return self.get(self.prefix, Query.LaserCurrentSetting)

        @current_setpoint.setter
        def current_setpoint(self, value: float) -> None:
            """
            Set the current setpoint for laser current.

            :param value: Desired current setpoint.
            :type value: float
            """
            if 0 > value > 100:
                reason = "exceeds 100%" if value > self.max_power else f"is below 0%"
                self.log.error(f"Cannot set laser to {value}ml because it {reason}")
            else:
                if self.constant_current == BoolVal.OFF:
                    self.log.warning(
                        "Laser is in constant power mode so changing power will not change intensity"
                    )
                self.set(self.prefix, Cmd.LaserCurrent, value)

    class L6CCCombiner(OxxiusController):

        def __init__(self, port: str):
            """
            Initialize the L6CCCombiner.

            :param port: Serial port name or Serial object.
            :type port: str
            """
            super().__init__(port)
            self.log = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        @property
        def percentage_split(self) -> str:
            """
            Get the percentage split between lasers.

            :return: Percentage split.
            :rtype: str
            """
            return self.get(self.prefix, Query.PercentageSplitStatus)

        @percentage_split.setter
        def percentage_split(self, value: float) -> None:
            """
            Set the percentage split between lasers.

            :param value: Desired percentage split (0-100).
            :type value: float
            """
            if value > 100 or value < 0:
                self.log.error(f"Impossible to set percentage spilt to {value}")
                return
            self.set(self.prefix, Cmd.PercentageSplit, value)

        @property
        def port_configuration(self) -> OxxiusUSBConfiguration:
            """
            Get the USB port configuration.

            :return: USB port configuration.
            :rtype: OxxiusUSBConfiguration
            """
            configuration = self.get(self.prefix, Query.USBConfiguration)
            return OxxiusUSBConfiguration(configuration)

        @property
        def cdrh(self) -> BoolVal:
            """
            Get the status of the 5-second CDRH emission delay.

            :return: CDRH emission delay status.
            :rtype: BoolVal
            """
            status = self.get(self.prefix, Query.FiveSecEmissionDelay)
            return BoolVal(status)

        @cdrh.setter
        def cdrh(self, status: BoolVal) -> None:
            """
            Set the status of the 5-second CDRH emission delay.

            :param status: Desired CDRH emission delay status.
            :type status: BoolVal
            """
            self.set(self.prefix, Cmd.FiveSecEmissionDelay, status)

        @property
        def laser_type(self) -> str:
            """
            Get the laser type.

            :return: Laser type.
            :rtype: str
            """
            return self.get(self.prefix, Query.LaserType)

        @property
        def interlock_status(self) -> BoolVal:
            """
            Get the interlock status.

            :return: Interlock status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.InterlockStatus))

        @property
        def emmision_key_status(self) -> BoolVal:
            """
            Get the emission key status.

            :return: Emission key status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.EmmissionKeyStatus))

        @property
        def LBX_constant_current_status(self) -> BoolVal:
            """
            Get the constant current status for LBX.

            :return: Constant current status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(self.prefix, Query.LaserDriverControlMode))

        @LBX_constant_current_status.setter
        def LBX_constant_current_status(self, status: BoolVal) -> None:
            """
            Set the constant current status for LBX.

            :param status: Desired constant current status.
            :type status: BoolVal
            """
            self.set(self.prefix, Cmd.LaserDriverControlMode, status)

        def digital_modualtion(self, prefix: str) -> BoolVal:
            """
            Get the digital modulation status for a given prefix.

            :param prefix: Command prefix for the laser.
            :type prefix: str
            :return: Digital modulation status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(prefix, Query.DigitalModulation))

        def set_digital_modulation(self, prefix: str, value: BoolVal) -> None:
            """
            Set the digital modulation status for a given prefix.

            :param prefix: Command prefix for the laser.
            :type prefix: str
            :param value: Desired digital modulation status.
            :type value: BoolVal
            """
            # If laser is in constant power mode, then digital modulation can't be turned on
            if self.get(prefix, Query.LaserDriverControlMode) == BoolVal.OFF:
                self.log.warning(
                    f"Laser {prefix} is in constant power mode and cannot be put in digital modulation mode"
                )
            else:
                self.set(Cmd.DigitalModulation + prefix, value)

        def external_control_mode(self, prefix: str) -> BoolVal:
            """
            Get the external power control mode status for a given prefix.

            :param prefix: Command prefix for the laser.
            :type prefix: str
            :return: External power control mode status.
            :rtype: BoolVal
            """
            return BoolVal(self.get(prefix, Query.ExternalPowerControl))

        def set_external_control_mode(self, prefix: str, value: BoolVal) -> None:
            """
            Set the external power control mode status for a given prefix.

            :param prefix: Command prefix for the laser.
            :type prefix: str
            :param value: Desired external power control mode status.
            :type value: BoolVal
            """
            self.set(prefix, Cmd.ExternalPowerControl, value)

    @property
    def emmision_key_status(self) -> BoolVal:
        """
        Get the emission key status.

        :return: Emission key status.
        :rtype: BoolVal
        """
        return BoolVal(self.get(self.prefix, Query.EmmissionKeyStatus))

    @property
    def LBX_constant_current_status(self) -> BoolVal:
        """
        Get the constant current status for LBX.

        :return: Constant current status.
        :rtype: BoolVal
        """
        return BoolVal(self.get(self.prefix, Query.LaserDriverControlMode))

    @LBX_constant_current_status.setter
    def LBX_constant_current_status(self, status: BoolVal) -> None:
        """
        Set the constant current status for LBX.

        :param status: Desired constant current status.
        :type status: BoolVal
        """
        self.set(self.prefix, Cmd.LaserDriverControlMode, status)

    def digital_modulation(self, prefix: str) -> BoolVal:
        """
        Get the digital modulation status for a given prefix.

        :param prefix: Command prefix for the laser.
        :type prefix: str
        :return: Digital modulation status.
        :rtype: BoolVal
        """
        return BoolVal(self.get(prefix, Query.DigitalModulation))

    def set_digital_modulation(self, prefix: str, value: BoolVal) -> None:
        """
        Set the digital modulation status for a given prefix.

        :param prefix: Command prefix for the laser.
        :type prefix: str
        :param value: Desired digital modulation status.
        :type value: BoolVal
        """
        # If laser is in constant power mode, then digital modulation can't be turned on
        if self.get(prefix, Query.LaserDriverControlMode) == BoolVal.OFF:
            self.log.warning(
                f"Laser {prefix} is in constant power mode and cannot be put in digital modulation mode"
            )
        else:
            self.set(Cmd.DigitalModulation + prefix, value)

    def external_control_mode(self, prefix: str) -> BoolVal:
        """
        Get the external power control mode status for a given prefix.

        :param prefix: Command prefix for the laser.
        :type prefix: str
        :return: External power control mode status.
        :rtype: BoolVal
        """
        return BoolVal(self.get(prefix, Query.ExternalPowerControl))

    def set_external_control_mode(self, prefix: str, value: BoolVal) -> None:
        """
        Set the external power control mode status for a given prefix.

        :param prefix: Command prefix for the laser.
        :type prefix: str
        :param value: Desired external power control mode status.
        :type value: BoolVal
        """
        self.set(prefix, Cmd.ExternalPowerControl, value)
