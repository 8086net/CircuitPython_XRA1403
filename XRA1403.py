# SPDX-FileCopyrightText: Copyright (c) 2024 Chris Burton
#
# SPDX-License-Identifier: MIT

"""
`xra1403`
====================================================

CircuitPython module for the XRA1403 SPI 16-pin I/O extender.
Port A is pins 0-7, port B is pins 8-15.

* Author(s): Chris Burton
"""


_XRA1403_REGISTER_GSR1 = const(0x00) # GPIO State for P0-P7
_XRA1403_REGISTER_GSR2 = const(0x01) # GPIO State for P8-P15
_XRA1403_REGISTER_OCR1 = const(0x02) # Output Control for P0-P7
_XRA1403_REGISTER_OCR2 = const(0x03) # Output Control for P8-P15
_XRA1403_REGISTER_PIR1 = const(0x04) # Input Polarity Inversion for P0-P7
_XRA1403_REGISTER_PIR2 = const(0x05) # Input Polarity Inversion for P8-P15
_XRA1403_REGISTER_GCR1 = const(0x06) # GPIO Configuration for P0-P7
_XRA1403_REGISTER_GCR2 = const(0x07) # GPIO Configuration for P8-P15
_XRA1403_REGISTER_PUR1 = const(0x08) # Input Internal Pull-up Resistor Enable/Disable for P0-P7
_XRA1403_REGISTER_PUR2 = const(0x09) # Input Internal Pull-up Resistor Enable/Disable for P8-P15
_XRA1403_REGISTER_IER1 = const(0x0A) # Input Interrupt Enable for P0-P7
_XRA1403_REGISTER_IER2 = const(0x0B) # Input Interrupt Enable for P8-P15
_XRA1403_REGISTER_TSCR1 = const(0xC) # Output Three-State Control for P0-P7
_XRA1403_REGISTER_TSCR2 = const(0x0D) # Output Three-State Control for P8-P15
_XRA1403_REGISTER_ISR1 = const(0x0E) # Input Interrupt Status for P0-P7
_XRA1403_REGISTER_ISR2 = const(0x0) # Input Interrupt Status for P8-P15
_XRA1403_REGISTER_REIR1 = const(0x10) # Input Rising Edge Interrupt Enable for P0-P7
_XRA1403_REGISTER_REIR2 = const(0x11) # Input Rising Edge Interrupt Enable for P8-P15
_XRA1403_REGISTER_FEIR1 = const(0x12) # Input Falling Edge Interrupt Enable for P0-P7
_XRA1403_REGISTER_FEIR2 = const(0x13) # Input Falling Edge Interrupt Enable for P8-P15
_XRA1403_REGISTER_IFR1 = const(0x14) # Input Filter Enable/Disable for P0-P7
_XRA1403_REGISTER_IFR2 = const(0x15) # Input Filter Enable/Disable for P8-P15
_XRA1403_REGISTER_READ = const(0x80) # Register Read Bit

from adafruit_bus_device.spi_device import SPIDevice
from busio import SPI

class XRA1403():
	def __init__(self, spi_bus: SPI, spi_cs: DigitalInOut, baudrate: int = 10000000, reset: bool = True, reset_interrupts: bool = True) -> None:
		self._spi = SPIDevice(spi_bus, spi_cs, baudrate=baudrate)
		self._output = bytearray([0,0])
		self._inversion = bytearray([0,0])
		self._configuration = bytearray([0,0])
		self._pullup = bytearray([0,0])
		self._interrupt_enable = bytearray([0,0])
		self._three_state = bytearray([0,0])
		self._rising_edge_interrupt = bytearray([0,0])
		self._falling_edge_interrupt = bytearray([0,0])
		self._input_filter = bytearray([0,0])

		if reset:
			self._reset(reset_interrupts)
		else:
			# A reset will also save state
			self._saveState()

	def _saveState(self):
		# Output controller
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_OCR1<<1,]))
			spi.readinto(self._output[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_OCR2<<1,]))
			spi.readinto(self._output[0:1])

		# Input Polarity Inversion
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_PIR1<<1,]))
			spi.readinto(self._inversion[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_PIR2<<1,]))
			spi.readinto(self._inversion[0:1])

		# GPIO Configuration
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_GCR1<<1,]))
			spi.readinto(self._configuration[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_GCR2<<1,]))
			spi.readinto(self._configuration[0:1])

		# Input Internal pull-up resistor
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_PUR1<<1,]))
			spi.readinto(self._pullup[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_PUR2<<1,]))
			spi.readinto(self._pullup[0:1])

		# Input Interrupt Enable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_IER1<<1,]))
			spi.readinto(self._interrupt_enable[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_IER2<<1,]))
			spi.readinto(self._interrupt_enable[0:1])

		# Output Three-State Control
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_TSCR1<<1,]))
			spi.readinto(self._three_state[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_TSCR2<<1,]))
			spi.readinto(self._three_state[0:1])

		# Input Rising Edge Interrupt Enable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_REIR1<<1,]))
			spi.readinto(self._rising_edge_interrupt[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_REIR2<<1,]))
			spi.readinto(self._rising_edge_interrupt[0:1])

		# Input Falling Edge Interrupt Enable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_FEIR1<<1,]))
			spi.readinto(self._falling_edge_interrupt[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_FEIR2<<1,]))
			spi.readinto(self._falling_edge_interrupt[0:1])

		# Input Filter Enable/Disable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_IFR1<<1,]))
			spi.readinto(self._input_filter[1:2])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_IFR2<<1,]))
			spi.readinto(self._input_filter[0:1])

	def _reset(self, reset_interrupts: bool = True):

		if reset_interrupts:
			buf = bytearray([0])
			# Reset interrupts by reading GPIO State Registers
			with self._spi as spi:
				spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_GSR1<<1,]))
				spi.readinto(buf)
			with self._spi as spi:
				spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_GSR2<<1,]))
				spi.readinto(buf)

		# Output controller
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_OCR1<<1, 0xFF]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_OCR2<<1, 0xFF]))

		# Input Polarity Inversion
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PIR1<<1, 0x00]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PIR2<<1, 0x0]))

		# GPIO Configuration
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_GCR1<<1, 0xFF]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_GCR2<<1, 0xFF]))

		# Input Internal pull-up resistor
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PUR1<<1, 0x00]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PUR2<<1, 0x00]))

		# Input Interrupt Enable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IER1<<1, 0x00]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IER2<<1, 0x00]))

		# Output Three-State Control
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_TSCR1<<1, 0x00]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_TSCR2<<1, 0x00]))

		# Input Rising Edge Interrupt Enable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_REIR1<<1, 0x00]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_REIR2<<1, 0x00]))

		# Input Falling Edge Interrupt Enable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_FEIR1<<1, 0x00]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_FEIR2<<1, 0x00]))

		# Input Filter Enable/Disable
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IFR1<<1, 0xFF]))
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IFR2<<1, 0xFF]))

		self._saveState()

	def read_gpio(self) -> int:
		return self.read_gpiob()<<8|self.read_gpioa()	

	def read_gpioa(self) -> int:
		buf = bytearray([0])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_GSR1<<1,]))
			spi.readinto(buf)
		return buf[0]

	def read_gpiob(self) -> int:
		buf = bytearray([0])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_GSR2<<1,]))
			spi.readinto(buf)
		return buf[0]

	def write_gpio(self, val: int) -> None:
		self.write_gpioa(val&0xFF)
		self.write_gpiob(val>>8&0xFF)

	def write_gpioa(self, val: int) -> None:
		self._output[1] = val&0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_OCR1<<1,val&0xFF]))

	def write_gpiob(self, val: int) -> None:
		self._output[0] = val&0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_OCR2<<1,val&0xFF]))

	def set_iodir(self, val: int) -> None:
		self.set_iodira(val&0xFF)
		self.set_iodirb(val>>8&0xFF)

	def set_iodira(self, val: int) -> None:
		self._configuration[1] = val&0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_GCR1<<1,val&0xFF]))

	def set_iodirb(self, val: int) -> None:
		self._configuration[0] = val&0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_GCR2<<1,val&0xFF]))

	def get_iodir(self, val: int) -> None:
		return self.get_iodirb()<<8|self.get_iodira()

	def get_iodira(self, val: int) -> None:
		return self._configuration[1]

	def get_iodirb(self, val: int) -> None:
		return self._configuration[0]

	def get_inv(self) -> int:
		return self.get_invb()<<8|self.get_inva()

	def get_inva(self) -> int:
		return self._inversion[1]

	def get_invb(self) -> int:
		return self._inversion[0]

	def set_inv(self, val: int) -> None:
		self.set_inva(val&0xFF)
		self.set_invb(val>>8&0xFF)

	def set_inva(self, val: int) -> None:
		self._invertion[1] = val&0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PIR1<<1,val&0xFF]))

	def set_invb(self, val: int) -> None:
		self._invertion[0] = val&0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PIR2<<1,val&0xFF]))

	def get_pullup(self) -> int:
		return self.get_pullupb()<<8|self.get_pullupa()

	def get_pullupa(self) -> int:
		return self._pullup[1]

	def get_pullupb(self) -> int:
		return self._pullup[0]

	def set_pullup(self, val: int) -> None:
		self.set_pullupa(val&0xFF)
		self.set_pullupb(val>>8&0xFF)

	def set_pullupa(self, val: int) -> None:
		self._pullup[1] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PUR1<<1,val&0xFF]))

	def set_pullupb(self, val: int) -> None:
		self._pullup[0] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_PUR2<<1,val&0xFF]))

	def get_interrupt_enable(self) -> int:
		return self.get_interrupt_enableb()<<8|self.get_interrupt_enablea()

	def get_interrupt_enablea(self) -> int:
		return self._interrupt_enable[1]

	def get_interrupt_enableb(self) -> int:
		return self._interrupt_enable[0]

	def set_interrupt_enable(self, val: int) -> None:
		self.set_interrupt_enablea(val&0xFF)
		self.set_interrupt_enableb(val>>8&0xFF)

	def set_interrupt_enablea(self, val: int) -> None:
		self._pullup[1] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IER1<<1,val&0xFF]))

	def set_interrupt_enableb(self, val: int) -> None:
		self._pullup[0] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IER2<<1,val&0xFF]))

	def get_three_state(self) -> int:
		return self.get_three_stateb()<<8|self.get_three_statea()

	def get_three_statea(self) -> int:
		return self._three_state[1]

	def get_three_stateb(self) -> int:
		return self._three_state[0]

	def set_three_state(self, val: int) -> None:
		self.set_three_statea(val&0xFF)
		self.set_three_stateb(val>>8&0xFF)

	def set_three_statea(self, val: int) -> None:
		self._three_state[1] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_TSCR1<<1,val&0xFF]))

	def set_three_stateb(self, val: int) -> None:
		self._three_state[0] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_TSCR2<<1,val&0xFF]))


	def get_interrupt_status(self) -> int:
		return self.get_interrupt_statusb()<<8|self.get_interrupt_statusa()

	def get_interrupt_statusa(self) -> int:
		buf = bytearray([0])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_ISR1<<1,]))
			spi.readinto(buf)
		return buf[0]

	def get_interrupt_statusb(self) -> int:
		buf = bytearray([0])
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_READ|_XRA1403_REGISTER_ISR1<<1,]))
			spi.readinto(buf)
		return buf[0]

	def get_rising_edge_interrupt(self) -> int:
		return self.get_rising_edge_interruptb()<<8|self.get_rising_edge_interrupta()

	def get_rising_edge_interrupta(self) -> int:
		return self._rising_edge_interrupt[1]

	def get_rising_edge_interruptb(self) -> int:
		return self._rising_edge_interrupt[0]

	def set_rising_edge_interrupt(self, val: int) -> None:
		self.set_rising_edge_interrupta(val&0xFF)
		self.set_rising_edge_interruptb(val>>8&0xFF)

	def set_rising_edge_interrupta(self, val: int) -> None:
		self._rising_edge_interrupt[1] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_REIR1<<1,val&0xFF]))

	def set_rising_edge_interruptb(self, val: int) -> None:
		self._rising_edge_interrupt[0] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_REIR2<<1,val&0xFF]))

	def get_falling_edge_interrupt(self) -> int:
		return self.get_falling_edge_interruptb()<<8|self.get_falling_edge_interrupta()

	def get_falling_edge_interrupta(self) -> int:
		return self._falling_edge_interrupt[1]

	def get_falling_edge_interruptb(self) -> int:
		return self._falling_edge_interrupt[0]

	def set_falling_edge_interrupt(self, val: int) -> None:
		self.set_falling_edge_interrupta(val&0xFF)
		self.set_falling_edge_interruptb(val>>8&0xFF)

	def set_falling_edge_interrupta(self, val: int) -> None:
		self._falling_edge_interrupt[1] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_FEIR1<<1,val&0xFF]))

	def set_falling_edge_interruptb(self, val: int) -> None:
		self._falling_edge_interrupt[0] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_FEIR2<<1,val&0xFF]))

	def get_input_filter(self) -> int:
		return self.get_input_filterb()<<8|self.get_input_filtera()

	def get_input_filtera(self) -> int:
		return self._input_filter[1]

	def get_input_filterb(self) -> int:
		return self._input_filter[0]

	def set_input_filter(self, val: int) -> None:
		self.set_input_filtera(val&0xFF)
		self.set_input_filterb(val>>8&0xFF)

	def set_input_filtera(self, val: int) -> None:
		self._input_filter[1] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IFR1<<1,val&0xFF]))

	def set_input_filterb(self, val: int) -> None:
		self._input_filter[0] = val & 0xFF
		with self._spi as spi:
			spi.write(bytearray([_XRA1403_REGISTER_IFR2<<1,val&0xFF]))

	def get_pin(self, pin: int) -> "DigitalInOut":
		assert 0<= pin <= 15
		return DigitalInOut(pin, self)

	def write_pin(self, pin: int, val: bool) -> None:
		if val:
			self.write_gpio(self._output | (1<<pin))
		else:
			self.write_gpio(self._output & ~(1<<pin))

	def read_pin(self, pin: int) -> bool:
		return (self.read_gpio() >> pin) & 0x1

class DigitalInOut:
	def __init__(self, pin_number: int, xra: XRA1200) -> None:
		self._pin = pin_number
		self._xra = xra 

	def switch_to_output(self, value: bool = False, **kwargs) -> None:
		if value:
			self._xra.write_gpio( self._xra.read_gpio() | ( 1<<self._pin ) )
		else:
			self._xra.write_gpio( self._xra.read_gpio() & ~( 1<<self._pin ) )
		self._xra.set_iodir( ((self._xra.get_iodir() & (1<<self._pin)) >> self._pin) & ~( 1<<self._pin ) )

	def switch_to_input(self, **kwargs) -> None:
		self._xra.set_iodir( ((self._xra.get_iodir() & (1<<self._pin)) >> self._pin) | ( 1<<self._pin ) )

	@property
	def value(self) -> bool:
		return (self._xra.read_gpio() & (1<<self._pin)) > 0

	@value.setter
	def value(self, val: bool) -> None:
		if val:
			self._xra.write_gpio( self._xra.read_gpio() | ( 1<<self._pin ) )
		else:
			self._xra.write_gpio( self._xra.read_gpio() & ~( 1<<self._pin ) )

	@property
	def direction(self) -> digitalio.Direction:
		if ((self._xra.get_iodir() & (1<<self._pin)) >> self._pin):
			return digitalio.Direction.INPUT
		else:
			return digitalio.Direction.OUTPUT

	@direction.setter
	def direction(self,val: digitalio.Direction) -> None:
		if val == digitalio.Direction.INPUT:
			self._xra.set_iodir( self._xra.get_iodir() | (1<<self._pin) )
		elif val == digitalio.Direction.OUTPUT:
			self._xra.set_iodir( self._xra.get_iodir() & ~(1<<self._pin) )
		else:
			raise ValueError("Expected INPUT or OUTPUT direction!")

	@property
	def invert_polarity(self) -> bool:
		return self._xra._inversion[0] & (1<<self._pin) > 0

	@invert_polarity.setter
	def invert_polarity(self, val: bool) -> None:
		if val:
			self._xra.set_inv( self._xra.get_inv() | (1<<self._pin) )
		else:
			self._xra.set_inv( self._xra.get_inv() & ~(1<<self._pin) )

	@property
	def pullup(self) -> bool:
		return self._xra.get_pullup() & (1<<self._pin) > 0

	@pullup.setter
	def pullup(self, val: bool) -> None:
		if val:
			self._xra.set_pullup( self._xra.get_pullup() | (1<<self._pin) )
		else:
			self._xra.set_pullup( self._xra.get_pullup() & ~(1<<self._pin) )

	@property
	def interrupt_enable(self) -> bool:
		return self._xra.get_interrupt_enable() & (1<<self._pin) > 0

	@interrupt_enable.setter
	def interrupt_enable(self, val: bool) -> none:
		if val:
			self._xra.set_interrupt_enable( self._xra.get_interrupt_enable() | (1<<self._pin) )
		else:
			self._xra.set_interrupt_enable( self._xra.get_interrupt_enable() & ~(1<<self._pin) )

	@property
	def three_state(self) -> bool:
		return self._xra.get_three_state() & (1<<self._pin) > 0
