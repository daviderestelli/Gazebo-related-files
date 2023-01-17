from nanolib_helper import *
import time

class Motor:
	def __init__(self, bus_index = 0, device_index = 0):
		'''setup connection'''
		nanolib_helper = NanolibHelper()

		# create access to the nanolib
		nanolib_helper.setup()

		# its possible to set the logging level to a different level
		nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

		# list all hardware available, decide for the first one
		bus_hardware_ids = nanolib_helper.get_bus_hardware()

		if bus_hardware_ids.empty():
			raise Exception('No bus hardware found.')

		# Use the selected bus hardware
		bus_hw_id = bus_hardware_ids[bus_index]

		# create bus hardware options for opening the hardware
		bus_hw_options = nanolib_helper.create_bus_hardware_options(bus_hw_id)

		# now able to open the hardware itself
		nanolib_helper.open_bus_hardware(bus_hw_id, bus_hw_options)
		nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

		# either scan the whole bus for devices (in case the bus supports scanning)
		device_ids = nanolib_helper.scan_bus(bus_hw_id)

		nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)
			
		if (device_ids.size() == 0):
			raise Exception('No devices found.')

		device_id = device_ids[device_index]

		device_handle = nanolib_helper.create_device(device_id)

		# now connect to the device
		nanolib_helper.connect_device(device_handle)
		
		self.nanolib_helper = nanolib_helper
		self.device_handle = device_handle
		self.bus_hw_id = bus_hw_id


	def close(self):
		'''close connection'''
		# cleanup and close everything
		self.nanolib_helper.disconnect_device(self.device_handle)
		self.nanolib_helper.close_bus_hardware(self.bus_hw_id)


	def example(self):
		'''execute the example from nanolib_example'''
		# now ready to work with the controller, here are some examples on how to access the
		# object dictionary:
		from nanolib_example import object_dictionary_access_examples
		object_dictionary_access_examples(self.nanolib_helper, self.device_handle)


	def read(self, index, subindex = 0x00):
		'''read some value from some index'''
		return self.nanolib_helper.read_number(self.device_handle, Nanolib.OdIndex(index, subindex))
		
	def print(self, index, subindex=0, binary=False):
		'''read and print some value from some index'''
		result = self.read(index, subindex)
		print(f'read(0x{index:x},0x{subindex:x}) =', f'0b{result:b}' if binary else result)

	def write(self, index, value, subindex=0x0, bits=16):
		'''write a command to the motor driver'''
		return self.nanolib_helper.write_number(self.device_handle, value, Nanolib.OdIndex(index, subindex), bits)
		
	def activate(self, value=1):
		'''set the motor status to active (is probably already set)'''
		return self.write(0x6060, value)
		
	def enable(self):
		'''enable the motor'''
		m.write(0x6040, 0x06)  # disable
		m.write(0x6040, 0x07)  # enable step 1
		m.write(0x6040, 0x0F)  # enable step 2
		
	def disable(self):
		'''disable the motor'''
		m.write(0x6040, 0x06)  # disable

	def set_velocity(self, value):
		'''set velocity in rev/h'''
		m.write(0x6060, 0x02)  # select velocity mode
		m.write(0x6042, value*2)  # set speed in 0.5*rev/h?

	def set_position(self, value):
		# TODO: doesn't work yet
		m.write(0x6060, 0x01)  # select position mode
		m.write(0x607A, value, bits=32)
		
		



if __name__ == '__main__':
	m = Motor()
	m.print(0x6040, binary=True)
	m.print(0x6060)
	m.print(0x607A)
	m.print(0x6041, binary=True)
	m.print(0x607D)
	
	m.set_velocity(10000)
	#m.set_position()
	m.print(0x6040)
	m.enable()
	m.print(0x6040)

	time.sleep(40)
	m.print(0x6040)
	m.disable()

	'''
	m.write(0x6060, -2)  # autosetup
	m.write(0x6040, 0b1000)	
	m.print(0x6041, binary=True)

	m.write(0x6040, 0b110)  # stop
	m.write(0x6060, 0)  # disable
	m.write(0x607A, 0)  # set motor position
	m.write(0x6060, 1)  # enable
	m.write(0x6040, 0b11110)  # start
	'''
	m.close()

