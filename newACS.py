# -*- coding: utf-8 -*-
# This is a fork of ACSpy library (c) Pete Bachant
# newACS v0.02

# NEW:
# - is_blocked() now works well
# - motor auto-stops if limit switch is pushed
# - motor won't start if blocked

"""
This module contains an [incomplete] object for communicating with an ACS controller.
"""
from __future__ import division, print_function
import acsc_modified as acsc
import time

'''ACS config'''

# acs_ip = '172.20.196.4' #STM ACS
acs_ip = '10.0.0.100' #GOGIN ACS - default ACS IP

acs_port = 701
test_axis_numbers = [0,] #a list of axes used in demo.py
test_axis_number = test_axis_numbers[0] #a single demo axis used in newACS's __main__
acs_flags = None
acs_wait = acsc.SYNCHRONOUS

class newAcsController(object):
	def __init__(self, ip, port, new_axis_names = dict(), contype="simulator", n_axes=8):
		self.contype = contype
		self.ip = ip
		self.port = port
		self.hc = None  # Инициализируем атрибут hc
		print('Connecting to ACS controller...')
		self.connect(ip, port)
		print('The action is completed')
		self.axes = []  #!Тот самый список осей
		for n in range(n_axes):
			if n in new_axis_names.keys():
				new_axis_name = new_axis_names[n]
			else:
				new_axis_name = 'ACS axis ' + str(n)
			new_axis = acsAxis(self, n, new_axis_name)
			self.axes.append(new_axis)

		
	def connect(self, address=acs_ip, port=acs_port):
		#if self.contype == "simulator":
		#	self.hc = acsc.openCommDirect()
		#elif self.contype == "ethernet":
		self.hc = acsc.openCommEthernetTCP(address=address, port=port)
		print(self.hc)
		return self.hc


	def enable_all(self, wait=acsc.SYNCHRONOUS):
		"""Enables all axes."""
		for a in self.axes:
			a.enable()

	def disable_all(self, wait=acsc.SYNCHRONOUS):
		"""Disables all axes."""
		for a in self.axes:
			a.disable()
		
	def disconnect(self):
		acsc.closeComm(self.hc)
		

class acsAxis(object):
	def __init__(self, controller, axisno, name=None):
		if isinstance(controller, newAcsController):
			self.controller = controller
		else:
			raise TypeError("ACS initialization error! ACS Controller object is not valid.")
		self.axisno = axisno
		if name:
			#controller.axisdefs[name] = axisno
			self.name = name
		else:
			self.name = str(axisno)
		
	def enable(self, wait=acsc.SYNCHRONOUS):
		acsc.enable(self.controller.hc, self.axisno, wait)

	def disable(self, wait=acsc.SYNCHRONOUS):
		acsc.disable(self.controller.hc, self.axisno, wait)
	
	@property
	def motor_state(self):
		"""Returns motor state dict."""
		return acsc.getMotorState(self.controller.hc, self.axisno)
		
		
		
		'''these are functions/slot with simple syntax to be used as an interface with other code'''
		
		
		
	def set_pos(self, pos): #ANY MOTOR
		acsc.setRPosition(self.controller.hc, self.axisno, pos)
		
	def get_pos(self): #ANY MOTOR
		return acsc.getFPosition(self.controller.hc, self.axisno) #! F means feedback motor position
	
	def get_Rpos(self):
		return acsc.getRPosition(self.controller.hc, self.axisno)
	
	def get_FVelosity(self):
		return acsc.getFVelocity(self.controller.hc, self.axisno)
		
	def to_point(self, pos): #ANY MOTOR
		if not self.is_blocked():
			acsc.toPoint(self.controller.hc, acs_flags, self.axisno, pos, acs_wait)
		else:
			print('ACS motor ', self.name, "won't move: LIMIT SWITCH")
			print('Rotate the axis MANUALLY to re-engage the motor.')

#! Done myself
	def go(self):
		if not self.is_blocked():
			acsc.go(self.controller.hc, self.axisno, acs_wait)
#! Done myself
		
	def stop(self): #ANY MOTOR
		acsc.halt(self.controller.hc, self.axisno)
		
	def is_moving(self): #CHECK IF MOVING #ANY MOTOR
		moving = not(self.motor_state["in position"]) # NOT (self.motor_state["moving"] or self.motor_state["accelerating"])
		blocked = self.is_blocked()
		if moving and blocked:
			self.stop() # AUTO-STOP
			print('ACS motor', self.name, ': LIMIT EXCEEDED. Auto-stop!')
			print('Rotate the axis MANUALLY to re-engage the motor.')
			print("NOTE: You don't have to stop the program to do it")
			return False
		else:
			return 'moving' #! ТУТ Я САМ ПОСТАВИЛ КАВЫЧКИ
		
	def is_blocked(self): #END-SWITCH ERROR CHECK #ANY MOTOR
		fault = acsc.getFault(self.controller.hc, self.axisno)
		fault_int = fault.value
		left_limit = fault_int % 2 # 1st bit of fault_int is left limit - CHECK RIGHT/LEFT, PROBABLY CONFUSED
		right_limit = ((fault_int - left_limit) // 2) % 2 # 2nd bit of fault_int is right limit - CHECK RIGHT/LEFT, PROBABLY CONFUSED
		return (right_limit or left_limit)
		
	def set_speed(self, speed): #ANY MOTOR
		acsc.setVelocity(self.controller.hc, self.axisno, speed)
		print('Скорость установлена успешно')

	def set_acceleration(self, acceleration):
		acsc.setAcceleration(self.controller.hc, self.axisno, acceleration)
		print('Ускорение установлено успешно')

	def set_deceleration(self, deceleration):
		acsc.setDeceleration(self.controller.hc, self.axisno, deceleration)
		print('Торможение установлено успешно')

# !Just a fantasy
	def set_kill_deceleration(self, kill_deceleration):
		acsc.setKillDeceleration(self.controller.hc, self.axisno, kill_deceleration)
		print('Экстренное торможение установлено')
# !Just a fantasy

	def set_jerk(self, jerk):
		acsc.setJerk(self.controller.hc, self.axisno, jerk)
		print('Рывок установлено успешно')
	
	def get_name(self):
		return self.name
		
	def set_name(self, name):
		self.name = name
		#self.controller.axisdefs[name] = self.axisno
		
		
		'''these are test functions used in demo.py only'''
		
		
		
	def test_move_A(self): #test slot for Qt
		test_shift = 2000
		pos = self.get_pos()
		print('Current pos of the newACS shifter no.', self.axisno, ' (in full steps):', pos)
		self.start(pos - test_shift)
	
	def test_move_B(self): #test slot for Qt
		test_shift = 2000
		pos = self.get_pos()
		print('Current pos of the newACS shifter no.', self.axisno, ' (in full steps):', pos)
		self.start(pos + test_shift)
'''
    def ptp(self, target, coordinates="absolute", wait=acsc.SYNCHRONOUS):
        """Performs a point to point move in either relative or absolute
        (default) coordinates."""
        if coordinates == "relative":
            flags = acsc.AMF_RELATIVE
        else:
            flags = None
        acsc.toPoint(self.controller.hc, flags, self.axisno, target, wait)
        
    def ptpr(self, distance, wait=acsc.SYNCHRONOUS):
        """Performance a point to point move in relative coordinates."""
        self.ptp(distance, coordinates="relative", wait=wait)
        
    
    def set_fpos(self, pos):
        acsc.setFPosition(self.controller.hc, self.axisno, pos)
    
    def set_rpos(self, pos):
        acsc.setRPosition(self.controller.hc, self.axisno, pos)
    
    @property
    def axis_state(self):
        """Returns axis state dict."""
        return acsc.getAxisState(self.controller.hc, self.axisno)
        
    @property
    def motor_state(self):
        """Returns motor state dict."""
        return acsc.getMotorState(self.controller.hc, self.axisno)
        
    @property
    def moving(self):
        return self.motor_state["moving"]
        
    @property
    def enabled(self):
        return self.motor_state["enabled"]
    
    @enabled.setter
    def enabled(self, choice):
        if choice == True:
            self.enable()
        elif choice == False:
            self.disable()
        
    @property
    def in_position(self):
        return self.motor_state["in position"]
        
    @property
    def accelerating(self):
        return self.motor_state["accelerating"]
        
    @property
    def rpos(self):
        return acsc.getRPosition(self.controller.hc, self.axisno)

    @property
    def fpos(self):
        return acsc.getFPosition(self.controller.hc, self.axisno)
        
    @property
    def rvel(self):
        return acsc.getRVelocity(self.controller.hc, self.axisno)

    @property
    def fvel(self):
        return acsc.getFVelocity(self.controller.hc, self.axisno)
        
    @property
    def vel(self):
        return acsc.getVelocity(self.controller.hc, self.axisno)
    @vel.setter
    def vel(self, velocity):
        """Sets axis velocity."""
        acsc.setVelocity(self.controller.hc, self.axisno, velocity)
        
    @property
    def acc(self):
        return acsc.getAcceleration(self.controller.hc, self.axisno)
    @acc.setter
    def acc(self, accel):
        """Sets axis velocity."""
        acsc.setAcceleration(self.controller.hc, self.axisno, accel)
        
    @property
    def dec(self):
        return acsc.getDeceleration(self.controller.hc, self.axisno)
    @dec.setter
    def dec(self, decel):
        """Sets axis velocity."""
        acsc.setDeceleration(self.controller.hc, self.axisno, decel)
'''
if __name__ == "__main__":
	stm_acs = newAcsController(acs_ip, acs_port, contype = 'Ethernet')
	#test_axis = stm_acs.axes[0]
	#test_axis.enable()
	#print(test_axis.get_pos())
	#print(test_axis.get_name())
	#test_axis.set_pos(0)
	#print(test_axis.get_pos())
	#time.sleep(2)
	#test_axis.start(2000)
	#time.sleep(2)
	#print(test_axis.get_pos())
	#test_axis.disable()
	#stm_acs.disconnect()