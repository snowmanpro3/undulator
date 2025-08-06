# -*- coding: utf-8 -*-
"""
ACSC
----
This module is a wrapper for the ACS C library using ctypes
"""
from __future__ import division, print_function
import ctypes
from ctypes import byref
import numpy as np
import platform

# Import the ACS C library DLL
if platform.architecture()[0] == "32bit":
    acs = ctypes.windll.LoadLibrary('ACSCL_x86.dll')
if platform.architecture()[0] == "64bit":
    acs = ctypes.windll.LoadLibrary('ACSCL_x64.dll')

int32 = ctypes.c_long
int64 = ctypes.c_int64
uInt32 = ctypes.c_ulong
uInt64 = ctypes.c_ulonglong
double = ctypes.c_double
char = ctypes.c_char
p = ctypes.pointer

# Define motion flags and constants
AMF_WAIT = 0x00000001
AMF_RELATIVE = 0x00000002
AMF_VELOCITY = 0x00000004
AMF_ENDVELOCITY = 0x00000008
AMF_CYCLIC = 0x00000100
AMF_CUBIC = 0x00000400

# Axis states
AST_LEAD = 0x00000001
AST_DC = 0x00000002
AST_PEG = 0x00000004
AST_PEGREADY = 0x00000010
AST_MOVE = 0x00000020
AST_ACC = 0x00000040
AST_SEGMENT = 0x00000080
AST_VELLOCK = 0x00000100
AST_POSLOCK = 0x00000200

# Motor states
MST_ENABLE = 0x00000001
MST_INPOS = 0x00000010
MST_MOVE = 0x00000020
MST_ACC = 0x00000040

SYNCHRONOUS = None
INVALID = -1
IGNORE = -1
ASYNCHRONOUS = -2
NONE = -1
COUNTERCLOCKWISE = 1
CLOCKWISE = -1
INT_BINARY = 4
REAL_BINARY	 = 8
INT_TYPE = 1
REAL_TYPE = 2

def openCommDirect():
    """Open simulator. Returns communication handle."""
    hcomm = acs.acsc_OpenCommDirect()
    if hcomm == -1:
        error = getLastError()
        print("ACS SPiiPlus Error", error)
    return hcomm


def openCommEthernetTCP(address="10.0.0.100", port=701):
    """Address is a string. Port is an int.
    Returns communication handle."""
    hcomm = acs.acsc_OpenCommEthernetTCP(address.encode(), port)
    return hcomm

def setVelocity(hcomm, axis, vel, wait=SYNCHRONOUS):  #Скорость
    """Sets axis velocity."""
    acs.acsc_SetVelocity(hcomm, axis, double(vel), wait)

def setAcceleration(hcomm, axis, acc, wait=SYNCHRONOUS):  #Ускорение
    """Sets axis acceleration."""
    acs.acsc_SetAcceleration(hcomm, axis, double(acc), wait)

# ! Just a fantasy
def setKillDeceleration(hcomm, axis, kdec, wait=SYNCHRONOUS):
    """Sets axis kill deceleration."""
    acs.acsc_SetKillDeceleration(hcomm, axis, double(kdec), wait)
# ! Just a fantasy

def setDeceleration(hcomm, axis, dec, wait=SYNCHRONOUS):  #Замедление
    """Sets axis deceleration."""
    acs.acsc_SetDeceleration(hcomm, axis, double(dec), wait)

def setJerk(hcomm, axis, jerk, wait=SYNCHRONOUS):  #Рывок
    acs.acsc_SetJerk(hcomm, axis, double(jerk), wait)

def getMotorEnabled(hcomm, axis, wait=SYNCHRONOUS):  #Провекра вкл или выкл двигатель на данной оси
    """Checks if motor is enabled."""
    state = ctypes.c_int()
    acs.acsc_GetMotorState(hcomm, axis, byref(state), wait)
    state = state.value
    return hex(state)[-1] == "1"  #Значение state преобразуется в шестнадцатеричный формат. Смотрим ласт символ ( 1 - вкл)

def getMotorState(hcomm, axis, wait=SYNCHRONOUS):
    """Gets the motor state. Returns a dictionary with the following keys:
      * "enabled"
      * "in position"
      * "moving"
      * "accelerating"
    """
    state = ctypes.c_int()
    acs.acsc_GetMotorState(hcomm, axis, byref(state), wait)
    state = state.value
    mst = {"enabled" : hex(state)[-1] == "1",
           "in position" : hex(state)[-2] == "1",
           "moving" : hex(state)[-2] == "2",
           "accelerating" : hex(state)[-2] == "4"}
    return mst

def getAxisState(hcomm, axis, wait=SYNCHRONOUS):
    """Gets the axis state. Returns a dictionary with the following keys
      * "lead"
      * "DC"
      * "PEG"
      * "PEGREADY"
      * "moving"
      * "accelerating"
      * "segment"
      * "vel lock"
      * "pos lock"
     """
    state = ctypes.c_int()
    acs.acsc_GetAxisState(hcomm, axis, byref(state), wait)
    state = state.value
    ast = {"lead" : hex(state)[-1] == "1",           #является ли ось ведущей 
           "DC" : hex(state)[-1] == "2",             #находится ли ось в режиме постоянного тока
           "PEG" : hex(state)[-1] == "4",            #активен ли режим PEG (Position Error Gate, задаёт удовлетворительный диапазон)
           "PEGREADY" : hex(state)[-2] == "1",       #готов ли режим PEG
           "moving" : hex(state)[-2] == "2",         #движется ли ось
           "accelerating" : hex(state)[-2] == "4",   #ускоряется ли ось
           "segment" : hex(state)[-2] == "8",        #активен ли сегментный режим
           "vel lock" : hex(state)[-3] == "1",       #заблокирована ли скорость
           "pos lock" : hex(state)[-3] == "2"}       #заблокирована ли позиция
    return ast

def registerEmergencyStop():
    """Register the software emergency stop."""
    acs.acsc_RegisterEmergencyStop()

def jog(hcomm, flags, axis, vel, wait=SYNCHRONOUS):  #Флаги описывают режим движения
    """Jog move."""
    acs.acsc_Jog(hcomm, flags, axis, double(vel), wait)

def toPoint(hcomm, flags, axis, target, wait=SYNCHRONOUS):
    """Point to point move."""
    acs.acsc_ToPoint(hcomm, flags, axis, double(target), wait)

def toPointM(hcomm, flags, axes, target, wait=SYNCHRONOUS):  #Выполняет многокоординатное движение
    """Initiates a multi-axis move to the specified target. Axes and target
    are entered as tuples. Set flags as None for absolute coordinates."""
    if len(axes) != len(target):
        print("Number of axes and coordinates don't match!")
    else:
        target_array = double*len(axes)
        axes_array = ctypes.c_int*(len(axes) + 1)
        target_c = target_array()
        axes_c = axes_array()
        for n in range(len(axes)):
            target_c[n] = target[n]
            axes_c[n] = axes[n]
        axes_c[-1] = -1
        errorHandling(acs.acsc_ToPointM(hcomm, flags, axes_c, target_c, wait))  #Обработка ошибок

def enable(hcomm, axis, wait=SYNCHRONOUS):   #Включает указанную ось (активирует двигатель)
    acs.acsc_Enable(hcomm, int32(axis), wait)

def disable(hcomm, axis, wait=SYNCHRONOUS):  #Выключает указанную ось (деактивирует двигатель)
    acs.acsc_Disable(hcomm, int32(axis), wait)

def getRPosition(hcomm, axis, wait=SYNCHRONOUS):  #Возвращает текущую позицию указанной оси (Real Position)
    pos = double()
    acs.acsc_GetRPosition(hcomm, axis, p(pos), wait)
    return pos.value

def getFPosition(hcomm, axis, wait=SYNCHRONOUS):  #Возвращает целевую позицию указанной оси (Final Position)
    pos = double()
    acs.acsc_GetFPosition(hcomm, axis, byref(pos), wait)
    return pos.value

def getRVelocity(hcomm, axis, wait=SYNCHRONOUS):  #Возвращает текущую скорость указанной оси
    rvel = double()
    acs.acsc_GetRVelocity(hcomm, axis, byref(rvel), wait)
    return rvel.value

def getFVelocity(hcomm, axis, wait=SYNCHRONOUS):  #Возвращает целевую скорость указанной оси
    vel = double()
    acs.acsc_GetFVelocity(hcomm, axis, byref(vel), wait)
    return vel.value

def getVelocity(hcomm, axis, wait=SYNCHRONOUS):  #Возвращает текущую скорость указанной оси
    """Returns current velocity for specified axis."""
    vel = double()
    acs.acsc_GetVelocity(hcomm, axis, byref(vel), wait)
    return vel.value

def getAcceleration(hcomm, axis, wait=SYNCHRONOUS):   #Возвращает текущее ускорение для указанной оси
    """Returns current acceleration for specified axis."""
    val = double()
    acs.acsc_GetAcceleration(hcomm, axis, byref(val), wait)
    return val.value

def getDeceleration(hcomm, axis, wait=SYNCHRONOUS):   #Возвращает текущее замедление для указанной оси
    """Returns current deceleration for specified axis."""
    val = double()
    acs.acsc_GetDeceleration(hcomm, axis, byref(val), wait)
    return val.value

def getFault(hcomm, axis, wait=SYNCHRONOUS): #Возвращает набор битов, которые указывают на ошибки двигателя или системы
    "Get the set of bits that indicate the motor or system faults"
    val = ctypes.c_long()
    ret = acs.acsc_GetFault(hcomm, axis, byref(val), wait) #Скорее всего ret это просто опечатка и её надо убрать
    return val

def closeComm(hcomm):  #Закрывает соединение с контроллером
    """Closes communication with the controller."""
    acs.acsc_CloseComm(hcomm)

def unregisterEmergencyStop():  #Отменяет регистрацию программной аварийной остановки
    acs.acsc_UnregisterEmergencyStop()

def getLastError():  #Возвращает код последней ошибки
    return acs.acsc_GetLastError()

def cleanBuffer(hcomm, buffno, startLine=0, endLine=1000, wait=SYNCHRONOUS):
    acs.acsc_ClearBuffer(hcomm, int32(buffno), int32(startLine), int32(endLine), wait)

def compileBuffer(hcomm, buffno, wait=SYNCHRONOUS):
    acs.acsc_CompileBuffer(hcomm, int32(buffno), wait)

def appendBuffer(hcomm, buffnumber, program, count=512, wait=SYNCHRONOUS):
    """Append one or several strings to a buffer into the ACS controller."""
    prgbuff = ctypes.create_string_buffer(str(program).encode(), count)
    rv = acs. acsc_AppendBuffer(hcomm, buffnumber, byref(prgbuff), count, wait)
    errorHandling(rv)

def runBuffer(hcomm, buffno, label=None, wait=SYNCHRONOUS): #Запускает буфер (программу) на контроллере
    """Runs a buffer in the controller."""
    if label is not None:
        label=label.encode()
    acs.acsc_RunBuffer(hcomm, int32(buffno), label, wait)

def stopBuffer(hcomm, buffno, wait=SYNCHRONOUS):
    """Stops a buffer running in the controller."""
    acs.acsc_StopBuffer(hcomm, int32(buffno), wait)

def getProgramState(hc, nbuf, wait=SYNCHRONOUS):  #Возвращает состояние программы (буфера) на контроллере
    """Returns program state"""
    state = ctypes.c_int()
    acs.acsc_GetProgramState(hc, nbuf, byref(state), wait)
    return state.value

def halt(hcomm, axis, wait=SYNCHRONOUS):  #Останавливает движение указанной оси (halt - остановка)
    """Halts motion on specified axis."""
    acs.acsc_Halt(hcomm, axis, wait)

#!!!!!! SelfMade
def haltM(hcomm, axes, wait=SYNCHRONOUS):  #Выполняет многокоординатное движение
    """The function terminates several motions using the full deceleration profile. Axes and target
    are entered as tuples. Set flags as None for absolute coordinates."""
    axes_array = ctypes.c_int*(len(axes) + 1)
    axes_c = axes_array()
    for n in range(len(axes)):
        axes_c[n] = axes[n]
    axes_c[-1] = -1
    errorHandling(acs.acsc_HaltM(hcomm, axes_c, wait))  #Обработка ошибок
#!!!!!! SelfMade
#!!!!!! SelfMade
def killAll(hcomm, wait=SYNCHRONOUS):  #Принудительно останавливает все оси (kill - убить)
    '''The function terminates all currently executed motions.'''
    acs.acsc_KillAll(hcomm, wait)
#!!!!!!! SelfMade

def declareVariable(hcomm, vartype, varname, wait=SYNCHRONOUS):  #Объявляет переменную в контроллере
    """Declare a variable in the controller."""
    acs.acsc_DeclareVariable(hcomm, vartype, varname.encode(), wait)

def readInteger(hcomm, buffno, varname, from1=None, to1=None, from2=None,
                to2=None, wait=SYNCHRONOUS):
    """Reads an integer(s) in the controller."""        #to1,2,  from1,2 - диапазоны дл чтения (опционально)
    intread = ctypes.c_int()
    acs.acsc_ReadInteger(hcomm, buffno, varname.encode(), from1, to1, from2,
                         to2, p(intread), wait)
    return intread.value

def writeInteger(hcomm, variable, val_to_write, nbuff=NONE, from1=NONE,
                 to1=NONE, from2=NONE, to2=NONE, wait=SYNCHRONOUS):
    """Writes an integer variable to the controller."""
    val = ctypes.c_int(val_to_write)
    acs.acsc_WriteInteger(hcomm, nbuff, variable.encode(), from1, to1,
                          from2, to2, p(val), wait)

def readReal(hcomm, buffno, varname, from1=NONE, to1=NONE, from2=NONE,
             to2=NONE, wait=SYNCHRONOUS):
    """Read real variable (scalar or array) from the controller."""
    if from2 == NONE and to2 == NONE and from1 != NONE:
        values = np.zeros((to1-from1+1), dtype=np.float64)
        pointer = values.ctypes.data
    elif from2 != NONE:
        values = np.zeros((to1-from1+1, to2-from2+1), dtype=np.float64)
        pointer = values.ctypes.data
    else:
        values = double()
        pointer = byref(values)
    acs.acsc_ReadReal(hcomm, buffno, varname.encode(), from1, to1, from2, to2,
                      pointer, wait)
    if from1 != NONE:
        return values
    else:
        return values.value

def writeReal(hcomm, varname, val_to_write, nbuff=NONE, from1=NONE, to1=NONE,
              from2=NONE, to2=NONE, wait=SYNCHRONOUS):
    """Writes a real value to the controller."""
    val = ctypes.c_double(val_to_write)
    acs.acsc_WriteReal(hcomm, nbuff, varname.encode(), from1, to1,
                       from2, to2, p(val), wait)

def uploadDataFromController(hcomm, src, srcname, srcnumformat, from1, to1, #Загружает данные из контроллера в файл на компьютере
            from2, to2, destfilename, destnumformat, btranspose, wait=0):
    acs.acsc_UploadDataFromController(hcomm, src, srcname, srcnumformat,
            from1, to1, from2, to2, destfilename, destnumformat,
            btranspose, wait)


def loadBuffer(hcomm, buffnumber, program, count=512, wait=SYNCHRONOUS):  #Загружает программу (буфер) в контроллер
    """Load a buffer into the ACS controller."""
    prgbuff = ctypes.create_string_buffer(str(program).encode(), count)
    rv = acs.acsc_LoadBuffer(hcomm, buffnumber, byref(prgbuff), count, wait)
    errorHandling(rv)


def loadBuffersFromFile(hcomm, filename, wait=SYNCHRONOUS):
    rv = acs.acsc_LoadBuffersFromFile(hcomm, filename.encode(), wait)
    errorHandling(rv)


def spline(hcomm, flags, axis, period, wait=SYNCHRONOUS):  #Можно через неё задавать плановое ускорение и замедление, сложные профили  скорости
    rv = acs.acsc_Spline(hcomm, flags, axis, double(period), wait)
    errorHandling(rv)

def addPVPoint(hcomm, axis, point, velocity, wait=SYNCHRONOUS):  #Добавляет точку PV (Position-Velocity) для указанной оси, задаёт позицию и скорость движения
    acs.acsc_AddPVPoint(hcomm, axis, double(point), double(velocity), wait)

def addPVTPoint(hcomm, axis, point, velocity, dt, wait=SYNCHRONOUS):  #Ещё и время достижения точки
    acs.acsc_AddPVTPoint(hcomm, axis, double(point), double(velocity),
                         double(dt), wait)

def multiPoint(hcomm, flags, axis, dwell, wait=SYNCHRONOUS):  #многокоординатное движение для указанной оси с заданной задержкой
    acs.acsc_MultiPoint(hcomm, flags, axis, double(dwell), wait) #dwell — это время задержки (ожидания) в конце движения

def addPoint(hcomm, axis, point, wait=SYNCHRONOUS):  #Добавляет точку для движения указанной оси
    acs.acsc_AddPoint(hcomm, axis, double(point), wait)

def extAddPoint(hcomm, axis, point, rate, wait=SYNCHRONOUS):  #Добавляет точку для движения с указанной скоростью (rate)
    acs.acsc_ExtAddPoint(hcomm, axis, double(point), double(rate), wait)

def endSequence(hcomm, axis, wait=SYNCHRONOUS):  #Завершает последовательность движения для указанной оси
    return acs.acsc_EndSequence(hcomm, axis, wait)

def go(hcomm, axis, wait=SYNCHRONOUS):  #Начинает движение указанной оси
    acs.acsc_Go(hcomm, axis, wait)

# ! By myself
def goM(hcomm, axes, wait=SYNCHRONOUS): #Начинает движение осей, ожидающих приказа, синхронно
    axes_array = ctypes.c_int*(len(axes) + 1)
    axes_c = axes_array()
    for n in range(len(axes)):
        axes_c[n] = axes[n]
        axes_c[-1] = -1
    acs.acsc_GoM(hcomm, axes_c, wait)
# ! By myself

def getOutput(hcomm, port, bit, wait=SYNCHRONOUS):  #Возвращает значение цифрового выхода контроллера
    """Returns the value of a digital output."""
    val = int32()
    acs.acsc_GetOutput(hcomm, port, bit, byref(val), wait)
    return val.value

def setOutput(hcomm, port, bit, val, wait=SYNCHRONOUS):  #Устанавливает значение цифрового выхода контроллера
    """Sets the value of a digital output."""
    acs.acsc_SetOutput(hcomm, port, bit, val, wait)

def errorHandling(returnvalue):  #Проверяет, произошла ли ошибка при выполнении функции
    if returnvalue == 0:
        error = getLastError()
        print("Error", error)

def printLastError():
    error = getLastError()
    if error != 0:
        print("ACS SPiiPlus Error", error)

def setRPosition(hcomm, axis, pos, wait=SYNCHRONOUS):  #Устанавливает реальную позицию (R Position) для указанной оси
    try:
        pos = ctypes.c_double(pos)
        acs.acsc_SetRPosition(hcomm, axis, pos, wait)
    except:
        raise IOError("Error: cannot set R position of the controller")

def setFPosition(hcomm, axis, pos, wait=SYNCHRONOUS):  #Устанавливает финальную позицию (F Position) для указанной оси
    try:
        pos = ctypes.c_double(pos)
        acs.acsc_SetFPosition(hcomm, axis, pos, wait)
    except:
        raise IOError("Error: cannot set F position of the controller")

#! All next functions made by myself!!!!!!!!!!
def smoothPointToPointMotion(hcomm, flags, axes, points, vel, wait=SYNCHRONOUS):
    acs.acss_SmoothPointToPointMotion(hcomm, flags, axes, points, vel, wait)



#!!!ДОБАВИТЬ ОБРАБОТКУ ВХОДНЫХ АРГУМЕНТОВ (double, tuple и т.д.)
def waitMotionEnd(hcomm, axis, timeout):
    """Waits for motion to end."""
    axis = int32(axis)
    timeout = int32(timeout)
    acs.acsc_WaitMotionEnd(hcomm, axis, timeout)  # Timeout - ms

def double_pointer(val):
    return ctypes.byref(ctypes.c_double(val)) if val is not None else ctypes.POINTER(ctypes.c_double)


def extendedSegmentedMotionV2(hcomm,
                              flags,
                              axes,
                              point,
                              vel,
                              endVel,
                              juncVel,
                              angle,
                              curveVel,
                              deviation,
                              radius,
                              maxLength,
                              starvMargin,
                              segments,
                              extLoopType,
                              minSegmentLength,
                              maxAllowedDeviation,
                              outputIndex,
                              bitNumber,
                              polarity,
                              motionDelay,
                              wait=SYNCHRONOUS
                              ):
    n = len(axes)
    axes_c = (ctypes.c_int * (n + 1))(*axes, -1)
    point_c = (ctypes.c_double * n)(*point)

    result = acs.acsc_ExtendedSegmentedMotionV2(
        hcomm,
        flags,
        axes_c,
        point_c,
        ctypes.byref(double(vel)),
        ctypes.byref(double(endVel)),
        ctypes.byref(double(juncVel)),
        ctypes.byref(double(angle)),
        ctypes.byref(double(curveVel)),
        ctypes.byref(double(deviation)),
        ctypes.byref(double(radius)),
        ctypes.byref(double(maxLength)),
        ctypes.byref(double(starvMargin)),
        double_pointer(segments),  #! Тут чар но почему-то работает с POINTER
        ctypes.byref(int64(extLoopType)),
        double_pointer(minSegmentLength),
        double_pointer(maxAllowedDeviation),
        ctypes.byref(int64(outputIndex)),
        ctypes.byref(int64(bitNumber)),
        ctypes.byref(int64(polarity)),
        double_pointer(motionDelay),
        wait=SYNCHRONOUS
    )

    if result == 0:
        raise RuntimeError("acsc_ExtendedSegmentedMotionV2 failed")
    return result


#!!!ДОБАВИТЬ ОБРАБОТКУ ВХОДНЫХ АРГУМЕНТОВ (double, tuple и т.д.)
def segmentArc2V2(hcomm,
                  flags,
                  axes,
                  center,
                  angle,
                  finalPoints,
                  vel,
                  endVel,
                  time,
                  values,
                  variables,
                  index,
                  masks,
                  extLoopType,
                  minSegmentLength,
                  maxAllowedDeviation,
                  lciState,
                  wait=SYNCHRONOUS):
   
    n = len(axes)
    axes_c = (ctypes.c_int * (n + 1))(*axes, -1)
    center_c = (ctypes.c_double * len(center))(*center)
    angle_c = ctypes.c_double(angle)
    finalPoints_c = (ctypes.c_double * len(finalPoints))(*finalPoints) if finalPoints else None

    
    result = acs.acsc_SegmentArc2V2(hcomm,
                           flags,
                           axes_c,
                           center_c,
                           angle_c,
                           finalPoints_c,
                           ctypes.byref(double(vel)),
                           ctypes.byref(double(endVel)),
                           ctypes.byref(double(time)),
                           ctypes.c_char_p(values.encode()) if values else None,
                           ctypes.c_char_p(variables.encode()) if variables else None,
                           ctypes.byref(int64(index)),
                           ctypes.c_char_p(masks.encode()) if masks else None,
                           ctypes.byref(int64(extLoopType)),
                           ctypes.byref(double(minSegmentLength)),
                           ctypes.byref(double(maxAllowedDeviation)),
                           ctypes.byref(int64(lciState)),
                           wait=SYNCHRONOUS)
    
    if result == 0:
        raise RuntimeError("acsc_ExtendedSegmentedMotionV2 failed")
    return result


def segmentLineV2(hcomm,
                  flags,
                  axes,
                  point,
                  vel,
                  endVel,
                  time,
                  values,
                  variables,
                  index,
                  masks,
                  extLoopType,
                  minSegmentLength,
                  maxAllowedDeviation,
                  lciState,
                  wait=SYNCHRONOUS):
    
    n = len(axes)
    axes_c = (ctypes.c_int * (n + 1))(*axes, -1)
    point_c = (ctypes.c_double * len(point))(*point)
    
    result = acs.acsc_SegmentLineV2(hcomm,
                           flags,
                           axes_c,
                           point_c,
                           ctypes.byref(double(vel)),
                           ctypes.byref(double(endVel)),
                           ctypes.byref(double(time)),
                           ctypes.c_char_p(values.encode()) if values else None,
                           ctypes.c_char_p(variables.encode()) if variables else None,
                           ctypes.byref(int64(index)),
                           ctypes.c_char_p(masks.encode()) if masks else None,
                           ctypes.byref(int64(extLoopType)),
                           ctypes.byref(double(minSegmentLength)),
                           ctypes.byref(double(maxAllowedDeviation)),
                           ctypes.byref(int64(lciState)),
                           wait=SYNCHRONOUS)
    
    if result == 0:
        raise RuntimeError("acsc_ExtendedSegmentedMotionV2 failed")
    return result

def group(hcomm, axes, wait=SYNCHRONOUS):
    """Groups multiple axes together for synchronized motion."""
    axes_array = ctypes.c_int*(len(axes) + 1)
    axes_c = axes_array()
    for n in range(len(axes)):
        axes_c[n] = axes[n]
    axes_c[-1] = -1
    acs.acsc_Group(hcomm, axes_c, wait)

def splitAll(hcomm, wait=SYNCHRONOUS):
    """Splits all axes in the controller."""
    acs.acsc_SplitAll(hcomm, wait)
    
def endSequenceM(hcomm, axes, wait=SYNCHRONOUS):
    axes_array = ctypes.c_int*(len(axes) + 1)
    axes_c = axes_array()
    for n in range(len(axes)):
        axes_c[n] = axes[n]
    axes_c[-1] = -1
    acs.acsc_EndSequenceM(hcomm, axes_c, wait)

def getProgramError(hcomm, nbuf, wait=SYNCHRONOUS):
    """
    Возвращает код последней ошибки выполнения для указанного буфера.
    Для команд, отправленных из Python по API, используется nbuf = -1.
    """
    # Создаем объект типа "integer", куда C-функция запишет результат
    error_code = ctypes.c_int()
    
    # Вызываем функцию из DLL, передавая error_code по ссылке (byref)
    acs.acsc_GetProgramError(hcomm, nbuf, byref(error_code), wait)
    
    # Возвращаем значение, которое теперь хранится в нашем объекте
    return error_code.value

def getErrorString(hcomm, error_code, wait=SYNCHRONOUS):
    """
    Возвращает текстовое описание для заданного кода ошибки.
    """
    # Создаем буфер для хранения строки, которую вернет C-функция.
    # 256 байт - более чем достаточно для большинства сообщений об ошибках.
    buffer_size = 256
    error_string_buffer = ctypes.create_string_buffer(buffer_size)
    
    # Вызываем функцию из DLL, передавая ей код ошибки и буфер для записи текста
    acs.acsc_GetErrorString(hcomm, error_code, error_string_buffer, buffer_size, wait)
    
    # C-функция возвращает байтовую строку.
    # Мы декодируем ее в обычную строку Python и возвращаем.
    # errors='ignore' поможет избежать проблем с кодировкой.
    return error_string_buffer.value.decode('utf-8', errors='ignore')


if __name__ == "__main__":  #Этот код выполнится только при запуске файла напрямую
    """Some testing can go here"""
    # hc = openCommEthernetTCP(address = "172.20.196.4")
    # print(hc)
    # print(getOutput(hc, 1, 16))
    # closeComm(hc)
    print("Type of flags:", type(AMF_VELOCITY))  # <- это должен быть <class 'int'>