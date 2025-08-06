# -*- coding: utf-8 -*-
"""
ACS Controller Executable Script
-----------------------------------
This script connects to the ACS controller and manages axis movement
"""

from __future__ import division, print_function
import acsc_modified as acsc
import newACS
import time
from PyQt6 import QtGui
import io
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QPlainTextEdit, QTabWidget
from PyQt6.QtGui import QTextCursor, QColor
from PyQt6.QtCore import Qt, QTimer, QObject, pyqtSignal, QThread, pyqtSlot
# Импортируем сгенерированный класс. Команда: pyuic6 undulator.ui -o undulator.py
from undulator import Ui_MainWindow
from workers import SingleAxisWorker, FFIMeasurementWorker, SFIMeasurementWorker, FindMagneticAxisWorker, CircularMotionWorker
import numpy as np
import csv
import matplotlib.pyplot as plt
import traceback


class ACSControllerGUI(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # Инициализация интерфейса
        self.stand = None
        self.fma_worker = None
        self.axis_workers = {}  # Словарь: {axis_id: worker}

        # Тестовый вывод
        self.dual_print("Программа запущена!")
        
        # Настройка окна для вывода принтов
        #?? Блок настройки логгера
        self._setup_logger(self.Console)  #!!!

        self.initTabText() #!!!

        # Создаём словари для 4 осей:
        '''Где есть getattr(self, f"чётотам{i}") - это функция, которая возвращает объект QLineEdit для ввода перемещения оси i.'''
        self.axes_data = {
            i: {
                "state": False,
                "speed_input": getattr(self, f"speed_input_{i}"),
                "acceleration_input": getattr(self, f"acceleration_input_{i}"),
                "deceleration_input": getattr(self, f"deceleration_input_{i}"),
                "kill_deceleration_input": getattr(self, f"kill_deceleration_input_{i}"),
                "jerk_input": getattr(self, f"jerk_input_{i}"),
                "axis_state_indicator": getattr(self, f"axis_state_indicator_{i}"),
                "pos_label": getattr(self, f"pos_label_{i}"),
                "is_moving_label": getattr(self, f"is_moving_label_{i}"),
                "is_acc_label": getattr(self, f"is_acc_label_{i}"),
                "is_in_pos_label": getattr(self, f"is_in_pos_label_{i}"),
                "is_in_pos_indicator": getattr(self, f"is_in_pos_indicator_{i}"),
                "choose_axis_button": getattr(self, f"choose_axis_{i}"),
                "enable_axis_button": getattr(self, f"enable_axis_button_{i}"),
                "start_axis_button": getattr(self, f"start_axis_{i}"),
                "move_distance": getattr(self, f"move_by_input_{i}"),
                "current_pos": 0.0,
                "axis_obj": None, #!Здесь хранятся ссылки на ось как объект из модуля newACS, к которым можно применять его методы
                "is_moving_indicator": getattr(self, f"is_moving_indicator_{i}")
            }
            for i in range(4)
        }

        self.connect_ui_elements()                               # Подключаем функции к элементам интерфейса
        self.selected_axes = []


    def connect_ui_elements(self):                               # Кнопки общего действия: старт, стоп, ресет
        self.connect_button.clicked.connect(self.connect_to_controller)
        self.reset_button.clicked.connect(self.set_default_values)
        self.zeropos_button.clicked.connect(self.zeropos_axes)
        self.start_choosen_axis_button.clicked.connect(self.startM)
        self.stop_button.clicked.connect(self.stop_all_axes)
        self.stop_button_2.clicked.connect(self.stop_all_axes)
        self.start_mode_motion.clicked.connect(self.check_mode_then_start)
        self.stop_button_test.clicked.connect(self.stop_all_axes)
        self.start_mode_motion_test.clicked.connect(self.check_mode_then_start_test)
        self.tab1.currentChanged.connect(self.currentTab)
        # self.findMagAxes_button.clicked.connect(self.findMagneticAxis)
        self.findMagAxes_button.clicked.connect(self.start_find_magnetic_axis_worker) # New
        

        for i in range(4):
            '''Перед connect стоит т.н. сигнал, а сам connect связывает сигнал с обработчиком'''
            data = self.axes_data[i]
            data["speed_input"].textChanged.connect(lambda text, ax=i: self.set_speed(ax, text))
            data["acceleration_input"].textChanged.connect(lambda text, ax=i: self.set_acceleration(ax, text))
            data["deceleration_input"].textChanged.connect(lambda text, ax=i: self.set_deceleration(ax, text))
            data["kill_deceleration_input"].textChanged.connect(lambda text, ax=i: self.set_kill_deceleration(ax, text))
            data["jerk_input"].textChanged.connect(lambda text, ax=i: self.set_jerk(ax, text))
            data["move_distance"].textChanged.connect(lambda text, ax=i: self.set_move_distance(ax, text))
            data["enable_axis_button"].clicked.connect(lambda checked, ax=i: self.toggle_axis(ax))
            data["start_axis_button"].clicked.connect(lambda checked, ax=i: self.start(ax))
            data["choose_axis_button"].stateChanged.connect(lambda state, ax=i: self.update_selected_axes(ax, state))

    def set_default_values(self): # Выставляет дефолтные параметры движения осей в общем окне
        for i in range(4):
            axis = self.axes_data[i]
            axis["speed_input"].setText("0.2")
            axis["acceleration_input"].setText("100")
            axis["deceleration_input"].setText("100")
            axis["kill_deceleration_input"].setText("166.67")
            axis["jerk_input"].setText("133.33")
    
    def dual_print(self, message, log_window=None):
        """
        Основной метод вывода:
        - message: текст сообщения
        - print(): вывод в консоль
        - appendPlainText(): вывод в GUI
        - _auto_scroll(): прокрутка вниз
        """
        if log_window == None:
            log_window = self.Console
        print(message)  # Консольный вывод
        log_window.appendPlainText(message)  # GUI-вывод
        self._auto_scroll(log_window)  # Автопрокрутка

    def _setup_logger(self, log_window=None):
        """Приватный метод для настройки логгера"""
        if log_window == None:
            log_window = self.Console
        # 1. Делаем лог только для чтения
        log_window.setReadOnly(True)
        
        # 2. Отключаем перенос строк (удобно для логов)
        log_window.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        
        # 3. Опционально: задаем шрифт с фиксированной шириной
        font = log_window.font()
        font.setFamily("Courier New")  # Моноширинный шрифт
        log_window.setFont(font)

    def _auto_scroll(self, log_window):
        """Приватный метод для автопрокрутки"""
        cursor = log_window.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        log_window.setTextCursor(cursor)

    def clear_logs(self, log_window=None):
        """Очистка лога"""
        if log_window == None:
            log_window = self.Console
        log_window.clear()

#!МБ ЭТО УБРАТЬ И СДЕЛАТЬ ЧЕРЕЗ ДИЗАЙНЕР
    def initTabText(self):
        # current_tab = self.tab1.currentIndex()   # Объект текущего окна
        # current_tab_name = self.tab1.tabText(current_tab)  # Название текущего окна
        
        self.tablePosition.item(0, 0).setText(f"{0}")
        self.tablePosition.item(1, 0).setText(f"{1}") # 4.15151:.3f
        self.tablePosition.item(2, 0).setText(f"{2}")
        self.tablePosition.item(3, 0).setText(f"{3}")

    def zeropos_axes(self):
        self.axes_data[0]["axis_obj"].set_pos(0)
        self.axes_data[1]["axis_obj"].set_pos(0)
        self.axes_data[2]["axis_obj"].set_pos(0)
        self.axes_data[3]["axis_obj"].set_pos(0)
        pass

    def connect_to_controller(self):
        """Подключается к контроллеру. Инициализирует оси как объекты в ключе 'axis_obj """
        self.stand = newACS.newAcsController(newACS.acs_ip, newACS.acs_port, contype='Ethernet', n_axes=4)
        if self.stand.connect() == -1:
            self.show_error("Ошибка подключения к контроллеру")
            self.label.setStyleSheet('background-color: red')
            self.stand = None
        else:
            self.label.setStyleSheet('background-color: green')
            # self.set_default_values() # Устанавливаем дефолтные значения при успешном подключении
            for i in range(4):        # После успешного подключения обновляем ссылки на оси в словарях
                self.axes_data[i]["axis_obj"] = self.stand.axes[i]
                

    def toggle_axis(self, axis):
        """Включает/выключает ось. Присваивает ключу 'state' значение True или False"""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        data = self.axes_data[axis] #! Короткая ссылка
        if data["state"]:
            data['axis_obj'].disable()
            data["axis_state_indicator"].setStyleSheet('background-color: red')
            data["state"] = False
        else:
            data['axis_obj'].enable()
            data["axis_state_indicator"].setStyleSheet('background-color: green')
            data["state"] = True

    def update_selected_axes(self, axis, state):
        """Обновляет список выбранных осей."""
        state = Qt.CheckState(state) # Эта строка преобразует число в понятное для Qt значение (мб 'checked')
        if state == Qt.CheckState.Checked:  # Если галочка поставлена
            if axis not in self.selected_axes:
                self.selected_axes.append(axis)
                self.selected_axes = sorted(self.selected_axes)
                print('Добавили в список')
        if state == Qt.CheckState.Unchecked:  # Если галочка снята
            if axis in self.selected_axes:
                self.selected_axes.remove(axis)
                print('Удалили из списка')
        print(f'Список выбранных осей: {self.selected_axes}')

    def set_speed(self, axis, text):
        if not self.stand:
            # self.show_error("Контроллер не подключён!")
            return
        
        data = self.axes_data[axis]
        try:
            speed = float(text)
            data['axis_obj'].set_speed(speed) # Передаём значение в контроллер
            data["speed"] = speed # Сохраняем скорость в словаре для оси `axis`
        except ValueError:
            self.show_error("Некорректный ввод скорости")

    def set_acceleration(self, axis, text):
        if not self.stand:
            # self.show_error("Контроллер не подключён!")
            return
        
        data = self.axes_data[axis]
        try:
            acceleration = float(text)
            data['axis_obj'].set_acceleration(acceleration) # Это другая set_acceleration из модуля newACS
            data["acceleration"] = acceleration  # Сохраняем ускорение в словаре для оси `axis`
        except ValueError:
            self.show_error("Некорректный ввод скорости")

    def set_deceleration (self, axis, text):
        if not self.stand:
            # self.show_error("Контроллер не подключён!")
            return
        
        data = self.axes_data[axis]
        try:
            deceleration = float(text)
            data['axis_obj'].set_deceleration(deceleration) # Передаём значение в контроллер
            data["deceleration"] = deceleration  # Сохраняем замедление в словаре для оси `axis`
        except ValueError:
            self.show_error("Некорректный ввод скорости")

    def set_kill_deceleration (self, axis, text):
        if not self.stand:
            # self.show_error("Контроллер не подключён!")
            return

        data = self.axes_data[axis]
        try:
            kill_deceleration = float(text)
            data['axis_obj'].set_kill_deceleration(kill_deceleration) # Передаём значение в контроллер
            data["kill_deceleration"] = kill_deceleration  # Сохраняем замедление в словаре для оси `axis`
        except ValueError:
            self.show_error("Некорректный ввод скорости")

    def set_jerk(self, axis, text):
        if not self.stand:
            # self.show_error("Контроллер не подключён!")
            return

        data = self.axes_data[axis]
        try:
            jerk = float(text)
            data['axis_obj'].set_jerk(jerk) # Передаём значение в контроллер
            data["jerk"] = jerk  # Сохраняем рывок в словаре для оси `axis`
        except ValueError:
            self.show_error("Некорректный ввод скорости")

    def set_move_distance(self, axis, text):
        if not self.stand:
            # self.show_error("Контроллер не подключён!")
            return

        data = self.axes_data[axis]
        try:
            distance = float(text)
            data["move_distance"] = distance
        except ValueError:
            self.show_error("Некорректный ввод перемещения")

    def start(self, axis):
        '''Простое перемещение одной оси (тестовое)'''
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        self.oneAxisLog = {  # Инициализация лога
            'time': [],
            'pos': [],
        }

        data = self.axes_data[axis]
        if data['state'] and axis in self.selected_axes:
            try:
                '''Здесь amf_relative - это флаг, который указывает, что перемещение будет относительным.'''
                acsc.toPoint(self.stand.hc, acsc.AMF_RELATIVE, axis, data['move_distance'], acsc.SYNCHRONOUS)
                data["is_in_pos_indicator"].setStyleSheet("background-color:rgb(255, 0, 0)")
                self.start_position_updates()
            except Exception as e:
                self.show_error(f"Ошибка при запуске движения оси {axis}: {e}")
        else:
            self.show_error(f"Ось {axis} не включена или не выбрана!")

    def startM(self):
        '''Синхронный старт движения выбранныхосей'''
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        self.dual_print(f"Открытая вкладка: {self.tab1.tabText(self.tab1.currentIndex())}")
        data = self.axes_data
        move_distances = []
        self.startpointM = data[self.selected_axes[0]]['axis_obj'].get_pos()
        leader = self.selected_axes[0]
        for i in self.selected_axes:
            if data[i]['state'] and data[i]['move_distance'] != 0:
                move_distances.append(data[i]['move_distance'])
        try:
            '''
            Здесь функция toPointM вызывается напрямую из модуля acsc_modified
            ТАМ УЖЕ ДОБАВЛЯЕТСЯ -1 В КОНЦЕ СПИСКА ОСЕЙ!!!!!!!!!!!!
            Здесь acsc.AMF_RELATIVE - это флаг, который указывает, что перемещение будет относительным.
            '''
            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(self.selected_axes), tuple(move_distances), acsc.SYNCHRONOUS)
            for i in self.selected_axes:
                data[i]["is_in_pos_indicator"].setStyleSheet("background-color:rgb(255, 0, 0)")
            print('Успешно запущенно движение осей')
            # Запускаем таймер обновления позиций
            self.start_position_updates()
        except Exception as e:
            self.show_error(f"Ошибка при запуске движения: {e}")

    def stop_all_axes(self):
        """Останавливает все оси."""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return

        try:
            acsc.killAll(self.stand.hc, acsc.SYNCHRONOUS)
        except Exception as e:
            self.show_error(f"Ошибка при остановке осей: {e}")

    def show_error(self, message):
        """Показывает сообщение об ошибке."""
        QMessageBox.critical(self, "Ошибка", message)

    def currentTab(self):
        self.currentTab = self.tab1.currentIndex()
        self.currentTabName = self.tab1.tabText(self.currentTab)
        self.dual_print(f'Вкладка переключена на "{self.currentTabName}"')

    def start_position_updates(self):
        """Запуск отдельного потока для каждой выбранной оси"""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return

        for axis_id in self.selected_axes:
            # Останавливаем предыдущий поток оси, если был
            if axis_id in self.axis_workers:
                self.axis_workers[axis_id].stop()

            # Создаём и настраиваем новый поток
            worker = SingleAxisWorker(self.stand, axis_id)
            worker.update_signal.connect(self.handle_axis_update)
            '''"Когда worker излучит update_signal, вызови мой метод handle_axis_update и передай ему данные из сигнала"'''
            worker.error_signal.connect(self.handle_axis_error)
            worker.start()
            
            self.axis_workers[axis_id] = worker

    def stop_position_updates(self):
        """Остановка всех потоков осей"""
        for axis_id, worker in self.axis_workers.items():
            worker.stop()
        self.axis_workers.clear()

    @pyqtSlot(int, float, bool, bool) 
    def handle_axis_update(self, axis_id, pos, moving, in_pos):
        """Обновление данных оси в GUI (выполняется в главном потоке)"""
        axis_data = self.axes_data[axis_id]

        # current_tab = self.tab1.currentIndex()   # Объект текущего окна
        # current_tab_name = self.tab1.tabText(current_tab)  # Название текущего окна
        #! Тут остановился!!!

        if self.currentTabName == "Settings":  # Замените на актуальное название вкладки
            # Обновляем значения
            axis_data["pos_label"].setText(f"Позиция: {pos:.4f}")
            axis_data["is_moving_label"].setText(f"Движение: {'Да' if moving else 'Нет'}")
            axis_data["is_in_pos_label"].setText(f"На месте: {'Да' if in_pos else 'Нет'}")

            # Меняем цвет индикаторов
            moving_color = "rgb(0, 128, 0)" if moving else "rgb(255, 0, 0)"  # Красный/Зелёный
            in_pos_color = "rgb(0, 128, 0)" if in_pos else "rgb(255, 0, 0)"
            
            axis_data["is_moving_indicator"].setStyleSheet(f"background-color: {moving_color}")
            axis_data["is_in_pos_indicator"].setStyleSheet(f"background-color: {in_pos_color}")

        elif self.currentTabName == "Выбор режимов движения":
            self.tablePosition.item(axis_id, 1).setText(f"Позиция: {pos:.4f}")

    @pyqtSlot(int, str)
    def handle_axis_error(self, axis_id, error_msg):
        """Обработка ошибок оси"""
        self.show_error(f"Ось {axis_id}: {error_msg}")
        if axis_id in self.axis_workers:
            self.axis_workers[axis_id].stop()

    def closeEvent(self, event):
        """Остановка потоков при закрытии окна"""
        self.stop_position_updates()
        super().closeEvent(event)


    def startCircularMotionWBuffer(self):
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        all_axes = [0, 1, 2, 3]
        try:
            speed = float(self.cm_speed_input.text())
            for axis in all_axes:
                self.axes_data[axis]['axis_obj'].enable()
                self.axes_data[axis]["state"] = True
                self.axes_data[axis]['axis_obj'].set_speed(speed)
                if axis not in self.selected_axes:
                    self.selected_axes.append(axis)
                    self.selected_axes = sorted(self.selected_axes)
            self.dual_print(f"Оси {all_axes} включены.")
            self.dual_print(f"Скорость {self.speed} мм/с установлена для осей {all_axes}.")
        except Exception as e:
            self.dual_print(f"Ошибка при включении осей или установке скорости: {e}")
            print(f"Ошибка при включении осей или установке скорости: {e}")
        
        try:
            radius = float(self.cm_radius_input.text())
            rotation = str(self.cm_rotation_input.text())
            angle = float(self.cm_angle_input.text())
            N = int(self.cm_number_of_rounds_input.text())
            self.dual_print(f"N: {N}, тип: {type(N)}")
        except Exception as e:
            self.dual_print(f"Ошибка чтения параметров кругового движения: {e}")

        try:
            nano = ktl(resource="GPIB0::7::INSTR", mode='meas')   #! Создаём экземпляр класса Keithley2182A
        except Exception as e:
            self.dual_print("Ошибка подключения к Keithley")
        else:
            self.dual_print("Успешное подключение к Keithley")


        self.cm_worker = CircularMotionWorker(self.stand, nano, speed, radius, rotation, N, angle)
        self.cm_worker.log_ready.connect(self.handle_cm_log)
        self.cm_worker.error_signal.connect(lambda msg: self.show_error(f"CM ошибка: {msg}"))
        self.cm_worker.progress_signal.connect(self.print_from_workers)
        self.cm_worker.start()
        self.start_position_updates()
        self.dual_print(f"Измерение FFI успешно запущено, идёт измерение...")


    @pyqtSlot(dict)
    def handle_cm_log(self, log):
        self.cm_motion_log = log
        fig1, fig2 = calc.harmonicAnalysis(log)

        try:
            buf = io.BytesIO()
            fig1.savefig(buf, format='png', dpi=300, bbox_inches='tight')
            buf.seek(0)
            pixmap = QtGui.QPixmap()
            pixmap.loadFromData(buf.getvalue())
            buf.close()
            plt.close(fig1)

            self.plot_pic.setPixmap(pixmap)
            self.plot_pic.setScaledContents(True)
            self.dual_print("График отображён в QLabel")
        except Exception as e:
            self.show_error(f"Ошибка отображения графика: {e}")
            if fig1:
                plt.close(fig1)


        try:
            buf1 = io.BytesIO()
            fig2.savefig(buf1, format='png', dpi=300, bbox_inches='tight')
            buf1.seek(0)
            pixmap2 = QtGui.QPixmap()
            pixmap2.loadFromData(buf1.getvalue())
            buf1.close()
            plt.close(fig2)

            self.plot_pic_2.setPixmap(pixmap2)
            self.plot_pic_2.setScaledContents(True)
            self.dual_print("График отображён в QLabel")
        except Exception as e:
            self.show_error(f"Ошибка отображения графика: {e}")
            if fig2:
                plt.close(fig2)


    def start_ffi_motion(self):
        """Проверяет режим движения и запускает соответствующий метод."""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        #! МОЖНО СДЕЛАТЬ QDoubleValidator и автоматическую замену запятой на точку
        try:
            distance = float(self.ffi_distance_input.text())
        except ValueError:
            self.show_error("Ошибка: введите число через точку")
            self.ffi_distance_input.setText('0.0')
            distance = 0.0  # или другое значение по умолчанию
        else:
            self.dual_print(f"Дистанция успешно введена и установлена")

        try:
            mode = (self.ffi_mode_input.text())
            if mode and distance != 0:
                if mode == 'X':
                    ffi_axes = [1,3]
                    self.selected_axes = ffi_axes
                    for axis in ffi_axes:
                        if not self.axes_data[axis]["state"]:
                            self.axes_data[axis]['axis_obj'].enable()
                            self.axes_data[axis]["state"] = True
                elif mode == 'Y':
                    ffi_axes = [0,2]
                    self.selected_axes = ffi_axes
                    for axis in ffi_axes:
                        if not self.axes_data[axis]["state"]:
                            self.axes_data[axis]['axis_obj'].enable()
                            self.axes_data[axis]["state"] = True
        except Exception as e:
            self.show_error("Ошибка: Введите капсом 'X' или 'Y'")
        else:
            self.dual_print(f"Мод успешно выбран")

        try:
            speed = float(self.ffi_speed_input.text())
            for axis in ffi_axes:  # Задаём скорость осям с поля ввода
                    self.axes_data[axis]['axis_obj'].set_speed(speed)
        except ValueError:
            self.show_error(" Ошибка: Что-то со скоростью мб")
        else:
            self.dual_print(f"Скорость успешно введена и установлена")

        try:
            nano = ktl(resource="GPIB0::7::INSTR", mode='meas')              #! Создаём экземпляр класса Keithley2182A
        except Exception as e:
            self.dual_print("Ошибка подключения к Keithley")
        else:
            self.dual_print("Успешное подключение к Keithley")

        
        self.ffi_worker = FFIMeasurementWorker(self.stand, ffi_axes, nano, distance, speed, mode)
        self.ffi_worker.log_ready.connect(self.handle_ffi_log)
        self.ffi_worker.error.connect(lambda msg: self.show_error(f"FFI ошибка: {msg}"))
        self.ffi_worker.progress_signal.connect(self.print_from_workers)
        self.ffi_worker.start()
        self.start_position_updates()
        self.dual_print(f"Измерение FFI успешно запущено, идёт измерение...")

    @pyqtSlot(dict)
    def handle_ffi_log(self, log):
        # self.ffi_motion_log = log
        fig = calc.firstFieldIntegral(log, self.ffi_worker.mode, self.ffi_worker.speed)

        try:
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=300, bbox_inches='tight')
            buf.seek(0)
            pixmap = QtGui.QPixmap()
            pixmap.loadFromData(buf.getvalue())
            buf.close()
            plt.close(fig)

            self.plot_pic.setPixmap(pixmap)
            self.plot_pic.setScaledContents(True)
            self.dual_print("График отображён в QLabel")
        except Exception as e:
            self.show_error(f"Ошибка отображения графика: {e}")
            if fig:
                plt.close(fig)

        
    def start_sfi_motion(self):
        """Проверяет режим движения и запускает соответствующий метод."""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        #! МОЖНО СДЕЛАТЬ QDoubleValidator и автоматическую замену запятой на точку
        try:
            distance = float(self.sfi_distance_input.text())
        except ValueError:
            self.show_error("Ошибка: введите число через точку")
            self.sfi_distance_input.setText('0.0')
            distance = 0.0  # или другое значение по умолчанию
        else:
            self.dual_print(f"Дистанция успешно введена и установлена")

        try:
            mode = (self.sfi_mode_input.text())
            if mode and distance != 0:
                if mode == 'X':
                    sfi_axes = [1,3]
                    self.selected_axes = sfi_axes
                    for axis in sfi_axes:
                        if not self.axes_data[axis]["state"]:
                            self.axes_data[axis]['axis_obj'].enable()
                            self.axes_data[axis]["state"] = True
                elif mode == 'Y':
                    sfi_axes = [0,2]
                    self.selected_axes = sfi_axes
                    for axis in sfi_axes:
                        if not self.axes_data[axis]["state"]:
                            self.axes_data[axis]['axis_obj'].enable()
                            self.axes_data[axis]["state"] = True
        except Exception as e:
            self.show_error("Ошибка: Введите капсом 'X' или 'Y'")
        else:
            self.dual_print(f"Мод успешно выбран")

        try:
            speed = float(self.sfi_speed_input.text())
            for axis in sfi_axes:  # Задаём скорость осям с поля ввода
                    self.axes_data[axis]['axis_obj'].set_speed(speed)
        except ValueError:
            self.show_error(" Ошибка: Что-то со скоростью мб")
        else:
            self.dual_print(f"Скорость успешно введена и установлена")
        
        try:
            nano = ktl(resource="GPIB0::7::INSTR", mode='meas')              #! Создаём экземпляр класса Keithley2182A
        except Exception as e:
            self.dual_print("Ошибка подключения к Keithley")
        else:
            self.dual_print("Успешное подключение к Keithley")

        self.sfi_worker = SFIMeasurementWorker(self.stand, sfi_axes, nano, distance, speed, mode)
        self.sfi_worker.log_ready.connect(self.handle_sfi_log)
        self.sfi_worker.progress_signal.connect(self.print_from_workers)
        self.sfi_worker.error.connect(lambda msg: self.show_error(f"SFI ошибка: {msg}"))
        self.sfi_worker.start()
        self.start_position_updates()
        self.dual_print(f"Измерение SFI успешно запущено, идёт измерение...")
            
    @pyqtSlot(dict)
    def handle_sfi_log(self, log):
        fig = calc.secondFieldIntegral(log, self.sfi_worker.mode, self.sfi_worker.speed)
            
        try:
            # 1. Создаем буфер в памяти
            buf = io.BytesIO()
            # 2. Сохраняем фигуру в буфер в формате PNG
            #    dpi можно подобрать для нужного размера/качества на экране (напр. 96)
            fig.savefig(buf, format='png', dpi=300, bbox_inches='tight')
            buf.seek(0) # Перемещаем указатель в начало буфера

            # 3. Загружаем данные из буфера в QPixmap
            pixmap = QtGui.QPixmap()
            pixmap.loadFromData(buf.getvalue())
            buf.close() # Закрываем буфер

            # !!! ВАЖНО: Закрываем фигуру Matplotlib после использования, чтобы освободить память !!!
            plt.close(fig)

            # 4. Устанавливаем QPixmap в ваш QLabel
            self.plot_pic.setPixmap(pixmap)
            # (Опционально) Масштабируем изображение под размер QLabel
            self.plot_pic.setScaledContents(True)
            print("График отображен в QLabel.")
        except Exception as e:
            self.show_error(f"Неизвестная ошибка в calc.sevondFieldIntegral или отображении графика: {e}")
            if fig: plt.close(fig) # Закрыть фигуру и при других ошибках


    def check_mode_then_start(self):
        """Проверяет режим движения и запускает соответствующий метод."""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        selected_mode = self.check_mode.currentText()
        print(f"Нажата кнопка 'Старт', выбран режим: {selected_mode}")

        if selected_mode == "По окружности":
            # self.start_circular_motion()
            self.startCircularMotionWBuffer()
        elif selected_mode == "Первый магнитный интеграл":
            self.start_ffi_motion()
        elif selected_mode == "Второй магнитный интеграл":  #Todo добавить возврат в ноль мб
            self.start_sfi_motion()


    def start_find_magnetic_axis_worker(self):
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return

        if self.fma_worker and self.fma_worker.isRunning():
            self.show_error("Процесс поиска магнитной оси уже запущен.")
            return

        try:
            distance = float(self.fma_distance_input.text())
            speed = float(self.fma_speed_input.text())
        except ValueError:
            self.dual_print("Ошибка: введите число через точку для дистанции/скорости.")
            return

        # Initialize Keithley if not already done (self.nano)
        if not hasattr(self, 'nano') or self.nano is None:
            try:
                nano = ktl(resource="GPIB0::7::INSTR", mode='meas')
                self.dual_print("Успешное подключение к Keithley для поиска оси.")
            except Exception as e:
                self.dual_print(f"Ошибка подключения к Keithley: {e}")
                return

        CONVERGENCE_THRESHOLD = 0.005  # mm, example value, make it configurable if needed
        MAX_ITERATIONS = 3             # Maximum number of iterations for convergence

        self.fma_worker = FindMagneticAxisWorker(
            self.stand,
            nano,
            distance,
            speed,
            CONVERGENCE_THRESHOLD,
            MAX_ITERATIONS
        )

        self.fma_worker.progress_signal.connect(self.print_from_workers)
        self.fma_worker.error_signal.connect(self.handle_fma_error)
        self.fma_worker.finished_signal.connect(self.handle_fma_finished)

        # Disable button, etc.
        self.findMagAxes_button.setEnabled(False)
        self.dual_print(f"Запуск поиска магнитной оси...")
        self.fma_worker.start()

    @pyqtSlot(str)
    def handle_fma_error(self, message):
        self.show_error(f"Ошибка поиска магн. оси: {message}")
        self.findMagAxes_button.setEnabled(True) # Re-enable button on error

    @pyqtSlot(dict)
    def handle_fma_finished(self, final_positions):
        self.dual_print("Поиск магнитной оси завершен.")
        # You can format and display final_positions as needed
        # for axis_id, pos in final_positions.items():
        #    self.dual_print(f"  {axis_id}: {pos:.4f} мм")
        self.findMagAxes_button.setEnabled(True) # Re-enable button on finish


    @pyqtSlot(str)
    def print_from_workers(self, message):
        self.dual_print(message)

    def circleBuffer(self):
        program = """
        ENABLE 0
        ENABLE 1
        ENABLE 2
        ENABLE 3
        MFLAGS(2).#DEFCON = 0
        CONNECT RPOS(2) = APOS(0)
        DEPENDS 2, 0
        MFLAGS(3).#DEFCON = 0
        CONNECT RPOS(3) = APOS(1)
        DEPENDS 3, 1
        GROUP (0,1,2,3)
        MSEG (0,1),0,0 
        ARC1 (0,1), 1,0,2,0,+ ! Add arc segment with center(1,0), final point (1,-1, clockwise rotation.
        ARC1 (0,1), 1,0,0,0,+
        ENDS (0,1)
        SPLITALL
        STOP
        """

        # Загрузка и запуск из Python:
        acsc.cleanBuffer(self.stand.hc, 0)
        acsc.loadBuffer(self.stand.hc, 0, program)
        acsc.compileBuffer(self.stand.hc, 0)
        acsc.runBuffer(self.stand.hc, 0)


    def check_mode_then_start_test(self):
        """Проверяет режим движения и запускает соответствующий метод."""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        selected_mode = self.check_mode_test.currentText()
        print(f"Нажата кнопка 'Старт', выбран режим: {selected_mode}")

        if selected_mode == "По окружности":
            self.circleBuffer()
            # self.simpleTest()
        elif selected_mode == "Первый магнитный интеграл":
            self.ffi_test()
        elif selected_mode == "Второй магнитный интеграл":  #Todo добавить возврат в ноль мб
            self.sfi_test()



if __name__ == '__main__':
    app = QApplication([])
    window = ACSControllerGUI()
    window.show()
    app.exec()
    # window.axisstate()
    # print(ACSControllerGUI.__dict__) # Shows all attributes the object have


# TODO В первой итерации из произвольной точки, выставление оси нужно будет добавить потом