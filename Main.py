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
# Импортируем сгенерированный класс. Команда: pyuic6 undulator2.ui -o undulator2.py
from undulator2 import Ui_MainWindow
from workers import SingleAxisWorker


class ACSControllerGUI(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # Инициализация интерфейса
        self.stand = None
        self.axis_workers = {}  # Словарь: {axis_id: worker}


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
                "is_moving_indicator": getattr(self, f"is_moving_indicator_{i}"),
                "masterslave": getattr(self, f"masterslave_{i}") #!!!!!!!!!!!!!!!!!!
            }
            for i in range(4)
        }

        self.connect_ui_elements()                               # Подключаем функции к элементам интерфейса
        self.selected_axes = []
        self.axis_pairs = {}
        self.masters = {1: "", 2: ""}
        self.slaves = {1: "", 2: ""}
        self.pair_1 = {"master": "", "slave": ""}
        self.pair_2 = {"master": "", "slave": ""}


    def connect_ui_elements(self):                               # Кнопки общего действия: старт, стоп, ресет
        self.connect_button.clicked.connect(self.connect_to_controller)
        self.reset_button.clicked.connect(self.set_default_values)
        self.zeropos_button.clicked.connect(self.zeropos_axes)
        self.start_choosen_axis_button.clicked.connect(self.startM)
        self.stop_button.clicked.connect(self.stop_all_axes)
        

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
            data["masterslave"].currentTextChanged.connect(lambda text, ax=i: self.update_masterslave_pairs(ax, text))

    def set_default_values(self): # Выставляет дефолтные параметры движения осей в общем окне
        for i in range(4):
            axis = self.axes_data[i]
            axis["speed_input"].setText("100")
            axis["acceleration_input"].setText("1000")
            axis["deceleration_input"].setText("1000")
            axis["kill_deceleration_input"].setText("1660")
            axis["jerk_input"].setText("1330")

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

    def update_masterslave_pairs(self, axis, role):
        """Обновляет пары masterslave."""
        # if not self.stand:
        #     self.show_error("Контроллер не подключён!")
        #     return

        for pair_num in range(1, 3):
            if self.masters[pair_num] == axis:
                self.masters[pair_num] = ""
            if self.slaves[pair_num] == axis:
                self.slaves[pair_num] = ""

        if role.startswith("Master pair"):
            pair_num = int(role[-1])
            self.masters[pair_num] = axis

        elif role.startswith("Slave pair"):
            pair_num = int(role[-1])
            self.slaves[pair_num] = axis

        self.axis_pairs = {
            pair_num: (self.masters[pair_num], self.slaves[pair_num])
            for pair_num in self.masters
            if self.masters[pair_num] != "" and self.slaves[pair_num] != ""
        }

        print(f"Обновлены пары: {self.axis_pairs}")  # Отладочный вывод
        self.axes_pairs.setText(f"Текущие пары: {self.axis_pairs}")

                        #???? ВОЗМОЖНО ЗАПОЛНИТЬ СЛОВАРИ С ПАРАМИ
        
        # Удаление всех старых связей (гарантированно для всех осей)
        try:
            delete_program = f"""
                MFLAGS({0}).#DEFCON = 1
                MFLAGS({1}).#DEFCON = 1
                MFLAGS({2}).#DEFCON = 1
                MFLAGS({3}).#DEFCON = 1
                """
            acsc.cleanBuffer(self.stand.hc, 3)
            acsc.loadBuffer(self.stand.hc, 3, delete_program)
            acsc.compileBuffer(self.stand.hc, 3)
            acsc.runBuffer(self.stand.hc, 3)
            print(f"Удалена зависимость для осей")
        except Exception as e:
            print(f"Не удалось очистить зависимости: {e}")

        for pair in self.axis_pairs:
            master, slave = self.axis_pairs[pair]
            try:
                program = f"""
                    MFLAGS({slave}).#DEFCON = 0
                    CONNECT RPOS({slave}) = APOS({master})
                    DEPENDS {slave}, {master}
                """
                acsc.cleanBuffer(self.stand.hc, 1)
                acsc.loadBuffer(self.stand.hc, 1, program)
                acsc.compileBuffer(self.stand.hc, 1)
                acsc.runBuffer(self.stand.hc, 1)

                print(f"Установлена связь: ось slave {slave} ← ось master {master}")
            except Exception as e:
                self.show_error(f"Ошибка при установке связи между master {master} и slave {slave}: {e}")


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
        
        if text == "" or text == "-":
            return  # Пропускаем проверку, пока ввод не заверен
    
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
        if data['state']:
            try:
                '''Здесь amf_relative - это флаг, который указывает, что перемещение будет относительным.'''
                acsc.toPoint(self.stand.hc, acsc.AMF_RELATIVE, axis, data['move_distance'], acsc.SYNCHRONOUS)
                data["is_in_pos_indicator"].setStyleSheet("background-color:rgb(255, 0, 0)")
                self.start_position_updates(axis)
            except Exception as e:
                self.show_error(f"Ошибка при запуске движения оси {axis}: {e}")
        else:
            self.show_error(f"Ось {axis} не включена или не выбрана!")

    def startM(self):
        '''Синхронный старт движения выбранныхосей'''
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        print(f"Открытая вкладка: {self.tab1.tabText(self.tab1.currentIndex())}")
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

    def start_position_updates(self, axis=None):
        """Запуск отдельного потока для каждой выбранной оси"""
        if not self.stand:
            self.show_error("Контроллер не подключён!")
            return
        
        for axis_id in range(4):
            # Останавливаем предыдущий поток оси, если был

            if axis_id in self.selected_axes or axis_id == axis or any(
                axis_id in (master, slave) 
                for master, slave in self.axis_pairs.values()
            ):
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

        # Обновляем значения
        axis_data["pos_label"].setText(f"Позиция: {pos:.4f}")
        axis_data["is_moving_label"].setText(f"Движение: {'Да' if moving else 'Нет'}")
        axis_data["is_in_pos_label"].setText(f"На месте: {'Да' if in_pos else 'Нет'}")

        # Меняем цвет индикаторов
        moving_color = "rgb(0, 128, 0)" if moving else "rgb(255, 0, 0)"  # Красный/Зелёный
        in_pos_color = "rgb(0, 128, 0)" if in_pos else "rgb(255, 0, 0)"
        
        axis_data["is_moving_indicator"].setStyleSheet(f"background-color: {moving_color}")
        axis_data["is_in_pos_indicator"].setStyleSheet(f"background-color: {in_pos_color}")

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

if __name__ == '__main__':
    app = QApplication([])
    window = ACSControllerGUI()
    window.show()
    app.exec()
    # window.axisstate()
    # print(ACSControllerGUI.__dict__) # Shows all attributes the object have


# TODO В первой итерации из произвольной точки, выставление оси нужно будет добавить потом