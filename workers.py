from PyQt6.QtCore import QThread, pyqtSignal
import acsc_modified as acsc
import time
import numpy as np
from scipy.signal import savgol_filter


class SingleAxisWorker(QThread):
    """Поток для опроса одной оси с максимальной частотой"""
    update_signal = pyqtSignal(int, float, bool, bool)  # axis_id, position, moving, in_position
    error_signal = pyqtSignal(int, str)  # axis_id, error_message
    progress_signal = pyqtSignal(str)  # To send informational messages (like dual_print)

    def __init__(self, stand, axis_id):
        super().__init__()
        self.stand = stand      # Ссылка на контроллер ACS
        self.axis_id = axis_id  # ID оси (0, 1, 2, 3)
        self.running = False    # Флаг работы потока

    def run(self):
        """Основной цикл потока
        Код внутри этого метода выполняется в отдельном потоке, когда вызывается worker.start()
        """
        self.running = True
        while self.running:
            try:
                # Получаем данные оси
                pos = acsc.getFPosition(self.stand.hc, self.axis_id)
                axis_state = acsc.getAxisState(self.stand.hc, self.axis_id)
                mot_state = acsc.getMotorState(self.stand.hc, self.axis_id)
                
                # Отправляем в главный поток информацию об оси в текущей итерации
                self.update_signal.emit(
                    self.axis_id,
                    pos,
                    axis_state['moving'],
                    mot_state['in position']
                )
            except Exception as e:
                self.error_signal.emit(self.axis_id, str(e))
            
            self.msleep(100)  # Пауза 10 мс (можно уменьшить для более частого опроса)
            #! Здесь определяется частота обновления позиций

    def stop(self):
        """Корректная остановка потока"""
        self.running = False
        self.wait(500)  # Ожидаем завершения (таймаут 500 мс)


class FFIMeasurementWorker(QThread):
    log_ready = pyqtSignal(dict)
    error = pyqtSignal(str)
    pos_new = pyqtSignal(float)
    progress_signal = pyqtSignal(str)  # To send informational messages (like dual_print)

    def __init__(self, stand, axes, keithley, distance, speed, mode):
        super().__init__()
        self.stand = stand
        self.ffi_axes = axes
        self.keithley = keithley
        self.distance = distance
        self.speed = speed
        self.mode = mode
        self.running = True

    def run(self):
        distances = [-(self.distance/2), -(self.distance/2)]
        try:
            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(self.ffi_axes), tuple(distances), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, self.ffi_axes[0], 20000)
        except Exception as e:
            self.progress_signal.emit(f"Ошибка при запуске синхронного движения: {e}")
            print(f"Ошибка при запуске синхронного движения: {e}")
        else:
            self.progress_signal.emit(f"Функция acsc.toPointM выполнена без ошибок, нить выведена на старт")
            print(f"Функция acsc.toPointM выполнена без ошибок, нить выведена на старт")
        time.sleep(0.2) #! Чтобы контроллер успел увидеть остановку оси???
        try:    
            distances = [self.distance, self.distance]
            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(self.ffi_axes), tuple(distances), acsc.SYNCHRONOUS)
            time.sleep(0.2)
            #*acsc.toPointM сама добавляет -1 в конец списка осей
        except Exception as e:
            self.progress_signal.emit(f"Ошибка при запуске основного синхронного движения: {e}")
            print(f"Ошибка при запуске основного синхронного движения: {e}")
        else:
            self.progress_signal.emit(f"Измерение FFI успешно запущено, идёт измерение...")
            print(f"Измерение FFI успешно запущено, идёт измерение...")
        try:
            master = self.ffi_axes[0]
            log = {
                'time': [],
                'x_pos': [],
                'y_pos': [],
                'eds': [],
            }
            start_time = time.time()
            while self.running:
                eds = self.keithley.get_voltage()
                pos = acsc.getFPosition(self.stand.hc, master)
                log['eds'].append(eds)
                log['time'].append(time.time() - start_time)
                if self.mode == 'X':
                    log['x_pos'].append(pos)
                    log['y_pos'].append(0.0)
                elif self.mode == 'Y':
                    log['y_pos'].append(pos)
                    log['x_pos'].append(0.0)

                mot_state = acsc.getMotorState(self.stand.hc, master)
                if mot_state['in position']:
                    break
                time.sleep(0.01)
            self.log_ready.emit(log)
        except Exception as e:
            self.error.emit(str(e))


class SFIMeasurementWorker(QThread):
    log_ready = pyqtSignal(dict)
    error = pyqtSignal(str)
    progress_signal = pyqtSignal(str)  # To send informational messages (like dual_print)

    def __init__(self, stand, axes, keithley, distance, speed, mode):
        super().__init__()
        self.stand = stand
        self.sfi_axes = axes
        self.keithley = keithley
        self.distance = distance
        self.speed = speed
        self.mode = mode
        self.running = True

    def run(self):
        distances = [-(self.distance/2), (self.distance/2)]
        try:
            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(self.sfi_axes), tuple(distances), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, self.sfi_axes[0], 20000)
        except Exception as e:
            self.progress_signal.emit(f"Ошибка при запуске синхронного движения: {e}")
            print(f"Ошибка при запуске синхронного движения: {e}")
        else:
            self.progress_signal.emit(f"Функция acsc.toPointM выполнена без ошибок, нить выведена на старт")
            print(f"Функция acsc.toPointM выполнена без ошибок, нить выведена на старт")
        time.sleep(0.2) #! Чтобы контроллер успел увидеть остановку оси???

        try:    
            distances = [self.distance, -self.distance]
            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(self.sfi_axes), tuple(distances), acsc.SYNCHRONOUS)
            #*acsc.toPointM сама добавляет -1 в конец списка осей
        except Exception as e:
            self.progress_signal.emit(f"Ошибка при запуске основного синхронного движения: {e}")
            print(f"Ошибка при запуске основного синхронного движения: {e}")
        else:
            self.progress_signal.emit(f"Измерение FFI успешно запущено, идёт измерение...")
            print(f"Измерение SFI успешно запущено, идёт измерение...")

        try:
            master = self.sfi_axes[0]
            slave = self.sfi_axes[1]
            log = {
                'time': [],
                'x_pos_0': [],
                'x_pos_1': [],
                'y_pos_0': [],
                'y_pos_1': [],
                'eds': [],
            }
            start_time = time.time()
            while self.running:
                eds = self.keithley.get_voltage()
                pos_m = acsc.getFPosition(self.stand.hc, master)
                pos_s = acsc.getFPosition(self.stand.hc, slave)
                log['time'].append(time.time() - start_time)
                log['eds'].append(eds)
                if self.mode == 'X':
                    log['x_pos_0'].append(pos_m)
                    log['x_pos_1'].append(pos_s)
                    log['y_pos_0'].append(0.0)
                    log['y_pos_1'].append(0.0)
                else:
                    log['y_pos_0'].append(pos_m)
                    log['y_pos_1'].append(pos_s)
                    log['x_pos_0'].append(0.0)
                    log['x_pos_1'].append(0.0)

                state = acsc.getMotorState(self.stand.hc, master)
                if state['in position']:
                    break
                time.sleep(0.01)
            self.log_ready.emit(log)
        except Exception as e:
            self.error.emit(str(e))


class CircularMotionWorker(QThread):
    progress_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    log_ready = pyqtSignal(dict)
    pos_new = pyqtSignal(float)

    def __init__(self, stand, keithley, speed, radius, rotation, N, angle):
        super().__init__()
        self.stand = stand
        self.keithley = keithley
        self.speed = speed
        self.radius = radius
        self.rotation = rotation
        self.N = N #!!! Количество оборотов. Учесть в задании сегментов
        self.angle = angle
        self.running = True
        self.masters = [0, 1]
        self.all_axes = [0, 1, 2, 3]

    def run(self):
        print(f"N: {self.N}, тип: {type(self.N)}")
        #* ПРЕДПОЛАГАЕТСЯ ЧТО НИТЬ НАХОДИТСЯ В ЦЕНТРЕ, Т.Е. НА МАГНИТНОЙ ОСИ
        try:
            #! Создаём две пары мастер слейв. Далее управляем только осями 0 и 1. 2 и 3 отражают их движение"
            program_0 = f"""
            MFLAGS(2).#DEFCON = 0
            CONNECT RPOS(2) = APOS(0)
            DEPENDS 2, 0
            MFLAGS(3).#DEFCON = 0
            CONNECT RPOS(3) = APOS(1)
            DEPENDS 3, 1
            GROUP (0,1,2,3)
            """

            acsc.cleanBuffer(self.stand.hc, 0)
            acsc.loadBuffer(self.stand.hc, 0, program_0)
            acsc.compileBuffer(self.stand.hc, 0)
            acsc.runBuffer(self.stand.hc, 0)

        except Exception as e:
            self.progress_signal.emit(f"Ошибка при создании master-slave пар: {e}")
            print(f"Ошибка при создании master-slave пар: {e}")
        else:
            self.progress_signal.emit(f"master-slave пары успешно созданы")
            print(f"master-slave пары успешно созданы")

        try:
            acsc.toPoint(self.stand.hc, acsc.AMF_RELATIVE, 1, -self.radius, acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, 1, 20000)
        except Exception as e:
            self.progress_signal.emit(f"Ошибка при запуске выводе нити на старт (acsc.toPoint): {e}")
            print(f"Ошибка при запуске выводе нити на старт (toPoint): {e}")
        else:
            self.progress_signal.emit(f"Функция acsc.toPoint выполнена без ошибок, нить выведена на старт")
            print(f"Функция acsc.toPoint выполнена без ошибок, нить выведена на старт")


        try:
            program_1 = f"""
            MSEG (0,1),{0},{-self.radius} 
            """


            acsc.cleanBuffer(self.stand.hc, 1)
            acsc.loadBuffer(self.stand.hc, 1, program_1)

            program_2 = f"""
            ARC1 (0,1), {0},{0},{0},{self.radius},{self.rotation} ! Add arc segment with center(1,0), final point (1,-1, clockwise rotation.
            ARC1 (0,1), {0},{0},{0},{-self.radius},{self.rotation}
            """
            
            for _ in range(self.N - 1):
                acsc.appendBuffer(self.stand.hc, 1, program_2)

            program_3 = f"""
            ARC1 (0,1), {0},{0},{0},{self.radius},{self.rotation} ! Add arc segment with center(1,0), final point (1,-1, clockwise rotation.
            ARC1 (0,1), {0},{0},{0},{-self.radius},{self.rotation}
            ENDS (0,1)
            SPLITALL
            STOP
            """

            acsc.appendBuffer(self.stand.hc, 1, program_3)
            acsc.compileBuffer(self.stand.hc, 1)
            acsc.runBuffer(self.stand.hc, 1)

        except Exception as e:
            self.progress_signal.emit(f"Ошибка при запуске движения по окружности: {e}")
            print(f"шибка при запуске движения по окружности: {e}")
        else:
            self.progress_signal.emit(f"Гармонический анализ успешно запущен, идёт измерение...")
            print(f"Гармонический анализ успешно запущен, идёт измерение...")

        
        try:
            # master = self.ffi_axes[0]
            log = {
                'time': [],
                'x_pos': [],
                'y_pos': [],
                'eds': [],
            }
            start_time = time.time()
            while self.running:
                eds = self.keithley.get_voltage()
                x_pos = acsc.getFPosition(self.stand.hc, self.masters[1])
                y_pos = acsc.getFPosition(self.stand.hc, self.masters[0])
                log['eds'].append(eds)
                log['time'].append(time.time() - start_time)
                log['x_pos'].append(x_pos)
                log['y_pos'].append(y_pos)

                mot_state = acsc.getMotorState(self.stand.hc, self.masters[0])
                if mot_state['in position']:
                    break
                time.sleep(0.01)
            self.log_ready.emit(log)
        except Exception as e:
            self.error.emit(str(e))

        try:
            acsc.toPoint(self.stand.hc, acsc.AMF_RELATIVE, 1, self.radius, acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, 1, 20000)
        except Exception as e:
            self.progress_signal.emit(f"Ошибка при возвращении нити в центр (acsc.toPoint): {e}")
            print(f"Ошибка при возвращении нити в центр (toPoint): {e}")
        else:
            self.progress_signal.emit(f"Функция acsc.toPoint выполнена без ошибок, нить возвращена в центр")
            print(f"Функция acsc.toPoint выполнена без ошибок, нить возвращена в центр")


class FindMagneticAxisWorker_PREVIOUS(QThread):
    # Signals to communicate with the main GUI thread
    progress_signal = pyqtSignal(str)  # To send informational messages (like dual_print)
    error_signal = pyqtSignal(str)    # To send error messages (like show_error)
    finished_signal = pyqtSignal(dict) # To send the final axis positions upon completion
    # Optional: intermediate_results_signal = pyqtSignal(str, str, float) # scan_type, mode, coordinate

    def __init__(self, stand, keithley, distance, speed, convergence_threshold, max_iterations):
        super().__init__()
        self.stand = stand
        self.keithley = keithley
        self.distance = distance
        self.speed = speed
        self.convergence_threshold = convergence_threshold
        self.max_iterations = max_iterations
        self.running = True
        self.L_wire = 2.0 # Wire length for SFI, or pass as parameter

    def _perform_scan_and_center_worker(self, scan_type, mode, axes_pair, move_distance, current_speed):
        # This method is adapted from ACSControllerGUI._perform_scan_and_center
        master = axes_pair[0]
        slave = axes_pair[1] # Used for SFI pair, FFI effectively uses master for logging

        try:
            for axis_id in axes_pair:
                # Ensure axis is enabled - direct call to controller
                acsc.enable(self.stand.hc, axis_id) # Or self.stand.axes[axis_id].enable() if newACS wraps it
                # Set speed - direct call to controller
                acsc.setVelocity(self.stand.hc, axis_id, current_speed) # Or self.stand.axes[axis_id].set_speed()

            self.progress_signal.emit(f"Скорость {current_speed} мм/с установлена для осей {axes_pair}.")

            log_data_points = {'time': [], 'eds': []}
            if scan_type == "FFI":
                log_data_points['pos'] = [] # For master axis position
            elif scan_type == "SFI":
                log_data_points['pos_0'] = [] # Master axis
                log_data_points['pos_1'] = [] # Slave axis
            
            self.progress_signal.emit(f"Подготовка к сканированию {scan_type} по оси {mode}...")
            if scan_type == "FFI":
                initial_moves = [-(move_distance / 2.0), -(move_distance / 2.0)]
                scan_moves = [move_distance, move_distance]
            elif scan_type == "SFI":
                initial_moves = [-(move_distance / 2.0), (move_distance / 2.0)]
                scan_moves = [move_distance, -move_distance]
            else:
                self.error_signal.emit(f"Неизвестный тип сканирования: {scan_type}")
                return None

            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(axes_pair), tuple(initial_moves), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, master, 30000) 
            time.sleep(0.2)
            self.progress_signal.emit(f"Перемещение на начальную точку сканирования {scan_type} {mode} завершено.")

            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(axes_pair), tuple(scan_moves), acsc.SYNCHRONOUS)
            self.progress_signal.emit(f"Начало сканирования {scan_type} {mode} ({move_distance} мм)...")

            scan_start_time = time.time()
            poll_interval = 0.1 
            max_log_duration = (move_distance / current_speed) * 1.5 + 10 # Increased buffer
            log_end_time = time.time() + max_log_duration

            while time.time() < log_end_time and self.running:
                pos_m = acsc.getFPosition(self.stand.hc, master)
                eds_v = self.keithley.get_voltage() # Assumes keithley object has get_voltage()
                current_t_rel = time.time() - scan_start_time

                log_data_points['time'].append(current_t_rel)
                log_data_points['eds'].append(eds_v)
                if scan_type == "FFI":
                    log_data_points['pos'].append(pos_m)
                elif scan_type == "SFI":
                    log_data_points['pos_0'].append(pos_m)
                    pos_s = acsc.getFPosition(self.stand.hc, slave)
                    log_data_points['pos_1'].append(pos_s)

                mot_state = acsc.getMotorState(self.stand.hc, master)
                if mot_state['in position']:
                    self.progress_signal.emit(f"Сканирование {scan_type} {mode}: Движение завершено, сбор данных остановлен.")
                    break
                time.sleep(poll_interval)
            else: 
                if not self.running:
                    self.progress_signal.emit(f"Сканирование {scan_type} {mode} прервано.")
                    acsc.killAll(self.stand.hc, acsc.SYNCHRONOUS)
                    return None
                self.progress_signal.emit(f"Сканирование {scan_type} {mode}: Превышено время ожидания сбора данных.")
                acsc.killAll(self.stand.hc, acsc.SYNCHRONOUS) # Ensure motion is stopped

            if not log_data_points['time'] or not log_data_points['eds']:
                self.progress_signal.emit(f"Нет данных для обработки {scan_type} {mode}.")
                return None

            min_coord = None
            # Get initial positions for calculating absolute target
            # These are absolute positions *before* this scan's centering move
            initial_pos_master_abs = acsc.getFPosition(self.stand.hc, master)
            initial_pos_slave_abs = acsc.getFPosition(self.stand.hc, slave)

            # Store initial absolute positions from before the scan's relative moves.
            # The `toPointM` for initial_moves was relative.
            # To get the absolute coordinate of the scan start:
            # Get current position, then subtract the scan_moves[0]/2 (or similar logic based on how you define 0)
            # Simpler: Use the recorded positions. The recorded 'pos' or 'pos_0' are already absolute.
            
            scan_path_abs_positions = np.array(log_data_points.get('pos', log_data_points.get('pos_0', [])))
            if len(scan_path_abs_positions) == 0:
                 self.error_signal.emit(f"Нет данных о позиции для {scan_type} {mode}.")
                 return None

            if scan_type == "FFI":
                integral_values = np.array(log_data_points['eds']) / current_speed
                min_id = np.argmin(np.abs(integral_values))
                min_coord_abs = scan_path_abs_positions[min_id]
            elif scan_type == "SFI":
                integral_values = (np.array(log_data_points['eds']) * self.L_wire) / (2.0 * current_speed)
                min_id = np.argmin(np.abs(integral_values))
                min_coord_abs = scan_path_abs_positions[min_id] # SFI minimum refers to master axis's absolute position

            self.progress_signal.emit(f"{scan_type} {mode}: Мин. значение интеграла ({integral_values[min_id]:.4e}) на абсолютной коорд. {min_coord_abs:.4f}")
            
            self.progress_signal.emit(f"Центрирование осей {axes_pair} на новой абсолютной координате {min_coord_abs:.4f}...")
            
            # Calculate relative moves to reach the absolute min_coord_abs from current positions
            current_pos_master_ax_abs = acsc.getFPosition(self.stand.hc, master)
            current_pos_slave_ax_abs = acsc.getFPosition(self.stand.hc, slave)

            move_master_rel = min_coord_abs - current_pos_master_ax_abs
            move_slave_rel = min_coord_abs - current_pos_slave_ax_abs # Both axes go to the same absolute coordinate

            centering_distances = [move_master_rel, move_slave_rel]
            # If axes_pair contains X axes (e.g., [1,3]), centering_distances will be [dx1, dx3]
            # If axes_pair contains Y axes (e.g., [0,2]), centering_distances will be [dy0, dy2]

            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, tuple(axes_pair), tuple(centering_distances), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, master, 30000)
            time.sleep(0.2)

            final_pos_master_abs = acsc.getFPosition(self.stand.hc, master)
            final_pos_slave_abs = acsc.getFPosition(self.stand.hc, slave)
            self.progress_signal.emit(f"{scan_type} {mode}: Оси перемещены. Итоговые позиции: {master}={final_pos_master_abs:.4f}, {slave}={final_pos_slave_abs:.4f} (цель была {min_coord_abs:.4f})")
            
            return min_coord_abs # Return the absolute coordinate found

        except Exception as e:
            self.error_signal.emit(f"Ошибка в _perform_scan_and_center_worker ({scan_type} {mode}): {str(e)}")
            # import traceback
            # self.progress_signal.emit(traceback.format_exc()) # For detailed debugging
            return None

    def run(self):
        self.progress_signal.emit(f"Запуск поиска магнитной оси: Дистанция={self.distance} мм, Скорость={self.speed} мм/с")

        current_iteration = 0
        
        # Log initial positions from the worker's perspective
        pos_data_initial = {}
        for i in range(4): # Assuming 4 axes
            pos_data_initial[f"axis_{i}_initial_pos"] = acsc.getFPosition(self.stand.hc, i)
        self.progress_signal.emit(f"Начальные позиции (0,1,2,3): ({pos_data_initial['axis_0_initial_pos']:.4f}, {pos_data_initial['axis_1_initial_pos']:.4f}, {pos_data_initial['axis_2_initial_pos']:.4f}, {pos_data_initial['axis_3_initial_pos']:.4f})")

        last_positions = {i: acsc.getFPosition(self.stand.hc, i) for i in range(4)}

        while current_iteration < self.max_iterations and self.running:
            self.progress_signal.emit(f"\n--- Итерация {current_iteration + 1} ---")
            
            iter_start_positions = {i: acsc.getFPosition(self.stand.hc, i) for i in range(4)}
            self.progress_signal.emit(
                f"Позиции в начале итерации {current_iteration + 1} (0,1,2,3): "
                f"({iter_start_positions[0]:.4f}, {iter_start_positions[1]:.4f}, "
                f"{iter_start_positions[2]:.4f}, {iter_start_positions[3]:.4f})"
            )

            # 1. FFI по X (axes 1, 3)
            if not self.running: break
            self.progress_signal.emit("Шаг 1: FFI по X...")
            new_x_center = self._perform_scan_and_center_worker('FFI', 'X', [1, 3], self.distance, self.speed)
            if new_x_center is None: self.progress_signal.emit("Ошибка в FFI X. Остановка."); break
            self.progress_signal.emit(f"FFI X: Новый целевой центр X = {new_x_center:.4f}")

            # 2. FFI по Y (axes 0, 2)
            if not self.running: break
            self.progress_signal.emit("Шаг 2: FFI по Y...")
            new_y_center = self._perform_scan_and_center_worker('FFI', 'Y', [0, 2], self.distance, self.speed)
            if new_y_center is None: self.progress_signal.emit("Ошибка в FFI Y. Остановка."); break
            self.progress_signal.emit(f"FFI Y: Новый целевой центр Y = {new_y_center:.4f}")

            # 3. SFI по X (axes 1, 3)
            if not self.running: break
            self.progress_signal.emit("Шаг 3: SFI по X...")
            new_x_center = self._perform_scan_and_center_worker('SFI', 'X', [1, 3], self.distance, self.speed)
            if new_x_center is None: self.progress_signal.emit("Ошибка в SFI X. Остановка."); break
            self.progress_signal.emit(f"SFI X: Новый целевой центр X = {new_x_center:.4f}")

            # 4. SFI по Y (axes 0, 2)
            if not self.running: break
            self.progress_signal.emit("Шаг 4: SFI по Y...")
            new_y_center = self._perform_scan_and_center_worker('SFI', 'Y', [0, 2], self.distance, self.speed)
            if new_y_center is None: self.progress_signal.emit("Ошибка в SFI Y. Остановка."); break
            self.progress_signal.emit(f"SFI Y: Новый целевой центр Y = {new_y_center:.4f}")

            current_positions = {i: acsc.getFPosition(self.stand.hc, i) for i in range(4)}
            self.progress_signal.emit(
                f"Позиции после итерации {current_iteration + 1} (0,1,2,3): "
                f"({current_positions[0]:.4f}, {current_positions[1]:.4f}, "
                f"{current_positions[2]:.4f}, {current_positions[3]:.4f})"
            )

            deltas = {i: abs(current_positions[i] - iter_start_positions[i]) for i in range(4)} # Мб убрать abs
            self.progress_signal.emit(f"Изменения за итерацию (Δ0,Δ1,Δ2,Δ3): "
                                      f"({deltas[0]:.4f}, {deltas[1]:.4f}, {deltas[2]:.4f}, {deltas[3]:.4f})")

            converged = all(deltas[i] < self.convergence_threshold for i in range(4))
            
            if converged:
                self.progress_signal.emit(f"Схождение достигнуто на итерации {current_iteration + 1}. Магнитная ось найдена.")
                break
            
            last_positions = current_positions
            current_iteration += 1
        
        if not self.running:
             self.progress_signal.emit("Поиск магнитной оси прерван пользователем.")
        elif current_iteration == self.max_iterations and not converged:
            self.progress_signal.emit(f"Достигнуто максимальное количество итераций ({self.max_iterations}) без схождения.")

        final_positions = {f"axis_{i}": acsc.getFPosition(self.stand.hc, i) for i in range(4)}
        self.progress_signal.emit(f"Финальные координаты концов нити (0,1,2,3):")
        self.progress_signal.emit(f"  Ось 0 (Y1): {final_positions['axis_0']:.4f} мм")
        self.progress_signal.emit(f"  Ось 1 (X1): {final_positions['axis_1']:.4f} мм")
        self.progress_signal.emit(f"  Ось 2 (Y2): {final_positions['axis_2']:.4f} мм")
        self.progress_signal.emit(f"  Ось 3 (X2): {final_positions['axis_3']:.4f} мм")
        
        self.finished_signal.emit(final_positions)

    def stop(self):
        self.running = False
        self.progress_signal.emit("Получен сигнал остановки...")
        # Optionally, if scans involve blocking calls that don't check self.running,
        # you might need to use acsc.killAll here if immediate stop is critical.
        # However, _perform_scan_and_center_worker already has a self.running check in its loop.



class FindMagneticAxisWorker(QThread):
    # Сигналы для связи с GUI
    progress_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)
    finished_signal = pyqtSignal(dict)

    def __init__(self, stand, keithley, scan_distance, scan_speed,
                 x_center, x_width, num_points_x,
                 y_center, y_width, num_points_y):
        super().__init__()
        self.stand = stand
        self.keithley = keithley
        self.scan_distance = scan_distance
        self.scan_speed = scan_speed
        
        self.x_coords = np.linspace(x_center - x_width / 2, x_center + x_width / 2, num_points_x)
        self.y_coords = np.linspace(y_center - y_width / 2, y_center + y_width / 2, num_points_y)
        
        self.running = True
        self.L_wire = 2.0

    def _perform_line_scan_and_find_center(self, scan_type, mode):
        """
        Выполняет одно длинное сканирование по линии, сглаживает данные
        и возвращает точную координату центра (пересечения нуля).
        """
        if mode == 'X':
            axes_pair = [1, 3]
            scan_coords = self.x_coords
        elif mode == 'Y':
            axes_pair = [0, 2]
            scan_coords = self.y_coords
        else:
            self.error_signal.emit(f"Неверный режим сканирования: {mode}")
            return None
        
        master = axes_pair[0]
        start_pos, end_pos = scan_coords[0], scan_coords[-1]
        scan_length = abs(end_pos - start_pos)
        
        try:
            log_data = {'pos': [], 'eds': []}
            
            acsc.toPointM(self.stand.hc, 0, axes_pair, (start_pos, start_pos), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, master, 30000)

            move_rel = end_pos - start_pos
            acsc.toPointM(self.stand.hc, acsc.AMF_RELATIVE, axes_pair, (move_rel, move_rel), acsc.SYNCHRONOUS)
            
            max_log_duration = (scan_length / self.scan_speed) * 1.5 + 10
            log_end_time = time.time() + max_log_duration
            while time.time() < log_end_time and self.running:
                pos_m = acsc.getFPosition(self.stand.hc, master)
                eds_v = self.keithley.get_voltage()
                log_data['pos'].append(pos_m)
                log_data['eds'].append(eds_v)
                
                mot_state = acsc.getMotorState(self.stand.hc, master)
                if mot_state['in position']: break
                time.sleep(0.05)

            if not log_data['pos'] or not log_data['eds']: return None

            pos = np.array(log_data['pos'])
            eds = np.array(log_data['eds'])
            
            if scan_type == "FFI":
                integral_values = eds / self.scan_speed
            else: # SFI
                integral_values = (eds * self.L_wire) / (2.0 * self.scan_speed)
            
            # --- КЛЮЧЕВОЙ ШАГ: ФИЛЬТРАЦИЯ ---
            # Параметры (window_length, polyorder) нужно подобрать под ваш уровень шума
            # window_length должен быть нечетным
            if len(integral_values) > 11:
                smoothed_integrals = savgol_filter(integral_values, window_length=11, polyorder=2)
            else:
                smoothed_integrals = integral_values

            # --- ПОИСК НУЛЯ ПО ОТФИЛЬТРОВАННЫМ ДАННЫМ ---
            sign = np.sign(smoothed_integrals)
            zero_crossings_indices = np.where(np.diff(sign))[0]
            
            if len(zero_crossings_indices) > 0:
                idx1 = zero_crossings_indices[0]
                idx2 = idx1 + 1
                
                y1, y2 = smoothed_integrals[idx1], smoothed_integrals[idx2]
                x1, x2 = pos[idx1], pos[idx2]
                center_coord = x1 - y1 * (x2 - x1) / (y2 - y1)
            else:
                self.progress_signal.emit(f"Предупреждение: Пересечение нуля не найдено для {scan_type}-{mode}. Ищем минимум модуля.")
                min_abs_idx = np.argmin(np.abs(smoothed_integrals))
                center_coord = pos[min_abs_idx]
                
            return center_coord

        except Exception as e:
            self.error_signal.emit(f"Ошибка в _perform_line_scan... ({scan_type} {mode}): {str(e)}")
            return None

    def run(self):
        self.progress_signal.emit("Запуск эффективного поиска оси (сканирование линиями)...")
        
        # --- Этап 1: Сканирование по строкам (поиск X-центра) ---
        x_centers_ffi, x_centers_sfi = [], []
        self.progress_signal.emit("\n--- Этап 1: Сканирование по строкам (поиск X центра) ---")
        for y_pos in self.y_coords:
            if not self.running: return
            self.progress_signal.emit(f"Сканируем строку при Y = {y_pos:.3f}")
            
            acsc.toPointM(self.stand.hc, 0, (0, 2), (y_pos, y_pos), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, 0, 30000)
            
            x_ffi = self._perform_line_scan_and_find_center('FFI', 'X')
            if x_ffi is None: return
            x_centers_ffi.append(x_ffi)

            x_sfi = self._perform_line_scan_and_find_center('SFI', 'X')
            if x_sfi is None: return
            x_centers_sfi.append(x_sfi)

        # --- Этап 2: Сканирование по столбцам (поиск Y-центра) ---
        y_centers_ffi, y_centers_sfi = [], []
        self.progress_signal.emit("\n--- Этап 2: Сканирование по столбцам (поиск Y центра) ---")
        for x_pos in self.x_coords:
            if not self.running: return
            self.progress_signal.emit(f"Сканируем столбец при X = {x_pos:.3f}")
            
            acsc.toPointM(self.stand.hc, 0, (1, 3), (x_pos, x_pos), acsc.SYNCHRONOUS)
            acsc.waitMotionEnd(self.stand.hc, 1, 30000)

            y_ffi = self._perform_line_scan_and_find_center('FFI', 'Y')
            if y_ffi is None: return
            y_centers_ffi.append(y_ffi)

            y_sfi = self._perform_line_scan_and_find_center('SFI', 'Y')
            if y_sfi is None: return
            y_centers_sfi.append(y_sfi)
            
        # --- Этап 3: Анализ и нахождение точки пересечения ---
        self.progress_signal.emit("\n--- Этап 3: Анализ данных и поиск центра ---")
        try:
            m1, c1 = np.polyfit(self.y_coords, x_centers_ffi, 1)
            m2, c2 = np.polyfit(self.x_coords, y_centers_ffi, 1)
            
            if abs(1 - m1 * m2) < 1e-9:
                self.error_signal.emit("Линии FFI параллельны, невозможно найти пересечение.")
                optimal_x_ffi, optimal_y_ffi = np.mean(x_centers_ffi), np.mean(y_centers_ffi)
            else:
                optimal_x_ffi = (m1 * c2 + c1) / (1 - m1 * m2)
                optimal_y_ffi = m2 * optimal_x_ffi + c2
            
            self.progress_signal.emit(f"Центр по FFI: X={optimal_x_ffi:.4f}, Y={optimal_y_ffi:.4f}")

        except Exception as e:
            self.error_signal.emit(f"Ошибка при анализе и поиске пересечения: {e}")
            return
            
        # --- Этап 4: Финальное перемещение ---
        self.progress_signal.emit(f"\nПеремещение на найденную магнитную ось (по FFI): X={optimal_x_ffi:.4f}, Y={optimal_y_ffi:.4f}")
        acsc.toPointM(self.stand.hc, 0, (1, 3), (optimal_x_ffi, optimal_x_ffi), acsc.SYNCHRONOUS)
        acsc.toPointM(self.stand.hc, 0, (0, 2), (optimal_y_ffi, optimal_y_ffi), acsc.SYNCHRONOUS)
        acsc.waitMotionEnd(self.stand.hc, 0, 30000)
        acsc.waitMotionEnd(self.stand.hc, 1, 30000)
        
        final_positions = {f"axis_{i}": acsc.getFPosition(self.stand.hc, i) for i in range(4)}
        self.finished_signal.emit(final_positions)

    def stop(self):
        self.running = False
        self.progress_signal.emit("Получен сигнал остановки...")    