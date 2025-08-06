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