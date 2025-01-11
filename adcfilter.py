# Write your code here :-)
from collections import deque


class ADCFilter:
    def __init__(self, window_size):
        """
        Инициализация фильтра.
        :param window_size: Количество точек для сглаживания.
        """
        self.window_size = window_size
        self.values = deque(maxlen=window_size)  # Очередь фиксированной длины

    def filter(self, new_value):
        """
        Фильтрация нового значения и возврат сглаженного значения.
        :param new_value: Новое значение отсчета от АЦП.
        :return: Сглаженное значение.
        """
        self.values.append(new_value)  # Добавляем новое значение в очередь
        smoothed_value = sum(self.values) / len(self.values)  # Вычисляем среднее
        return smoothed_value

# Пример использования
adc_filter = ADCFilter(window_size=3)
# Пример значений от АЦП
adc_data = [10, 12, 14, 13, 15, 20, 18, 17, 16, 15]

for value in adc_data:
    smoothed_value = adc_filter.filter(value)
    print(f"Новое значение: {value}, Сглаженное значение: {smoothed_value:.2f}")
