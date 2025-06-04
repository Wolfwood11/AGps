#ifndef MathUtils_H
#define MathUtils_H

#include <Structs.h>
#include <TinyGPSPlus.h>
#include <cmath>

class MathUtils {
public:
    // Константы для логики таймера
    static constexpr float trackLengthMeters = 1200.0; // Примерная длина трассы
    static constexpr float maxSpeedKmh = 300.0; // Максимальная ожидаемая скорость
    static constexpr unsigned long minLapTimeMs = (unsigned long)((trackLengthMeters / (maxSpeedKmh / 3.6)) * 1000.0); // Минимальное разумное время круга
    static constexpr unsigned long gpsTimeoutMs = 5000; // Таймаут отсутствия GPS фикса в мс

    // Референсные координаты для проекции Меркатора (центр вашей трассы)
    static constexpr double lat0 = 52.4449; // Пример: широта
    static constexpr double lon0 = 20.6400; // Пример: долгота

    static unsigned long lastCrossMillis; // Для дебаунсинга пересечения линии (на основе millis)

    static void latLonToMeters(double lat, double lon, double& x, double& y)
    {
        const double R = 6378137.0; // Радиус Земли в метрах
        x = R * (lon - lon0) * M_PI / 180.0 * cos(lat0 * M_PI / 180.0);
        y = R * (lat - lat0) * M_PI / 180.0;
    }

    static double calculateBearing(double x1, double y1, double x2, double y2)
    {
        double bearing = atan2(x2 - x1, y2 - y1) * 180.0 / M_PI; // В atan2 сначала Y потом X для навигационного азимута
                                                                 // Здесь X2-X1, Y2-Y1 (dx, dy). Если нужно от севера по часовой: atan2(dx, dy)
                                                                 // atan2(y2-y1, x2-x1) -> угол от оси X против часовой
                                                                 // Для навигационного (от севера по часовой): atan2(x2-x1, y2-y1)
        bearing = 90.0 - bearing; // Конвертация в навигационный азимут (0 - север, 90 - восток)
        if (bearing < 0)
            bearing += 360.0;
        return bearing;
    }

    // Расчет разницы в мс между двумя метками времени NMEA (для интервалов между фиксами GPS)
    // Упрощенная версия, корректно обрабатывает один переход через полночь.
    static long calculate_nmea_fix_interval_ms(const FullNmeaTime& t_current, const FullNmeaTime& t_previous)
    {
        if (!t_current.isValid || !t_previous.isValid)
            return 100; // Стандартный интервал для 10Гц

        long prev_ms_of_day = t_previous.getMsOfDay();
        long current_ms_of_day = t_current.getMsOfDay();

        // Проверка на тот же день или переход через одну полночь
        if (t_current.year == t_previous.year && t_current.month == t_previous.month) {
            if (t_current.day == t_previous.day) {
                return current_ms_of_day - prev_ms_of_day;
            } else if (t_current.day == t_previous.day + 1) { // Переход на следующий день
                return (24 * 3600000L - prev_ms_of_day) + current_ms_of_day;
            }
        }
        // Если даты сильно расходятся или предыдущий день "позже" текущего (ошибка данных),
        // возвращаем стандартный интервал или разницу в тот же день (может быть отрицательной).
        // Для последовательных фиксов это маловероятно.
        long diff = current_ms_of_day - prev_ms_of_day;
        if (abs(diff) > 12 * 3600000L)
            return 100; // Если разница > 12 часов, вероятно ошибка, вернуть 100мс
        return diff; // Может быть отрицательным, если время "прыгнуло"
    }

    // Расчет длительности круга в мс.
    // ВНИМАНИЕ: Эта функция УПРОЩЕНА. Для полной корректности при пересечении сложных границ дат
    // (несколько дней, конец месяца/года) нужна полноценная арифметика дат или библиотека.
    // Эта версия корректна для кругов в пределах одного дня или с одним пересечением полуночи.
    static unsigned long calculate_lap_duration_ms(const FullNmeaTime& t_end, const FullNmeaTime& t_start)
    {
        if (!t_end.isValid || !t_start.isValid)
            return 0;

        // Простой случай: тот же день, месяц и год
        if (t_end.year == t_start.year && t_end.month == t_start.month && t_end.day == t_start.day) {
            long diff = t_end.getMsOfDay() - t_start.getMsOfDay();
            return (diff >= 0) ? (unsigned long)diff : 0;
        }

        // Случай: пересечение одной полуночи (t_end на следующий день)
        // Это очень упрощенная проверка! Не учитывает конец месяца/года.
        if (t_end.year == t_start.year && t_end.month == t_start.month && t_end.day == t_start.day + 1) {
            long diff = (24 * 3600000L - t_start.getMsOfDay()) + t_end.getMsOfDay();
            return (unsigned long)diff;
        }

        // Если даты более сложные, эта функция вернет 0 или неверный результат.
        // Требуется более надежный метод (например, конвертация в секунды с эпохи).
        // Serial.println("Warning: Complex date difference in calculate_lap_duration_ms not fully supported.");
        return 0;
    }

    // Добавление мс к времени NMEA (для интерполяции).
    // ВНИМАНИЕ: Упрощенная версия. Корректно для небольших ms_to_add (не меняющих дату).
    // Не обрабатывает изменение дня, месяца, года.
    static FullNmeaTime add_ms_to_nmea_time(FullNmeaTime base_time, long ms_to_add)
    {
        if (!base_time.isValid)
            return base_time;

        long current_total_ms_in_day = base_time.getMsOfDay();
        current_total_ms_in_day += ms_to_add;

        FullNmeaTime result_time = base_time; // Копируем исходную дату (Y,M,D)

        // Обработка перехода мс в пределах суток (упрощенно, без изменения даты)
        if (current_total_ms_in_day >= 24 * 3600000L) {
            current_total_ms_in_day -= 24 * 3600000L;
            // ДАТА НЕ ОБНОВЛЯЕТСЯ! Для интерполяции (<100мс) это обычно не меняет дату.
        } else if (current_total_ms_in_day < 0) {
            current_total_ms_in_day += 24 * 3600000L;
            // ДАТА НЕ ОБНОВЛЯЕТСЯ!
        }

        result_time.hour = current_total_ms_in_day / 3600000L;
        current_total_ms_in_day %= 3600000L;
        result_time.minute = current_total_ms_in_day / 60000L;
        current_total_ms_in_day %= 60000L;
        result_time.second = current_total_ms_in_day / 1000L;
        current_total_ms_in_day %= 1000L;
        result_time.hundredths = current_total_ms_in_day / 10L;

        return result_time;
    }

    // Проверка пересечения отрезков (геометрия)
    static bool segmentsIntersect(double p0_x, double p0_y, double p1_x, double p1_y,
        double p2_x, double p2_y, double p3_x, double p3_y,
        double& i_x, double& i_y, double& tRatio)
    {
        double s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;
        s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;
        s2_y = p3_y - p2_y;

        double s, t;
        double det = -s2_x * s1_y + s1_x * s2_y;

        // Если отрезки параллельны или коллинеарны (det близок к 0)
        if (fabs(det) < 1e-9)
            return false;

        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / det;
        t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / det;

        if (s >= -1e-9 && s <= 1.0 + 1e-9 && t >= -1e-9 && t <= 1.0 + 1e-9) { // с допуском
            i_x = p0_x + (t * s1_x);
            i_y = p0_y + (t * s1_y);
            tRatio = t; // t - это отношение для первого отрезка (траектории GPS)
            tRatio = fmax(0.0, fmin(1.0, tRatio)); // Ограничиваем tRatio [0,1]
            return true;
        }
        return false; // Нет пересечения в пределах отрезков
    }

    // Проверка пересечения линии с интерполяцией
    static bool crossedLineInterpolated(double lat_prev, double lon_prev, double lat_curr, double lon_curr,
        TrapLine& trap, double& tRatioCalculated)
    {
        double x1_m, y1_m, x2_m, y2_m;
        latLonToMeters(lat_prev, lon_prev, x1_m, y1_m);
        latLonToMeters(lat_curr, lon_curr, x2_m, y2_m);

        double ix, iy; // Координаты точки пересечения (не используются дальше, но нужны функции)

        if (!segmentsIntersect(x1_m, y1_m, x2_m, y2_m, trap.x1, trap.y1, trap.x2, trap.y2, ix, iy, tRatioCalculated)) {
            return false;
        }

        // Проверка направления пересечения
        double gps_segment_bearing = calculateBearing(x1_m, y1_m, x2_m, y2_m);
        double bearing_diff = fabs(trap.bearing - gps_segment_bearing);
        if (bearing_diff > 180.0)
            bearing_diff = 360.0 - bearing_diff;

        if (bearing_diff > 90.0)
            return false; // Пересечение не в ту сторону

        // Проверка на пересечение выполнена и направление корректно
        return true;
    }
};
#endif
