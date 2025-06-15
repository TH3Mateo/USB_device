# Projekt STM32 USB Device

## Opis projektu

Projekt systemu wbudowanego oparty na mikrokontrolerze **STM32F411**, zbudowany z użyciem **FreeRTOS (CMSIS-RTOS2)**. System integruje kilka komponentów i podsystemów do pracy w czasie rzeczywistym, obejmujących pomiar, sterowanie i komunikację:

- Komunikacja USB CDC (serial over USB)
- Wyświetlacz LCD do prezentacji statusu i danych
- Pomiar temperatury oraz regulacja PID z użyciem ADC i PWM
- Pomiar odległości za pomocą czujnika **VL53L1X** (Time-of-Flight)

Każda główna funkcja została zaimplementowana jako osobny task FreeRTOS, co zapewnia modularność i wysoką responsywność systemu.

---

## Wykorzystane komponenty

- **Mikrokontroler**: STM32F411 (ARM Cortex-M4)
- **RTOS**: FreeRTOS (poprzez CMSIS-RTOS2)
- **USB CDC**: Komunikacja szeregowa z komputerem
- **LCD**: Interfejs tekstowy użytkownika
- **ADC i PWM**: Pomiar temperatury i sterowanie grzałką
- **VL53L1X**: Czujnik Time-of-Flight do pomiaru odległości

---

## Taski FreeRTOS

| Task               | Opis                                                                   |
|--------------------|------------------------------------------------------------------------|
| `COM_manager`      | Obsługuje komunikację USB CDC: odbiór i wysyłanie danych przez bufor   |
| `DISPLAY_manager`  | Inicjalizuje LCD i okresowo aktualizuje wyświetlane informacje         |
| `TEMP_manager`     | Cyclicznie odczytuje temperaturę i steruje wyjściem PWM na podstawie PID |
| `TOF_manager`      | Inicjalizuje czujnik VL53L1X i udostępnia bieżące pomiary odległości    |

---