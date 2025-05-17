# Код до SLAVE борди

Код написаний на Arduino фреймворку, за допомогою platformio у VScode.

## Структура

```
slaveboard_f401rc/
├── .pio/                   # PlatformIO робоча директорія, генерується автоматично
├── lib/                    # Бібліотеки
│   └── ism330bx/           # ліба для IMU. Підключена як гіт субмодуль (лежить на окремому репо)
├── src/                    # 
│   ├── main.cpp            # Основний код
└── platformio.ini          # Конфіг PlatformIO
```

### Вимоги
- Розшиерення PlatformIO в VSCode

## Налаштування platformio.ini

Якщо ви збираєтеся програмувати не через st-link, а через dfu - замініть значення в upload_protocol на dfu

```ini
[env:genericSTM32F401RC]
platform = ststm32
board = genericSTM32F401RC
upload_protocol = stlink <--- ТУТ
...

```

## Встановлення та запуск

1. Клонуйте репо з флажком --recursive (щоб також стнути лібу для imu) :
   ```bash
   git clone --recursive https://github.com/ugv-rws/rws-slaveboard.git
   ```

2. Відкрийте репо як папку у воркспейсі vscode (щоб platformio його побачив)

3. Завантажуйте код через upload/upload and monitor в меню platfomio

## Ліба до imu ISM330BX

Бібліотека ISM330BX інтегрована як підмодуль в `./lib/ISM330BX`:
Альтернатив поки для цього imu нема, бо він вийшов хіба недавно. Тому прийшлося нашкрябати. Там тільки необхідний функціонал.

Ось посилання на його даташит: https://www.st.com/resource/en/data_brief/steval-mki245ka.pdf
