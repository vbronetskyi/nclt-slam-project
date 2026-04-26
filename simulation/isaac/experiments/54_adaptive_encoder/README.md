# exp 54 - real-speed cruise + adaptive SLAM/encoder blend

тестив динамічний blend між SLAM і encoder pose на основі того як вони
розходяться - якщо відстань між двома > 0.5 м то більша вага encoder, а
якщо все ок то ведучий SLAM. Хотілось щоб система не телепортувалась
коли SLAM губить tracking, і не drift'ила коли encoder пробуксовує

цей run залишився незавершеним - переключились на v55 з visual landmark
matcher що дав надійніше рішення тієї ж проблеми

