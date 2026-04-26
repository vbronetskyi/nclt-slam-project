# exp 30: teach-and-repeat (перший прототип)

ідея проста - спочатку проїхати маршрут і запамʼятати його (teach phase),
потім повторити цей же маршрут (repeat) без повноцінної глобальної SLAM.
Локалізація на repeat: wheel encoders + IMU gyro + depth-нав + ORB anchor
matching кожні 2 м

## результат

на forest south-route (196 m round-trip):
- iteration 1, GT controller baseline - 196 m, cross-track 0.68 m, 0 зіткнень
- iteration 2, GT+odom_s+stuck recovery - 191 m, 2 stuck events
- iteration 3, чисто visual matching - 76 m і cascade failure (низький confidence -> неправильна якорна позиція -> drift)
- iteration 4, cmd_vel + IMU без GT - 24 m, 37% slippage error на skid-steer ламає геть усе
- iteration 5, encoder + IMU - 21 m, encoder drift +-2 m але anchor correction overcounts turning distance

## проблеми

ORB anchor matching на same-session дає 0.97 confidence, а на cross-session
лише 0.30 - viewpoint divergence 1-2 m між teach і repeat достатня щоб
зламати feature matching. На skid-steer cmd_vel і encoder обидва ненадійні
(slippage), і єдине що рятує це GT - але це не справжня локалізація
