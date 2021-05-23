1. new
: reward 1번만 world좌표 계산으로 사용
: weight는 1. (상관 없음)
: action은 steering (-200 ~ 200) 여기에 0.01곱해서 사용
: self.snake.moveSin(steering = action[0],#action[0],
                            forces = self.forces, dt = 1./30.,
                            wavePeriod = 1.5, waveLength = 4, waveAmplitude = 0.8)
: force는 400
: 길이 20