## Snake Locomotion Implementation
**Result Video** https://youtu.be/Vwp6FH1EtQc

21/03/18 3pm

승수("editCaps", "editCaps2") 
* 'snake-v0' -> 'Snake-v0'
* 'snakeEnv' -> 'SnakeEnv' 수정
* 중간에 추가되어버린 디버그 string들 정리("editCaps", "editCaps2")
* 'gym_test', 'ppo2_test' 정상 작동 확인 (계산머신에서도 돌려봐야할듯)

21/03/18 5pm

승수("editSnake")

- snake.py의 render()에서 p.connect 및 load 중복 삭제("editSnake")

21/03/21 1pm

혜원("set state and action")

- snake.py의 init함수와 step함수 수정

21/03/27 1pm

혜원("change state and edit reset(),init()")

- snake.py의 init, reset함수 수정. 승수가 톡방에 올린 그림 참고.

21/03/27 7pm

혜원("edit step()")

- snake.py의 step 함수 수정. render() 수정 필요.

21/03/28 7pm

진("model training test")

- ppo2_test.py 수정 

(1) loading a model without environment error 

 model save && load 주석 처리(학습 잘 된 모델 아직 없어서 저장 할 필요 x)

(2) ppo2 학습 횟수에 따른 test reward 확인

1000번 학습했을 때 test reward가 368.0으로 가장 높다, 5000번은 296으로 overfitting 가능성 -> 하지만 쓰레쉬홀드를 임의로 설정했으므로 정확히 구현하고 다시 돌려볼 필요o

21/03/31 1pm

혜원("hyewon" 브랜치 생성, "change threshold and reward")

- __init__ 함수의 action space와 observation space 의 threshold 수정.
- reset함수에 resetSimulation 함수 추가함에 따라 world 초기화와 snakeId 생성하는 코드의 위치 변경
- makeSnakeBody()함수에서 cylinderHeight 0.2로 수정
- step()함수에서 머리에서 꼬리까지의 축에 대해 더 많이 앞으로 나아갈 때 리워드를 받도록 수정.
