# rokey 6주차 주행 2

---
### 최종
- 라이다를 통해 맵을 한바퀴 돌고 odom을 통해 시작 위치와 현 위치를 비교
- 비슷한 위치라면 map 파일 저장

- pnp를 통한 이미지 좌표 가져옴 (실패...) (subin_code.py) 
- 지원님이 A-star 알고리즘 사용해서 위치 찾는 알고리즘 개발 (실패...) (ji_code.py)

- 발표 ppt는 canva의 '재난 구조 로봇' 참조

- start.txt는 로봇을 구동할때 사용한 명령어 모음

### slam을 통해 mapping을 진행할때 터널 내부에서 맵이 틀어지는 현상을 고치는 방법(slam.yaml)
1. loop_search_maximum_distance를 줄인다.(default value: 3.0)
   
   -> 해당 파라미터는 로봇이 loop close를 진행할 범위를 지정하는 것
        값을 줄일 수록 좁은 범위의 값을 참고하여 맵을 보정함.
        
2. do_loop_closing을 false로 한다.(default value: True)

   -> 해당 파라미터는 loop_closing을 진행 여부를 표시하는 것임.
