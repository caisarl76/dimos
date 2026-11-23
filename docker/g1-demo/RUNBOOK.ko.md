---
source: RUNBOOK.md
source_hash: a1cfffba2689747679c42f993785efbb55285bdfda0777f26102ee97090c4ff3
lang: ko
---

# G1 데모 런북 (docker compose)

모든 작업은 데모 워크스테이션의 이 디렉터리에서 실행한다. 1~3단계는 점검이고 4단계부터가 데모다. 로봇은 `demo.sh up`이 여는 tmux pane에서 구동한다. 각 pane에는 명령이 입력만 되어 있고 실행되지는 않은 상태이므로, 적힌 순서대로 Enter를 누른다. dimos 체크아웃은 필요 없다.
번들은 `compose.yaml`, `compose.demo.yaml`, `demo.sh`, `.env`, `volo/deploy/g1-loop/compose.yaml`
과 로드된 이미지 두 개(`dimos-g1-demo`, `volo-g1-workstation:g1-loop-final-20261028`)로 구성된다.

## 0. 새 워크스테이션, 한 번만

compose 파일 옆에 `.env`를 둔다(compose와 `demo.sh`가 모두 읽는다).

```
VOLO_DIR=./volo
PC2_SSH_HOST=unitree@192.168.0.223
PC2_222_SSH_HOST=unitree@192.168.0.222
SONIC_ENCODER=<path>/models--nvidia--GEAR-SONIC/snapshots/9c0ff22b4ffec27c5392e8e284eb2f2df7a5b4e2/sonic_v1_1/model_encoder.onnx
GEAR_SONIC_G1_DIR=<path>/g1
WORKSTATION_IP=<this machine's address on the robot network>
SSH_DIR=<path>/ssh
```

`WORKSTATION_IP`는 222의 loop profile(`policy_host`)과 loop의 VLM URL에 들어간다. 222의 harness는 이 주소로 policy 서버와 VLM 서버를 찾는다. `demo.sh`는 이 값이 없으면 시작을 거부한다(기본값 없음). `SSH_DIR`는 `id -u`가 1000이 아닐 때 필요하다. 컨테이너는 uid 1000으로 실행되어 다른 사용자의 `~/.ssh`를 읽지 못하며 이 문제는 tunnel이 "Host key verification failed"와 함께 재시작되는 증상으로 나타난다. 사본은 한 번만 만든다.

```
sudo install -d -o 1000 -g 1000 -m 700 <path>/ssh
sudo install -o 1000 -g 1000 -m 600 ~/.ssh/id_rsa <path>/ssh/id_rsa      # the key the robots accept
ssh-keyscan 192.168.0.223 192.168.0.222 | sudo tee <path>/ssh/known_hosts >/dev/null
sudo chown 1000:1000 <path>/ssh/known_hosts
```

## 1. 초기화

```
./demo.sh down
```

## 2. 파일, ssh 사본, 이미지 확인

```
grep -c SSH_DIR compose.yaml                                   # 2
grep -E 'WORKSTATION_IP|SSH_DIR|SSH_HOST' .env
sudo ls -la "$(sed -n 's/^SSH_DIR=//p' .env)"                  # drwx------ 1000, id_rsa 600, known_hosts non-empty
docker images | grep -E 'dimos-g1-demo|volo-g1-workstation'
docker compose -f compose.demo.yaml --profile workstation config --services   # tunnel dimos policy vlm
```

## 3. 컨테이너가 쓰는 그대로의 tunnel ssh 점검

```
docker compose -f compose.demo.yaml run --rm --entrypoint ssh tunnel -o BatchMode=yes unitree@192.168.0.223 true && echo tunnel-ssh-ok
```

"Host key verification failed": 마운트한 `known_hosts`에 pc2가 없거나 디렉터리를 읽을 수 없는 경우다(0단계 참조). "Permission denied (publickey)": 사본의 키 파일이 잘못되었다.

## 4. GPU 서버와 pane

```
./demo.sh up
```

policy와 vlm을 시작하고 tunnel과 dimos가 멈춰 있는지 확인한 뒤 tmux 세션 `g1-demo`를 연다.
`ws` 창은 서버 로그를 따라간다. `ready on port 5571`이 나올 때까지 기다린다(디스크가 식은 상태에서 약 14분).
세션이 이미 있으면 `up`은 거부한다. 해당 세션에 attach하거나 먼저 `demo.sh down`을 실행한다.

## 5. 223 창, pane 순서

- `ctrl`: pc2의 sudo 비밀번호, 시계 확인, guarded controller. stance가 될 때까지 기다린다.
- `planner`: idle planner 미리보기 후 시작한다. planner가 확인을 마치고 종료하면 같은 줄이 tunnel과 dimos를 올린다. 확인: `docker compose -f compose.demo.yaml ps tunnel`이 Up이고 계속 Up으로 유지된다.
- `log`: blueprint가 MCP tool 목록을 출력할 때까지 기다린다(약 1분).
- `test`: 자연스러운 초대 제스처 한 번과 그에 맞는 대사.
- `greeter`: 본 공연. Ctrl-C로 끝내며, 실행 중인 제스처는 마저 끝난다.

"SONIC is not in confirmed planner idle": controller가 tunnel을 통해 붙지 않았거나, planner pane이 완료되지 않았거나, test를 첫 1분 안에 실행한 경우다. 어느 쪽인지는 아래 명령으로 가린다.

```
docker compose -f compose.demo.yaml logs --tail 20 tunnel
docker compose -f compose.demo.yaml exec dimos python -c "from dimos.porcelain.dimos import Dimos; a=Dimos.connect(); s=a.SonicExecutionModule.get_status(); print({k: s[k] for k in ('state','subscriptions_ready','feedback_fresh','feedback_age_s','ready','error')}); f=a.SonicExecutionModule.get_feedback(); print(None if f is None else f.reference_guard); a.stop()"
```

`subscriptions_ready`가 false이거나 `feedback_fresh`가 false이면 tunnel 또는 controller 문제다. 둘 다 true인데 `planner_confirmed`가 false이거나 `armed`가 true이면 planner pane을 다시 한다(7단계의 note 참조).

"stream did not arm (tick deadline missed); control returned to the planner": blueprint 자체의 streaming loop(dimos 컨테이너 안)가 arming 중 100 ms를 넘게 멈췄고 세션이 스스로 중단되었다는 뜻이다. pc2는 planner idle에 머문다. 모든 journal 이벤트는 이 loop에서 fsync되며 journal은 `dimos-state` volume에 있다. policy 서버가 동시에 로딩 중인 HDD에 Docker data root가 있거나, GPU 서버를 시작하는 동안 CPU가 부족하면 둘 다 이 증상을 낸다. `ready on port 5571`이 뜨고 시스템이 조용해질 때까지 기다린 뒤 test pane에서 Enter를 다시 누른다. 유휴 상태에서도 반복되면 `docker info --format '{{.DockerRootDir}}'`, `cat /proc/loadavg`, `docker stats --no-stream`을 확인한다.

## 6. 222 창, pane 순서

- `T0`: :5555의 RealSense 카메라 서버. 계속 실행해 둔다.
- `T1`: planner hold 상태의 SONIC deploy. `Init Done`이 나올 때까지 기다린다.
- `T2` 전에 222에서 policy 서버를 warm-up한다(harness는 첫 요청에 10 s를 한 번만 주는데, cold 서버는 이를 맞추지 못한다). 서버와만 통신하며 로봇에는 아무것도 전달되지 않는다.

  ```
  ssh unitree@192.168.0.222 'cd ~/GR00T-WholeBodyControl-loop-20261026 && PYTHONPATH=$PWD ~/GR00T-WholeBodyControl-harness-20261022/venv/bin/python -c "import numpy as np, time; from gear_sonic.utils.inference.lerobot_sonic_policy import LeRobotSonicPolicyClient as C; c=C(\"<WORKSTATION_IP>\", 5571, 30000); t=time.monotonic(); a,m=c.get_action({\"lerobot\":{\"state\":np.zeros(28,np.float32),\"image\":np.zeros((480,640,3),np.uint8),\"task\":\"probe\"}}); print(\"warm\", round(time.monotonic()-t,2), \"s\", m)"'
  ```

  `warm <seconds> {...}`가 출력된다. 시간이 10 s를 한참 밑돌 때까지 반복한다.
- `T2`: `policy_host: <WORKSTATION_IP>`를 출력한 뒤 harness를 실행한다.
- `k`: T2가 subscribe하기를 기다렸다가 `k`를 한 번 보내고 상태를 출력한다. `controller_running: true`, `policy_ready: true`. 두 번 보내지 말 것. `k`는 토글이다.
- `rec`: telemetry recorder.
- `T3`: loop. "Policy prewarm unavailable": harness에 policy가 없거나(`policy_host` 오류, policy 서버 미준비) 카메라 프레임이 없다(T0). "upright_check: APIConnectionError": loop가 `--vlm-base-url`의 VLM에 닿지 못한다. 원인은 셋 중 하나다. `WORKSTATION_IP`를 설정하기 전에 연 pane이 옛 주소를 그대로 갖고 있는 경우, vlm 컨테이너가 아직 서비스 중이 아닌 경우(워크스테이션에서 `curl -s http://127.0.0.1:8301/v1/models`를 실행했을 때 `qwen3-vl-4b`가 나오면 서비스 중이다), 워크스테이션 방화벽이 로봇 네트워크의 8301과 5571을 막는 경우(222에서 `curl -s --max-time 5 http://<WORKSTATION_IP>:8301/v1/models`; `sudo ufw allow from 192.168.0.0/24 to any port 8301 proto tcp`, 5571도 동일).
- `T4`: verdict console. trial이 끝날 때까지 출력이 없다.

harness는 policy를 정확히 한 번만 prewarm한다. 첫 카메라 프레임에서 10 s deadline의 요청을 한 번 보내고 `policy_ready`는 그 harness가 살아 있는 동안 그 요청이 돌려준 값으로 고정된다.
따라서 그 시점에 문제(T0 없음, 잘못된 `policy_host`, 서버 로딩 중, cold 서버, 방화벽)가 하나라도 있으면 원인을 고친 뒤에도 `policy_ready: false`가 유지된다. 원인을 고치고 서버를 warm-up(위 참조)하고 T2를 재시작한 다음 `k`를 한 번 보낸다. policy 서버를 재시작해도 harness가 같은 식으로 무효가 된다.

## 7. 종료

```
docker compose -f compose.demo.yaml exec dimos dimos mcp call stop_arm   # wait for its tool update in the log pane
# 222 panes: Ctrl-C T3, rec, T2, T1, T0; 223 panes: Ctrl-C greeter, then the controller in ctrl
./demo.sh down
```

policy 서버만 내리려면 `docker compose -f compose.demo.yaml --profile workstation stop policy`를 실행한다(`up -d --no-build policy`로 다시 올린다).

데모 도중 pc2에서 무언가를 재시작하면, planner 스크립트에 pc2의 5556 포트가 비어 있어야 하는데 이 포트는 tunnel이 잡고 있다. `docker compose -f compose.demo.yaml stop tunnel dimos`를 실행한 뒤 planner pane을 다시 한다.

## 8. 알려진 문제 (2026-10-29 데모)

- 223: 제스처와 음성이 실행 도중 끊겼다. 둘 다 제스처마다 워크스테이션과 pc2 사이의 네트워크를 건넌다. tick은 ssh tunnel(`-R 5556`)을 통해 나가고 feedback은 같은 tunnel(`-L 5557`)로 돌아오며 reference가 오래되면 pc2의 guard가 planner로 해제하고 음성 한 줄마다 pc2로 ssh 명령을 보낸다. 링크가 약 100 ms 넘게 멈추면 동작이 끊기고 ssh가 멈추면 대사가 끊긴다. 미해결이다. 다음에는 먼저 측정하고(제스처 중 `ping -i 0.2 192.168.0.223`, 재전송 확인용 `ss -ti dst 192.168.0.223`, 워크스테이션이 Wi-Fi인지 여부), 워크스테이션을 유선 로봇 네트워크에 연결하거나 tick 소스를 pc2로 옮겨 링크에는 명령만 흐르게 한다.
- 222: 데모 공간과 학습 데이터 사이의 시각적 차이 때문에 불안정하다. 여기서는 범위 밖이다.
