# ⚡ [05] 젯슨(Jetson) 헤드리스 자동 로그인 & NetBird/SSH 원격 부팅 가이드

> **문서 번호**: `docs/jetson_plan/05_jetson_headless_boot_and_autologin_guide.md`  
> **작성 일자**: 2026년 8월 24일 (KST)  
> **문서 소유자**: **민석 (Hardware, Sensor & Deployment Lead)** & **Antigravity Supervisor**  
> **대상 장비**: Unitree Go2 온보드 Jetson Orin NX (Ubuntu 20.04 LTS / ROS 2 Foxy / CUDA 11.4)  
> **IP 구성**: 유선랜 `192.168.123.99` (Go2 DDS), NetBird VPN `100.96.204.119` (`wt0`)

---

## 📌 1. 개요 및 목적
모니터, 키보드, 마우스 없이 운용되는 실물 4족보행 로봇(Unitree Go2) 환경에서, **Jetson 전원 인가 시 비밀번호 입력 대기 없이 즉시 우분투 세션으로 자동 로그인(Auto-login)되고, NetBird 가상망과 SSH 키 원격 접속이 부팅 즉시 활성화되도록 구성하는 시스템 엔지니어링 가이드**입니다.

---

## ⚙️ 2. GDM3 디스플레이 매니저 자동 로그인 설정

우분투 기본 설정에서는 사용자가 화면에 비밀번호를 쳐야 네트워크/서비스가 시작됩니다. 이를 헤드리스 모드로 변경합니다:

* **설정 파일**: `/etc/gdm3/custom.conf`

```ini
[daemon]
WaylandEnable=false

# Enabling automatic login
AutomaticLoginEnable = true
AutomaticLogin = unitree
```

* **적용 명령어**:
```bash
sudo sed -i 's/#  AutomaticLoginEnable = true/AutomaticLoginEnable = true/' /etc/gdm3/custom.conf
sudo sed -i 's/#  AutomaticLogin = user1/AutomaticLogin = unitree/' /etc/gdm3/custom.conf
```

---

## 🌐 3. NetBird VPN & SSH 서비스 부팅 자동 시작 보장

```bash
sudo systemctl enable netbird
sudo systemctl enable ssh
```

* **동작 원리**:
  - Jetson 전원이 켜지면 systemd에 의해 `netbird.service`가 즉시 기동됩니다.
  - 유선랜(`eth0`) 또는 Wi-Fi가 연결되는 즉시 P2P 터널이 형성되어 `100.96.204.119` IP가 할당됩니다.
  - 사용자는 외부 어디서든 태블릿이나 노트북에서 `ssh unitree@100.96.204.119`로 1초 만에 접속할 수 있습니다.

---

## 🌡️ 4. 하드웨어 발열 및 GStreamer 가속 점검

* **열화상 센서 정상 범위**:
  - CPU / GPU / SoC 온도: **$53^\circ\text{C} \sim 57^\circ\text{C}$** (정상 동작 범위, 쓰로틀링 한계 $85^\circ\text{C}$ 대비 안전)
* **GStreamer 하드웨어 가속 플러그인**:
  - `nvv4l2decoder` 정상 탑재 ➔ Go2 전면 카메라 H.264 하드웨어 0% CPU 디코딩 지원.
