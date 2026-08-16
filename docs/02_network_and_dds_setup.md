# 🌐 [02] 네트워크 IP 토폴로지 및 CycloneDDS 통신 세팅 가이드

> **문서 소유자**: **민석 (Minseok)**  
> **문서 목적**: Unitree Go2 사족보행 로봇 내부 이더넷(`eth0`), 등판 Jetson Orin NX 온보드 모듈, 도커 컨테이너, 외부 Pro 6000 GPU 서버 간의 **IP 주소 체계, CycloneDDS XML 설정, Host-Docker 소켓 브릿지 명세**입니다.

---

## 📌 목차 (Table of Contents)
1. [네트워크 IP 주소 체계 (IP Topology)](#1-네트워크-ip-주소-체계-ip-topology)
2. [CycloneDDS 통신 설정 (cyclonedds.xml)](#2-cyclonedds-통신-설정-cycloneddsxml)
3. [Host-Docker 루프백 소켓 브릿지 구조](#3-host-docker-루프백-소켓-브릿지-구조)

---

## 🗺️ 1. 네트워크 IP 주소 체계 (IP Topology)

```text
========================================================================================
                         UNITREE GO2 NETWORK IP TOPOLOGY
========================================================================================
• Go2 메인 모션 제어 보드 (Main Motion Controller) : 192.168.123.161
• Go2 등판 젯슨 보드 1 (Jetson Orin NX Master)     : 192.168.123.15 (우리가 SSH 접속하는 보드)
• Go2 등판 젯슨 보드 2 (Jetson Sub-processor)      : 192.168.123.14
• 외부 VLM 추론 서빙 서버 (RTX Pro 6000 Server)    : 192.168.123.100 (또는 연구실 공유 IP)
• 민석 님 개발 및 모니터링 노트북                  : 192.168.123.99
========================================================================================
```

---

## ⚙️ 2. CycloneDDS 통신 설정 (`cyclonedds.xml`)

호스트 및 도커 컨테이너 내에서 ROS 2 Foxy 및 Jazzy 패킷 충돌을 방지하기 위한 표준 DDS 바인딩 파일입니다:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
        </General>
        <Discovery>
            <Peers>
                <Peer address="192.168.123.15"/>
                <Peer address="192.168.123.161"/>
            </Peers>
            <ParticipantIndex>auto</ParticipantIndex>
        </Discovery>
        <Internal>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
    </Domain>
</CycloneDDS>
```

---

## 🔌 3. Host-Docker 루프백 소켓 브릿지 구조

* **도커 내부 (`scratch/docker_bridge.py`)**: S2E 노드가 발행한 궤적 속도 명령을 수신하여 `127.0.0.1:5005` 로컬 루프백으로 송신 ($0.1\text{ms}$ 지연).
* **호스트 OS (`scratch/host_bridge.py`)**: 루프백 패킷을 수신하여 `unitree_sdk2_python` 기반 `SportClient.Move(vx, vy, vyaw)`로 변환 후 $50\text{Hz}$로 로봇 모터 제어 보드(`192.168.123.161`)에 직접 인가.
