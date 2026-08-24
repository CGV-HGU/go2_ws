# 🤖 [Docker Control 03] VLM 모델 실시간 자동 감지(Auto-Discovery) 및 Zero-Config 운영 가이드

> **대상 시스템**: Ubuntu 24.04 Docker (`sdam_go2_container`), `nav_memory_qwen.vlm_client`  
> **문서 목적**: 서버의 VLM 모델 교체 시 클라이언트 수정 없이 100% 자동 감지 및 바인딩하는 무설정(Zero-Config) 파이프라인 가이드

---

## 1. 🔍 VLM 모델 자동 감지 메커니즘 (`auto_detect_served_model`)

1. **엔드포인트 조회**:
   - `GET http://100.96.60.15:8000/v1/models`
2. **응답 스키마 파싱**:
   ```json
   {
     "object": "list",
     "data": [
       {
         "id": "qwen3.5-9b-instruct",
         "object": "model",
         "created": 1724490000,
         "owned_by": "vllm"
       }
     ]
   }
   ```
3. **자동 바인딩**:
   - `models[0]`을 추출하여 이후 모든 `/v1/chat/completions` 요청의 `"model"` 파라미터로 자동 적용합니다.

---

## 2. ⚡ 검증 및 테스트 실행법

```bash
# Docker 내부에서 4-Tier 파이프라인 및 VLM 자동 감지 1-Click 테스트
docker exec sdam_go2_container python3 /workspace/go2_ws_antarctica/scratch/test_docker_to_sport_cmd_vel_pipeline.py
```
