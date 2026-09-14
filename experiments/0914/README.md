# 2026-09-14 Jetson 로그 검토

- [야간 주행 분석·에피소드별 궤적·Ours/Direct 비교·와이파이 장애](night_log_review/README.md)
- [회차별 지표 CSV](night_log_review/episodes_summary.csv)
- [4번 궤적 비교](night_log_review/figures/goal4_comparison.png)
- [PixelNav 반대 방향 주행·원본 118단계 재생 대조](pixnav_direction_review/README.md)
- [팀 공유용 PixelNav 정량 평가·4번 5회 결과·1·2·3번 주행 로직 점검](pixnav_quantitative_evaluation/README.md)
- [전체 제한 1800초 후속 검토](jetson_timeout_review.md)
- [1800초 적용·기록기 상한 수정·새 10회차 준비 검증](episode_timeout_1800/README.md)
- [로그 용량·18.2GB 정리 후보·필수 기록 범위](storage_review/README.md)
- [주행 속도·5번 실패·센서 처리 지연 재현·Direct 300초 제한](drive_timing_review/README.md)

야간 로그·PixelNav 원본 대조는 분석만 수행했다. 후속 1800초 작업에서는 실행기·준비기·기록 상한과 캠페인 설정을 수정했다. 로봇 주행·네트워크 설정 변경·커밋/푸시는 수행하지 않았다.
