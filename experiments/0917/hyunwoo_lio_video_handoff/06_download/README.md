# 현우님 전달용 ZIP

아래 **두 ZIP을 모두 받아 같은 위치에 압축을 풀어 주세요.** 각각 독립적인 ZIP이므로 분할 압축 합치기 프로그램은 필요 없습니다. 압축 안의 `hyunwoo_lio_video_handoff` 폴더를 합치면 MP4 5개와 지도·궤적·판단·설명 자료가 모두 갖춰집니다.

- [hyunwoo_goal4_part1.zip](hyunwoo_goal4_part1.zip): ESCAPE RGB 중심 원속도, ESCAPE RGB+VLM 4배속, 지도·궤적 CSV·판단·설명 자료.
- [hyunwoo_goal4_part2.zip](hyunwoo_goal4_part2.zip): ESCAPE RGB+VLM 원속도, PixelNav 전방 RGB 원속도·4배속.

영상은 재인코딩하지 않았습니다. 기존 LIO Goal 4 자료이며 외부 휴대폰 촬영 영상과 원본 bag은 포함하지 않습니다.

GitHub의 [파일당 100MiB 제한](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)에 맞춰 두 개로 나눴습니다. MP4 자체는 Git 제외 대상이지만 이 ZIP 두 개는 커밋할 수 있습니다. 단순 `git push` 전에 새 ZIP도 `git add`와 `git commit`에 포함해야 합니다.

Jetson의 기존 단일 ZIP `/home/unitree/Downloads/hyunwoo_lio_goal4_handoff_20260917.zip`은 그대로 보존했습니다.
