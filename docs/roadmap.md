# FlightDynamics 6-DOF Task & Roadmap

미구현 기능, 테스트 대기 항목, 향후 추가 개발 백로그를 단계별로 관리하는 문서입니다.

---

## 1. 추가 개발 백로그 (Feature Backlog)

### Step 1. 기체 설계 고도화 (`AeroDesignPanel`)
- [ ] **형상 설정 팝업 창**: 드론(Multirotor) / 고정익(Fixed-Wing) / VTOL 기체의 로터 위치·방향, 날개 스팬·코드·익형, 동체 치수를 직관적으로 편집할 수 있는 전용 팝업 창 구현
- [ ] **실시간 공력 계수 계산 연동**: 팝업에서 수정한 형상 파라미터로부터 `StabilityAndTrimAnalyzer` / `CcBladeBemtSolver` / `VlmAndLiftingLineSolver` 공력 계수를 즉시 재계산·미리보기 및 저장
- [ ] **JSBSim / YASim 호환 기체 정의 Import/Export**: JSBSim XML(`<fdm_config>`, `<metrics>`, `<mass_balance>`, `<aerodynamics>`, `<propulsion>`) 및 YASim XML(`<airplane>`, `<wing>`, `<thruster>`, `<ballast>`) 양식과 호환되는 기체 설계 입출력 지원

### Step 1-B. 비행 동역학(FDM) 확장 (YASim & JSBSim 참고)
- [ ] **다점 착륙장치(Multi-Gear) & 서스펜션 모델** *(JSBSim `<ground_reactions>` / YASim `<gear>` 참고)*: 현재 단일 점 `LandingGearHeightMeters` 접촉 모델을 노즈/메인 기어 개별 좌표·스트럿 스프링/댐퍼·노즈 스티어링·차등 브레이크로 확장
- [ ] **동적 질량·CG·관성 텐서 갱신** *(JSBSim `<mass_balance>` / `<pointmass>` 참고)*: 페이로드 투하, 연료/배터리 소모에 따른 `MassKg`, `CenterOfGravityFrd`, `Ixx/Iyy/Izz` 실시간 업데이트
- [ ] **전압 강하–추진 연동 & 액추에이터 슬루율 제한** *(JSBSim `<propulsion>` / `<flight_control>` 참고)*: 배터리 전압 강하(Sag)에 따른 최대 RPM/추력 저하 반영 및 서보·모터 속도/가속도(Slew Rate) 제한 추가
- [ ] **다차원 공력 테이블(`<table>`) & 형상 기반 자동 밸러스트/트림 솔버** *(JSBSim `<aerodynamics>` / YASim `<ballast>`, `<cruise>`, `<approach>` 참고)*: $\alpha/\beta/\text{Mach}/(h/b)$ 다차원 보간 테이블 지원 및 목표 순항·접근 조건 기반 CG/항력/트림 역산 솔버 추가

### Step 2. 인프라 UI 정리 (`FlightDynamicsEngineeringWizard` / `InfraControlPanel`)
- [ ] **상단 상태 박스 원클릭 제어**: 메인 창 상단 상태 스트립(`VM`, `containerd`, `unity-px4-sitl`, `unity-ros2-dds`)에서 작은 버튼으로 직접 켜고 끌 수 있도록 추가
- [ ] **중복·조잡한 버튼 정리**: 상단/패널 간 중복되는 인프라 제어 버튼을 통폐합해 UI 단순화

### Step 3. 시뮬레이션 환경 고도화 (`SimulationEnvironmentPanel` / `IsaAtmosphereAndTurbulence`)
- [ ] **고도별 돌풍/바람 프로파일**: 고도 구간별 풍향·풍속·돌풍 강도(Wind Shear / Gust Layer) 설정 및 실시간 반영
- [ ] **환경 프리셋 및 대기 조건 확장**: 난기류/기압/온도 오프셋 등 상세 환경 설정 UI 확장

### Step 4. PX4 튜닝 / 파라미터 구조 (`AirframeControlParameterTable`)
- [ ] **C# 내부 제어기(400 Hz) vs PX4 SITL(50 Hz) 게인 분리**: 하나의 테이블이 두 제어 주기를 겸용하면서 생기는 튜닝 간섭 제거

### Step 5. 계측/로그 그래프 고도화 (`TelemetryLogPanel`)
- [ ] **가로축(시간) 이동 및 줌 기능**: 로그 시간축 팬(Pan) / 줌(Zoom) 스크롤 지원
- [ ] **명령-응답(Command vs Response) 비교 그래프**: 목표 자세·고도·속도(Target/Setpoint)와 실제 응답(Actual)을 한눈에 비교할 수 있는 오버레이 구성
- [ ] **동적 그래프 추가/커스터마이징**: 사용자가 원하는 텔레메트리 채널을 선택해 그래프를 동적으로 추가·배치할 수 있는 기능

---