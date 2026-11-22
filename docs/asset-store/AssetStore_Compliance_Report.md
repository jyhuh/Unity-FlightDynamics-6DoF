# Unity Asset Store 업로드 기술 검토 및 Colima 외부 SITL 이슈 해결 보고서

## 1. Colima / 외부 PX4 SITL 시뮬레이션 구조가 에셋 스토어 심사에서 문제가 되는 이유

유니티 에셋 스토어 심사 가이드라인(**Unity Asset Store Submission Guidelines — Section 2 & Section 4**)에 따르면, 업로드되는 모든 에셋 패키지는 다음과 같은 엄격한 기술 요건을 충족해야 합니다.

| 심사 항목 | 기존 Colima / 외부 SITL 전용 구조의 문제점 | 심사 거절(Rejection) 위험도 |
| :--- | :--- | :---: |
| **1. 자체 완결성 (Self-Contained Out-of-the-Box)** | 심사관 및 구매자는 Colima, Docker, Linux VM, ROS 2 컨테이너, 또는 3GB 규모의 `PX4-Autopilot` 이미지가 없는 순수 Unity 환경에서 데모 씬을 실행합니다. 외부 컨테이너가 없다고 기체가 이륙/비행하지 않으면 **"Not functional out of the box"** 사유로 거절됩니다. | **매우 높음 (치명적)** |
| **2. 크로스 플랫폼 호환성 (Windows / macOS / Linux)** | 기존 `Px4SitlProcessLauncher`는 Play 진입 시 무조건 `/bin/bash -lc ".../scripts/px4-sitl.sh ..."` 실행을 시도했습니다. Windows 환경이나 새 Unity 프로젝트에는 `/bin/bash` 및 `scripts/px4-sitl.sh`가 존재하지 않아 Play 즉시 `Win32Exception` / `Console Error`가 발생합니다. | **매우 높음 (치명적)** |
| **3. 프로젝트 루트 오염 및 기존 씬 덮어쓰기 금지** | 1) Play 시 구매자의 프로젝트 루트에 `docker/config/*.params` 폴더를 강제 생성하던 동작<br/>2) `[InitializeOnLoad]`에서 활성 씬과 `EditorBuildSettings.scenes`를 자동 덮어쓰던 동작은 심사 가이드라인(**2.1.d Non-intrusive Import**) 위반입니다. | **높음** |
| **4. 단일 에셋 폴더 구조 (`Assets/FlightDynamics6DoF/`)** | 에셋 스토어 업로드는 `Packages/`가 아닌 `Assets/<PackageName>/` 단일 루트 폴더 하위에 모든 런타임 DLL, 스크립트, 에디터 툴, 데모 씬, 문서가 포함되어야 합니다. | **필수 요건** |

---

## 2. 해결 방안: Dual-Mode 아키텍처 (내장 C# Standalone Autopilot 기본 탑재 + 외부 PX4/ROS 2 SITL 선택적 연동)

본 에셋의 코어 엔진(`FlightDynamics6DoF.Core.dll`)에는 이미 외부 리눅스 컨테이너 없이도 회전익(Multirotor), 고정익(Fixed-Wing), 복합형 수직이착륙기(QuadPlane VTOL)를 정밀 제어할 수 있는 **순수 C# 비선형 비행제어기(`CascadedFlightController`: SO(3) 쿼터니언 자세 제어 + PID 위치/속도 제어 + 고정익 활주로 이륙 활주/TECS/L1 유도 제어)**가 내장되어 있습니다.

이를 활용하여 **에셋 스토어 완벽 호환을 위한 5가지 핵심 코드 개선**을 적용했습니다.

### 2.1 외부 SITL 스크립트 존재 여부 자동 감지 (`IsSitlWorkspaceAvailable`)
- `Px4SitlProcessLauncher.IsSitlWorkspaceAvailable` 및 `SitlInfraController.IsSitlWorkspaceAvailable` 프로퍼티를 추가하여 프로젝트 루트에 `scripts/px4-sitl.sh`가 존재하는지 자동 판별합니다.
- 일반 에셋 스토어 설치 환경(`scripts/px4-sitl.sh` 없음)에서는:
  1. `/bin/bash` 외부 프로세스를 **절대 호출하지 않습니다** (Windows/macOS/Linux Console Error 0건 보장).
  2. 프로젝트 루트에 `docker/config/` 폴더를 **생성하지 않습니다**.
  3. Play 모드 진입 시 즉시 **Standalone Built-in C# Autopilot Mode (`UseInternalFlightController = true`)**로 자동 전환되어 외부 프로그램 없이 즉시 비행 가능합니다.

### 2.2 MAVLink TCP 4560 무중단 자동 전환 (Seamless Handover)
- 기존 `MavlinkNetworkNode.FixedUpdate()`가 외부 PX4 연결 여부와 무관하게 매 물리 프레임마다 `Vehicle.UseInternalFlightController = false`로 덮어쓰던 문제를 수정했습니다.
- 이제 `MavlinkNetworkNode`는 백그라운드에서 `TCP :4560` (외부 PX4 SITL 대기) 및 `UDP :14550` (QGroundControl 텔레메트리) 포트를 열어두되:
  - **외부 PX4 SITL 미연결 시**: 내장 C# `CascadedFlightController`가 100% 독립적으로 기체를 제어합니다.
  - **외부 PX4 SITL 연결 시 (`IsConnected && TotalHilActuatorFramesReceived > 0`)**: 자동으로 내장 제어기를 해제하고 외부 PX4 SITL의 `HIL_ACTUATOR_CONTROLS` Lockstep 제어로 즉시 전환합니다.

### 2.3 HUD 및 Workbench 비행 명령의 내장 C# Autopilot 양방향 라우팅
- 런타임 HUD 및 에디터 워크벤치(Step 4)의 모든 비행 명령 버튼(**`Arm`**, **`Disarm`**, **`Takeoff`**, **`Loiter`**, **`Start Autonomous Circular Orbit (10m)`**, **`RTL`**, **`Land`**, **`VTOL -> Fixed-Wing`**, **`VTOL -> Multicopter`**)이 외부 PX4가 없을 때는 **내장 C# `CascadedFlightController`의 메서드를 직접 실행**하도록 구현했습니다.
- 특히 `CascadedFlightController`에 다음 기능을 추가 검증했습니다:
  - `CommandCircularOrbit(...)`: ROS 2 컨테이너 없이도 순수 C# 웨이포인트 루프를 통해 10m 반경 자율 원형 선회 비행 수행
  - `CommandVtolTransition(bool toFixedWing, ...)`: QuadPlane VTOL 기체의 수직이착륙(MC) ↔ 전진 고정익 크루즈(FW) 천이 비행 수행
  - `CommandLoiterHold(...)`: 현재 고도 및 위치 홀드

### 2.4 에디터 Workbench UI (Step 2: Infra)의 Standalone 안내 모드
- 외부 SITL 스크립트가 없는 새 유니티 프로젝트에서 `Tools > FlightDynamics 6-DOF > Unified 5-Step Engineering Workbench`를 열면:
  - 상단 상태 배지가 `STOPPED` 경고 대신 **`Backend: STANDALONE`** / **`Autopilot: BUILT-IN GNC`** (초록색 정상 배지) 및 **`unity-px4-sitl: OPTIONAL (TCP:4560)`**으로 표시됩니다.
  - Step 2 (`2. Infra`) 탭 상단에 Standalone C# Autopilot이 활성화되어 외부 컨테이너 없이 즉시 시뮬레이션이 가능하다는 친절한 안내 박스가 표시됩니다.
  - 런타임 HUD에서도 불필요한 Colima VM 관리 버튼(`VM Status`, `Repair containerd`, `Start VM`)을 숨겨 깔끔한 항공 시뮬레이션 UI를 제공합니다.

### 2.5 비침습적(Non-Intrusive) 에셋 임포트 보장
- `UnityEditorCliAndSceneBootstrap` (`[InitializeOnLoad]`)가 사용자의 기존 작업 씬이나 `EditorBuildSettings.scenes`를 임의로 덮어쓰지 않도록 보호 로직을 적용했습니다.

---

## 3. 에셋 스토어 패키징 구조 (`Assets/FlightDynamics6DoF/`)

생성된 `FlightDynamics6DoF_AssetStore_v1.0.0.unitypackage`는 임포트 시 다음과 같이 단일 루트 폴더(`Assets/FlightDynamics6DoF/`) 하위에 깔끔하게 설치됩니다.

```text
Assets/FlightDynamics6DoF/
├── package.json
├── README.md
├── ThirdPartyNotices.md
├── Runtime/
│   ├── FlightDynamics6DoF.Unity.asmdef
│   ├── Plugins/
│   │   ├── FlightDynamics6DoF.Core.dll       # 난독화/최적화된 .NET Standard 2.1 코어 물리/공력/GNC/MAVLink 엔진
│   │   └── FlightDynamics6DoF.Core.xml       # IDE IntelliSense API 문서
│   ├── Assets/
│   │   └── AircraftProfileAsset.cs           # 기체 제원 및 Stage-1 공력 DB ScriptableObject
│   └── Components/
│       ├── FlightDynamicsVehicle.cs          # 6-DOF 다중 정밀도 항공기 플랜트 및 기즈모
│       ├── FlightSimulationManager.cs        # 씬 매니저 및 인터랙티브 엔지니어링 HUD
│       ├── MavlinkNetworkNode.cs             # Zero-Dependency MAVLink v2 TCP/UDP 브릿지
│       ├── Px4SitlProcessLauncher.cs         # Standalone C# 라우터 & 선택적 외부 PX4 SITL 런처
│       ├── ProceduralAircraftVisualBuilder.cs# 3종 항공기(회전익/고정익/VTOL) 및 활주로 절차적 3D 모델러
│       ├── Ros2LidarAndObstacleCourse.cs     # 360° 2D Raycast LiDAR 및 장애물 코스 생성기
│       ├── FlightCameraController.cs         # 4모드 비행 카메라 (Chase, Follow, FPV, Top-Down)
│       └── FlightLogRetentionSettings.cs     # 학술용 CSV/HTML 텔레메트리 로거 보존 정책
├── Editor/
│   ├── FlightDynamics6DoF.Editor.asmdef
│   ├── FlightDynamicsEngineeringWizard.cs    # 6-Step 통합 엔지니어링 워크벤치 윈도우
│   ├── AeroDesignPanel.cs                    # Step 1: 공력 해석 및 BEMT/Airfoil 계산기
│   ├── InfraControlPanel.cs                  # Step 2: Standalone 안내 및 선택적 SITL 컨테이너 제어
│   ├── SimulationEnvironmentPanel.cs         # Step 3: 3D 씬 원클릭 빌드 및 충실도(L0~L3)/풍향 제어
│   ├── Px4TuningPanel.cs                     # Step 4: 기체별 제어 게인 튜닝 및 비행 명령
│   ├── TelemetryLogPanel.cs                  # Step 5: 실시간 멀티채널 차트 및 CSV/HTML 리포트
│   ├── Ros2Nav2Panel.cs                      # Step 6: 2D LiDAR & 장애물 회피 / 오프보드 선회 제어
│   ├── SitlInfraController.cs                # 백그라운드 상태 컨트롤러
│   ├── TelemetryChartRenderer.cs             # 에디터 벡터 차트 렌더러
│   ├── TelemetryChannelRegistry.cs           # 텔레메트리 채널 레지스트리
│   ├── TelemetryLogRepository.cs             # 로그 저장소
│   ├── TelemetryTimeViewport.cs              # 타임라인 뷰포트
│   ├── FlightLogSettingsWindow.cs            # 로그 용량 관리 윈도우
│   └── UnityEditorCliAndSceneBootstrap.cs    # 원클릭 데모 씬 빌더 메뉴
└── Demo/
    ├── Scenes/
    │   └── FlightDynamics6DoF_EngineeringDemo.unity  # 즉시 실행 가능한 3D 엔지니어링 데모 씬
    └── Profiles/
        └── EngineeringAircraftProfile.asset          # 사전 베이킹된 항공기 프로파일 에셋
```
