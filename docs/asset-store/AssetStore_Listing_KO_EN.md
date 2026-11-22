# Unity Asset Store Publisher Portal 등록 자료 (English & 한국어)

이 문서는 **Unity Asset Store Publisher Portal (https://publisher.unity.com/)**에 에셋을 등록할 때 각 입력 필드에 바로 복사하여 붙여넣을 수 있도록 규격에 맞춰 작성된 공식 등록 자료입니다.

---

## 1. 기본 메타데이터 (Basic Metadata)

- **Package Title (영문 공식 명칭)**:
  `FlightDynamics 6-DOF: Drone & UAV Suite`
- **GitHub Repository / Documentation URL**:
  `https://github.com/jyhuh/Unity-FlightDynamics-6DoF`
- **Category (카테고리)**:
  `Tools` > `Physics` (또는 `Tools` > `Simulation`)
- **Keywords / Tags (최대 15개, 쉼표 구분)**:
  `flight dynamics, 6dof, uav, drone, vtol, fixed wing, aerodynamics, aircraft physics, mavlink, px4, sitl, ros2, lidar, autopilot, simulation`
- **Supported Unity Versions**:
  `Unity 2021.3 LTS` 이상 및 `Unity 6 (6000.0 / 6000.3+)` 완벽 지원 (.NET Standard 2.1)
- **Render Pipeline Compatibility**:
  - **Universal Render Pipeline (URP)**: Compatible (Default Shader auto-detection)
  - **Built-in Render Pipeline (BIRP)**: Compatible (Automatic Standard shader fallback)
  - **High Definition Render Pipeline (HDRP)**: Compatible via material fallback
- **License**:
  Standard Unity Asset Store EULA (100% Clean-Room C# implementation, zero GPL/LGPL dependencies; third-party academic references documented in `ThirdPartyNotices.md`)

---

## 2. Short Summary (요약 설명 — 200자 이내)

### [English] (198 characters)
Engineering-grade 6-DOF UAV, Fixed-Wing & eVTOL flight physics suite for Unity. Features 3D VLM & BEMT aerodynamics, RK4 physics (L0-L3), C# SO(3)/TECS autopilot, 2D LiDAR & MAVLink v2 / PX4 bridge.

### [한국어]
유니티용 항공우주 엔지니어링급 6자유도(6-DOF) 회전익 드론, 고정익 UAV 및 eVTOL 비행역학 시뮬레이션 스위트입니다. 3D VLM 및 BEMT 공력 해석기, 다중 정밀도 RK4 물리 엔진(Level 0~3), 내장 C# 비선형 비행제어기, 2D LiDAR, 학술용 텔레메트리 리포트 및 MAVLink v2 / PX4 SITL 연동을 제공합니다.

---

## 3. Description (상세 설명 — Publisher Portal Description 필드용)

### [English Version — Paste into Publisher Portal `Description`]

**FlightDynamics 6-DOF: Drone & UAV Suite** is an aerospace-grade flight dynamics, aerodynamic analysis, and autonomous GNC (Guidance, Navigation, and Control) engineering suite for Unity. Designed for UAV researchers, aerospace engineers, robotics developers, and high-fidelity flight simulator creators, it bridges offline aerodynamic solver accuracy with real-time 6-DOF simulation.

**Works 100% Standalone Out-of-the-Box (Windows, macOS, Linux)**
No external simulators or containers are required. The built-in pure C# **Cascaded Nonlinear SO(3) Attitude, PID Position, and Fixed-Wing TECS/L1 Flight Controller** flies Multirotor drones, Fixed-Wing aircraft (including runway ground roll & rotation), and Hybrid QuadPlane eVTOLs immediately upon pressing Play. For advanced hardware/software-in-the-loop (HIL/SITL) workflows, it also includes a zero-dependency **MAVLink v2 TCP/UDP bridge** for external **PX4-Autopilot SITL (TCP :4560)**, **QGroundControl (UDP :14550)**, and **ROS 2 (UDP :8888 / :8890)**.

**Suitable Genres & Applications**
- **Drone & Flight Simulators**: Realistic fixed-wing runway takeoff/landing, quadcopter agility, and eVTOL transition physics.
- **Aerospace Research & Robotics (Digital Twins)**: Synthetic IMU/GPS/Baro/Pitot/2D LiDAR sensor generation, Dryden turbulence testing, and CSV/HTML5 academic telemetry analysis.
- **Autonomous UAV & Defense Training**: Waypoint missions, circular orbit loitering, obstacle avoidance courses, and optional PX4/QGroundControl/ROS 2 integration.

**Fully Customizable**
- **Custom 3D Airframes**: Attach `FlightDynamicsVehicle` to any 3D aircraft mesh and use the 1-click **Extract Geometry from Child Meshes** helper to estimate inertia tensors and aerodynamic reference areas automatically.
- **ScriptableObject & JSON Workflows**: Save, edit, and share custom vehicle specs via `AircraftProfileAsset` or JSON import/export.
- **Configurable Physics & Control**: Choose between 4 physics fidelity levels (`Level 0` to `Level 3`), adjust RK4 sub-step rates (`100–1000 Hz`), toggle ground effect / blade flapping / wind gusts, and tune PID/TECS gains live inside the 6-Step Editor Workbench.

---

### [한국어 버전 — 홍보 및 국내 문서용]

**FlightDynamics 6-DOF: Drone & UAV Suite**는 유니티에서 항공우주 공학 수준의 공력 해석(Aerodynamics), 6자유도(6-DOF) 비선형 비행역학, 그리고 자율 비행 제어(GNC)를 수행할 수 있는 올인원 엔지니어링 스위트입니다.

- **외부 프로그램 없이 100% 단독 동작 (Windows / macOS / Linux 완벽 호환)**:
  내장된 순수 C# 비선형 비행제어기(`CascadedFlightController` — SO(3) 자세 제어, PID 위치 제어, 고정익 활주로 이륙 및 TECS/L1 유도 제어)를 통해 별도의 컨테이너나 외부 프로그램 설치 없이 데모 씬에서 즉시 회전익 드론, 고정익 항공기, QuadPlane VTOL 수직이착륙기를 비행시키고 데이터를 로깅할 수 있습니다.
- **선택적 외부 PX4 SITL / QGroundControl / ROS 2 연동**:
  외부 의존성이 전혀 없는 순수 C# MAVLink v2 소켓 브릿지(`MavlinkNetworkNode`)를 내장하여, 필요 시 외부 PX4-Autopilot SITL(`TCP :4560` Lockstep), QGroundControl(`UDP :14550`), ROS 2 Humble(`UDP :8888 / :8890`)과 즉시 연동됩니다.

---

## 4. Technical Details (핵심 기능 & 기술 사양 — Publisher Portal `Technical details` Tools 템플릿용)

### [English Version — Paste into Publisher Portal `Technical details`]

**Features**
- **Two-Stage Aerodynamic & Stability Pipeline (Stage 1)**:
  - 2D Airfoil Polar Solver (CST parameterization, viscous boundary-layer approximation, Viterna +/-180 deg post-stall, Du-Selig 3D stall delay)
  - 3D Vortex Lattice Method (VLM) horseshoe downwash & control surface effectiveness decay
  - Fuselage Slender-Body & Cross-Flow drag solver
  - CCBlade BEMT Rotor Solver (Prandtl hub/tip loss, Glauert high-thrust, Cheeseman-Bennett IGE ground effect, contracting slipstream wake)
  - Stability & Trim Analyzer (CL_alpha, Cm_alpha, Cn_beta, Neutral Point x_NP, Static Margin % MAC, damped pseudoinverse mixer)
- **Multi-Fidelity Real-Time 6-DOF Engine (Stage 2: Levels 0–3)**:
  - Level 0: Unity Rigidbody lumped Kt/Kq forces
  - Level 1: 400 Hz RK4 surrogate using baked derivatives
  - Level 2: Real-time nonlinear lifting-line strip theory & BEMT rotor inflow
  - Level 3: Real-time 3D VLM wake, rotor-wing slipstream impingement, blade flapping & MIL-F-8785C Dryden turbulence
- **Three Ready-to-Fly Vehicle Presets & Runway Mechanics**:
  - Multirotor (Quad X-450), Fixed-Wing (Talon-180 with Coulomb runway friction mu = 0.025 & rotation takeoff), and Hybrid eVTOL (Millie QuadPlane with MC <-> FW transition)
- **Built-in Standalone C# Autopilot & Optional MAVLink v2 Bridge**:
  - Cascaded SO(3) attitude, PID position/velocity, circular orbit, RTL, auto-land, and Fixed-Wing TECS/L1 cruise
  - Zero-dependency TCP :4560 (PX4 SITL lockstep), UDP :14550 (QGroundControl), and UDP :8890 (360-degree 2D Raycast LiDAR for ROS 2)
- **Unified 6-Step Editor Engineering Workbench**:
  - Airframe & BEMT/Airfoil calculators, 1-click scene builder, live GNC/PX4 parameter sliders, real-time telemetry plots with RMSE metrics, and 1-click CSV + HTML5/SVG report export
- **Architecture**: Pre-compiled `.NET Standard 2.1` core (`FlightDynamics6DoF.Core.dll` + XML docs), open Runtime/Editor C# source, zero external dependencies, Built-in & URP compatible, Fast Enter Play Mode ready

**Supported OS**
- Windows, macOS (Apple Silicon & Intel), and Linux (100% Standalone C# mode runs natively on all platforms; optional external PX4/ROS 2 SITL container workflows supported on macOS & Linux)

**Link to documentation**
- Online Documentation: https://github.com/jyhuh/Unity-FlightDynamics-6DoF/tree/main/docs
- Offline Manual (included in package): `Assets/FlightDynamics6DoF/Documentation/FlightDynamics6DoF_Manual.pdf`

---

## 5. Notes to Asset Store Reviewer (심사관 전달 메모 — Submission Notes 필드용)

> **Quick Verification Steps for Reviewer:**
> 1. Import the package into a clean Unity project (URP or Built-in). All files are self-contained under `Assets/FlightDynamics6DoF/`.
> 2. Open the included demo scene: `Assets/FlightDynamics6DoF/Demo/Scenes/FlightDynamics6DoF_EngineeringDemo.unity` (or click `Tools > FlightDynamics 6-DOF > Quick-Build 3D Demo Scene Now` from the top menu).
> 3. Press **Play**:
>    - The package runs in **Standalone Built-in C# Autopilot Mode** out-of-the-box with zero external software required.
>    - In the top-left on-screen Engineering HUD, click **`Takeoff`**, **`Start Autonomous Circular Orbit (10m)`**, **`Loiter`**, **`RTL`**, or **`Land`** to command autonomous flight.
>    - Click **`Quad X-450 (iris)`**, **`Fixed-Wing (plane)`**, or **`QuadPlane (standard_vtol)`** in Section 2 of the HUD to switch aircraft types live (click **`Takeoff`** after switching to Fixed-Wing to observe realistic runway ground-roll acceleration and rotation takeoff, or click **`Transition to Fixed-Wing (FW)`** on the QuadPlane VTOL).
> 4. Open `Tools > FlightDynamics 6-DOF > Unified 5-Step Engineering Workbench` to inspect the 6-tab engineering editor tools, run Stage-1 aerodynamic analysis, view real-time telemetry charts, and export CSV/HTML5 academic flight reports.

---

## 6. 에셋 스토어 이미지 및 스크린샷 매핑 가이드 (`docs/asset-store/images/`)

Unity Asset Store Publisher Portal의 **Media** 탭에 업로드할 규격별 이미지 파일 목록입니다:

| 이미지 용도 | 필수 해상도 규격 (px) | 생성된 파일 경로 (`docs/asset-store/images/`) | 설명 |
| :--- | :---: | :--- | :--- |
| **Cover Image (메인 커버)** | `1950 x 1300` | `CoverImage_1950x1300.png` | 3종 항공기(Multirotor, Fixed-Wing, VTOL) + 6-DOF 공력/물리 아키텍처 타이틀 배너 |
| **Card Image (스토어 카드)** | `420 x 280` | `CardImage_420x280.png` | 스토어 검색 목록 노출용 썸네일 카드 이미지 |
| **Social Media Image** | `1200 x 630` | `SocialMediaImage_1200x630.png` | 소셜 미디어 공유용 와이드 프로모션 이미지 |
| **Icon Image (패키지 아이콘)** | `160 x 160` | `IconImage_160x160.png` | 패키지 매니저 및 스토어 아이콘 |
| **Screenshot 1** | `1920 x 1080` | `Screenshot_01_Multirotor_QuadX450_3DFlight.png` | 회전익 드론(Quad X-450) 헬리패드 이륙 및 3D 비행 뷰 |
| **Screenshot 2** | `1920 x 1080` | `Screenshot_02_FixedWing_Talon180_RunwayAndFlight.png` | 고정익 UAV(Talon-180) 활주로 이륙 활주 및 상승 비행 뷰 |
| **Screenshot 3** | `1920 x 1080` | `Screenshot_03_HybridVTOL_QuadPlane_Transition.png` | 복합형 수직이착륙기(QuadPlane eVTOL) 천이 비행 뷰 |
| **Screenshot 4** | `1920 x 1080` | `Screenshot_04_LiDAR_ObstacleCourse_Autonomy.png` | 360° 2D Raycast LiDAR 레이저 빔 및 3D 장애물 회피 코스 뷰 |
| **Screenshot 5** | `1920 x 1080` | `Screenshot_05_EngineeringTelemetry_AcademicReport.png` | 6-Step 엔지니어링 워크벤치 및 멀티채널 학술 텔레메트리 리포트 |
| **Screenshot 6** | `1920 x 1080` | `Screenshot_06_SystemArchitecture_TwoStageSolvers.png` | Stage-1 공력 해석(CST/VLM/BEMT) & Stage-2 다중 정밀도 RK4 시스템 아키텍처 |
