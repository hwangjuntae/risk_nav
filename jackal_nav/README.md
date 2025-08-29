# Jackal Navigation Package

Jackal 로봇을 위해 특별히 제작된 독립적인 navigation 패키지입니다. Carter navigation의 로직을 통합하여 안전하고 효율적인 자율 주행을 제공합니다.

## 📋 기능

- **Nav2 기반 자율 주행**: 최신 ROS2 Navigation2 스택 사용
- **Risk Assessment**: LiDAR와 카메라 데이터를 통한 실시간 위험도 평가
- **Adaptive Speed Control**: 위험도에 따른 속도 조절
- **Jackal 최적화**: Jackal 로봇의 물리적 특성에 맞춘 파라미터
- **Real-time Visualization**: RViz를 통한 실시간 상태 모니터링
- **Isaac Sim 호환**: Isaac Sim 환경에서의 완전한 navigation 지원

## 🚀 주요 구성 요소

### 1. Navigation Parameters
- **jackal_navigation_params.yaml**: Jackal에 최적화된 Nav2 파라미터
- Jackal의 크기와 성능에 맞게 최적화된 navigation 파라미터

### 2. Risk Assessment Node
- **jackal_risk_navigation.py**: 실시간 위험도 평가 및 속도 제어
- LiDAR 기반 장애물 감지
- 카메라 기반 시각적 위험 요소 감지
- 적응형 속도 제어

### 3. Launch Files
- **jackal_navigation.launch.py**: 기본 navigation 시스템 실행 (Isaac Sim 호환)

## 💻 사용법

### 1. 환경 설정
```bash
cd /home/teus/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2. Navigation 시스템 실행
```bash
# 전체 navigation 시스템 실행
ros2 launch jackal_nav jackal_navigation.launch.py

# 특정 맵 파일 지정
ros2 launch jackal_nav jackal_navigation.launch.py map:=/path/to/your/map.yaml

# 시뮬레이션 모드 (기본값)
ros2 launch jackal_nav jackal_navigation.launch.py use_sim_time:=true
```



### 4. 개별 노드 실행
```bash
# Risk assessment 노드만 실행
ros2 run jackal_nav jackal_risk_navigation.py
```

## 📁 패키지 구조

```
jackal_nav/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/              # 설정 파일들
├── maps/                # 맵 파일들
│   ├── office.png
│   └── office.yaml
├── params/              # Navigation 파라미터들
│   └── jackal_navigation_params.yaml
├── launch/              # Launch 파일들
│   └── jackal_navigation.launch.py          # 기본 Navigation (Isaac Sim 호환)
├── rviz/                # RViz 설정
│   └── jackal_navigation.rviz
├── src/                 # Python 소스 코드
│   └── jackal_risk_navigation.py
└── jackal_nav/          # Python 패키지
    └── __init__.py
```

## 🎛️ 주요 파라미터

### Navigation Parameters (jackal_navigation_params.yaml)
- **Robot Footprint**: `[0.26, 0.17], [0.26, -0.17], [-0.26, -0.17], [-0.26, 0.17]`
- **Max Linear Velocity**: 2.0 m/s
- **Max Angular Velocity**: 1.0 rad/s
- **Inflation Radius**: 0.8 m
- **Costmap Resolution**: 0.05 m

### Risk Assessment Parameters
- **Min Safe Distance**: 1.0 m
- **Warning Distance**: 2.0 m
- **Max Safe Speed**: 1.0 m/s
- **Processing Rate**: 10 Hz

## 🔧 Jackal 로봇 최적화 특징

1. **Robot Size**: Jackal의 footprint `[0.26, 0.17], [0.26, -0.17], [-0.26, -0.17], [-0.26, 0.17]`에 맞게 조정
2. **Velocity Limits**: Jackal의 성능에 맞는 속도 제한 (선형: 2.0 m/s, 각속도: 1.0 rad/s)
3. **Sensor Configuration**: 단일 LiDAR 설정으로 단순화
4. **Topic Names**: Jackal 표준 토픽명 사용 (`/scan`, `/odom`, `/cmd_vel`)

## 📊 토픽 정보

### Subscribed Topics
- `/Lidar/laser_scan` (sensor_msgs/LaserScan): Isaac Sim LiDAR 데이터
- `/Lidar/point_cloud` (sensor_msgs/PointCloud2): Isaac Sim 3D LiDAR 포인트클라우드
- `/Camera/rgb` (sensor_msgs/Image): Isaac Sim RGB 카메라 이미지
- `/Camera/depth` (sensor_msgs/Image): Isaac Sim 깊이 카메라 이미지
- `/Camera/camera_info` (sensor_msgs/CameraInfo): 카메라 정보
- `/chassis/odom` (nav_msgs/Odometry): Isaac Sim 오도메트리
- `/chassis/imu` (sensor_msgs/Imu): Isaac Sim IMU 데이터
- `/cmd_vel` (geometry_msgs/Twist): 원본 속도 명령

### Published Topics
- `/cmd_vel_risk` (geometry_msgs/Twist): 위험도 조정된 속도 명령
- `/risk_assessment/image` (sensor_msgs/Image): 위험도 시각화 이미지
- `/risk_map` (nav_msgs/OccupancyGrid): 위험도 맵
- `/scan_converted` (sensor_msgs/LaserScan): 변환된 2D LiDAR 스캔 (포인트클라우드에서 변환)

## ⚠️ 주의사항

1. **의존성**: 다음 패키지들이 설치되어 있어야 합니다:
   - `navigation2`
   - `nav2_bringup`
   - `cv_bridge`
   - `opencv-python`

2. **센서 데이터**: LiDAR와 카메라 토픽이 올바르게 퍼블리시되고 있는지 확인하세요.

3. **맵 파일**: navigation 시작 전에 적절한 맵 파일이 준비되어 있어야 합니다.

## 🐛 문제 해결

### 빌드 오류
```bash
# 의존성 재설치
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 워크스페이스 클린 빌드
cd /home/teus/ws
rm -rf build/ install/
colcon build --packages-select jackal_nav
```

### 런타임 오류
```bash
# 토픽 확인
ros2 topic list | grep -E "(scan|camera|cmd_vel)"

# 노드 상태 확인
ros2 node list
ros2 node info /jackal_risk_navigation
```

## 📚 참고 자료

- [ROS2 Navigation2 Documentation](https://navigation.ros.org/)
- [Jackal Robot Documentation](https://docs.clearpathrobotics.com/docs/ros/tutorials/outdoor/jackal) 