"""bringup.launch.py — Jetson 인지 쪽 전체 실행: 카메라 2 대 + perception + 확인용 압축 영상 (+ 선택 Nav2 · mission)

역할
  - 손목 D435 (realsense2_camera) · 광각 Arducam (usb_cam) · perception_node 를 한 번에 띄움
  - publish_debug:=true 면 손목 · 광각 디버그 영상을 Jetson 에서 JPEG 로 압축해 /…/compressed 로 냄 (Wi-Fi 로 보기용)
  - nav:=true · mission:=true 면 navigation/nav2.launch.py · mission/mission.launch.py 도 같이
  - 로봇 본체 (바퀴 · 팔 · 라이다 · slam_toolbox) 는 여기서 안 띄움 → tools/jetson/run_robot.sh (포트 · 안전 확인을 그 스크립트가 함)
입력: launch 인자 (아래 DeclareLaunchArgument)
출력 (띄우는 노드가 내는 것)
  - /camera/camera/color/image_raw (+ /compressed) · aligned_depth_to_color/image_raw · color/camera_info   손목 D435, 30 Hz
  - /wide_cam/image_raw (+ /compressed, JPEG 품질 jpeg_quality) · camera_info   광각 1280×720, 30 Hz
  - /perception/wide/detections · wrist/detections · traffic_light · task_text   perception_node
  - /perception/wrist_debug_image/compressed · wide_debug_image/compressed   publish_debug:=true 일 때
실행 (Jetson)
  source ~/Desktop/Project2_Turtle_Ninja/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=17
  ros2 launch bringup bringup.launch.py
  ros2 launch bringup bringup.launch.py publish_debug:=true                     # 확인용 영상까지
  ros2 launch bringup bringup.launch.py wrist_cam:=false wide_model:=<광각>.engine
note
  - Fast DDS 전송 = FASTDDS_BUILTIN_TRANSPORTS (기본 LARGE_DATA, 큰 영상은 TCP). 이 launch 의 모든 노드에 적용.
    밖에서 보는 쪽 (개발 PC rqt · ros2 topic · 팀원) 도 같은 값을 export 해야 서로 보임 (한쪽만 다르면 그 노드가 안 보임)
    기본 UDP 는 큰 영상 조각 하나만 빠져도 장 전체를 버림 → Wi-Fi 에서 압축 영상 1.5 Hz (2026-11-03 실측, LARGE_DATA 면 30 Hz)
  - Wi-Fi 로는 raw 영상을 못 봄 (실측 처리량 약 3.4 MB/s ↔ 손목 raw 30 Hz 27.6 MB/s) → 개발 PC 는 /compressed 를 받아서 봄
  - 광각 장치는 /dev/video 번호 대신 by-id 경로를 실제 장치로 풀어 넘김 (usb_cam 은 by-id 링크의 상대 경로를 못 풂, 번호는 D435 연결 순서로 바뀜)
  - 광각 usb_cam: mjpeg2rgb + av_device_format YUV420P (없으면 첫 장 변환에서 segfault, 실측)
  - cv_bridge 를 OpenCV 4.8 로 다시 빌드한 워크스페이스 (~/deps_ws) 를 쓰면, perception 밖 노드 (카메라 · republish) 에서는
    그 cv_bridge 를 LD_LIBRARY_PATH 에서 뺌 → 시스템 OpenCV 4.6 으로 빌드된 패키지와 한 프로세스에 섞이지 않게
  - 시험 전용 고정 TF (odom → d435_color_optical_frame) 는 여기 없음. 로봇 없이 손목 처리를 돌려 볼 때만 따로 띄움
  - navigation 은 bringup 을 exec_depend 함 → 여기서는 package.xml 에 navigation 을 안 넣음 (순환), 실행 때 찾음
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WIDE_BY_ID = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0"


def share(pkg, *path):
    return os.path.join(get_package_share_directory(pkg), *path)


def base_env():
    """perception 밖 노드용 LD_LIBRARY_PATH: 시스템 밖에서 빌드한 cv_bridge 의 lib 를 뺌. 해당 없으면 {}"""
    try:
        prefix = get_package_prefix("cv_bridge")
    except PackageNotFoundError:
        return {}
    if prefix.startswith("/opt/ros/"):
        return {}
    lib = os.path.join(prefix, "lib")
    paths = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p and os.path.normpath(p) != lib]
    return {"LD_LIBRARY_PATH": ":".join(paths)}


def make(context):
    arg = lambda name: context.perform_substitution(LaunchConfiguration(name))  # noqa: E731
    on = lambda name: arg(name).lower() == "true"  # noqa: E731
    env = base_env()
    quality = int(arg("jpeg_quality"))
    out = []
    if arg("dds_transport"):   # 이 뒤에 띄우는 모든 노드에 적용
        out.append(SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", arg("dds_transport")))
    if env:
        out.append(LogInfo(msg=f"cv_bridge = {get_package_prefix('cv_bridge')} → 카메라 · republish 에서는 뺌"))

    # 손목 D435: 컬러 640×480 · 깊이 848×480, 30 Hz (장치 기본값), 깊이를 컬러에 맞춤
    if on("wrist_cam"):
        rs = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("realsense2_camera", "launch", "rs_launch.py")),
            launch_arguments={"align_depth.enable": "true"}.items())
        # rs_launch 안 노드에도 같은 LD_LIBRARY_PATH. scoped 라 perception 쪽 환경은 안 바뀜
        out.append(GroupAction(
            [SetEnvironmentVariable(k, v) for k, v in env.items()] + [rs], scoped=True, forwarding=True))

    # 광각 Arducam: MJPG 1280×720 30 Hz, 내부 파라미터 config/camera_wide.yaml
    if on("wide_cam"):
        dev = os.path.realpath(os.path.expanduser(arg("wide_device")))
        if not os.path.exists(dev):
            raise RuntimeError(f"광각 장치 {arg('wide_device')} 없음 — v4l2-ctl --list-devices 로 확인, wide_device:= 로 지정")
        out.append(Node(
            package="usb_cam", executable="usb_cam_node_exe", name="usb_cam", namespace="wide_cam", output="screen",
            additional_env=env,
            parameters=[{
                "video_device": dev, "image_width": 1280, "image_height": 720, "framerate": 30.0,
                "pixel_format": "mjpeg2rgb", "av_device_format": "YUV420P",
                "camera_name": "wide_cam", "frame_id": "birdeye_camera_optical_frame",
                "camera_info_url": "file://" + share("bringup", "config", "camera_wide.yaml"),
                "..image_raw.compressed.jpeg_quality": quality,   # 이름이 '..' 로 시작함 (ros2 param list 로 확인)
            }]))

    # perception: 설정 = config/perception.yaml (bringup share), 엔진 경로만 인자로
    if on("perception"):
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("perception", "launch", "perception.launch.py")),
            launch_arguments={"wide_model": arg("wide_model"), "wrist_model": arg("wrist_model"),
                              "publish_debug": arg("publish_debug")}.items()))

    # 확인용 디버그 영상 → JPEG (perception 이 raw 로 냄, 1280×720 raw 는 Wi-Fi 로 1 Hz 도 안 나옴)
    if on("perception") and on("publish_debug"):
        for name in ("wrist", "wide"):
            topic = f"/perception/{name}_debug_image"
            out.append(Node(
                package="image_transport", executable="republish", name=f"{name}_debug_republish", output="log",
                additional_env=env,
                parameters=[{"in_transport": "raw", "out_transport": "compressed",
                             "out.compressed.jpeg_quality": quality}],
                remappings=[("in", topic), ("out/compressed", topic + "/compressed")]))

    if on("nav"):
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("navigation", "launch", "nav2.launch.py")),
            launch_arguments={"localization": arg("localization")}.items()))
    if on("mission"):
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share("mission", "launch", "mission.launch.py")),
            launch_arguments={"localization": arg("localization")}.items()))
    return out



def generate_launch_description():
    home = os.path.expanduser("~")
    return LaunchDescription([
        DeclareLaunchArgument("dds_transport", default_value="LARGE_DATA",
                              description="FASTDDS_BUILTIN_TRANSPORTS. 보는 쪽 PC 도 같은 값. '' 면 안 바꿈"),
        DeclareLaunchArgument("wrist_cam", default_value="true", description="손목 D435 realsense2_camera"),
        DeclareLaunchArgument("wide_cam", default_value="true", description="광각 Arducam usb_cam"),
        DeclareLaunchArgument("wide_device", default_value=WIDE_BY_ID, description="광각 장치 (by-id 경로, 실제 장치로 풀어 넘김)"),
        DeclareLaunchArgument("perception", default_value="true"),
        DeclareLaunchArgument("wide_model", default_value=os.path.join(home, "models", "best_wide.engine"),
                              description="광각 TensorRT 엔진 (544×960), 옆에 같은 이름 .names.txt"),
        DeclareLaunchArgument("wrist_model", default_value=os.path.join(home, "models", "best_wrist.engine"),
                              description="손목 TensorRT 엔진 (480×640), 옆에 같은 이름 .names.txt"),
        DeclareLaunchArgument("publish_debug", default_value="false",
                              description="true 면 손목 · 광각 디버그 영상 + Jetson 에서 JPEG 압축"),
        DeclareLaunchArgument("jpeg_quality", default_value="30",
                              description="광각 카메라 · 디버그 영상 JPEG 품질. 광각 95 = 362 KB/장 → Wi-Fi 4.5 Hz, 30 = 85 KB → 27.7 Hz"),
        DeclareLaunchArgument("nav", default_value="false", description="navigation/nav2.launch.py (로봇 · slam 은 run_robot.sh 로 먼저)"),
        DeclareLaunchArgument("mission", default_value="false", description="mission/mission.launch.py"),
        DeclareLaunchArgument("localization", default_value="slam", description="nav · mission 에 넘김: slam | amcl"),
        OpaqueFunction(function=make),
    ])
