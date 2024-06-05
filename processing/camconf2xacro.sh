echo "<!-- Please update simulation/static_agent_launcher/description/camera_config.xacro with these camera xacros  -->"
echo "<!-- =================================== -->"
echo "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">"
python3  cameraCalibration.py intrinsic/160.yaml extrinsic/160.yaml 160
python3  cameraCalibration.py intrinsic/164.yaml extrinsic/164.yaml 164

echo "</robot>"