## Delivery 2 (Dec 2023) - Hamid Ebadi
- TODO

## Jackal Robot (Mar 2024) - Christoffer Johannesson

- Added dyno fork of jackal repo from Clearpath Robotics.
- Updated to Humble, added bringup and support for namespacing. Jackal can be spawned in Gazebo and controlled through the keyboard.
- Added .devcontainer folder with Dockerfile and devcontainer.json to set up project container in VS Code.
- Added docker-compose to link all needed files and set environment variables.
- Added .vscode folder with settings and tasks for easy building of the project.
- Updated README with info on how to use Docker setup in VS Code, and some features to make it easy to share the same setup with others.
- Features includes: python3 dependency install with pip, cloning of other git repositories and how to make changes to those repositories.

## Delivery 1 (Dec 2023) - Supervisor: Hamid Ebadi

- Basic warehouse model 1.0.0: Anders Bäckelie
- CAD modelling (eur-pallet, boxes, shelf, support_pole, traffic-cone, steel_drum) 1.0.0 : Jacob Rohdin
- Physics (collision, inertia), visuals and gazebo compatible mesh creation 1.0.0: Anders Bäckelie
- Walking actor using scripted trajectories 1.0.0 : Anders Bäckelie
- Infobot_gazebo_environment 1.0.0: ROS2 launcher to start Gazebo world : Hamid Ebadi
- static_agent_launcher 1.0.0: Camera and Aruco tags : Hamid Ebadi
- camera-viewer 1.0.0: Python code to get Gazebo camera feed : Hamid Ebadi

### Experimental features:
- Volvo warehouse 0.0.1 : Hamid Ebadi
- Volvo camera calibration in Gazebo 0.0.1 : Hamid Ebadi
- Integrate Infobot_agent 0.0.2: InfoBot differential-drive AMR (Autonomous Mobile Robot) URDF and ROS launcher (GOPAL and forklift): Hamid Ebadi
- Integrate Infobot_cartographer 2.1.5: cartographer for creating PGM maps
- Integrate nav2_commander 0.0.2: ROS package to command Infobot where the destination is : Hamid Ebadi
- Integrate Infobot_navigation2 2.1.5: Standard Nav2 stack launcher : Hamid Ebadi
- Integrate Infobot_teleop 0.0.2: Teleoperation for InfotBot