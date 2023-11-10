<!-- Please fill out the following pull request template for non-trivial changes to help us process your PR faster and more efficiently.-->

---

## Basic Info

| Info | Please fill out this column |
| ------ | ----------- |
| Ticket(s) this addresses   | (Closes #IssueNumber) |
| Primary OS tested on | (Ubuntu, MacOS, Windows) |
| Robotic platform tested on | (Steve's Robot, gazebo simulation of Tally, hardware turtlebot) |

---
## How to run this PR feature

<details>
    <summary>Run simulation</summary>

```bash
xhost +local:root
docker run -it --rm --net host --privileged --name simulation --gpus all --env DISPLAY=$DISPLAY --env-file simulation.env --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix umdlife/umd-simulation-dev:ros2 ros2 launch umd_simulation ros2_iris_mavros.launch.py
```

</details>

<details>
  <summary>Run docker compose (uncomment docking service if needed)</summary>

```yaml
version: "3.4"
services:
  copter100:
    container_name: copter100
    image: umdlife/umd-copter-dev:latest
    network_mode: host
    env_file:
      - ./simulation.env
    command: ros2 launch umd_robot_executor robot_bt_navigator.launch.py run_mode:=sim
  mission:
    container_name: mission
    image: umdlife/umd-mission-dev:latest
    network_mode: host
    env_file:
      - ./simulation.env
    command: ros2 launch umd_mission_core mission_core.launch.py
  database:
    image: postgis/postgis:14-3.3
    container_name: database
    network_mode: host
    restart: always
    env_file:
      - ./simulation.env
    healthcheck:
      test: pg_isready -U postgres -d postgres
      interval: 10s
      timeout: 3s
      retries: 3
  backend:
    container_name: backend
    image: umdlife/umd-web-dev:latest
    depends_on:
      database:
        condition: service_healthy
    network_mode: host
    env_file:
      - ./simulation.env
    environment:
      NETWORK: local_old
  # docking100:
  #   container_name: docking100
  #   image: umdlife/umd-docking-dev:latest
  #   network_mode: host
  #   env_file:
  #     - ./simulation.env
  #   environment:
  #     ROBOT_MODEL: heisha_d135
  #     ROBOT_ID: 100
  #     DEVICE_NAME: sfkKrZ40sdOSwVIAdq8N
  #   volumes:
  #     - ./heisha_params.yaml:/umd2_ws/install/umd_docking_hal/share/umd_docking_hal/config/heisha_params.yaml
```

</details>

<details>
  <summary>Environmental variables: simulation.env file</summary>

```
#GENERAL
ROS_DOMAIN_ID=90
ROBOT_MODEL=iris
ROBOT_ID=100
NETWORK=local
DDS_IP_1=localhost
DDS_IFACE=auto
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
DDS_VERBOSITY=info

#WEB - POSTGRESS
POSTGRES_USER=postgres
POSTGRES_PASSWORD=postgres
POSTGRES_DB=users

#WEB - BACKEND
TOKEN_SECRET=1234567890
VITE_MAPBOX_URL=pk.eyJ1IjoiZ25leWhhYnViIiwiYSI6ImNsbmlvbW13ajEzeGMycW1pZzN0cnhvengifQ.Lrk1Nk5Ln0EfubS5GfhOQQ
VITE_BASE_URL=http://localhost:8081
VITE_SOCKET_URL=ws://localhost:8081
DATABASE_URL=postgresql://postgres:postgres@localhost:5432/umd_web_database?schema=public
VPN=disable

#SIMULATION 
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=graphics
ROBOT_NAME=1

```
</details>

<details>
  <summary>Params for docking: heisha_params.yaml file</summary>

```yaml
/**:

  ros__parameters:
    mqtt_server_ip: '127.0.0.1'
    mqtt_server_port: 1883
    topic_header: 'heisha/dnest'
    login_topics: ['client/login','server/login_reply']
    status_topics: ['client/post','server/post_reply']
    service_topics: ['client/service_reply','server/service']
    heartbeat_topics: ['client/heartbeat', 'server/heartbeat_reply']
    heartbeat_timeout_sec: 3600

```
</details>

Open the interface and interact with the buttons. Link the robot to the docking station if needed. Create and upload a mission and monitor the behavior. At any point during the mission, verify that all the robot's functionalities are running correctly.

## Description of contribution in a few bullet points

<!--
* I added this neat new feature
* Also fixed a typo in a parameter name in nav2_costmap_2d
-->

## Related Pull-Requests

<!--

* Add related PRs links.
  -->

## Description of documentation updates required from your changes

<!--
* Added new parameter, so need to add that to default configs and documentation page
* I added some capabilities, need to document them
-->

---

## Future work that may be required in bullet points

<!--
* I think there might be some optimizations to be made from STL vector
* I see alot of redundancy in this package, we might want to add a function `bool XYZ()` to reduce clutter
* I tested on a differential drive robot, but there might be issues turning near corners on an omnidirectional platform
-->

## Review Check

- [ ] There are no conflicts with target branch.

- [ ] Description is intelligible and well written.

- [ ] Passes the CI tests.

- [ ] README.md is updated.  





