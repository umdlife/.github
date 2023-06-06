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
<!--
* Branches to be checkout
* Instruction to run the docker/command/ blah blah
-->

<details>
    <summary>Run simulation</summary>

```bash
xhost +local:root
docker run -it --rm --net host --privileged --gpus all -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=graphics -e ROBOT_NAME=1 --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix umdlife/umd-simulation-dev:latest roslaunch umd_simulation three_iris_mavros_sitl.launch
```

</details>

<details>
  <summary>Run docker compose</summary>

```yaml
version: "3.4"

services:
  bridge:
    container_name: bridge
    image: umdlife/umd-bridge:v2.0.4
    network_mode: host
    environment:
      ROS_DOMAIN_ID: 60
      NETWORK: local
    command: ros2 run ros1_bridge parameter_bridge /bridge_topics /bridge_service_1_to_2 /bridge_service_2_to_1
  copter100:
    container_name: copter100
    image: umdlife/umd-copter-dev:latest
    network_mode: host
    environment:
      ROBOT_MODEL: iris
      ROBOT_ID: 100/sitl
      ROS_DOMAIN_ID: 60
      NETWORK: local
    command: ros2 launch umd_robot_executor robot_bt_navigator.launch.py run_mode:=sim
  mission:
    container_name: mission
    image: umdlife/umd-mission-dev:latest
    network_mode: host
    environment:
      VPN: disable
      ROS_DOMAIN_ID: 60
      NETWORK: local
    command: ros2 launch umd_mission_core mission_core.launch.py
  database:
    image: postgis/postgis:14-3.3
    container_name: database
    network_mode: host
    restart: always
    environment:
      - POSTGRES_USER=postgres
      - POSTGRES_PASSWORD=postgres
      - POSTGRES_DB=users
    # ports:
    #   - '5432:5432'
  redis:
    image: redis:6.2.5
    container_name: redis
    network_mode: host
    restart: always
    environment:
      - REDIS_PASSWORD=redis
    # ports:
    #   - '6379:6379'
  web-backend:
    container_name: web-backend
    image: umdlife/umd-web-dev:latest
    depends_on:
      - database
      - redis
    network_mode: host
    environment:
        ROS_DOMAIN_ID: 60
        NETWORK: local
        VPN: disable
        # VITE_DISABLE_AUTH: 1
        TOKEN_SECRET: 1234567890
        VITE_MAPBOX_URL: pk.eyJ1IjoiZ25leWhhYnViIiwiYSI6ImNsYmk4MjMyZTA3Y2Ezbm9kYm5keTVhNDQifQ.61SwBpBAbsm3r87qdHBsZQ
        VITE_BASE_URL: http://localhost:8081
        VITE_SOCKET_URL: ws://localhost:8081
        DATABASE_URL: postgresql://postgres:postgres@localhost:5432/umd_web_database?schema=public
        REDIS_URL: redis://localhost:6379
    volumes:
      - type: bind
        source: /home/unmanned/umd2_ws/src/umd_mission/umd_utils/cfg
        target: /umd2_ws/install/umd_utils/share/umd_utils/cfg
   

```

</details>
 Open the interface in the browser and execute a mission. Interact with all the buttons to make sure it works as before.

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





