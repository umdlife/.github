<!-- Please fill out the following pull request template for non-trivial changes to help us process your PR faster and more efficiently.-->

---

## Basic Info

| Info | Please fill out this column |
| ------ | ----------- |
| Ticket(s) this addresses   | (Closes #IssueNumber) |
| Primary OS tested on | (Ubuntu 22.04) |
| Robotic platform tested on | (Gazebo PX4 Simulation, DJIM300 HITL, DJIM300) |

---

## Description of contribution in a few bullet points
<!--
* I added this neat new feature
* Also fixed a typo in a parameter name in nav2_costmap_2d
-->

## Related Pull-Requests
<!--
* Add related PRs links.
-->

## How to run this PR feature

Open the interface and interact with the buttons. Link the robot to the docking station if needed. Create and upload a mission and monitor the behavior. At any point during the mission, verify that all the robot's functionalities are running correctly.

## A new package is added? 

  - [ ] Yes
  - [ ] No

  If yes, please update the following files:
  
  - [ ] repo_where_you_have_your_pkg/docker/Dockerfile
  - [ ] docker/debians/create_debians.sh
  - [ ] docker/debians/rosdep.yaml
  - [ ] .github/workflows/repo_where_you_need_the_pkg_build_image.yaml


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

- [ ] USP ID is present in the PR title 





