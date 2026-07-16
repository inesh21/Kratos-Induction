'''Induction Assignment 4: Autonomous Waypoint Navigation &

OpenCV ArUco Search
Project Kratos – Autonomous Subsystem

Mission Briefing
Welcome back to the Autonomous Subsystem of Project Kratos.
In the previous assignments, you drove the TurtleBot using manual teleoperation and basic geometry

scripts. Now, we are raising the stakes. The rover needs to navigate a hazardous environment au-
tonomously, locate scattered targets, and log their identities.

Your job this week: write a ROS 2 node that reads a series of rough search zone coordinates from
aruco_waypoints.yaml. Your node must use the Nav2 navigation stack to drive the rover to each
coordinates. Since these waypoints only represent rough coordinates, once the rover arrives at a waypoint,
your node must switch to a visual search routine (e.g., rotating or executing a scanning pattern). Using
the robot’s onboard RGB camera feed and OpenCV, you must find the nearby vertical ArUco marker
(a cube with marker texture on all sides), detect its ID, record the ID and the robot’s coordinates at the
moment of detection, and then proceed to the next waypoint.
The pipeline you are implementing:
Read YAML Waypoints –> Nav2 Go-to Goal –> Visual Search Rotation –> OpenCV ArUco Detection
–> Save Log (.txt)
Objective
The goal of this assignment is to develop an autonomous search-and-identify script that coordinates
high-level path planning (using Nav2) with local vision-based target acquisition (using OpenCV ArUco
detection) and file reporting.
Topics Covered
• Parsing configuration files in Python (pyyaml)
• ROS 2 Action Clients (nav2_msgs/action/NavigateToPose)
• Subscription to camera 1 topics (sensor_msgs/msg/Image)
• ROS 2 image processing with cv_bridge and OpenCV
• ArUco marker detection (cv2.aruco)
• State machine coordination / logical transitions in Python
Setup Requirements
Verify the following setup before writing your code:

SIMULATOR UPDATE: You must run git pull inside your local cloned genesis_sim di-
rectory to receive the latest updates. This contains the updated simulation configurations, 3D

ArUco marker assets, obstacle cubes, and the waypoints layout.
1. Ubuntu 22.04 LTS (or compatible Linux distribution)
2. ROS 2 Humble Hawksbill
3. Genesis Physics Engine (version 1.1.0 or newer)
4. Genesis ROS Bridge (genesis_ros) — built and sourced inside ~/ros2_ws
5. Genesis TurtleBot Simulator (genesis_sim) — updated via git pull
6. Python packages for OpenCV and YAML parsing:

Assignment 4: Autonomous Waypoint Navigation & OpenCV ArUco Search 1

Induction 2026 Project Kratos – Autonomous Subsystem

pip install opencv-python opencv-contrib-python pyyaml
Assignment Tasks
You are required to complete the following tasks:
1. YAML Parsing & Nav2 Navigation
Write a script that reads the waypoints coordinates from the aruco_waypoints.yaml file.
1a. YAML Parsing
• Read and parse the rough target positions (x, y) from aruco_waypoints.yaml.
• Print the coordinates of all target waypoints to the console on startup.
1b. Nav2 Action Client
• Initialize an Action Client for nav2_msgs/action/NavigateToPose.
• Send the coordinates to Nav2 to drive the robot to the first waypoint.
• Monitor feedback and wait until Nav2 reports that the robot has arrived at the goal pose before
triggering the search state.
2. Visual Search & OpenCV ArUco Detection
Once Nav2 arrives at a waypoint, the robot must search for the nearby ArUco tag.
2a. Visual Search Routine
• Implement a local searching state (for example, rotate the robot slowly by publishing low angular
velocities on /cmd_vel).
• The robot must scan the area until the onboard camera sees the ArUco marker.
2b. ArUco Marker Detection & Approaching
• Subscribe to the RGB camera feed on the topic /turtlebot/rgb.
• Use cv_bridge to convert the incoming ROS Image messages into OpenCV-compatible images.
• Detect ArUco markers using OpenCV’s ArUco module with the dictionary configuration DICT_4X4_1000
(Various IDs are placed in the scene).
• Once detected, drive the robot forward to stop exactly 0.25 meters away from the ArUco
tag (e.g., using visual servoing, estimated marker distance, or simple camera geometry metrics).
3. Data Logging & Execution
Record the findings of the mission.
3a. Save Target Logs
• For each discovered marker, capture its ID and the current (x, y) coordinates of the robot (read
from the /odom topic or /tf).
• Write this information in real-time to a file named detected_targets.txt. The file must append
each new target as it is found:
Target found! ID: 67, Rover Position: (2.48, 1.76)
Target found! ID: 69, Rover Position: (-1.95, 2.18)
Target found! ID: 88, Rover Position: (0.97, -2.44)
3b. Multi-Goal Sequencing
• Once the target is found and logged, transition back to the navigation state and dispatch the next
waypoint coordinates to Nav2.
• Repeat until all targets listed in the YAML configuration are found and logged.'''

'''Step 1: Detect the markers

You already have

corners, ids, rejected = cv2.aruco.detectMarkers(
    image,
    self.aruco_dict,
    parameters=self.parameters
)

Good.

Now ask yourself:

What if no marker is visible?

Then

ids

will be

None

So immediately after detection add

if ids is None:
    return None

Now your function becomes

def detect_markers(self, image):

    corners, ids, rejected = cv2.aruco.detectMarkers(
        image,
        self.aruco_dict,
        parameters=self.parameters
    )

    if ids is None:
        return None

Test this mentally:

No marker

↓

Return None

Mission node knows nothing was found.

Step 2: Get the marker ID

Suppose

ids

looks like

[[67]]

OpenCV stores IDs as a NumPy array.

You only want

67

So add

marker_id = int(ids[0][0])

Now

marker_id

contains

67
Step 3: Get the corners

This is the part most beginners find confusing.

Suppose OpenCV found one marker.

Then

corners

looks something like

[
 [
   [100,200],
   [180,200],
   [180,280],
   [100,280]
 ]
]

The first [ ] is

list of detected markers

The second [ ] is

the four corners of one marker

So get the first marker

marker_corners = corners[0]

Now

marker_corners

contains only

[
 [100,200],
 [180,200],
 [180,280],
 [100,280]
]
Step 4: Compute the center

A marker has four corners.

(0)------(1)
 |        |
 |        |
(3)------(2)

The center is simply

Average X

Average Y

NumPy makes this easy.

Add

center = np.mean(marker_corners[0], axis=0)

Let's understand this.

Suppose

marker_corners[0]

=

[
 [100,200],
 [180,200],
 [180,280],
 [100,280]
]

Average x

100+180+180+100
---------------
      4

=140

Average y

200+200+280+280
---------------
      4

=240

So

center

becomes

[140,240]
Step 5: Store center x and y

Now split it.

center_x = int(center[0])
center_y = int(center[1])

Now you have

center_x

140

and

center_y

240
Step 6: Compute marker width

Why do we need width?

Because

Far marker

↓

Small

Near marker

↓

Big

We'll use this later to stop 25 cm away.

The easiest method

Measure top edge

(0)------(1)

So

top_left = marker_corners[0][0]
top_right = marker_corners[0][1]

Now compute distance.

Use

width = np.linalg.norm(top_right - top_left)

Suppose

Top Left

(100,200)

Top Right

(180,200)

Distance

80 pixels

That's your width.

Step 7: Return everything

Now your function finally has

marker_id

center_x

center_y

width

marker_corners

So

return {
    "id": marker_id,
    "center_x": center_x,
    "center_y": center_y,
    "width": width,
    "corners": marker_corners
}
Final flow inside the detector
Image
   │
   ▼
detectMarkers()
   │
   ▼
No marker?
   │
 ┌─Yes──► return None
 │
 ▼
Marker found
   │
   ▼
Get ID
   │
   ▼
Get corners
   │
   ▼
Compute center
   │
   ▼
Compute width
   │
   ▼
Return dictionary
One small improvement

You're using the older OpenCV ArUco API:

cv2.aruco.Dictionary_get()
cv2.aruco.DetectorParameters_create()
cv2.aruco.detectMarkers()'''

import cv2
import numpy as np


class ArucoDetector:

    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_1000):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)

    def detect_markers(self, image):
        cv2.imshow("Robot Camera", image)
        cv2.waitKey(1)
        corners, ids, rejected = self.detector.detectMarkers(image)
        # No marker found
        if ids is None:
            return None

        # First detected marker
        marker_id = int(ids[0][0])
        marker_corners = corners[0]

        # Calculate marker center
        center = np.mean(marker_corners[0], axis=0)
        center_x = int(center[0])
        center_y = int(center[1])

        # Calculate marker width (top edge)
        top_left = marker_corners[0][0]
        top_right = marker_corners[0][1]
        width = np.linalg.norm(top_right - top_left)

        return {
            "id": marker_id,
            "center_x": center_x,
            "center_y": center_y,
            "width": width,
            "corners": marker_corners
        }