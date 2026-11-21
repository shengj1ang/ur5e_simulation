#!/usr/bin/env python3
"""The camera rig and the world-fixed reference, spawned as one static model.

Why a spawned model rather than the Gazebo GUI:

  - the camera pose is numeric and identical on both sides of the comparison,
    which a hand-placed GUI viewpoint is not;
  - Gazebo writes camera frames on the simulation clock, so a real-time factor
    below one stretches the wall-clock recording without changing the frames.
    Screen capture would have to be re-timed afterwards; this does not;
  - nothing of the desktop, the GUI chrome or the mouse ends up in the frame.

Both cameras are aimed at a centre given in world coordinates, from a distance,
an azimuth and an elevation. Azimuth is measured from world +X towards +Y: at
0 the camera looks straight down -X, so a Y disturbance is exactly horizontal
in frame. It is kept small rather than zero because dead-on, this arm points
at the camera and hides its own tool; the horizontal component of a Y motion
is then cos(azimuth), which at 35 degrees is still 0.82.

  wide   the arm and its mount, around 1 mm per pixel. This is where four
         years of difference is legible: the tool either rides with the
         platform or it does not.
  macro  a narrow field across the tool centre, around 0.05 mm per pixel, so
         that one millimetre is tens of pixels. This is the only framing in
         which the dynamics feedforward's own contribution (0.2 to 1.2 mm RMS)
         is larger than a pixel.

The reference rod is a thin static cylinder at the tool's nominal position,
offset towards the camera so it is never hidden by the arm. Without a
world-fixed datum in frame a perspective view of a moving arm is unreadable:
with it, "the tool stays on the rod" and "the tool swings off the rod" are
immediate.

The rod is laid perpendicular to the disturbance: a Y disturbance moves the
tool across the frame, so the datum is a vertical rod; a Z disturbance moves it
up and down, so the datum is a horizontal one. A rod parallel to the motion
would show nothing at all.
"""

import math

WIDTH = 1280
HEIGHT = 720

WIDE = dict(distance=2.40, azimuth=math.radians(35.0), elevation=math.radians(12.0),
            field=1.00)
MACRO = dict(distance=1.20, azimuth=math.radians(35.0), elevation=math.radians(6.0),
             field=0.250)


def _place(centre, distance, azimuth, elevation):
    """Camera pose that looks at `centre` from the given spherical offset."""
    cx, cy, cz = centre
    horizontal = distance * math.cos(elevation)
    x = cx + horizontal * math.cos(azimuth)
    y = cy + horizontal * math.sin(azimuth)
    z = cz + distance * math.sin(elevation)
    yaw = math.atan2(cy - y, cx - x)
    pitch = elevation  # Gazebo's camera looks along +X; positive pitch tips it down.
    return x, y, z, pitch, yaw


def _fov(field, distance):
    return 2.0 * math.atan(0.5 * field / distance)


def _camera(name, centre, spec, frames_dir):
    x, y, z, pitch, yaw = _place(centre, spec["distance"], spec["azimuth"],
                                 spec["elevation"])
    fov = _fov(spec["field"], spec["distance"])
    return f"""      <sensor name="{name}" type="camera">
        <pose>{x:.6f} {y:.6f} {z:.6f} 0 {pitch:.6f} {yaw:.6f}</pose>
        <camera>
          <horizontal_fov>{fov:.6f}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.02</near><far>60</far></clip>
          <save enabled="true"><path>{frames_dir}/{name}</path></save>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
"""


def build(tip, frames_dir, wide_centre=None, rod_axis="z"):
    """SDF for the rig, aimed at a tool centre measured in world coordinates."""
    x, y, z = tip
    centre = wide_centre or (x, y, z - 0.12)
    # Just in front of the tool as the wide camera sees it, so the arm never
    # occludes the datum.
    ax, ay = math.cos(WIDE["azimuth"]), math.sin(WIDE["azimuth"])
    rod = (x + 0.10 * ax, y + 0.10 * ay, z)
    # A cylinder stands along its own Z; roll it a quarter turn to lay it
    # along world Y, which is the datum a vertical disturbance needs.
    rod_rpy = "1.5707963 0 0" if rod_axis == "y" else "0 0 0"
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="video_rig">
    <static>true</static>

    <link name="reference_rod">
      <pose>{rod[0]:.6f} {rod[1]:.6f} {rod[2]:.6f} {rod_rpy}</pose>
      <visual name="rod">
        <geometry><cylinder><radius>0.0016</radius><length>0.24</length></cylinder></geometry>
        <material>
          <ambient>0.9 0.15 0.1 1</ambient>
          <diffuse>0.9 0.15 0.1 1</diffuse>
          <emissive>0.4 0.04 0.02 1</emissive>
        </material>
      </visual>
    </link>

    <link name="cameras">
      <pose>0 0 0 0 0 0</pose>
{_camera("wide", centre, WIDE, frames_dir)}{_camera("macro", (x, y, z), MACRO, frames_dir)}    </link>
  </model>
</sdf>
"""


def scales():
    """Millimetres per pixel for each camera, for captioning the frames."""
    return {name: 1000.0 * spec["field"] / WIDTH
            for name, spec in (("wide", WIDE), ("macro", MACRO))}


if __name__ == "__main__":
    for name, mm in scales().items():
        print(f"{name}: {mm:.4f} mm/px, field {mm*WIDTH:.1f} mm wide")
    print(build((-0.810, 0.134, 1.734), "/tmp/frames"))
