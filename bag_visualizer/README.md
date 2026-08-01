# Bag Visualizer

Plays back a recorded `ros2 bag` (per `rosbag_record.txt`) as an animated
top-down map: robot positions/headings, peer detections, and the four debug
force vectors (avoidance, quark/cohesion, directional, net) from
`swarm_captain`. Two robots are expected — the root robot (no topic prefix)
and `robot1` (`/robot1/...` prefix) — matching the recorded topic layout.

No ROS install required for the viewer itself; bag parsing uses the pure-Python
[`rosbags`](https://pypi.org/project/rosbags/) library.

## Usage

```bash
pip install -r requirements.txt
python3 extract_bag.py /path/to/bag_dir -o data.json
```

If your bag has no embedded type definitions (rosbags will raise
`AnyReaderError: Bag contains no type definitions`), pin a fallback typestore
matching how it was recorded, e.g.:

```bash
python3 extract_bag.py /path/to/bag_dir -o data.json --ros-distro ROS2_HUMBLE
```

Optionally overlay the synthetic temperature field used for gradient-following
testing:

```bash
python3 extract_bag.py /path/to/bag_dir -o data.json \
    --temp-field ../fake_tempratures/wilsons_landing_smaller_multimodal.mat
```

Then serve this folder and open it through **http://**, not by
double-clicking `index.html` — browsers block `fetch()` of local JSON over
`file://` with a CORS error:

```bash
python3 serve.py
# open http://127.0.0.1:8000/index.html
```

Use the "Data" field in the footer to point at a different extracted
`data.json` without re-running the server, then click "Load".

## Notes

- Robot positions come from `.../mavros/global_position/global` (lat/lon),
  converted to a local east/north meter frame relative to the first GPS fix
  seen in the bag.
- Peer detections and force vectors are published in the robot's own body
  frame (x-forward/y-left); the extractor rotates them into the map frame
  using the robot's compass heading so they can be overlaid at each robot's
  absolute position.
- The player resamples all topics onto a fixed-rate timeline (default 10 Hz,
  matching `swarm_captain`'s control rate) using last-value-hold, since the
  underlying topics don't publish in lockstep.
- The "Satellite basemap" layer fetches real imagery tiles from Esri's public
  World Imagery service directly in your browser, aligned to the same GPS
  origin as everything else. It needs internet access at view time (won't
  work fully offline) and isn't loaded until you check the box.
