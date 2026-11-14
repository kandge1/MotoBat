# What one sensor returned in one capture, and nothing else (defectgen.scan.ScanResult,
# without the ground truth the simulator also keeps). One message per capture.
#
# Frame: header.frame_id is the module (pallet datum) frame, Z up.
# Units: metres, radians (REP 103). defectgen works in mm; convert at the boundary.

std_msgs/Header header

string name                   # capture name, e.g. "pre_busbar_line_scan"
string sensor                 # sensor model: line_scan | snapshot | profilometer
string params_hash            # sha256 of the frozen sensor parameters (task 1.3)

# Organised cloud: height x width = the sensor's own ray grid (rows x columns), so the ray
# index is implicit in the position. A ray that returned nothing is NaN.
# Fields: x y z (float32, m), signal (float32, returned light, white diffuse = 1),
# sigma (float32, m: the 1-sigma noise along the measuring direction; coverage uses it).
sensor_msgs/PointCloud2 points

# Sensor pose (sensor -> module) per profile (line scan, profilometer) or per shot (snapshot).
geometry_msgs/Pose[] sensor_poses

# How the sensor was driven (speed, pitch, exposure, seed ...), and what was scanned
# (stage, run). Free-form JSON: these differ per sensor and are logged, not computed on.
string settings_json
string source_json
