# One defect the detector reports (defectgen.detect.Finding).

# Class names, as in defectgen/configs/dataset.yaml.
string CELL_PLACEMENT=cell_placement
string MISSING_CELL=missing_cell
string REVERSED_POLARITY=reversed_polarity
string TERMINAL_HEIGHT=terminal_height
string TERMINAL_TILT=terminal_tilt
string BUSBAR_POSE=busbar_pose
string BUSBAR_LIFTOFF=busbar_liftoff
string CARRIER_SEATING=carrier_seating
string MISSING_WELD=missing_weld
string SHORT_WELD=short_weld
string SEAM_DEVIATION=seam_deviation
string SPATTER=spatter
string FOD=fod
string ANOMALY=anomaly        # a residual region no rule could name

string cls
float64 magnitude             # in `unit`: mm, deg, mm2, or 1 (a count or a fraction)
string unit
geometry_msgs/Point location  # m, module frame
geometry_msgs/Point bounds_min
geometry_msgs/Point bounds_max
string capture                # which capture it was found in
string feature                # e.g. "cell 3", "busbar 4", "seam 2 (busbar 0, pin 2)"
KeyValue[] measured           # the numbers behind the finding
