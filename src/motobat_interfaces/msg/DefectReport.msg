# What an inspection station tells the line about one module at one stage
# (defectgen.detect.DefectReport).

uint8 OK=0
uint8 NOK=1                   # at least one finding
uint8 INCOMPLETE=2            # nothing found, but some feature was never seen: not a pass

string PRE_BUSBAR=pre_busbar  # station 6: cell pose, terminal height and tilt, polarity
string PRE_WELD=pre_weld      # station 8: busbar pose, busbar-to-terminal gap
string POST_WELD=post_weld    # station 10: seams, spatter
string S1_PLACEMENT=s1_placement  # step 3: each cell measured by the S1 stack camera after placing (task 3.4)

std_msgs/Header header        # frame_id: module (pallet datum) frame
string module_id
string stage
uint8 result

Finding[] findings
Measurement[] measurements
Uncovered[] uncovered
CaptureInfo[] captures
string detect_hash            # sha256 of the frozen detector parameters
