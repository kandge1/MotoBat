# One measured number, with the unit it was measured and judged in.
# Values keep the detector's own units (mm, deg) so they compare directly with the
# tolerances in defectgen/configs/detect.yaml; only typed geometry (Point, Pose,
# PointCloud2) is converted to SI, per REP 103. unit "1" = a count or a fraction.
string key
float64 value
string unit
