# One feature's measurement, found or not (defectgen DefectReport.measurements).
# kind: terminals | cells | polarity | adjacent_dh | busbars | gaps | carrier | seams
string kind
string id                     # cell index, pin, busbar index ...; "" for one-off features (carrier)
string capture
bool seen
KeyValue[] values             # deviations from nominal; list entries flattened as key[0], key[1] ...
