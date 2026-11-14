# One capture a DefectReport was built from: what it was, and how well it registered.
string name
string sensor
string params_hash
uint32 points
float64 icp_fitness           # fraction of datum points matched
float64 icp_rmse              # m
string[] registered_axes      # axes the datum fixed, e.g. [x, y, yaw]; the rest come from the fixture
KeyValue[] timings            # s, per detector step
