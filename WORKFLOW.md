## Static defect generation and visualization on a battery in RVIZ ## 

# Terminal 1
cd ~/projects/MotoBat && source motobat_env.sh
rviz2 -d src/motobat_description/rviz/defects.rviz

with 

# Terminal 2 #all defect classes
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective

or

# Terminal 2 #only welding defects
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_welds --hide cover cmc

or

# Terminal 2 #no defects, perfect welds 
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_clean --hide cover cmc


## Terminal 2 options ##
# Ctrl-C the running show_module.py before starting another; each run clears
# the old markers. Always /usr/bin/python3, never mb-venv (rclpy needs 3.12,
# the venv is 3.11).

# realistic severities (tenths of a mm) -- read them off the boxes and tags
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_realistic

# an earlier stage: pre_busbar | pre_weld | post_weld (default)
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --stage pre_busbar
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --stage pre_weld

# overlay the clean module, translucent blue, to see what moved
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --ghost data/preview_clean

# hide bodies before publishing (or untick them live in RViz)
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --hide cover cmc carrier
#   groups: cooling_plate cells terminals_pos terminals_neg pads end_plates
#           side_plates carrier busbars welds cmc cover fod

# see-through bodies (0 = invisible, 1 = opaque)
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --alpha 0.4

# one cell + its terminals against the Fusion cell_assembled (placement check)
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/cell_check --stage pre_busbar --ghost data/cell_check/reference

# In RViz: Defects -> Namespaces toggles any part/, defect/ or consequence/
# group live. Untick consequence/labels to see the welds without text.


## Regenerate the preview data ##
# data/ is gitignored: run after a fresh clone or any defectgen change

cd ~/projects/MotoBat && mb-venv
python -m defectgen --out data/preview_clean
python -m defectgen --out data/preview_defective --demo-defects
python -m defectgen --out data/preview_realistic --demo-defects realistic
python -m defectgen --out data/preview_welds --demo-defects welds
python -m defectgen --out data/cell_check --cells 0 3

# a different random scatter (spatter positions only)
python -m defectgen --out data/preview_defective --demo-defects --seed 7


## Tests ##

cd ~/projects/MotoBat/defectgen && mb-venv
pytest -p no:launch_testing -p no:launch_ros            # full suite, ~2.5 min
pytest -p no:launch_testing -p no:launch_ros -k weld    # just the weld tests
# -p no:... -- ROS's pytest plugins load when ROS is sourced and break collection


## If RViz shows stale markers ##
# an old show_module.py is still publishing
pgrep -af show_module.py
pkill -f show_module.py
