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
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_welds --hide cover

or

# Terminal 2 #no defects, perfect welds 
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_clean --hide cover


## More things Terminal 2 can show ##

# Only one show_module.py at a time. Ctrl-C the one that is running, then
# start the next -- it wipes the old markers out of RViz by itself.
# Always /usr/bin/python3 here, never mb-venv: ROS's rclpy is built for
# Python 3.12 and the venv is 3.11, so it will not import inside the venv.

# the same defects at the size a real line makes them (tenths of a mm).
# Too small to see by eye -- read them off the boxes and labels instead.
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_realistic

# the module at an earlier step of the line. The default is post_weld.
#   pre_busbar = cells, terminals and housing only
#   pre_weld   = busbars placed, not welded yet
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --stage pre_busbar
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --stage pre_weld

# lay a see-through blue copy of the perfect module over the defective one,
# so anything that moved shows up as a step between the two
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --ghost data/preview_clean

# leave parts out from the start (you can also untick them live in RViz)
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --hide cover cmc carrier
#   parts you can hide: cooling_plate cells terminals_pos terminals_neg pads
#                       end_plates side_plates carrier busbars welds cmc cover fod

# make every part see-through (0 = invisible, 1 = solid)
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective --alpha 0.4

# placement check: one normal cell and one flipped cell with their terminals,
# overlaid on the cell_assembled from Fusion. If they don't line up, the
# generator is placing terminals wrong.
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/cell_check --stage pre_busbar --ghost data/cell_check/reference

# In RViz, under Defects -> Namespaces, every part (part/...), every defect
# type (defect/...) and every weld result (consequence/...) has its own tick
# box. Untick consequence/labels to see the welds without the text.


## Preliminary sensor scans (task 1.3) ##
# Simulated sensor point clouds of a generated module, shown on top of it.
# Post-weld scans leave the CMC and cover off: inspection happens before
# they go on.
#
# Four terminals. Terminals 1 and 2 are exactly the ones from the top of
# this file -- RViz, and the module you want to scan. Terminal 3 runs the
# scan, which is a one-off: it writes a file and finishes. Terminal 4 puts
# that file into RViz on top of the module.
#
# Scan the same module Terminal 2 is showing (data/preview_welds below),
# otherwise the cloud won't line up with the parts.

# Terminal 1 #start rviz
cd ~/projects/MotoBat && source motobat_env.sh
rviz2 -d src/motobat_description/rviz/defects.rviz

# Terminal 2 #the module to scan, here the welding defects
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_welds --hide cover cmc

# Terminal 3 #run the scans
# This one DOES use the venv (mb-venv): the scanner needs Open3D, which only
# lives in the venv. Pick whichever sensors you want. Each one prints what it
# is doing as it goes: the sensor settings, the scan plan, a progress line per
# batch (how many rays, how many points kept, what the rest were lost to),
# then a table by surface material: points kept, noise, how much light came
# back, and why points were lost (miss, range, grazing, shadow, dark, glare).
cd ~/projects/MotoBat && source motobat_env.sh
mb-venv                                  # alias from motobat_env.sh
# or, if motobat_env.sh isn't sourced in this terminal:
source ~/motobat/venv/bin/activate
# prompt should now start with (venv). "Command 'python' not found" means you
# skipped this: Ubuntu has no system `python`, only the venv provides it.
python -m defectgen.scan data/preview_welds --sensor line_scan      # the fixed Ranger3 station: the whole module passes under it
python -m defectgen.scan data/preview_welds --sensor profilometer   # the Keyence on the robot wrist: one sweep along the +Y busbar row
python -m defectgen.scan data/preview_welds --sensor snapshot       # the Zivid on the robot wrist: one 3D picture from above
python -m defectgen.scan data/preview_welds --sensor photos         # the Zivid's 2D camera: four close-up photos (see below)
# the three 3D sensors each write data/preview_welds/scans/post_weld_<sensor>.npz
# (and a .ply you can open in MeshLab; the snapshot also writes
# post_weld_snapshot_2d.png, its one wide 2D picture).

# --sensor photos: one 2D photo per quadrant -- half the module's length on
# one terminal row, 6 terminals each. Terminals are numbered like a 24-pin
# chip: looking down with the cell-0 end at the top, T1 is top-left, T1-T12
# run down the left row and T13-T24 back up the right. The camera is centred
# between T3/T4, T9/T10, T15/T16 and T21/T22, from as close as the Zivid can
# work (300 mm), at its full 2448 x 2048 resolution. Each photo is cropped
# to its quadrant, defects are boxed, every terminal is labelled T1..T24, and
# the four are stitched into one picture of the whole module. About 25 s.
#   --no-stitch           skip the stitched picture
#   --include cmc cover   put the CMC and cover back in (post-weld inspection
#                         happens before they go on, so they are left off)
# Everything lands in one new folder,
# data/scan_logs/<date>_<time>_preview_welds_post_weld_photos/
#   photos/q1_pins_1-6.png ... q4_pins_19-24.png    the four quadrant photos
#   photos/stitched.png                             the whole module, stitched
#   annotated/                                      all five, defects boxed, terminals labelled
#   defects/NN_<defect>.png                         one zoomed close-up per defect
# Open the latest set in an image viewer to zoom in:
xdg-open "$(ls -d data/scan_logs/*_photos | tail -1)/annotated/"
# Resolution is the real camera's: 0.12 mm per pixel at 300 mm, so a quadrant
# photo is about 720 x 1480 pixels. The pictures are a display render, not a
# frozen sensor model -- good for looking, not for measuring.

# speed knob for the two line sensors: how fast the robot sweeps (or the
# conveyor moves, for line_scan) in mm/s, and how many profiles the sensor
# fires per second. Points along the sweep land speed/rate mm apart, so
# slower = denser, and the log prints how long the real scan would take.
python -m defectgen.scan data/preview_welds --sensor profilometer --speed 50  --rate 1000   # 0.05 mm apart (the default spacing)
python -m defectgen.scan data/preview_welds --sensor profilometer --speed 200 --rate 1000   # 4x faster, 4x sparser
python -m defectgen.scan data/preview_welds --sensor line_scan    --speed 100 --rate 1000   # conveyor at 100 mm/s
# Without --speed/--rate, the spacing comes from the frozen sensor config.
# Faster isn't free: firing more profiles per second means each one is
# exposed for less time, so less light comes back -- watch "dark" and the
# noise column climb on copper and welds. The log prints the exposure. Each
# sensor has a maximum rate (Keyence 16000/s, Ranger3 7000/s) and refuses
# more. The motion itself is perfect for now -- no robot vibration yet.

# height knob for the two line sensors: how far above the terminal tops the
# sensor sits, in mm (default: its reference distance -- 245 for the Keyence,
# 500 for the Ranger3 station). Farther = wider line but coarser points and
# more noise (noise grows with distance squared). The Keyence only measures
# within 245 +/- 34 mm; outside that, the log warns and points drop as "range".
python -m defectgen.scan data/preview_welds --sensor profilometer --distance 215   # close: fine and quiet
python -m defectgen.scan data/preview_welds --sensor profilometer --distance 275   # far: wider, noisier

# other options: --stage pre_busbar|pre_weld, --seed N for different noise,
# --out somewhere/else.npz, --quiet for just the last line

# Every run also gets its own folder in data/scan_logs/, kept as a history:
#   data/scan_logs/<date>_<time>_<run>_<stage>_<sensor>/
#     scan.log       everything it printed
#     summary.json   the numbers: settings, sensor params hash, points kept
#                    and lost per reason and per material
# (the point clouds themselves stay in data/<run>/scans/, they are big)

# Terminal 4 #show the scans in rviz, on top of the module
# Back to system python (no mb-venv) -- same reason as Terminal 2.
# Each sensor shows up as its own display in RViz's left panel:
# "Line scan (Ranger3)", "Profilometer (LJ-X8200)", "Snapshot (Zivid)", plus
# "Zivid 2D", the Zivid's one wide picture, and "Zivid photos (stitched)",
# the four quadrant photos stitched -- shown automatically when
# --sensor photos has been run. Tick and untick them to compare.
# (Restart RViz from Terminal 1 once after updating, to pick these up.)
cd ~/projects/MotoBat && source motobat_env.sh

# one sensor
/usr/bin/python3 src/motobat_description/scripts/show_scan.py data/preview_welds/scans/post_weld_profilometer.npz

# all three at once, on top of the module (toggle them in RViz)
/usr/bin/python3 src/motobat_description/scripts/show_scan.py data/preview_welds/scans/post_weld_*.npz

# all three next to each other instead of stacked: line scan on one side,
# profilometer in the middle (on the module), snapshot on the other side
/usr/bin/python3 src/motobat_description/scripts/show_scan.py data/preview_welds/scans/post_weld_*.npz --side-by-side

# To change what is shown, Ctrl-C and run it again.
#   --colour height     (default) rainbow by height -- these colours are NOT the
#                       parts, every dot is a measured point coloured by how high it is
#   --colour material   one colour per surface it landed on: copper, aluminium, plastic, ...
#   --colour signal     how much laser light came back to the camera:
#                       blue = barely enough (nearly dropped), red = nearly glare
#   --max 2000000       show at most this many points per sensor (RViz slows above ~2 M)
#
# What you are looking at: each dot is one simulated measurement. The grain is
# sensor noise. The dark outline around terminals and busbars is triangulation
# shadow -- spots the laser lit but the camera could not see, so the sensor
# returns nothing there. Untick "Defects" to see the clouds on their own.
#
# The Zivid 2D picture is also saved as a file you can open directly:
#   data/preview_welds/scans/post_weld_snapshot_2d.png
# The washed-out white on the metal under the camera is glare -- the same
# glare that drops 3D points there.


## Detection (task 1.4) ##
# Point clouds in, a list of defects out, scored against the labels the
# generator wrote. Detection itself needs only the venv; RViz is for looking
# at the result. Same four terminals as the scans above:
#   Terminal 1  RViz, as always
#   Terminal 2  the module, at the stage being inspected
#   Terminal 3  scan, then detect (venv)
#   Terminal 4  what detection found, boxed on the module (NEW: show_detections.py)
#
# Use the matching --stage everywhere: pre_busbar finds cell, terminal,
# polarity, pad and FOD defects; pre_weld busbar pose, busbar gap and carrier
# seating; post_weld the welds and spatter.

# Terminal 1 #start rviz (restart it once after updating, to get the new "Detections" display)
cd ~/projects/MotoBat && source motobat_env.sh
rviz2 -d src/motobat_description/rviz/defects.rviz

# Terminal 2 #the module being inspected, here the weld demo
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_welds --hide cover cmc
# for the clean check instead:
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_clean --stage pre_busbar

# Terminal 3 #scan, then detect
cd ~/projects/MotoBat && source motobat_env.sh && mb-venv
python -m defectgen.scan data/preview_clean --stage pre_busbar --sensor line_scan
python -m defectgen.detect data/preview_clean --stage pre_busbar     # a clean module: expect no findings
python -m defectgen.scan data/preview_welds --stage post_weld --sensor line_scan
python -m defectgen.detect data/preview_welds --stage post_weld      # the weld demo, scored

# Terminal 4 #what detection found, on top of the module
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_detections.py data/preview_welds --stage post_weld
# One box per finding (findings on the same part share one), with a tag:
#   green   found        right class, right place
#   red     false alarm  nothing there
#   amber   knock-on     a finding on a defect of another class (see "other" below)
#   magenta MISSED       the truth box of a defect it did not find; the tag says
#                        what, if anything, was found there instead
#   cyan    unscored     the run has no labels
# Untick "Defects" (Terminal 2's truth boxes) to see only what detection said.
# Under Detections -> Namespaces, each colour toggles on its own.
# show_scan.py still works exactly as before (Terminal 4 of the scan section),
# in another terminal at the same time if you want the cloud too:
/usr/bin/python3 src/motobat_description/scripts/show_scan.py data/preview_welds/scans/post_weld_line_scan.npz

# Which scans it uses: every data/<run>/scans/<stage>_*.npz that is a scan of
# the module as it is now. A scan is skipped, with a line saying why and the
# command that rescans it, when it was saved before 6 Nov 2026 (no ray index),
# is of another stage, or is of the module before it was last regenerated.
# Scans also record --include, so detection compares against the same parts.
# Scan with more than one sensor and each feature is measured by whichever
# capture saw it best. Or name them (then a stale one is an error, not a skip):
python -m defectgen.detect data/preview_welds --stage post_weld \
    --scans data/preview_welds/scans/post_weld_profilometer.npz

# What it prints:
#   the alignment per capture (what it registered to: the housing),
#   one line per finding: class, size (mm or deg), which feature, where, which capture,
#   anything no capture saw ("not seen by any capture") -- not passed, just not looked at,
#   and, if <stage>.labels.json is there, a table per class:
#     TP     found, right class, right place
#     FN     a defect it should have found and did not ("missed as" says what it
#            found there instead, if anything)
#     FP     a finding with no defect under it
#     other  a finding on a defect of another class -- usually a real knock-on
#            effect, e.g. busbars riding high on a carrier a reversed cell lifted
# The full report goes to data/<run>/detections/<stage>.report.json: every
# measurement (each terminal's height, each cell's dx/dy/yaw, each gap, each
# seam's offset/bow/width), not just the findings. The scoring, finding by
# finding and label by label, goes next to it as <stage>.evaluation.json --
# that is what Terminal 4 draws.
#
# The demo modules stack many defects on purpose; the realistic one has a
# reversed cell that lifts the carrier ~3 mm and spoils every weld after it.
# One defect per module is what a detector is judged on (task 1.7 builds that
# dataset); the demos are for looking at.
# Thresholds: defectgen/src/defectgen/configs/detect.yaml (tagged S/D/A).


## Regenerate the preview data ##
# Everything under data/ is generated, not in git. Run these after a fresh
# clone, or after any change to defectgen, or the viewers show old modules.
# Uses the venv.

cd ~/projects/MotoBat && source motobat_env.sh && mb-venv
python -m defectgen --out data/preview_clean                                # no defects
python -m defectgen --out data/preview_defective --demo-defects             # every defect class, exaggerated
python -m defectgen --out data/preview_realistic --demo-defects realistic   # every class, real-line sizes
python -m defectgen --out data/preview_welds --demo-defects welds           # weld defects only
python -m defectgen --out data/cell_check --cells 0 3                       # the placement check

# same defects, different random scatter (only the spatter moves)
python -m defectgen --out data/preview_defective --demo-defects --seed 7


## Tests ##
# Run these after changing anything in defectgen. Uses the venv.

cd ~/projects/MotoBat/defectgen && mb-venv
pytest -p no:launch_testing -p no:launch_ros            # everything, a few minutes
pytest -p no:launch_testing -p no:launch_ros -k weld    # only the weld tests
pytest -p no:launch_testing -p no:launch_ros -k scan    # only the sensor tests
pytest -p no:launch_testing -p no:launch_ros tests/test_detect.py   # only detection
# The -p no:... flags matter: with ROS sourced, pytest picks up ROS's own
# test plugins and dies before running anything.


## If RViz shows old markers or an old point cloud ##
# Something from an earlier run is still publishing. Find it and stop it.
pgrep -af "show_module.py|show_scan.py"
pkill -f show_module.py
pkill -f show_scan.py
