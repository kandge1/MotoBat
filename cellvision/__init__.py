"""cellvision: the S1 overhead 2D cameras in numpy (task 3.2). No ROS.

camera.py   midiCam2 pinhole model and box-scene renderer
calib.py    dot-grid target -> plane homography (normalised DLT)
locate.py   cell top -> (x, y, yaw, sizes) by subpixel edges and a rectangle fit
scenes.py   the infeed and the stack, as box scenes in station coordinates
study.py    accuracy studies:  python -m cellvision.study
"""
