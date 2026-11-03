## Static defect generation and visualization on a battery in RVIZ ## 

# Terminal 1
cd ~/projects/MotoBat && source motobat_env.sh
rviz2 -d src/motobat_description/rviz/defects.rviz

# Terminal 2
cd ~/projects/MotoBat && source motobat_env.sh
/usr/bin/python3 src/motobat_description/scripts/show_module.py data/preview_defective
