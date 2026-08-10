# RViz configs go here

`aurora.launch.py` opens RViz pointed at `aurora.rviz` in this directory by
default (`rviz_config` launch argument). That file doesn't exist yet --
follow the walkthrough in the top-level README (or ask) to set up the
displays and save it here, so future launches open with everything already
configured.

This placeholder file exists so the directory itself gets installed by
colcon even before `aurora.rviz` exists -- with `--symlink-install`,
saving directly into
`install/inchiscope_bringup/share/inchiscope_bringup/rviz/aurora.rviz`
works immediately, no rebuild needed. Copy it back into this source
directory afterwards (and it's worth committing) if you want the config
kept under version control.
