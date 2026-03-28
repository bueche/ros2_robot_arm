# Switch to manual exposure and reduce it significantly
v4l2-ctl --device=/dev/video0 --set-ctrl=auto_exposure=1
v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_time_absolute=50

# Reduce brightness and backlight compensation
v4l2-ctl --device=/dev/video0 --set-ctrl=brightness=-30
v4l2-ctl --device=/dev/video0 --set-ctrl=backlight_compensation=0

# Reduce gain to prevent camera from brightening dark areas
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=0
