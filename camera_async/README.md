## Setup
* Pan/Tilt Arduino Controller
  * /dev/ttyACM0
  * ``~/arduino-cli compile camera_async --fqbn arduino:avr:uno -p /dev/ttyACM0;  ~/arduino-cli upload camera_async --fqbn arduino:avr:uno -p /dev/ttyACM0``
* Zoom Arduino Controller
  * /dev/ttyACM1
  * ``~/arduino-cli compile camera_zoom_async --fqbn arduino:avr:uno -p /dev/ttyACM1;  ~/arduino-cli upload camera_zoom_async --fqbn arduino:avr:uno -p /dev/ttyACM1``

* Camera Control Python Service
* ``sudo systemctl restart camera-control.service``