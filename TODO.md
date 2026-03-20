### TODO.md

For functionality parity with `radar_pi`:

* (M)ARPA target tracking is still defective
* EBL/VRM support in GUI
* Timed Transmit
* Garmin xHD support (on hold until developer shows up)

Bugs:

* Rotation of the PPI window doesn't work, image is always HeadsUp whereas heading check marks
  are north up.
* Check doppler packets sent when no chartplotter present and disallow doppler status when
  no heading is on radar spokes.
* Furuno brand support needs more work. (-> Dirk)

For parity with branch `v2`: 

* Re-implement the radar recording and playback. 
* Re-implement the debugger. Or a better one?
