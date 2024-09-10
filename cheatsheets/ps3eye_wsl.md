# How to use PS3 Eye camera on Docker Container

## Links

- [github project](https://github.com/bakercp/ofxPS3EyeGrabber) giving the idea of bypassing the standard kernel driver
- 

Not used but might be of help
- https://www.reddit.com/r/hoggit/comments/vaddx6/trouble_installing_drivers_ps3_eye_camera_windows/

## WSL2

Open a wsl2 terminal (not docker) and follow


```bash
$ sudo apt install kmod
$ sudo modprobe uvcvideo
modprobe: FATAL: Module uvcvideo not found in directory /lib/modules/5.15.153.1-microsoft-standard-WSL2+

$ sudo apt install udev
$ sudo udevadm trigger

```
