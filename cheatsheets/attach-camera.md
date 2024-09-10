# Connect camera device to docker container (Windows)

## Help

```powershell
usbipd-win 4.2.0

Description:
  Shares locally connected USB devices to other machines, including Hyper-V guests and WSL 2.

Usage:
  usbipd [command] [options]

Options:
  --version       Show version information
  -?, -h, --help  Show help and usage information

Commands:
  attach   Attach a USB device to a client
  bind     Bind device
  detach   Detach a USB device from a client
  license  Display license information
  list     List USB devices
  policy   Manage policy rules
  server   Run the server on the console
  state    Output state in JSON
  unbind   Unbind device
```

## Share the device

```powershell
>> usbipd list
Connected:
BUSID  VID:PID    DEVICE                                   STATE
1-3    0b05:19b6  Dispositivo de entrada USB               Not shared
2-1    1bcf:2281  HD WEBCAM                                Not shared
```

```powershell
>> usbipd bind --busid 2-1
>> usbipd list
Connected:
BUSID  VID:PID    DEVICE                                   STATE
1-3    0b05:19b6  Dispositivo de entrada USB               Not shared
2-1    1bcf:2281  HD WEBCAM                                Shared
```

## Attach the device to a WSL distribution

```powershell
>> wsl -l -v
  NAME              STATE           VERSION
* Ubuntu            Running         2
  docker-desktop    Running         2
  Ubuntu-22.04      Stopped         2

>> usbipd attach --wsl docker-desktop --busid 2-1
```

## Check the camera

Open a WSL terminal and check with `lsusb` if your usb port is listed.  
Then check if `/dev/video0` exist. If it does, you're good to go.  

If it doesn't, the camera drivers might not be installed in your WSL2 kernel. If so, check out [this](./wsl-kernel.md).

If you're using a PS3 Eye camera, check out [this](./ps3eye_wsl.md).
