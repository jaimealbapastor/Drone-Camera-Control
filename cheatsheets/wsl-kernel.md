# Modify WSL Kernel to accept USB Cams

[AWESOME VIDEO MODIFY KERNEL](https://www.youtube.com/watch?v=t_YnACEPmrM&t=183s)  
[Video found from here](https://github.com/docker/for-win/issues/13940)  

```bash
> wsl
$ cd /usr/src
```

> If not done, download clean Kernel from this [list](https://github.com/microsoft/WSL2-Linux-Kernel/releases)
>
> ```bash
> uname -r
> ```
>
> ```bash
> VERSION=5.15.153.1
> ```
>
> ```bash
> sudo git clone -b linux-msft-wsl-${VERSION} https://github.com/microsoft/WSL2-Linux-Kernel.git ${VERSION}-microsoft-standard && cd ${VERSION}-microsoft-standard
> ```

Copy and extract the configuration file of the current Kernel

```bash
sudo cp /proc/config.gz config.gz && sudo gunzip config.gz && sudo mv config .config
```

Now select the options (look youtube video)

```bash
sudo make menuconfig
```

And compile it

```bash
sudo make -j$(nproc) && sudo make modules_install -j$(nproc) && sudo make install -j$(nproc)
```

