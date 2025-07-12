# 安装

## 下载

选择平台和版本[下载安装包](https://github.com/Jelatine/JellyCAD/releases)

## Windows

下载`JellyCAD_Setup_v0.2.2.exe`双击安装

## Ubuntu24.04

```bash
wget https://github.com/Jelatine/JellyCAD/releases/download/v0.2.2/JellyCAD_Setup_v0.2.2.sh
chmod +x JellyCAD_Setup_v0.2.2.sh
./JellyCAD_Setup_v0.2.2.sh
```

安装依赖：(后续可能在安装包中解决?)

```bash
# libxcb-icccm.so.4: cannot open shared object file
sudo apt install libxcb*
# libSM.so.6: cannot open shared object file
sudo apt-get install libsm6
# libxkbcommon-x11.so.0: cannot open shared object file
sudo apt-get install libxkbcommon-x11-0
```

