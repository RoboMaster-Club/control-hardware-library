## About The Library

This is a shareable hardware library for robomaster embedded system. It contains board support packages, device-drivers, and algorithms for control systems.
## How To
Add the submodule to your project
```
git submodule add <https://github.com/jia-xie/control-hardware-library.git> FOLDER-NAME
```
Initialize the submodule
```
git submodule update --init
```
The submodule will exist as a folder in the project. In Git environment, it is a link pointing to a specific commit version. To manipulate the library itself, you can open the library directory and treat it as a regular Git repository.
```
cd FOLDER-NAME
git status
```
You should be able to view git information about this library.
## Modifications
- Change ```samepleFreq``` in [MahonyAHRS.c](Algo/Src/MahonyAHRS.c?plain=1#L23), this will affect the fusion result
- Initialize a task for imu in FreeRTOS environment
