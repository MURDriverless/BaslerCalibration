# BaslerCalibration
Camera calibration script used to run on Basler cameras

## Usage
```
./main [options]
Options:
 -s        size of square in mm
 -x        chessboard columns
 -y        chessboard rows
 -f        calibration frames
```
Default has `s = 20`, `(x, y) = (11, 7)` and `f = 50`.

Changing `x` or `y` without the other will leave the other at default, e.g.
```
./main -x 10
```
is the same as
```
./main -x 10 -y 7
```

Device name to used is currently hard coded at line,
```
di.SetFriendlyName("CameraLeft (40022599)");
```

## Requirements
- Pylon
- Opencv2

## Compilation instructions
```
mkdir build
cd build
cmake ..
make
```

## TODO
- [ ] Remove hard code of camera name
- [ ] Generic GeniCAM interface? (Harvester or Aravis)
- [ ] Extend frame grabbing logic, [like the image pipe line in ros](https://github.com/ros-perception/image_pipeline/tree/melodic/camera_calibration).