# Camera Calibration

Calibration is a painful on-time per camera process which if not done correctly, can result in a catastrophic experience with SLAM.

## Preparation

You need **COMPLETE FLAT** checkerboards. You can achieve this by putting the checker board in-between two pieces of glass or inside a picture frame where it completely flats out. **This is really important. If your checker boards are not flat, calibration results are invalid**.

## Option 1 - KIT's online calibrator

You can use the [online calibrator provided by KIT](http://www.cvlibs.net/software/calibration/index.php). **READ THEIR INSTRUCTIONS CAREFULLY**. Make sure checkerboards fill up the entire picture. Make sure your INI file matches what KIT's examples are. In the end, KIT will return something like this:

```TEXT
S_01: 6.400000000000e+02 4.800000000000e+02
^___ this is source picture dimensions

K_01: 5.039630403708e+02 4.491364345717e-04 3.187466804000e+02 0.000000000000e+00 4.990782697476e+02 2.458400371100e+02 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00
^___ this is the camera intrinsic matrix so: fx 0 cx, 0 fy cy, 0 0 1

D_01: 6.567503662024e-02 -4.898695307849e-01 1.252667542191e-03 1.807506292790e-04 9.823352309818e-01
^___ this is the distortion parameters so: k1 k2 p1 p2 k3

___ Below parameters are unused by our SLAM implementation for now
R_01: 9.999709749288e-01 -6.455330100142e-03 -4.046975819075e-03 6.484506085142e-03 9.999527791032e-01 7.238145107599e-03 4.000060101265e-03 -7.264177659247e-03 9.999656150299e-01
T_01: -4.560403448807e-03 5.268657208575e-05 -4.260939879871e-03
```

## Option 2 - Matlab's calibration toolbox

[You can use Matlab to do the calibration for you](https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html). It's easier to investigate the individual captures this way and fine tune the result. Downside is that, you need to take many pictures for it to be accurate and reliable.
