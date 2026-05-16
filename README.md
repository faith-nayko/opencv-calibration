# OpenCV Camera Calibration

A C++ implementation of camera self-calibration using OpenCV, applied to a real
photogrammetric dataset and validated against the results of an established
photogrammetric bundle adjustment package.

**Author:** Faith Nayko
**Contact:** faithnayko@gmail.com | faith.nayko@ucalgary.ca

---

## Project purpose

I built this project to demonstrate working knowledge of OpenCV applied to a
real-world photogrammetric problem. Rather than running the standard
checkerboard calibration tutorial, I reimplemented a camera self-calibration
that I originally performed in [FEMBUN](#about-fembun) for my MSc thesis, using
the same input data, and compared the results.

The headline result: **the OpenCV-calibrated radial distortion coefficients
agree with the FEMBUN reference values to within ~1–4%, with focal length
agreement to within 0.1%**, confirming that the OpenCV implementation
reproduces the FEMBUN calibration to within typical implementation-specific
variation.

## The calibration data

The dataset comes from an indoor calibration field at the University of
Calgary developed for my [MSc thesis](https://hdl.handle.net/1880/123784) on
RTLS-constrained smartphone indoor photogrammetry. It consists of:

- **Camera:** iPhone 12 Pro, native 12 MP resolution (4032 × 3024)
- **Control points:** 135 surveyed targets distributed across two perpendicular
  walls, with coordinates established by a terrestrial laser scan
- **Image network:** 30 exposures from varied positions and orientations,
  including alternating landscape and portrait rotations to break projective
  coupling between intrinsic and extrinsic parameters
- **Image observations:** ~3,400 pixel measurements obtained through a
  semi-automated workflow (Otsu thresholding → boundary extraction → least-squares
  ellipse fitting), with sub-pixel measurement precision

## What the code does

The program reads six input files describing the camera, the calibration field,
and the image network; runs `cv::calibrateCamera` with the appropriate
parameter-fixing flags; and prints the calibrated intrinsics alongside the
FEMBUN reference values for comparison.

### Pipeline

1. **Read inputs** — IOPs (initial intrinsics), additional-parameter flags,
   bundle config, EOPs, control points, and image observations.
2. **Group observations** — match each 2D pixel observation to its corresponding
   3D control point, then group by image.
3. **Convert exterior orientations** from FEMBUN convention (degrees, intrinsic
   rotation order R = R<sub>κ</sub>·R<sub>φ</sub>·R<sub>ω</sub>) to OpenCV's
   pose representation (Rodrigues vector + translation).
4. **Translate flags** — convert the boolean flag struct into OpenCV's
   `CALIB_*` bitmask.
5. **Run calibration** — call `cv::calibrateCamera` with all inputs assembled.
6. **Report** — print the refined intrinsics, then a side-by-side comparison
   with FEMBUN reference values.

### Results

| Parameter | FEMBUN reference | OpenCV result | Diff   | Rel diff |
|-----------|------------------|---------------|--------|----------|
| fx        | 3020.67          | 3023.68       | +3.01  | +0.10%   |
| cx        | 2030.77          | 2032.26       | +1.49  | +0.07%   |
| cy        | 1512.28          | 1497.72       | −14.56 | −0.96%   |
| k1        | +0.09771         | +0.09901      | +0.0013| +1.3%    |
| k2        | −0.18865         | −0.19634      | −0.0077| +4.1%    |
| k3        | +0.11160         | +0.10699      | −0.0046| −4.1%    |

Final RMS reprojection error: **2.06 px**.

### A note on the residual differences

OpenCV's `calibrateCamera` treats all 3D object points as known and fixed. The
original FEMBUN calibration also included tie points whose positions were
estimated jointly with the camera parameters; those observations were dropped
from this OpenCV run since they aren't supported by the API. Some of the
residual difference between the two calibrations (particularly in cy, where
the principal point is known to correlate with tangential distortion in
self-calibration problems) likely reflects this methodological constraint
rather than an algorithmic discrepancy.

## Build

Built with Visual Studio 2022 and OpenCV 4.x. Open the `.sln` file, configure
the OpenCV include and library directories in project properties, place the
`data/` folder of input files inside the project directory, and build for
either Debug or Release x64.

## Code organization

```
opencv-calibration/
├── 2026-05-14_Calibration/
│   ├── main.cpp              entry point and result reporting
│   ├── types.h               data structures for IOPs, EOPs, GCPs, etc.
│   ├── config.h / config.cpp readers for the six input files
│   ├── geometry.h / .cpp     FEMBUN ↔ OpenCV pose conversion
│   ├── calibration.h / .cpp  the calibration pipeline
│   └── data/                 input files
└── README.md
```

## About FEMBUN

FEMBUN is a constrained self-calibrating bundle adjustment software developed
by Lichti and Chapman (1997) at the University of Calgary. It implements
bundle adjustment with weighted-observation support for ground control points
and exterior orientation parameters, enabling flexible integration of external
measurements with photogrammetric observations.

**Reference:** Lichti, D. D., & Chapman, M. A. (1997). Constrained FEM
self-calibration. *Photogrammetric Engineering and Remote Sensing*, 63(9),
1111–1119.
