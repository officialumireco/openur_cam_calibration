# OpenUR Camera Calibration

A lightweight Python utility for calibrating a camera using a circular calibration pattern. The project reads a set of calibration images, detects the calibration target, estimates the camera matrix and distortion coefficients, and writes the results to an OpenCV XML file.

This repository is intentionally simple and focused: it is designed to work as a standalone calibration script for image datasets captured with a single camera.

## Project purpose

The script in `calibration_single_camera.py`:

- loads calibration images from a folder
- converts images to grayscale and detects circular calibration markers
- finds the target grid with OpenCV
- computes camera intrinsics and lens distortion
- saves the calibration parameters to `stereoMap.xml`
- stores intermediate detected images in a processing folder for inspection

## Repository structure

- `calibration_single_camera.py` — main calibration script
- `samples/` — example output files, including a generated `stereoMap.xml`
- `LICENSE` — MIT license

## Requirements

- Python 3.x
- OpenCV (`opencv-python`)
- NumPy

Install dependencies:

```bash
pip install numpy opencv-python
```

## Quick start

1. Prepare a folder of calibration images.
2. Update the configuration in `calibration_single_camera.py`.
3. Run the script:

```bash
python calibration_single_camera.py
```

## Configuration

The script uses a dictionary named `dict_cfg` near the bottom of the file. Example:

```python
dict_cfg = {
    "processing": {
        "id": "BOSS-A",
        "calib_board": {
            "width": 100,
            "height": 100,
            "dia_circle": 24,
            "dia_circle_centre": 30,
            "num_rows": 7,
            "num_cols": 7
        },
        "clean": False
    },
    "paths": {
        "dir_images": "/images",
        "dir_detected": "/process"
    }
}
```

Key settings:

- `dir_images`: directory containing calibration images
- `dir_detected`: directory where processed and detected images are saved
- `num_rows` / `num_cols`: board grid dimensions
- `calib_board` values: pattern geometry used by the detection step

## Output

The calibration script writes the intrinsic calibration result to an XML file named `stereoMap.xml` in the detected output directory.

The saved file contains values such as:

- `K` — camera matrix
- `D` — distortion coefficients

## Notes

- The detection logic is tuned for circular calibration patterns and may need adjustment for different boards or lighting conditions.
- If the pattern is not detected reliably, adjust the SimpleBlobDetector settings in `_make_blob_detector()`.
- The script writes visual debug images to the detection folder so you can inspect what was found before trusting the calibration result.

## License

This project is distributed under the MIT License. See `LICENSE` for details.
