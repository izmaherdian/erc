# Event-based Deformation Control in Confined Space Environment
This repository presents the following article in Python:

Duy-Nam Bui, Manh Duong Phung, Hung Pham Duy, **"MPPDC: Model Prediction-based Perceptual Deformation Control for Multiple Robots in Narrow Space Environments,"**.

## Installation
```
git clone git@github.com:duynamrcv/edc.git
cd edc
```

## Demo
We implement two controllers, includes the proposed controller `EDC` and the pure behavior controller `BC`, which are presented in `robot_edc.py` and `robot_bc.py`, respectively. To choose the tested controller, edit parameter `CONTROLLER` in `config.py`. To run the demo, run:
```
python3 main.py
```
The data will be saved in `*.txt`. For the visualization, run:
```
python3 visualization.py
```

## Results
| Pentagon shape | V-shape |
| :---:      |     :---:  |
| <img src="results/gif_edc_shape1.gif" alt="" width="100%"/> | <img src="results/gif_edc_shape2.gif" alt="" width="100%"/> |