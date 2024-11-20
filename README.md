# Event-based Reconfiguration Control for Time-varying Robot Formation in Narrow Spaces
This repository presents the following article in Python:

Duy-Nam Bui, Manh Duong Phung, Hung Pham Duy, **"Event-based Reconfiguration Control for Time-varying Robot Formation in Narrow Spaces,"**.

## Installation
```
git clone git@github.com:duynamrcv/erc.git
pip install imageio numpy matplotlib
```

## Demo
We implement two controllers, includes the proposed controller `ERC` and the pure behavior controller `BC`, which are presented in `robot_erc.py` and `robot_bc.py`, respectively. To choose the tested controller, edit parameter `CONTROLLER` in `config.py`. To run the demo, run:
```
cd erc
python3 main.py
```
The data will be saved in `*.txt`. For the visualization, run:
```
python3 visualization.py
```

## Results
|Method| Pentagon shape | V-shape |
|:---:| :---:      |     :---:  |
|ERC| <img src="results/gif_erc_shape1.gif" alt="" width="100%"/> | <img src="results/gif_erc_shape2.gif" alt="" width="100%"/> |
|BC|<img src="results/gif_bc_shape1.gif" alt="" width="100%"/> | <img src="results/gif_bc_shape2.gif" alt="" width="100%"/> |