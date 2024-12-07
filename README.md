# DInCIKF-covisible for multi-robot object based pose estimation
Distributed Invariant Covariance-Intersection Kalman filter for multi robots pose estimation and environmental objects pose estimation. 

## File Description
### Matlab Scripts

Experiment 1 is the numerical simulation based on MATLAB. You can run main1.mlx to get the result of experiment 1. Run main2.mlx to get the result of experiment 2.

### Python Scripts
Experiment2 Tools files are python scripts, which are for the real world dataset experiment (YCB Video). We use Foundation Pose to get the pose estimation results, which are saved in file yml2jsons as .yml format. You need to download from the following dataset link to obtain the suitable format.

Dataset link: [https://bop.felk.cvut.cz/home/](https://bop.felk.cvut.cz/home/)

Foundation Pose link: [https://github.com/NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose)

To use the objects' poses in MATLAB, you can use convertyml2json.py to change the .yml output to .json.

#### Coppeliasim Scenario

Coppeliasim office scenario and visuslization scripts of path is in file coppeliasim.

## Related Paper

```
@article{li2024distributed,
  title={Distributed Invariant Kalman Filter for Object-level Multi-robot Pose SLAM},
  author={Li, Haoying and Zeng, Qingcheng and Li, Haoran and Zhang, Yanglin and Wu, Junfeng},
  journal={arXiv preprint arXiv:2409.09410},
  year={2024}
}
```
