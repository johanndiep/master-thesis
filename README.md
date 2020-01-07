# Ultra-Wideband Localization Error Correction using Gaussian Process for Teach-and-Repeat UAV Flight

This GitLab repository contains the code for the "Ultra-Wideband Localization Error Correction using Gaussian Process for Teach-and-Repeat UAV Flight" project. It was developed in the context of a master thesis at ETH Zurich.

## Context

In recent years, unmanned aerial vehicles (UAV) have gained more interests in the research community. Due to their improved videography capabilities, they have the potential to bridge the gap between precise and flexible filming. However, their current localization methods are
not reliable enough for tasks such as the following of a predefined trajectory.

Motivated by this gap, the work in the context of this thesis is to establish an accurate localization system for UAVs based on commercially available ultra-wideband (UWB) technology. It describes the implementation of a system based on filtered multilateration using distance measurements between the drone-mounted tag and several environmentfixed anchors. Here, methods are presented to estimate the anchor placements as well as to correct for the systematic biases of the range readings. The former is solved by means of Gauss-Newton optimization using inter-anchor distance estimations. For the latter, in order to correct for the systematic distance measurement errors, a teach-andrepeat framework is proposed, where Gaussian Process models are derived from training data given by a motion capture system and incorporated in the corresponding equation of the state estimation algorithm.

## Installation

This installation was tested for Ubuntu 16.04 LTS. For other operating systems, changes or additional packages might be required. The following
packages were used:

* [Python 3.6.5](https://www.python.org/downloads/source/)
* [CUDA Toolkit 9.0](https://developer.nvidia.com/cuda-90-download-archive/)
* [PyTorch](https://pytorch.org/)
* [Neural 3D Mesh Renderer](http://hiroharu-kato.com/projects_en/neural_renderer.html) (A modified version was used for this project which is not publicly available at the time of writing.)
* [pyquaternion](http://kieranwynn.github.io/pyquaternion/)
* [meshzoo 0.4.3](https://pypi.org/project/meshzoo/)
* [OpenCV 3.4.4](https://www.learnopencv.com/install-opencv-3-4-4-on-ubuntu-16-04/)
* [tqdm](https://github.com/tqdm/tqdm)
* [scikit-image 0.13.1](http://scikit-image.org/docs/0.13.x/install.html)
* [matplotlib 3.0.2](https://matplotlib.org/users/installing.html) (optional)
* [MeshLab 2016](http://www.meshlab.net/#download/) (optional)

Additional libraries might be required, install on request.

## Dataset

For evaluation, rendered dataset by Peidong Liu can be used:

* [Realistic Rendering](https://gitlab.com/jdiep/semester-thesis/tree/master/RelisticRendering-dataset)
* [Urban City](https://gitlab.com/jdiep/semester-thesis/tree/master/UrbanCity%20dataset)

## Code Structure

The [source code](https://gitlab.com/jdiep/semester-thesis/tree/master/neural_mesh_renderer/OBMBACPE/lib) folder contains all the essential classes with its corresponding methods:

* **dataset.py:**<br/>
This class is responsible for reading out the information from the rendered dataset. Additionally, it also contains the methods to return the scaled as well as the perturbed information.
* **framework.py:**<br/>
This class sets up the framework for the optimization process. It initializes the pose with a disturbance which is subsequently mapped to se(3)-formulation. Further, the objective function is defined here.
* **imagegeneration.py:**<br/>
This class contains methods to generate rendered depth maps, sharp and blurred images at arbitrary poses.
* **meshgeneration.py:**<br/>
This class contains the construction of the 3D polygon-mesh.
* **optimization.py:**<br/>
The optimization with PyTorch automatic differentiation package is contained here.
* **posetransformation.py:**
Contains the exponential and logarithmic mapping methods.
* **randomizer.py:**<br/> 
Responsible for creating a cardinal directed vector with a defined length as well as an angle axis vector corresponding to an elemental rotation with a defined angle.
* **renderer.py:**<br/> 
Initializes the external renderer module, which is used for generating depth maps.

The [main](https://gitlab.com/jdiep/semester-thesis/tree/master/neural_mesh_renderer/OBMBACPE) folder contains the executables of the classes mentioned above which can be run individually depending on the desired output:

* **pose estimation.py:**<br/>
This is the main executable file of this project. It finds the pose of the blurred input image via an optimization process.
* **3d representation.py:**<br/> 
This program produces a 3D pointcloud and polygon-mesh representation of the environment in the intial frame. The generated txt- and obj-file can be observed in Meshlab.
* **image generator.py:**<br/> 
This program generates sharp and blurry images at arbitrary poses.

## Example: Optimization Process

Artificial blurred images generated during optimization.

![](https://media.giphy.com/media/Y3e0dGYLjS0JegOLAW/giphy.gif)

## Version

* 7. January 2020: Version 1.0, [Documentation](https://gitlab.com/jdiep/semester-thesis/tree/master/documentation)

## Authors

* Johann Diep (MSc Student Mechanical Engineering, ETH Zurich, jdiep@student.ethz.ch)