# Art Skills

A project to capture, perceive, and render artistic skills.

- Capture: really, motion capture, cameras, etc. LOW level.
- Perceive: infer parameters from low-level input by optimization over a generative model of the strokes.
- Render: possibly style-transfer, scale transfer to a an output device like a giant cable-robot, a super-duper Kuka, or ShodaFranka.

oFX & MATLAB setup and workflow (04/23/21):
1) Install oFX in Visual Studio 19 (there are guides on youtube)
2) Go to the following link to access and download the project: https://www.dropbox.com/s/v2wpm659orm6itd/Stroke_Capture.zip?dl=0
3) Use the oFX project launcher to open the project in VS19
4) Modify the ofstream output path to the matlab folder in the art_skills folder
5) Run the project and collect input stroke
6) Open the matlab file and run

## Instructions from template:

- Python 3.6+ is required, since we support these versions.
- To install the wrap package via `GTSAM`:

  - Set the CMake flag `GTSAM_BUILD_PYTHON` to `ON` to enable building the Pybind11 wrapper.
  - Set the CMake flag `GTSAM_PYTHON_VERSION` to `3.x` (e.g. `3.7`), otherwise the default interpreter will be used.
  - You can do this on the command line as follows:

    ```sh
    cmake -DGTSAM_BUILD_PYTHON=ON -DGTSAM_PYTHON_VERSION=3.7 ..
    ```
- Alternatively, you can install the wrap package directly from the [repo](https://github.com/borglab/wrap), but you will still need to install `GTSAM`.

## INSTALL

- In the top-level directory, create the `build` directory and `cd` into it.
- Run `cmake ..`.
- Run `make`, and the wrapped module will be installed to the `python` directory in the top-level.
- To install the wrapped module, simply run `make python-install`.
- You can also run `python main.py` which calls the wrapped module, to get a flavor of how it works.

## DOCUMENTATION

For more detailed information, please refer to the [tutorial](TUTORIAL.md).
