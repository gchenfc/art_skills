# Art Skills

## GTSAM chebyshev alphabet stuff
To get `cpp/chebyshev.cpp` to run:

In GTSAM folder:
1. `git checkout feature/basis` - to get Varun's basis function code
2. `git merge develop`          - to get latest gtsam updates
3. resolve merge conflicts (gtsam.i - use "incoming" from develop)
    I think this should probably work but not sure:  
        `git checkout feature/basis`  
        `git merge -X theirs develop`
4. `cd build`
5. `ccmake ..` - this is a "GUI" interface to edit cmake variables.  scroll down to `GTSAM_WITH_EIGEN_UNSUPPORTED` and change it from "OFF" to "ON".  Hit "c" for "configure" and keep doing that until you get the option for "g" for "generate".  or:
    `cmake -DGTSAM_WITH_EIGEN_UNSUPPORTED=ON ..`
6. `cmake -DCMAKE_INSTALL_PREFIX=./install ..`
7. `make -j8 install`

Then back in art_skills,
```bash
cd build
cmake ..
make jd00_chebyshevAlphabet
./cpp/jd00_chebyshevAlphabet
```
should work

---

A project to capture, perceive, and render artistic skills.

- Capture: really, motion capture, cameras, etc. LOW level.
- Perceive: infer parameters from low-level input by optimization over a generative model of the strokes.
- Render: possibly style-transfer, scale transfer to a an output device like a giant cable-robot, a super-duper Kuka, or ShodaFranka.

oFX & MATLAB setup and workflow (04/26/21):
1) Install oFX for Visual Studio 17 from the following webstie: https://openframeworks.cc/download/
2) Do not open oFX projects manually, instead use to the "projectGenerator.exe" in the install folder to create or launch projects
3) Go to the following link to access and download the project: https://www.dropbox.com/s/v2wpm659orm6itd/Stroke_Capture.zip?dl=0
4) Alternatively, create en empty project and replace the project files with the scripts in the "art_skills/cpp/Stroke_Capture" folder
5) Use the oFX project launcher to open the project in Visual Studio, you will be prompted to rebase, which you should for it to work
6) Modify the ofstream output path to the /art_skills/matlab folder in the art_skills folder to save text files where the "vel_processing.m" file can read them
7) Run the oFX project and collect input strokes
8) Open the matlab file and run to see generated plots

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
