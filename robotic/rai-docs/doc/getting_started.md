# Getting Started

## Quick Install

Install the `robotic` python package. On a standard Ubuntu, the
following should be sufficient:

    sudo apt install liblapack3 freeglut3 libglu1-mesa libfreetype6 fonts-ubuntu python3 python3-pip
    #latest Ubuntu: libglut3.12 and 'cd /usr/lib/x86_64-linux-gnu/ && sudo ln -s libglut.so.3.12 libglut.so.3'
    python3 -m pip install --user robotic numpy

A standard test is

    python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'
    python3 -c 'import robotic as ry; ry.test.RndScene()'

## Compiling from source, cmd line tools, & docstrings

* **Compiling for the real robot:** The `robotic` pip package does not
  have the real robot drivers (for the Franka Panda robot) included,
  and therefore only supports simulation mode. However, the interface
  and code is designed so that everything directly transfers to the
  real robot: All you have to do is re-compile the python package on
  your machine, including the libfranka and librealsense drivers. This
  is done by installing the
  [robotic](https://github.com/MarcToussaint/robotic) package.
* **Command line tools:** There are currently three little cmd line tools, which should be in `~/.local/bin`:
  * `ry-view`: to view robot/scene model files (see [this tutorial](tutorials/configurationEditing.html))
  * `ry-bot`: to test basic operations with the real robot (have a look at help and simple source)
  * `urdf2rai.py`: to help convert from urfl to model files (see, again, [this tutorial](tutorials/configurationEditing.html))
* **Docstrings and tab completion:** The package comes with a 'stubs'
  file (should be in
  `~/.local/lib/python*/site-packages/robotic/_robotic.ipy`), which
  includes docstrings most methods and should enable autocompletion in
  your IDE. The docstrings are not yet great for all methods.

  If tab-autocomplete for jupyter does not work, try `python3 -m pip
  install --user jedi==0.17.2`

## Downloading the tutorial notebooks

    git clone https://github.com/MarcToussaint/rai-tutorials.git
	cd rai-tutorials
	jupyter-notebook .

If you don't have jupyter installed yet:

	pip3 install --user jupyter nbconvert matplotlib
