# Arm Control
The package is providing python api to control robot manipulator and its end-effector.

### How to Use The Module
```python
from arm_control import ArmTask, SuctionTask
```

### Making Python Module for Another Package
* catkin [user guide](http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html) for setup

1. Uncomment following line in CMakeLists.txt
```cmake
catkin_python_setup()
```

2. Let structure of package like following:
```
-- package_name/
   |-- some_folder/
   |-- src/
       |-- package_name/
           |-- __init__.py
           |-- some_package/
           |-- some_module.py
   |-- CMakeLists.txt
   |-- package.xml
   |-- setup.py
```

3. Make a setup.<span></span>py in package
```python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['package_name'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```

4. Build the package
```bash
cd SOMEWORKSPACE
catkin_make
```
