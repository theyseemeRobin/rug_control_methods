from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['controller_interface', 'moveit_fk', 'moveit_ik'],
    package_dir={'' : 'src'},
)
setup(**d)