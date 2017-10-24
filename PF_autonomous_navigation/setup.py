## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD 
from distutils.core import setup 
from catkin_pkg.python_setup import generate_distutils_setup 
 
# fetch values from package.xml 
setup_args = generate_distutils_setup( 
	packages=['PF_autonomous_navigation'], 
	package_dir={'': 'include/PF_autonomous_navigation'}, 
) 
setup(**setup_args)
