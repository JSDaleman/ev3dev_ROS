from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'mqtt'  # Reemplaza con el nombre de tu paquete

setup_args = generate_distutils_setup(
    packages=[package_name],
    package_dir={'': 'src'},  # Indica que el código fuente está en 'src/'
)

setup(**setup_args)
