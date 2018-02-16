from setuptools import setup, Extension
setup(name='PyIMU',
      version='0.0.1b',
      description='Module for IMU sensor',
      author='Artem',
      ext_modules=[Extension('PyIMU', sources=['src/PyIMU.c', 'src/IMUlib.c'], include_dirs =['src', '/usr/local/include'],library_dirs = ['/usr/local/lib'], libraries = ['m'])])
