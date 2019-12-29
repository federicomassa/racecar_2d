from setuptools import setup

setup(
   name='racecar_2d',
   version='0.1',
   description='A 2D racing car simulator',
   author='Federico Massa',
   author_email='fedemassa91@gmail.com',
   packages=['racecar_2d'],
   install_requires=['pygame', 'numpy', 'scipy', 'tqdm'],
)
