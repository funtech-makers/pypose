#!/usr/bin/env python3


from setuptools import setup, find_packages


setup(name='pypose',
      version='2.0',
      description='A tool for controling Dynamixels with an Arbotix',
      url='https://github.com/funtech-makers/pypose',
      author='Ewen BRUN',
      author_email='ewen.brun@ecam.fr',
      license='GPLv3',
      packages=find_packages(),
      zip_safe=False,
      install_requires=[
          'pyserial',
          'wxpython',
      ],
      entry_points={
          'console_scripts': ['pypose=pypose.pypose:main'],
      }
      )
