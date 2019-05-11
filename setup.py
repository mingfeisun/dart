﻿import contextlib
import os
import re
import sys
import platform
import subprocess

from codecs import open  # To use a consistent encoding
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion
import distutils.log

# Get the current directory path.
here = os.path.abspath(os.path.dirname(__file__))

# Extract the description from the top-level README.
with open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()
description = 'dartpy provides python bindings for DART.'

distutils.log.set_verbosity(distutils.log.DEBUG)  # Set DEBUG level


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    """ Wrapper class that builds the extension using CMake. """

    def run(self):
        """ Build using CMake from the specified build directory. """
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: " +
                ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(
                re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.8.0':
                raise RuntimeError("CMake >= 3.8.0 is required on Windows")

        distutils.log.set_verbosity(distutils.log.DEBUG)  # Set DEBUG level

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(
            os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = [
            '-DDART_BUILD_DARTPY=ON',
            # '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DPYTHON_EXECUTABLE=' + sys.executable
        ]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            # cmake_args += [
            #     '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(
            #         cfg.upper(), extdir)
            # ]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(
            env.get('CXXFLAGS', ''), self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args,
                              cwd=self.build_temp,
                              env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args,
                              cwd=self.build_temp)
        subprocess.check_call(['cmake', '--build', '.', '--target', 'install'],
                              cwd=self.build_temp)


# Set up the python package wrapping this extension.
setup(
    name='dartpy',
    version='0.0.1',
    description=description,
    long_description=long_description,
    ext_modules=[CMakeExtension('dartpy')],
    url='https://github.com/dartsim/dart',
    author='Jeongseok Lee',
    author_email='jslee02@gmail.com',
    license='BSD',
    keywords='dartsim robotics',
    classifiers=[
        'Development Status :: 1 - Planning',
        'License :: BSD',
        'Intended Audience :: Developers',
    ],
    cmdclass=dict(build_ext=CMakeBuild),
)