from setuptools import setup, find_packages
from os import path

import versioneer

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='modact',
    version=versioneer.get_version(),
    cmdclass=versioneer.get_cmdclass(),
    description='Multi-Objective Design of Actuators',

    long_description=long_description,
    long_description_content_type='text/markdown',

    author='Cyril Picard',
    author_email='cyril.picard@epfl.ch',

    classifiers=[
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
    ],

    packages=find_packages(exclude=['contrib', 'docs', 'tests']),

    # List additional URLs that are relevant to your project as a dict.
    #
    # This field corresponds to the "Project-URL" metadata fields:
    # https://packaging.python.org/specifications/core-metadata/#project-url-multiple-use
    #
    # Examples listed include a pattern for specifying where the package tracks
    # issues, where the source is hosted, where to say thanks to the package
    # maintainers, and where to support the project financially. The key is
    # what's used to render the link text on PyPI.
    project_urls={  # Optional
        'Bug Reports': 'https://github.com/epfl-lamd/modact/issues',
        'Source': 'https://github.com/epfl-lamd/modact/',
    },
    url='https://github.com/epfl-lamd/modact/',
    setup_requires=["pytest-runner"],
    tests_require=["pytest", "pytest-cov"]
)
