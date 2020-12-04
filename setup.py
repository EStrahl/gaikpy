from setuptools import find_packages, setup

from os import path
this_directory = path.abspath(path.dirname(__file__))
#with open(path.join(this_directory, 'README.md'), encoding='utf-8') as f:
with open(path.join(this_directory, 'README.md')) as f:
    long_description = f.read()

setup(
    name='gaikpy',
    version='0.3.4.0',
    author='Erik Strahl',
    author_email='strahl@informatik.uni-hamburg.de',
    description='Calculates and visualises forward and (full-pose) inverse kinematic realised with a genetic algorithm (ga) for URDF models',
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/knowledgetechnologyuhh/gaikpy",
    platforms='Posix; Linux; MacOS X; Windows',
    #package_dir={'gaikpy':'gaikpy'},
    #packages=find_packages(),
    packages=['gaikpy','gaikpy.examples','gaikpy.resources'],
    #packages=find_packages(where='./.'),
    #packages=find_packages(where='./src'),
    #packages=find_packages(),
    package_dir={
        'gaikpy': './src/gaikpy/',
        'gaikpy.examples': './examples/gaikpy/',
        'gaikpy.resources': './resources/',
    },
    
    #include_package_data=True,
    setup_requires=(
        'pytest-runner',
    #    'setuptools_scm'
    ),
    #package_data={'': ['resources/data/nico/*.p','resources/urdf/nico/*.urdf','resources/urdf/nico/meshes/*.STL']},
    package_data={'gaikpy.resources': ['data/nico/nico_right_20_new.p',
                                        'urdf/nico/complete_modified_bounds.urdf',
                                        'urdf/nico/meshes/*.STL'
                                        ]},

    #package_data = {
    #'' : ['*.STL'],
    #'' : ['*.p'],
    #'' : ['complete_modified_bounds.urdf'],
    #},


    install_requires=['Sphinx', 'm2r2','sphinxcontrib-napoleon',
                    'numpy', 'scipy==1.4.1', 'sympy', 'math3d', 
                    "matplotlib","transforms3d","urdfpy",
                    "ikpy==3.0.1"],
    classifiers=[
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Natural Language :: English',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        "Topic :: Scientific/Engineering",
    ],
    
    tests_require=(
        'pytest-cov',
    ),

    scripts=['demo.py']
)