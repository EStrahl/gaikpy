from setuptools import find_packages, setup

setup(
    name='gaikpy',
    version='0.2.0',
    author='Erik Strahl',
    author_email='strahl@informatik.uni-hamburg.de',
    description='Calculates and visualises forward and (full-pose) inverse kinematic realised with a genetic algorithm (ga) for URDF models',
    platforms='Posix; Linux; MacOS X; Windows',
    packages=find_packages(where='./src'),
    package_dir={
        '': 'src'
    },
    include_package_data=True,
    setup_requires=(
        'pytest-runner',
    ),
    install_requires=['numpy', 'scipy==1.4.1', 'sympy', 'math3d', "matplotlib","transforms3d","urdfpy","ikpy==3.0.1"],  
    classifiers=[
        
        'Natural Language :: English',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        "Topic :: Scientific/Engineering",
    ],
    
    tests_require=(
        'pytest-cov',
    )
)