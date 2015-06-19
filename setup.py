from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import numpy as np
extensions = [
    Extension( 
        name="pySLAM",
        sources=['pySLAM.pyx', 'SimpleOutput3DWrapper.cpp'],
        include_dirs = [ np.get_include(),
                        'lsd_slam/lsd_slam_core/src',
                        'lsd_slam/lsd_slam_core/thirdparty/Sophus', 
                        '/usr/include/eigen3'],
        #include_dirs = [ np.get_include(),
        #          '../lsd_slam_noros/lsd_slam',
        #          '../lsd_slam_noros/thirdparty/Sophus', 
        #          '/usr/include/eigen3'],
        libraries = ['lsdslam'],
        library_dirs = ['lsd_slam/lsd_slam_core/lib'],
        #library_dirs = ['../lsd_slam_noros/lib'],
        extra_link_args=['-Wl,-Rlsd_slam/lsd_slam_core/lib,-R/usr/local/lib'],
        #extra_link_args=['-Wl,-R../lsd_slam_noros/lib,-R/usr/local/lib'],
        extra_compile_args=["-std=c++11"],
        language="c++")
]

setup( 
    name="pySLAM",
    version="0.1",
    packages = ['pySLAM'],
    description="OpenGL UI powered by Cython",
    url="https://github.com/pupil-labs/pySLAM",
    author='Pupil Labs',
    author_email='info@pupil-labs.com',
    license='MIT',
    ext_modules=cythonize(extensions)
)