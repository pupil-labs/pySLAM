from libcpp.string cimport string
from io_wrapper cimport Output3DWrapper

#cdef extern from "<Eigen/Eigen.h>" namespace "Eigen":
    

cdef extern from "SlamSystem.h" namespace "lsd_slam":
    ctypedef struct Matrix3f:
        pass
    
    #ctypedef struct Output3DWrapper:
    #    pass
    ctypedef struct Frame:
        pass

    cdef cppclass SlamSystem:
        SlamSystem(int, int, Matrix3f, bool) except +
        
        void randomInit(unsigned char*, double, int)
        void trackFrame(unsigned char*, unsigned int, bool, double)
        
        void finalize()
        
        void setVisualization(Output3DWrapper*)
        
        Frame* getCurrentKeyframe()