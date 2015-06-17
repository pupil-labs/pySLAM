from libcpp.string cimport string
from io_wrapper cimport Output3DWrapper

cdef extern from "<Eigen/Eigen>" namespace "Eigen":
    cdef cppclass Matrix3f:
        Matrix3f() except + 
        Matrix3f(int rows, int cols) except + 
        float * data()

cdef extern from "SlamSystem.h" namespace "lsd_slam":
    ctypedef struct Frame:
        pass

    cdef cppclass SlamSystem:
        SlamSystem(int, int, Matrix3f, bool) except +
        
        void randomInit(unsigned char* image, double timeStamp, int id)
        void trackFrame(unsigned char* image, unsigned int frameID, bint blockUntilMapped, double timestamp)
        
        void finalize()
        
        void setVisualization(Output3DWrapper*)
        
        Frame* getCurrentKeyframe()
        
cdef extern from "<util/Undistorter.h>" namespace "lsd_slam":
    cdef cppclass Undistorter:
        Undistorter() except +
        
        @staticmethod
        Undistorter* getUndistorterForFile(const char* configFilename)
        
        void undistort(unsigned char*, unsigned char*)
        