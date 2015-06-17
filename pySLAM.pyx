from slam_system cimport Matrix3f
from slam_system cimport SlamSystem 
from slam_system cimport Undistorter

cdef class Slam_Context:
    cdef SlamSystem *thisptr
    def __cinit__(self, int w, int h,float[::1] K, enableSLAM=True):
        cdef Matrix3f _K
        cdef float * K_d = _K.data()

        for x in range(9):
            K_d[x] = K[x]
            print K_d[x]
        self.thisptr = new SlamSystem(w, h, _K, enableSLAM)
        
    def __init__(self, int w, int h, enableSLAM=True):
        pass
             
    def __dealloc__(self):
        del self.thisptr
        
    def init(self, unsigned char[:,::1] image,int id, double ts):
        self.thisptr.randomInit(&image[0,0], ts, id)
        
    def track_frame(self,unsigned char[:,::1] image, int id, double ts, bint blockedUntilMapped):
        self.thisptr.trackFrame(&image[0,0], id, blockedUntilMapped, ts)
        
    def finalize(self):
        self.thisptr.finalize()

    #def set_output_wrapper(self, output_wrapper):
    #   self.thisptr.setVisualization(output_wrapper)

cdef class Slam_Undistorter:
    cdef Undistorter *thisptr
    
    def __cinit__(self):
        pass
    
    def get_instance(self, camera_config_path):
        self.thisptr = Undistorter.getUndistorterForFile(camera_config_path)