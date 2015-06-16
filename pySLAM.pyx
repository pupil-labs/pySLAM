from slam_system cimport Matrix3f
from slam_system cimport SlamSystem 
'''
cdef class pySLAM:
    cdef SlamSystem *thisptr
    def __cinit__(self, int w, int h, K, enableSLAM=True):
        self.thisptr = new SlamSystem(w, h, K, enableSLAM)
    def __dealloc__(self):
        del self.thisptr
        
    def init(self, image, ts, id):
        self.thisptr.randomInit(image, ts, id)
    def track_frame(self, image, id, blockedUntilMapped, ts):
        self.thisptr.trackFrame(image, id, blockedUntilMapped, ts)
    def finalize(self):
        self.thisptr.finalize()
        
    #def set_output_wrapper(self, output_wrapper):
    #   self.thisptr.setVisualization(output_wrapper)
'''