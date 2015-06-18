from slam_system cimport Matrix3f
from slam_system cimport SlamSystem 
cimport slam_system as sls
cimport numpy as np
import numpy as np

cdef class Slam_Context:
    cdef SlamSystem *thisptr
    def __cinit__(self, int w, int h,float[::1] K, enableSLAM=True):
        self.test()
        cdef Matrix3f _K
        cdef float * K_d = _K.data()

        for x in range(9):
            K_d[x] = K[x]
        self.thisptr = new SlamSystem(w, h, _K, enableSLAM)
        
    def __init__(self, int w, int h, enableSLAM=True):
        pass
             
    def __dealloc__(self):
        del self.thisptr
        
    def init(self, unsigned char[:,::1] image,int id, double ts):
        self.thisptr.randomInit(&image[0,0], ts, id)
        
    def track_frame(self,unsigned char[:,::1] image, int id, double ts, bint blockedUntilMapped):
        with nogil:
            self.thisptr.trackFrame(&image[0,0], id, blockedUntilMapped, ts)
        
    def finalize(self):
        self.thisptr.finalize()
        
    cdef test(self):
        sls.minUseGrad = 5       #1, 50
        sls.cameraPixelNoise2 = 16  #1, 50
        
        sls.KFUsageWeight = 4.0   #0.0, 20
        sls.KFDistWeight = 3.0    #0.0, 20
        
        sls.doSlam = True
        sls.doKFReActivation = True
        sls.doMapping = True
        sls.useFabMap = False

        sls.allowNegativeIdepths = True
        sls.useSubpixelStereo = True
        sls.useAffineLightningEstimation = False
        sls.multiThreading = True
        
        sls.maxLoopClosureCandidates = 10 #0, 50
        sls.loopclosureStrictness = 1.5 #0.0, 100
        sls.relocalizationTH = 0.7 #0, 1
        
        sls.depthSmoothingFactor = 1 # 0, 10

    #def set_output_wrapper(self, output_wrapper):
    #   self.thisptr.setVisualization(output_wrapper)

cdef class Slam_Undistorter:
    cdef sls.Undistorter *thisptr
    
    def __cinit__(self,camera_config_path):
        pass
    def __init__(self,camera_config_path):
        self.thisptr = sls.Undistorter.getUndistorterForFile(camera_config_path)
    
    def get_output_size(self):
        height,width = self.thisptr.getOutputWidth(), self.thisptr.getOutputHeight()
        return width,height
    
    def undistort(self, unsigned char[:,::1] raw_image):
        cdef np.ndarray[np.uint8_t,ndim=2] undistorted_image = np.zeros(self.get_output_size(),dtype=np.uint8)
        self.thisptr.undistort2(&raw_image[0,0], &undistorted_image[0,0])
        return undistorted_image