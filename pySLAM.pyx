cimport slam_system as slam
cimport numpy as np
import numpy as np

cdef class Slam_Context:
    cdef slam.SlamSystem *thisptr
    cdef slam.Output3DWrapper* vis
    
    def __cinit__(self, int w, int h,float[::1] K, enableSLAM=True):
        self.configure()
        self.vis = slam.SimpleOutput3DWrapper.getInstance()
        self.set_visualization(self.vis)
        print "got it"
        cdef slam.Matrix3f _K
        cdef float * K_d = _K.data()

        for x in range(9):
            K_d[x] = K[x]
        self.thisptr = new slam.SlamSystem(w, h, _K, enableSLAM)
        
    def __init__(self, int w, int h, enableSLAM=True):
        pass
             
    def __dealloc__(self):
        del self.thisptr
        
    @property
    def size(self):
        return self.thisptr.width, self.thisptr.height
        
    @property
    def K(self):
        ret = np.ndarray(shape=(9), dtype=np.float32)
        cdef float* tmp_K = self.thisptr.K.data()
        for x in range(9):
            ret[x] = tmp_K[x]
        return ret
        
    def init(self, unsigned char[:,::1] image,int id, double ts):
        self.thisptr.randomInit(&image[0,0], ts, id)
        
    def track_frame(self,unsigned char[:,::1] image, int id, double ts, bint blockedUntilMapped):
        with nogil:
            self.thisptr.trackFrame(&image[0,0], id, blockedUntilMapped, ts)
        
    def finalize(self):
        self.thisptr.finalize()

    @property
    def minUseGrad(self):
        return slam.minUseGrad
    @minUseGrad.setter
    def minUseGrad(self, value):
        slam.minUseGrad = value
        
    @property
    def cameraPixelNoise2(self):
        return slam.cameraPixelNoise2
    @cameraPixelNoise2.setter
    def cameraPixelNoise2(self, value):
        slam.cameraPixelNoise2 = value
        
    @property
    def KFUsageWeight(self):
        return slam.KFUsageWeight
    @KFUsageWeight.setter
    def KFUsageWeight(self, value):
        slam.KFUsageWeight = value
        
    @property
    def KFDistWeight(self):
        return slam.KFDistWeight
    @KFDistWeight.setter
    def KFDistWeight(self, value):
        slam.KFDistWeight = value
        
    @property
    def doSlam(self):
        return slam.doSlam
    @doSlam.setter
    def doSlam(self, value):
        slam.doSlam = value
        
    @property
    def doKFReActivation(self):
        return slam.doKFReActivation
    @doKFReActivation.setter
    def doKFReActivation(self, value):
        slam.doKFReActivation = value
        
    @property
    def doMapping(self):
        return slam.doMapping
    @doMapping.setter
    def doMapping(self, value):
        slam.doMapping = value
        
    @property
    def useFabMap(self):
        return slam.useFabMap
    @useFabMap.setter
    def useFabMap(self, value):
        slam.useFabMap = value
        
    @property
    def allowNegativeIdepths(self):
        return slam.allowNegativeIdepths
    @allowNegativeIdepths.setter
    def allowNegativeIdepths(self, value):
        slam.allowNegativeIdepths = value
        
    @property
    def useSubpixelStereo(self):
        return slam.useSubpixelStereo
    @useSubpixelStereo.setter
    def useSubpixelStereo(self, value):
        slam.useSubpixelStereo = value
        
    @property
    def useAffineLightningEstimation(self):
        return slam.useAffineLightningEstimation
    @useAffineLightningEstimation.setter
    def useAffineLightningEstimation(self, value):
        slam.useAffineLightningEstimation = value
        
    @property
    def multiThreading(self):
        return slam.multiThreading
    @multiThreading.setter
    def multiThreading(self, value):
        slam.multiThreading = value
        
    @property
    def maxLoopClosureCandidates(self):
        return slam.maxLoopClosureCandidates
    @maxLoopClosureCandidates.setter
    def maxLoopClosureCandidates(self, value):
        slam.maxLoopClosureCandidates = value
        
    @property
    def loopclosureStrictness(self):
        return slam.loopclosureStrictness
    @loopclosureStrictness.setter
    def loopclosureStrictness(self, value):
        slam.loopclosureStrictness = value
        
    @property
    def relocalizationTH(self):
        return slam.relocalizationTH
    @relocalizationTH.setter
    def relocalizationTH(self, value):
        slam.relocalizationTH = value
        
    @property
    def depthSmoothingFactor(self):
        return slam.depthSmoothingFactor
    @depthSmoothingFactor.setter
    def depthSmoothingFactor(self, value):
        slam.depthSmoothingFactor = value
    
    def configure(self):
        slam.minUseGrad = 5       #1, 50
        slam.cameraPixelNoise2 = 16  #1, 50
        
        slam.KFUsageWeight = 4.0   #0.0, 20
        slam.KFDistWeight = 3.0    #0.0, 20
        
        slam.doSlam = True
        slam.doKFReActivation = True
        slam.doMapping = True
        slam.useFabMap = False

        slam.allowNegativeIdepths = True
        slam.useSubpixelStereo = True
        slam.useAffineLightningEstimation = False
        slam.multiThreading = True
        
        slam.maxLoopClosureCandidates = 10 #0, 50
        slam.loopclosureStrictness = 1.5 #0.0, 100
        slam.relocalizationTH = 0.7 #0, 1
        
        slam.depthSmoothingFactor = 1 # 0, 10

    cdef set_visualization(self, slam.Output3DWrapper* output_wrapper):
        self.thisptr.setVisualization(output_wrapper)

cdef class Slam_Undistorter:
    cdef slam.Undistorter *thisptr
    
    def __cinit__(self,camera_config_path):
        pass
    def __init__(self,camera_config_path):
        self.thisptr = slam.Undistorter.getUndistorterForFile(camera_config_path)
    
    def get_output_size(self):
        height,width = self.thisptr.getOutputWidth(), self.thisptr.getOutputHeight()
        return width,height
    
    def undistort(self, unsigned char[:,::1] raw_image):
        cdef np.ndarray[np.uint8_t,ndim=2] undistorted_image = np.zeros(self.get_output_size(),dtype=np.uint8)
        self.thisptr.undistort2(&raw_image[0,0], &undistorted_image[0,0])
        return undistorted_image
    
cdef class SimpleOutput3DWrapper(Output3DWrapper):
    cdef slam.SimpleOutput3DWrapper *thisptr
    
    def __cinit__(self,camera_config_path):
        pass
    def __init__(self,camera_config_path):
        self.thisptr = slam.SimpleOutput3DWrapper.getInstance()

cdef class Output3DWrapper:
    pass
#     cdef slam.Output3DWrapper *thisptr
#     
#     def __cinit__(self):
#         if type(self) is Output3DWrapper:
#             self.thisptr = new slam.Output3DWrapper()
#             print "constr"
#             print self
#     def __dealloc__(self):
#         if type(self) is Output3DWrapper:
#             print "destr"
#             print self
#             del self.thisptr
    
# cdef class SimpleOutput3DWrapper(Output3DWrapper):
#     cdef slam.SimpleOutput3DWrapper *derptr
#      
#     def __cinit__(self):
#         if type(self) is SimpleOutput3DWrapper:
#             self.derptr = new slam.SimpleOutput3DWrapper()
#             self.thisptr = self.derptr
#             print "constr"
#             self.derptr.test()
#             print self
#     def __dealloc__(self):
#         if type(self) is SimpleOutput3DWrapper:
#             print "destr"
#             print self
#             del self.derptr
#             del self.thisptr