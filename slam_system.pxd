from libcpp.string cimport string
from io_wrapper cimport Output3DWrapper

cdef extern from "<Eigen/Eigen>" namespace "Eigen":
    cdef cppclass Matrix3f:
        Matrix3f() except + 
        Matrix3f(int rows, int cols) except + 
        float * data()
        
cdef extern from "<util/settings.h>" namespace "lsd_slam":
    # keystrokes
    extern bint autoRun
    extern bint autoRunWithinFrame
    extern int debugDisplay
    extern bint displayDepthMap
    extern bint onSceenInfoDisplay
    extern bint dumpMap
    extern bint doFullReConstraintTrack
    
    # dyn
    extern bint printPropagationStatistics
    extern bint printFillHolesStatistics
    extern bint printObserveStatistics
    extern bint printObservePurgeStatistics
    extern bint printRegularizeStatistics
    extern bint printLineStereoStatistics
    extern bint printLineStereoFails
    
    extern bint printTrackingIterationInfo
    extern bint printThreadingInfo
    
    extern bint printKeyframeSelectionInfo
    extern bint printConstraintSearchInfo
    extern bint printOptimizationInfo
    extern bint printRelocalizationInfo
    
    extern bint printFrameBuildDebugInfo
    extern bint printMemoryDebugInfo
    
    extern bint printMappingTiming
    extern bint printOverallTiming
    extern bint plotTrackingIterationInfo
    extern bint plotSim3TrackingIterationInfo
    extern bint plotStereoImages
    extern bint plotTracking
    
    
    extern bint allowNegativeIdepths
    extern bint useMotionModel
    extern bint useSubpixelStereo
    extern bint multiThreading
    extern bint useAffineLightningEstimation
    
    extern float freeDebugParam1
    extern float freeDebugParam2
    extern float freeDebugParam3
    extern float freeDebugParam4
    extern float freeDebugParam5
    
    
    extern float KFDistWeight
    extern float KFUsageWeight
    extern int maxLoopClosureCandidates
    extern int propagateKeyFrameDepthCount
    extern float loopclosureStrictness
    extern float relocalizationTH
    
    
    extern float minUseGrad
    extern float cameraPixelNoise2
    extern float depthSmoothingFactor
    
    extern bint useFabMap
    extern bint doSlam
    extern bint doKFReActivation
    extern bint doMapping
    
    extern bint saveKeyframes
    extern bint saveAllTracked
    extern bint saveLoopClosureImages
    extern bint saveAllTrackingStages
    extern bint saveAllTrackingStagesInternal
    
    extern bint continuousPCOutput

cdef extern from "SlamSystem.h" namespace "lsd_slam":
    ctypedef struct Frame:
        pass

    cdef cppclass SlamSystem:
        SlamSystem(int, int, Matrix3f, bool) except +
        
        void randomInit(unsigned char* image, double timeStamp, int id)
        void trackFrame(unsigned char* image, unsigned int frameID, bint blockUntilMapped, double timestamp) nogil
        
        void finalize()
        
        void setVisualization(Output3DWrapper*)
        
        Frame* getCurrentKeyframe()
        
cdef extern from "<util/Undistorter.h>" namespace "lsd_slam":
    cdef cppclass Undistorter:
        Undistorter() except +

        void undistort2(unsigned char*, unsigned char*)
        
        int getOutputWidth()
        int getOutputHeight()
    
        @staticmethod
        Undistorter* getUndistorterForFile(const char* configFilename)

    
