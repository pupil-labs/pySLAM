from libcpp.string cimport string
from libcpp.vector cimport vector 

cdef extern from "<Eigen/Eigen>" namespace "Eigen":
    cdef cppclass Matrix3f:
        Matrix3f() except + 
        Matrix3f(int rows, int cols) except + 
        float * data()
    cdef cppclass Vector3f:
        Vector3f() except + 
        Vector3f(int rows, int cols) except + 
        float * data()
        
cdef extern from "<sophus/se3.hpp>" namespace "Sophus":
    cdef cppclass SE3f:
        pass
        
cdef extern from "<util/SophusUtil.h>":
    ctypedef SE3f SE3
        
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
        SlamSystem(int w, int h, Matrix3f K, bint enableSLAM) except +
        
        int width, height
        Matrix3f K
        const bint SLAMEnabled
        bint trackingIsGood
        
        void randomInit(unsigned char* image, double timeStamp, int id)
        void gtDepthInit(unsigned char* image, float* depth, double timeStamp, int id)
        void trackFrame(unsigned char* image, unsigned int frameID, bint blockUntilMapped, double timestamp) nogil
        void finalize()
        void optimizeGraph()        
        Frame* getCurrentKeyframe()
        SE3 getCurrentPoseEstimation()
        void setVisualization(Output3DWrapper*)
        void requestDepthMapScreenshot(const string& filename)
        bint doMappingIteration()
        int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bint forceParent=True, bint useFABMAP=True, float closeCandidatesTH=1.0)
        bint optimizationIteration(int itsPerTry, float minChange)
        void publishKeyframeGraph()
        #vector[FramePoseStruct*, Eigen::aligned_allocator<lsd_slam::FramePoseStruct*> > getAllPoses();
        
        float msTrackFrame, msOptimizationIteration, msFindConstraintsItaration, msFindReferences
        int nTrackFrame, nOptimizationIteration, nFindConstraintsItaration, nFindReferences
        float nAvgTrackFrame, nAvgOptimizationIteration, nAvgFindConstraintsItaration, nAvgFindReferences

cdef extern from "<IOWrapper/Output3DWrapper.h>" namespace "lsd_slam":
    ctypedef struct Frame:
        pass
    
    cdef cppclass Output3DWrapper:
        pass
 
cdef extern from "SimpleOutput3DWrapper.h" namespace "lsd_slam":
    ctypedef struct Frame:
        pass
 
    cdef cppclass SimpleOutput3DWrapper(Output3DWrapper):
        void publishKeyframe(Frame* kf)
          
        void publishTrackedFrame(Frame* kf)
          
        void publishTrajectory(vector[Vector3f] trajectory, string identifier)
        void publishTrajectoryIncrement(Vector3f pt, string identifier)
  
        void test()
        #void publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
        
        @staticmethod
        SimpleOutput3DWrapper* getInstance()


cdef extern from "<util/Undistorter.h>" namespace "lsd_slam":
    cdef cppclass Undistorter:
        Undistorter() except +

        void undistort2(unsigned char*, unsigned char*)
        
        int getOutputWidth()
        int getOutputHeight()
    
        @staticmethod
        Undistorter* getUndistorterForFile(const char* configFilename)

    
