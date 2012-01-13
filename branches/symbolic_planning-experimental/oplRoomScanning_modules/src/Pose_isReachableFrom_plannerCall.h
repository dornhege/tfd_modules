
#ifndef RoomScanning_Pose_isReachableFrom_H_
#define RoomScanning_Pose_isReachableFrom_H_

#ifdef __cplusplus
extern "C" {
#endif

double Pose_isReachableFrom_plannerCall(const modules::ParameterList & parameterList,
        modules::predicateCallbackType predicateCallback,
        modules::numericalFluentCallbackType numericalFluentCallback,
        int relaxed);

#ifdef __cplusplus
}
#endif

#endif 
