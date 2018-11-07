function set_matlab_utils_path()

path_to_utils = '../../../../matlab/DMP_EKF/utils/';

addpath(path_to_utils);
addpath([path_to_utils '/lib/DMP_lib/']);
addpath([path_to_utils '/lib/math_lib/']);
addpath([path_to_utils '/lib/io_lib/']);
addpath([path_to_utils '/lib/filter_lib/']);
addpath([path_to_utils '/lib/KalmanFilter_lib/']);

end
