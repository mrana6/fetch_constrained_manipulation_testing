function arm_model = generateFetchArm(arm_origin)

import gtsam.*
import gpmp2.*

base_pose = Pose3(Rot3(eye(3)), Point3(arm_origin));

theta = [0, -pi/2, 0, 0, 0, 0, 0]';
d = [0.06, 0, 0.219+0.133, 0, 0.197+0.1245, 0, 0.1385+0.16645]';
alpha_r = [-pi/2 -pi/2 pi/2 -pi/2 pi/2 -pi/2 0]';
a = [0.117, 0, 0, 0, 0, 0, 0]';
abs_arm = Arm(7, a, alpha_r, d, base_pose, theta);
% physical arm
% sphere data [id x y z r]
spheres_data = [...
   0   0.0   0.0  0.0  0.01

   1   0.0  0.0    0.0  0.01   

   2   0.0  0.0    0.0  0.01
   
   3   0.0  0.0    0.0  0.01
   
   4   0.0  0.0    0.0  0.01
   
   5   0.0  0.0    0.0  0.01
   
   6  0.0  0.0  0.0  0.01
];

nr_body = size(spheres_data, 1);

sphere_vec = BodySphereVector;
for i=1:nr_body
    sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
        Point3(spheres_data(i,2:4)')));
end
arm_model = ArmModel(abs_arm, sphere_vec);

end

