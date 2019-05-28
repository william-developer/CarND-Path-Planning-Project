# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## The project consists of three parts
* PREDICTION

the prediction component estimates what actions other objects might take in the future.In this prediction module, we use three status,car head,car left,car right, and find out close following.
when check_car_lane equals current lane,I define car head. when check_car_lane is more than lane,I define car right. when check_car_lane is less than lane,I define car left.
and When their distance is less than 30 meters,I think they are too closed.The code is as follows
```
// current lane
if (lane == check_car_lane) {
 if (abs(car_s-check_car_s)<30) {
   car_ahead = true;
 }
}

// right lane
if (lane < check_car_lane) {
 if (abs(car_s-check_car_s)<30) {
   car_right = true;
 }
}
// left lane
if (lane > check_car_lane) {
 if (abs(car_s-check_car_s)<30) {
   car_left = true;
 }
}


```
* BEHAVIOUR

the behavioral planning component determines what behavior the vehicle should exhibit at any point in time.
If there is no car ahead, keep the speed within 49.5.
Otherwise,
If there is no car on the left and it is not in the leftmost lane, choose the left lane.
If there is no car on the right and there is no more right lane, choose the right lane.
If there are cars on the left and right, slow down.
```
if(car_ahead) {
    if(!car_left && lane -1>= 0) {
      lane--;
    } else if(!car_right && lane+1 <=2) {
      lane++;
    } else {
      ref_vel -= .224;
    }
} else if(ref_vel < 49.5){
    ref_vel += .224;
}
```
* TRAJECTORY

based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
reference x,y,yaw states
```
double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);
```
init data,if previous size is empty, use the car as starting reference
```
if (previous_size < 2) {
double prev_car_x = car_x - cos(car_yaw);
double prev_car_y = car_y - sin(car_yaw);

ptsx.push_back(prev_car_x);
ptsx.push_back(car_x);
ptsy.push_back(prev_car_y);
ptsy.push_back(car_y);
}
else {
ref_x = previous_path_x[previous_size-1];
ref_y = previous_path_y[previous_size-1];
double ref_x_prev = previous_path_x[previous_size-2];
double ref_y_prev = previous_path_y[previous_size-2];
ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

// use two points that make the path tangent to the previous path's end point
ptsx.push_back(ref_x_prev);
ptsx.push_back(ref_x);
ptsy.push_back(ref_y_prev);
ptsy.push_back(ref_y);
}
```
In frenet add evenly 30m spaced points ahead of the starting reference
```
vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

for (int i=0; i<ptsx.size(); i++) {
// shift car reference angle to 0 degrees
double shift_x = ptsx[i]-ref_x;
double shift_y = ptsy[i]-ref_y;
ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
}
```
We use a simple method called spline to generate trajectory
```
tk::spline s;
s.set_points(ptsx, ptsy);
```
The number of points has to be calculated is 50. We already added remaining previous points to the vectors next_x_vals and next_y_vals. We need to get the spline points of 50-previous_points.
```
for (int i = 1; i <= 50-previous_path_x.size(); ++i) {
    double N = target_dist/(.02*ref_vel/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
    y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
}

```
