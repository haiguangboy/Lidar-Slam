rosbag问题汇总

  gnss:

​        topic_name: /ublox_node/fix

​        frame_id: imu_link

​        queue_size: 1000000

rosbag play 2019-04-28-20-58-02.bag
[ INFO] [1618408239.556823752]: Opening 2019-04-28-20-58-02.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time: 1556456283.040277   Duration: 0.000000 / 487.531016        [RUNNING]  Bag Time: 1556456283.041300   Duration: 0.001023 / 487.531016  



 gnss:

​        topic_name: /navsat/fix 

​        frame_id: imu_link

​        queue_size: 1000000

位于：test_frame_node.cpp

![设定数据格式](doc/设定数据格式.png)

