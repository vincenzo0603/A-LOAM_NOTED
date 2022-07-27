//lidar坐标系:x轴向前，y轴向左，z轴向上的右手坐标系
//处理频率:与激光帧率一致
void FeatureDt::getFeaturePoints(pcl::PointCloud<PointType> &laser_cloud_in,
                                          uint64_t ts,
                                          pcl::PointCloud<PointType>::Ptr &laser_cloud_in_range,
                                          pcl::PointCloud<PointType>::Ptr &corner_sharp,
                                          pcl::PointCloud<PointType>::Ptr &corner_less_sharp,
                                          pcl::PointCloud<PointType>::Ptr &surf_flat,
                                          pcl::PointCloud<PointType>::Ptr &surf_less_flat)
{
  int cloud_in_size = laser_cloud_in.points.size();

  //获取点云的开始和结束水平角度, 确定一帧中点的角度范围
  //此处需要注意一帧扫描角度不一定<2pi, 可能大于2pi, 角度需特殊处理
  //角度范围用于确定每个点的相对扫描时间, 用于运动补偿
  float start_yaw;
  float end_yaw;
  getYawRange(laser_cloud_in, cloud_in_size, start_yaw, end_yaw);

  //至于此处half_passed的作用, 文中有详述
  bool half_passed = false;

  int cloud_filted_size = cloud_in_size;
  PointType point;

  //每一线存储为一个单独的线(SCAN), 针对单个线计算特征点
  std::vector<PointCloudType> laser_cloud_per_scan(N_SCANS);

  //把点根据几何角度(竖直)分配到线中
  for (int i = 0; i < cloud_in_size; i++)
  {
    point.x = laser_cloud_in.points[i].x;
    point.y = laser_cloud_in.points[i].y;
    point.z = laser_cloud_in.points[i].z;

    //scan_id由竖直角度映射而得, 具体参考lidar的user manual
    int scan_id = getScanIDOfPoint(point);
    if (scan_id > (N_SCANS - 1) || scan_id < 0)
    {
      cloud_filted_size--;
      continue;
    }

    float point_yaw;
    getYawOfOnePoint(start_yaw, end_yaw, point, point_yaw, half_passed);

    //计算此点在此帧中扫描的相对时间, 用来作为后续的运动补偿
    float yaw_percent = (point_yaw - start_yaw) / (end_yaw - start_yaw);

    //复用这个字段传递参数,整数部分是SCANID, 小数部分是相对时间
    point.intensity = toIntensity(scan_id, yaw_percent);

    //往对应的scan中, 新增一个point
    laser_cloud_per_scan[scan_id].push_back(point);
  }

  //将点云按scan顺序排列, 重新组成大点云
  for (int i = 0; i < N_SCANS; i++)
  {
    *laser_cloud_in_range += laser_cloud_per_scan[i];
  }

  //记录每个scanid的开始和结束点
  std::vector<int> scan_start_ind(N_SCANS, 0);
  std::vector<int> scan_end_ind(N_SCANS, 0);

  calcCurvature(laser_cloud_in_range, cloud_filted_size, scan_start_ind, scan_end_ind);

  detectFeaturePoint(laser_cloud_in_range, cloud_filted_size, scan_start_ind, scan_end_ind, corner_sharp, corner_less_sharp, surf_flat, surf_less_flat);
}

//这三个函数涉及服用点云中点的intersity字段,存储scanid(intensity的整数部分)和相对扫描时间(intensity小数部分)
inline float toIntensity(int scanID, float yawPercent)
{
    return scanID + yawPercent;
}

inline int toScanID(float intensity)
{
    return int(intensity);
}

inline float toReltiveTime(float intensity)
{
    return intensity - int(intensity);
}

void FeatureDt::getYawRange(pcl::PointCloud<PointType> &laser_cloud_in, 
                            int cloud_size, 
                            float &start_yaw, float &end_yaw
                            )
{
  //第一个点和最后一个点对应的是第一和最末一束线
  //velodyne是顺时针增大, 而坐标轴中的yaw是逆时针增加, 所以这里要取负号
  start_yaw = -atan2(laser_cloud_in.points[0].y, laser_cloud_in.points[0].x);
  end_yaw = -atan2(laser_cloud_in.points[cloud_size - 1].y, laser_cloud_in.points[cloud_size - 1].x) + 2 * M_PI;

  //atan2得到的角度是[-pi, pi], 所以， 如果都用标准化的角度， 那么end_yaw可能小于或者接近start_yaw， 这不利于后续的运动补偿
  //因为运动补偿需要从每个点的水平角度确定其在一帧中的相对时间
  //我们需要转换到end_yaw > start_yaw 且end_yaw-start_yaw接近2*M_PI的形式， 所以就有了以下代码
  if (end_yaw - start_yaw > 3 * M_PI)
  {
    end_yaw -= 2 * M_PI;
  }
  else if (end_yaw - start_yaw < M_PI)
  {
    end_yaw += 2 * M_PI;
  }
}

//yaw决定了点的相对扫描时间
void FeatureDt::getYawOfOnePoint(float &start_yaw, 
                                  float &end_yaw, 
                                  PointType point, 
                                  float &yaw, 
                                  bool &half_passed
                                   )
{
  yaw = -atan2(point.y, point.x);

  //因为转一圈可能会超过2pi， 故角度a可能对应a或者2pi + a
  //如何确定是a还是2pi+a呢， half_passed 利用点的顺序与时间先后相关这一点解决了这个问题
  if (!half_passed)
  {
    if (yaw < start_yaw - M_PI / 2)
    {
      yaw += 2 * M_PI;
    }
    else if (yaw > start_yaw + M_PI * 3 / 2)
    {
      yaw -= 2 * M_PI;
    }

    if (yaw - start_yaw > M_PI)
    {
      half_passed = true;
    }
  }
  else
  {
    yaw += 2 * M_PI;

    if (yaw < end_yaw - M_PI * 3 / 2)
    {
      yaw += 2 * M_PI;
    }
    else if (yaw > end_yaw + M_PI / 2)
    {
      yaw -= 2 * M_PI;
    }
  }
}

//计算曲率
void FeatureDt::calcCurvature(pcl::PointCloud<PointType>::Ptr laser_cloud, 
                             int cloud_size, 
                             std::vector<int> &scan_start_ind, 
                             std::vector<int> &scan_end_ind
                            )
{
  int scan_count = -1;

  //针对每个点求其特征
  for (int i = 5; i < cloud_size - 5; i++)
  {
    //用周围的10个点计算其描述子, 边界的点省略掉， 因为他们周围的点不足5个
    float diff_x = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
    float diff_y = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
    float diff_z = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;

    //曲率计算公式
    cloud_curvature_[i] = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
    cloud_sort_ind_[i] = i;

    //特征点默认为less_smooth, 后续会在曲率筛选中改变这个label
    //neighbor_picked意义在于,当一个点被选为corner或者surf时, 周边N个点不要被选, 让特征点尽量分布广, 不聚集
    cloud_neighbor_picked_[i] = 0;
    cloud_label_[i] = CURV_LESS_SMOOTH;

    if (toScanID(laser_cloud->points[i].intensity) != scan_count)
    {
      scan_count = toScanID(laser_cloud->points[i].intensity);

      if (scan_count > 0 && scan_count < N_SCANS)
      {
        //设置本scan的头
        scan_start_ind[scan_count] = i + 5;

        //设置上个scan的尾
        scan_end_ind[scan_count - 1] = i - 5;
      }
    }
  }

  //最前和最后5个点不方便计算曲率, 抛弃
  //不要认为一个SCAN首尾可以相接, 运动状态导致的畸变会使得首尾差距很大
  scan_start_ind[0] = 5;
  scan_end_ind.back() = cloud_size - 5;

  //paper中(a) (b)这两种特殊情况的点不会被选为corner orsurface
  for (int i = 5; i < cloud_size - 6; i++)
  {
    //计算曲率
    float diff_x = laser_cloud->points[i + 1].x - laser_cloud->points[i].x;
    float diff_y = laser_cloud->points[i + 1].y - laser_cloud->points[i].y;
    float diff_z = laser_cloud->points[i + 1].z - laser_cloud->points[i].z;
    float diff = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;

    //曲率阈值过滤
    if (diff > 0.1)
    {
      float depth1 = sqrt(laser_cloud->points[i].x * laser_cloud->points[i].x +
                          laser_cloud->points[i].y * laser_cloud->points[i].y +
                          laser_cloud->points[i].z * laser_cloud->points[i].z);

      float depth2 = sqrt(laser_cloud->points[i + 1].x * laser_cloud->points[i + 1].x +
                          laser_cloud->points[i + 1].y * laser_cloud->points[i + 1].y +
                          laser_cloud->points[i + 1].z * laser_cloud->points[i + 1].z);

      //针对paper中(b)情况 
      if (depth1 > depth2)
      {
        diff_x = laser_cloud->points[i + 1].x - laser_cloud->points[i].x * depth2 / depth1;
        diff_y = laser_cloud->points[i + 1].y - laser_cloud->points[i].y * depth2 / depth1;
        diff_z = laser_cloud->points[i + 1].z - laser_cloud->points[i].z * depth2 / depth1;

        if (sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / depth2 < 0.1)
        {
    
          cloud_neighbor_picked_[i - 5] = 1;
          cloud_neighbor_picked_[i - 4] = 1;
          cloud_neighbor_picked_[i - 3] = 1;
          cloud_neighbor_picked_[i - 2] = 1;
          cloud_neighbor_picked_[i - 1] = 1;
          cloud_neighbor_picked_[i] = 1;
        }
      }
      else
      {
        diff_x = laser_cloud->points[i + 1].x * depth1 / depth2 - laser_cloud->points[i].x;
        diff_y = laser_cloud->points[i + 1].y * depth1 / depth2 - laser_cloud->points[i].y;
        diff_z = laser_cloud->points[i + 1].z * depth1 / depth2 - laser_cloud->points[i].z;

        if (sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / depth1 < 0.1)
        {
          cloud_neighbor_picked_[i + 1] = 1;
          cloud_neighbor_picked_[i + 2] = 1;
          cloud_neighbor_picked_[i + 3] = 1;
          cloud_neighbor_picked_[i + 4] = 1;
          cloud_neighbor_picked_[i + 5] = 1;
          cloud_neighbor_picked_[i + 6] = 1;
        }
      }
    }

    //针对paper中(a)情况 
    float diff_x_2 = laser_cloud->points[i].x - laser_cloud->points[i - 1].x;
    float diff_y_2 = laser_cloud->points[i].y - laser_cloud->points[i - 1].y;
    float diff_z_2 = laser_cloud->points[i].z - laser_cloud->points[i - 1].z;
    float diff_2 = diff_x_2 * diff_x_2 + diff_y_2 * diff_y_2 + diff_z_2 * diff_z_2;

    float dis = laser_cloud->points[i].x * laser_cloud->points[i].x + laser_cloud->points[i].y * laser_cloud->points[i].y + laser_cloud->points[i].z * laser_cloud->points[i].z;

    if (diff > 0.0002 * dis && diff_2 > 0.0002 * dis)
    {
      cloud_neighbor_picked_[i] = 1;
    }
  }
}

void FeatureDt::detectFeaturePoint(pcl::PointCloud<PointType>::Ptr laser_cloud, 
                                  int cloud_size, 
                                  std::vector<int> &scan_start_ind, 
                                  std::vector<int> &scan_end_ind, 
                                  pcl::PointCloud<PointType>::Ptr &corner_sharp, 
                                  pcl::PointCloud<PointType>::Ptr &corner_less_sharp, 
                                  pcl::PointCloud<PointType>::Ptr &surf_flat, 
                                  pcl::PointCloud<PointType>::Ptr &surf_less_flat)
{
  //还是每束scan单独处理
  for (int i = 0; i < N_SCANS; i++)
  {
    pcl::PointCloud<PointType>::Ptr surf_points_less_flat_scan(new pcl::PointCloud<PointType>);
    int less_sharp_num = 0;

    //将每个线等分为六段，分别进行处理（sp、ep分别为各段的起始和终止位置）
    for (int j = 0; j < 6; j++)
    {
      //先求每段的开始和结束点
      int sp = (scan_start_ind[i] * (6 - j) + scan_end_ind[i] * j) / 6;
      int ep = (scan_start_ind[i] * (5 - j) + scan_end_ind[i] * (j + 1)) / 6 - 1;

      //在每一段，排序, 小的在前, 大的在后
      for (int k = sp + 1; k <= ep; k++)
      {
        for (int l = k; l >= sp + 1; l--)
        {
          if (cloud_curvature_[cloud_sort_ind_[l]] < cloud_curvature_[cloud_sort_ind_[l - 1]])
          {
            swap(cloud_sort_ind_[l - 1], cloud_sort_ind_[l]);
          }
        }
      }

      //选取角点
      int largest_picked_num = 0;
      for (int k = ep; k >= sp; k--)
      {
        //k = ep, cloud_sort_ind_[k]对应这一组最末位置的index, 也就是曲率最大的index
        int ind = cloud_sort_ind_[k];

        //如果邻居没被选中并且自己够格
        if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] > 0.1)
        {
          largest_picked_num++;

          //取x个认为是sharp的点
          if (largest_picked_num <= 6)
          {
            cloud_label_[ind] = CURV_SHARP;
            corner_sharp->push_back(laser_cloud->points[ind]);
            corner_less_sharp->push_back(laser_cloud->points[ind]);
            less_sharp_num++;
          }
          //取y个认为是lesssharp的点
          else if (largest_picked_num <= 24)
          {
            cloud_label_[ind] = CURV_LESS_SHARP;
            corner_less_sharp->push_back(laser_cloud->points[ind]);
            less_sharp_num++;
          }
          else
          {
            break;
          }

          //选中的点标记为已选
          cloud_neighbor_picked_[ind] = 1;

          //向后五个点
          for (int l = 1; l <= 5; l++)
          {
            //之前的曲率是前后各五个点, 这里计算相邻两点的变化率
            float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
            float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
            float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;

            //遇到某点曲率高不标记, 还有机会被选上
            if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
            {
              break;
            }

            //否则, 标记, 因为邻居是角点, 你不能再做角点
            cloud_neighbor_picked_[ind + l] = 1;
          }

          //向前五个点, 逻辑用上
          for (int l = -1; l >= -5; l--)
          {
            float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
            float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
            float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;

            if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
            {
              break;
            }

            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      //选取平面点
      int smallest_picked_num = 0;
      for (int k = sp; k <= ep; k++)
      {
        //!! k = sp, cloud_sort_ind_[k]对应这一组最先位置的index, 也就是曲率最小的index
        int ind = cloud_sort_ind_[k];
        if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] < 0.1)
        {
          cloud_label_[ind] = CURV_SMOOTH;
          surf_flat->push_back(laser_cloud->points[ind]);

          smallest_picked_num++;
          if (smallest_picked_num >= 8)
          {
            break;
          }

          //已选中的点, 对临近点进行标记
          cloud_neighbor_picked_[ind] = 1;

          //向后遍历五个点
          for (int l = 1; l <= 5; l++)
          {
            float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
            float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
            float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;

            //此处发生突变, 停止标记临近点
            if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
            {
              break;
            }

            cloud_neighbor_picked_[ind + l] = 1;
          }

          //向前遍历五个点, 逻辑同上
          for (int l = -1; l >= -5; l--)
          {
            float diff_x = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
            float diff_y = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
            float diff_z = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;

            if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
            {
              break;
            }

            cloud_neighbor_picked_[ind + l] = 1;
          }
        }
      }

      //取平面点 less smooth, 之前没被选中的都会被标记为less smooth
      for (int k = sp; k <= ep; k++)
      {
        // <= CURV_LESS_SMOOTH means smooth or less smooth
        if (cloud_label_[k] <= CURV_LESS_SMOOTH)
        {
          surf_points_less_flat_scan->push_back(laser_cloud->points[k]);
        }
      }
    }

    // 对lessFlatScan进行降采样
    pcl::PointCloud<PointType> surf_points_less_flat_scan_ds;
    pcl::VoxelGrid<PointType> down_size_filter;
    down_size_filter.setInputCloud(surf_points_less_flat_scan);
    down_size_filter.setLeafSize(0.15, 0.15, 0.15);
    down_size_filter.filter(surf_points_less_flat_scan_ds);

    //sp 是个step, 这里使用了一种简单的点过滤方法
    int sp = 1;

    if (less_sharp_num == 0)
    {
      sp = floor(1.0 * surf_points_less_flat_scan_ds.size() / 100);
    }
    else
    {
      sp = floor(1.0 * surf_points_less_flat_scan_ds.size() / less_sharp_num / 3);
    }

    sp = sp > 0 ? sp : 1;
    for (int k = 0; k < surf_points_less_flat_scan_ds.size(); k += sp)
    {
      surf_less_flat->push_back(surf_points_less_flat_scan_ds.points[k]);
    }
  }
}

int FeatureDt::getScanIDOfPoint(PointType &point)
{
  //线与水平面的夹角,计算线束的角度, 以确定属于哪条线, 单位 °
  float scan_pitch = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;

  //根据scanPitch确定scan ID, 范围0~N_SCANS - 1, scanID在后面寻找最近直线时将发挥重要作用
  int rounded_scan_pitch = int(scan_pitch / 2.0 + (scan_pitch < 0.0 ? -0.5 : +0.5));
  int scan_id = 0;

  //让SCANID在物理上连续
  scan_id = rounded_scan_pitch + 7;

  return scan_id;
}
laserMapping


void LaMapping::solveWithGaussNewton(cv::Mat &mat_x,
                                        int iter_count, 
					pcl::PointCloud<PointType>::Ptr points_selected 
					pcl::PointCloud<PointType>::Ptr coeff_selected)
{
	int laser_cloud_sel_num = points_selected->size();

	bool is_degenerate = false;
	cv::Mat mat_p(6, 6, CV_32F, cv::Scalar::all(0));

	//预先计算三个欧拉角的sin 和cos, 对应文章中的sin(ex), cos(ex), 
	//sin(ey), cos(ey), sin(ez), cos(ez),  
	float srx = sin(lidar_pose_in_map_r_[0]);
	float crx = cos(lidar_pose_in_map_r_[0]);
	float sry = sin(lidar_pose_in_map_r_[1]);
	float cry = cos(lidar_pose_in_map_r_[1]);
	float srz = sin(lidar_pose_in_map_r_[2]);
	float crz = cos(lidar_pose_in_map_r_[2]);
 
	//高斯牛顿求解中用到的一些矩阵， 对应正规方程（normal equation）， AT*A*x = AT*b
	cv::Mat mat_a(laser_cloud_sel_num, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat mat_a_t(6, laser_cloud_sel_num, CV_32F, cv::Scalar::all(0));
	cv::Mat mat_a_t_a(6, 6, CV_32F, cv::Scalar::all(0));
	cv::Mat mat_b(laser_cloud_sel_num, 1, CV_32F, cv::Scalar::all(0));
	cv::Mat mat_a_t_b(6, 1, CV_32F, cv::Scalar::all(0));

	//将每个观测项构建最小二乘问题, 公式i
	for (int i = 0; i < laser_cloud_sel_num; i++)
	{
		PointType point_ori, coeff;
		point_ori = points_selected->points[i];
		coeff = coeff_selected->points[i];

		//coeff.x, coeff.y, coeff.z 为loss对pose中x, y, z的偏导
		//coeff.intensity 为loss
		//以下代码中的arx计算对应公式h
		float arx = (crx * sry * srz * point_ori.x + crx * crz * sry * point_ori.y - srx * sry * point_ori.z) * coeff.x 
		           + (-srx * srz * point_ori.x - crz * srx * point_ori.y - crx * point_ori.z) * coeff.y 
				   + (crx * cry * srz * point_ori.x + crx * cry * crz * point_ori.y - cry * srx * point_ori.z) * coeff.z;
		float ary = ((cry * srx * srz - crz * sry) * point_ori.x + (sry * srz + cry * crz * srx) * point_ori.y + crx * cry * point_ori.z) * coeff.x 
				   + ((-cry * crz - srx * sry * srz) * point_ori.x + (cry * srz - crz * srx * sry) * point_ori.y  - crx * sry * point_ori.z) * coeff.z;
		float arz = ((crz * srx * sry - cry * srz) * point_ori.x  + (-cry * crz - srx * sry * srz) * point_ori.y) * coeff.x 
				   + (crx * crz * point_ori.x - crx * srz * point_ori.y) * coeff.y 
				   + ((sry * srz + cry * crz * srx) * point_ori.x + (crz * sry - cry * srx * srz) * point_ori.y) * coeff.z;

		//见公式i
		mat_a.at<float>(i, 0) = arx;
		mat_a.at<float>(i, 1) = ary;
		mat_a.at<float>(i, 2) = arz;
		mat_a.at<float>(i, 3) = coeff.x;
		mat_a.at<float>(i, 4) = coeff.y;
		mat_a.at<float>(i, 5) = coeff.z;
		mat_b.at<float>(i, 0) = -coeff.intensity;
	}

	//构建normal equation, 见公式k
	cv::transpose(mat_a, mat_a_t);
	mat_a_t_a = mat_a_t * mat_a;
	mat_a_t_b = mat_a_t * mat_b;

	//高斯牛顿法, 直接解normal equation求步长, QR分解是一种解法
	cv::solve(mat_a_t_a, mat_a_t_b, mat_x, cv::DECOMP_QR);

	//具体描述见Loam作者Zhang J的<<On Degeneracy of Optimization-based State Estimation Problems>>
	//大概方法是通过Jacobian的eigenvalue判断哪个分量的约束不足, 不更新那个方向上的迭代
	if (iter_count == 0)
	{
		cv::Mat mat_e(1, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat mat_v(6, 6, CV_32F, cv::Scalar::all(0));
		cv::Mat mat_v2(6, 6, CV_32F, cv::Scalar::all(0));

		cv::eigen(mat_a_t_a, mat_e, mat_v);
		mat_v.copyTo(mat_v2);

		is_degenerate = false;

		float eign_thre[6] = {100, 100, 100, 100, 100, 100};

		for (int i = 5; i >= 0; i--)
		{
			if (mat_e.at<float>(0, i) < eign_thre[i])
			{
				for (int j = 0; j < 6; j++)
				{
					mat_v2.at<float>(i, j) = 0;
				}

				is_degenerate = true;
			}
			else
			{
				break;
			}
		}

		mat_p = mat_v.inv() * mat_v2;
	}

	if (is_degenerate)
	{
		cv::Mat mat_x2(6, 1, CV_32F, cv::Scalar::all(0));
		mat_x.copyTo(mat_x2);
		mat_x = mat_p * mat_x2;
	}
}

//更新迭代的结果
void LaMapping::updateTransformFromOptimize(cv::Mat &mat_x)
{
	lidar_pose_in_map_r_[0] += mat_x.at<float>(0, 0);
	lidar_pose_in_map_r_[1] += mat_x.at<float>(1, 0);
	lidar_pose_in_map_r_[2] += mat_x.at<float>(2, 0);

	lidar_pose_in_map_t_[0] += mat_x.at<float>(3, 0);
	lidar_pose_in_map_t_[1] += mat_x.at<float>(4, 0);
	lidar_pose_in_map_t_[2] += mat_x.at<float>(5, 0);
}

bool LaMapping::isConverged(cv::Mat &mat_x)
{
	//判断是否已收敛, 这里的判断方法很简单
	float delta_r = sqrt(
		pow(radToDeg(mat_x.at<float>(0, 0)), 2) +
		pow(radToDeg(mat_x.at<float>(1, 0)), 2) +
		pow(radToDeg(mat_x.at<float>(2, 0)), 2));

	float delta_t = sqrt(
		pow(mat_x.at<float>(3, 0) * 100, 2) +
		pow(mat_x.at<float>(4, 0) * 100, 2) +
		pow(mat_x.at<float>(5, 0) * 100, 2));

	return (delta_r < 0.1 && delta_t < 0.3);
}

void LaMapping::doOptimize(int max_iteration)
{
	//复用point cloud结构存储偏导数,
	pcl::PointCloud<PointType>::Ptr coeff_selected boost::make_shared<pcl::PointCloud<PointType>>();
	//存储匹配成功的点,与coeff_selected一一对应
	pcl::PointCloud<PointType>::Ptr points_selected boost::make_shared<pcl::PointCloud<PointType>>();

	//限制迭代次数
	for (int iter_count = 0; iter_count < max_iteration; iter_count++)
	{
		//分别处理corner特征和feature特征, 建立loss成功的点(以下称为有效点), 会加入到features_selected
		//可以见到, 每次迭代, 我们会重新做一次特征点匹配
		procLossAboutCornerPoints(corner_feature_points, points_selected, coeff_selected);
		procLossAboutSurfPoints(surf_feature_points, points_selected, coeff_selected);

		//如果有效点数小于特定值, 我们认为约束不够, 放弃此次优化
		//无法优化的话, 会直接使用pose的预测值
		if (points_selected()->size() < features_slected_num_enough_for_optimize)
		{
			break;
		}

		cv::Mat mat_x(6, 1, CV_32F, cv::Scalar::all(0));

		//构建norm equation, 求解pose增量
		solveWithGaussNewton(mat_x, iter_count, points_selected, coeff_selected);

		//根据高斯牛顿迭代结果, 更新pose
		updateTransformFromOptimize(mat_x);

		if (isConverged(mat_x))
		{
			//如果迭代趋于收敛, 退出
			break;
		}
	}
}

void LaMapping::procLossAboutSurfPoints(pcl::PointCloud<PointType>::Ptr surf_feature_points,
                                        pcl::PointCloud<PointType>::Ptr points_selected, 
					pcl::PointCloud<PointType>::Ptr coeff_selected
					)
{
	for (int i = 0; i < surf_feature_points->size(); i++)
	{
		//将点转换到世界坐标系
		PointType point_sel = transPointToMapCoordinate(surf_feature_points[i]);

		std::vector<int> point_search_ind;
		std::vector<float> point_search_sq_dis;

		//从map对应的kdtree中, 搜索半径一米内的五个surf特征点
		if (surf_kdtree_ptr->radiusSearch(point_sel, 1.0, point_search_ind, point_search_sq_dis, 5) < 5)
		{
			//没有搜到足够多的点, 本点匹配失败, 此点不对后续的优化贡献约束
			continue;
		}

		//这些变量用来求解平面方程, 平面方程为AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
		//其中(X,Y,Z)是点的坐标, 对应这里的mat_a0, 是已知数
		//A/D, B/D, C/D 对应mat_x0, 是待求的值
		//等式右边的-1对应mat_b0
		cv::Mat mat_a0(5, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat mat_b0(5, 1, CV_32F, cv::Scalar::all(-1));
		cv::Mat mat_x0(3, 1, CV_32F, cv::Scalar::all(0));

		//构建五个最近点的坐标矩阵
		for (int j = 0; j < 5; j++)
		{
			mat_a0.at<float>(j, 0) = map_ptr->points[point_search_ind[j]].x;
			mat_a0.at<float>(j, 1) = map_ptr->points[point_search_ind[j]].y;
			mat_a0.at<float>(j, 2) = map_ptr->points[point_search_ind[j]].z;
		}

		//求解 (A/D)X+(B/D)Y+(C/D)Z = -1 中的 A/D, B/D, C/D 
		cv::solve(mat_a0, mat_b0, mat_x0, cv::DECOMP_QR);

		float pa = mat_x0.at<float>(0, 0);
		float pb = mat_x0.at<float>(1, 0);
		float pc = mat_x0.at<float>(2, 0);

		//对应之前的-1, (A/D)X+(B/D)Y+(C/D)Z = -1 <=> (A/D)X+(B/D)Y+(C/D)Z +1 = 0
		float pd = 1;  

		//ps为平面法向量的模
		//求得(pa, pb, pc)为法向量, 模为1, pd为平面到原点的距离
		float ps = sqrt(pa * pa + pb * pb + pc * pc);
		pa /= ps;
		pb /= ps;
		pc /= ps;
		pd /= ps;

		//确定拟合出的平面与用来拟合的点都足够接近, 表示平面拟合的有效性
		bool plane_valid = false;
		for (int j = 0; j < 5; j++)
		{
			if (fabs(pa * map_ptr->points[point_search_ind[j]].x +
					 pb * map_ptr->points[point_search_ind[j]].y +
					 pc * map_ptr->points[point_search_ind[j]].z + pd) > 0.2)
			{
				plane_valid = true;
				break;
			}
		}

		if(plane_valid)
		{
			//平面无效, 本点匹配失败, 此点不对后续的优化贡献约束
			continue;
		}

		//点到平面的距离, 参考点到平面距离公式, 分母部分为1
		float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;

		//这里的s是个权重, 表示s在这个least-square问题中的置信度, 每个点的置信度不一样
		//理论上, 这个权重, 与点到面距离负相关, 距离越大, 置信度越低, 这里相当于是一个在loss之外加了一个鲁棒性函数, 用来过减弱离群值的影响
		//源代码中"sqrt(sqrt(point_sel.x * point_sel.x + point_sel.y * point_sel.y + point_sel.z * point_sel.z)" 这部分, 并没有什么逻辑性可言
		//你可以设计自己的鲁棒性函数来替代这一行代码
		float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(point_sel.x * point_sel.x + point_sel.y * point_sel.y + point_sel.z * point_sel.z));

		//最终确定的可优化点
		if (s > 0.1)
		{
			points_selected.push_back(point_sel);

			//复用PointType传递偏导数, (pa, pb, pc)是法向量,点到平面的垂线, 也是点面距离相对与点坐标的偏导, 详见文章的公式推导部分.
			//pd2是loss
			PointType coeff;
			coeff.x = s * pa;
			coeff.y = s * pb;
			coeff.z = s * pc;
			coeff.intensity = s * pd2;
			coeff_selected.push_back(corner_feature_points[i]);
		}
		else
		{
			//距离无效, 本点匹配失败, 此点不对后续的优化贡献约束
		}
	}
}

void LaMapping::procLossAboutCornerPoints(pcl::PointCloud<PointType>::Ptr corner_feature_points,
                                          pcl::PointCloud<PointType>::Ptr points_selected, 
					 pcl::PointCloud<PointType>::Ptr coeff_selected
					 )
{
	for (int i = 0; i < corner_feature_points->size(); i++)
	{
		//将点转换到世界坐标系
		PointType point_sel = transPointToMapCoordinate(corner_feature_points[i]);


		std::vector<int> point_search_ind;
		std::vector<float> point_search_sq_dis;

		//从map对应的kdtree中, 搜索半径一米内的五个corner特征点
		if (surf_kdtree_ptr->radiusSearch(point_sel, 1.0, point_search_ind, point_search_sq_dis, 5) < 5)
		{
			//没有搜到足够多的点, 本点匹配失败, 此点不对后续的优化贡献约束
			continue;
		}

		//将五个最近点的坐标加和求平均
		float cx = 0;
		float cy = 0;
		float cz = 0;

		cv::Mat mat_a1(3, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat mat_d1(1, 3, CV_32F, cv::Scalar::all(0));
		cv::Mat mat_v1(3, 3, CV_32F, cv::Scalar::all(0));

		for (int j = 0; j < 5; j++)
		{
			cx += corner_map->points[point_search_ind[j]].x;
			cy += corner_map->points[point_search_ind[j]].y;
			cz += corner_map->points[point_search_ind[j]].z;
		}

		//坐标均值
		cx /= 5;
		cy /= 5;
		cz /= 5;

		//求均方差
		float a11 = 0;
		float a12 = 0;
		float a13 = 0;
		float a22 = 0;
		float a23 = 0;
		float a33 = 0;

		for (int j = 0; j < 5; j++)
		{
			float ax = corner_map->points[point_search_ind[j]].x - cx;
			float ay = corner_map->points[point_search_ind[j]].y - cy;
			float az = corner_map->points[point_search_ind[j]].z - cz;

			a11 += ax * ax;
			a12 += ax * ay;
			a13 += ax * az;
			a22 += ay * ay;
			a23 += ay * az;
			a33 += az * az;
		}

		//协方差矩阵的6个元素(3*3矩阵, 对角元素重复)
		a11 /= 5;
		a12 /= 5;
		a13 /= 5;
		a22 /= 5;
		a23 /= 5;
		a33 /= 5;

		//构建协方差矩阵
		mat_a1.at<float>(0, 0) = a11;
		mat_a1.at<float>(0, 1) = a12;
		mat_a1.at<float>(0, 2) = a13;
		mat_a1.at<float>(1, 0) = a12;
		mat_a1.at<float>(1, 1) = a22;
		mat_a1.at<float>(1, 2) = a23;
		mat_a1.at<float>(2, 0) = a13;
		mat_a1.at<float>(2, 1) = a23;
		mat_a1.at<float>(2, 2) = a33;

		//对协方差矩阵进行Eigenvalue decomposition, 以分析空间点的分布规律
		cv::eigen(mat_a1, mat_d1, mat_v1);

		//如果最大特征值相对第二大特征值的比例足够大, 那么反应点的分布趋于一条直线
		if (mat_d1.at<float>(0, 0) > 3 * mat_d1.at<float>(0, 1))
		{
			//(x0, y0, z0)世界坐标系下的特征点
			float x0 = point_sel.x;
			float y0 = point_sel.y;
			float z0 = point_sel.z;

			//(x1,y1,z1), (x2,y2,z2) 用来表示特征值最大方向对应的直线
			float x1 = cx + 0.1 * mat_v1.at<float>(0, 0);
			float y1 = cy + 0.1 * mat_v1.at<float>(0, 1);
			float z1 = cz + 0.1 * mat_v1.at<float>(0, 2);
			float x2 = cx - 0.1 * mat_v1.at<float>(0, 0);
			float y2 = cy - 0.1 * mat_v1.at<float>(0, 1);
			float z2 = cz - 0.1 * mat_v1.at<float>(0, 2);

			//a012为点到直线距离计算分子部分
			//两向量的叉乘
			float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
			                 * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
			                 + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
			                 * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
			                 + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))
			                 * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

			//l12为点到直线距离公式分母部分，(x1,y1,z1), (x2,y2,z2)两点的距离
			float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

			//(la, lb, lc)为单位向量, 模为1 ,方向为从垂足->点p
			float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;
			float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
			float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

			//ld2为点到直线的距离
			float ld2 = a012 / l12;

			//这里的s是个权重, 表示s在这个least-square问题中的置信度, 每个点的置信度不一样
			//理论上, 这个权重, 与点到面距离负相关, 距离越大, 置信度越低, 这里相当于是一个在loss之外加了一个鲁棒性函数, 用来过减弱离群值的影响
			//源代码中只是简单的用1-距离表示权重
			//你可以设计自己的鲁棒性函数来替代这一行代码
			float s = 1 - 0.9 * fabs(ld2);

			if (s > 0.1)
			{		
				//复用PointType传递偏导数, (la, lb, lc)是点到直线的垂线对应的向量, 也是点线距离相对与点坐标的偏导, 详见文章的公式推导部分.
				//ld2是loss
				PointType coeff;
				coeff.x = s * la;
				coeff.y = s * lb;
				coeff.z = s * lc;
				coeff.intensity = s * ld2;

				points_selected.push_back(corner_feature_points[i]);
			}
			else
			{
				//距离无效, 本点匹配失败, 此点不对后续的优化贡献约束
			}
		}
	}
}

void LaMapping::newLaserProc(pcl::PointCloud<PointType>::Ptr laser)
{
	frame_count_++;

	//第一帧, 完成初始化工作
	if(frame_count_ == 1)
	{
		//记录IMU odom(也可以用其他形式的odometry)
		prepareForNextFrame();
	
		//第一帧根据当前pose转换到map坐标系下, 作为初始地图
		updateFeatureMaps();

		//地图生成kdtree, 便于后续查找
		updateKdTrees();

		return;
	}

	//每隔几帧处理一次
	if (frame_count_ % 5 == 0)
	{
		//根据odometry进行位姿的predict
		predictTransformByOdom();

		//迭代优化位姿
		doOptimize();
		
		//迭代结束更新相关的转移矩阵
		prepareForNextFrame();

		//局部地图都要更新
		updateFeatureMaps();

		//地图生成kdtree, 便于后续查找
		updateKdTrees();
	}
}
航位推算

#include <cmath>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

void matrixToEulerYXZ(Eigen::Matrix3d& m, Eigen::Vector3d &out)
{
    //对照WIKIPEDIA的表运算（YXZ外旋）
    double z = atan2(m(1,0), m(1,1));

    double cos_x = m(1,1) / cos(z);
    double x = atan2(-m(1,2), cos_x);

    double y = atan2(m(0,2), m(2,2));

    out[0] = x;
    out[1] = y;
    out[2] = z;
}

void matrixToRtYXZ(Eigen::Matrix4d& matrix, Eigen::Vector3d &euler_r, Eigen::Vector3d &euler_t)
{
    Eigen::Matrix3d matrix_r = matrix.block(0,0,3,3);

    euler_t[0] = matrix(0,3);
    euler_t[1] = matrix(1,3);
    euler_t[2] = matrix(2,3);

    matrixToEulerYXZ(matrix_r, euler_r);
}

void eulerToMatrixYXZ(Eigen::Matrix3d& m, Eigen::Vector3d &euler)
{
    m = Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
}

void rtToMatrixYXZ(Eigen::Vector3d &r, Eigen::Vector3d &t, Eigen::Matrix4d& m)
{
    Eigen::Matrix3d m_r;
    eulerToMatrixYXZ(m_r, r);

    m.block(0,0,3,3) = m_r;
    m(0,3) = t[0];
    m(1,3) = t[1];
    m(2,3) = t[2];

    m(3, 0) = 0;
    m(3, 1) = 0;
    m(3, 2) = 0;
    m(3, 3) = 1;
}

//全局变量

//前一帧的ODOM pose， r是欧拉角，t是平移向量， 请自行维护
Eigen::Vector3d odometry_pre_r_；
Eigen::Vector3d odometry_pre_t_；

//当前的ODOM pose， r是欧拉角，t是平移向量， 请自行维护
Eigen::Vector3d odometry_r_；
Eigen::Vector3d odometry_t_；

//前一帧的pose， r是欧拉角，t是平移向量， 请自行维护
Eigen::Vector3d pose_pre_r_；
Eigen::Vector3d pose_pre_r_；

//基于predict得到transformToOptimize_, 作为优化的初始值
void predictTransformByOdom(Eigen::Vector3d &dst_r, Eigen::Vector3d &dst_t)
{
	Eigen::Matrix4d odom_m1, odom_m2, odom_trans;

    //LOAM中的变换都是欧拉角表示， YXZ顺序外旋的欧拉角

    //将前一帧的ODOM（R， T）转换为4*4变换矩阵形式
	rtToMatrixYXZ(odometry_pre_r_, odometry_pre_t_, odom_m1);
    
    //将当前帧的ODOM（R， T）转换为4*4变换矩阵形式
	rtToMatrixYXZ(odometry_r_, odometry_t_, odom_m2);

    //求两帧之间的变换矩阵， 当前帧变换到前一帧
    //W1 * 12 = W2 ->12 = W1逆*W2
    odom_trans = odom_m1.inverse()*odom_m2;

    //将前一帧的pose（R，T）转换为4*4变换矩阵形式
	Eigen::Matrix4d pose;
	rtToMatrixYXZ(pose_pre_r_, pose_pre_t_, pose);

    //将ODOM得到的变换矩阵作用于前一帧POSE得到当前帧的POSE估计
	//W2 = W1*12
	pose = pose*odom_trans;

    //4*4矩阵格式的POSE转换为（R，T）格式的POSE
	matrixToRtYXZ(pose, dst_r, dst_t);

    //now，dst_r， dst_t就是我们预测后的pose
}