
// 全ての平面を除去(下記の平面除去関数をこんな感じでwhile文で使う)
while (true){
	bool flag = planeRemoval(pc, this->planeThreshold);
	if (flag){
		break;
	}
}
//平面除去関数
bool planeRemoval(pcl::PointCloud::Ptr cloud, double threshold){
	// 点群が0でないか確認
	if (cloud->points.size() == 0){
		std::cout << "no cloud data" << std::endl;
		return true;
	}

	//　平面検出（pclのチュートリアル通り）
	pcl::SACSegmentation seg;
	seg.setOptimizeCoefficients(true);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	seg.setInputCloud(cloud);
	seg.setModelType(pcl::SACMODEL_PLANE); //モデル
	seg.setMethodType(pcl::SAC_RANSAC);	//検出手法
	seg.setMaxIterations(200);
	seg.setDistanceThreshold(threshold); //閾値
	seg.segment(*inliers, *coefficients);

	// 平面除去を終わらせるかどうか：検出した平面が，前の平面除去した状態の点群のfinishRemovePlaneRatioで指定された割合未満の点群サイズであれば，それは平面でないとみなして点群除去処理を終了（finishRemovePlaneRatioは0.0~1.0:環境に合わせてください）
	if (inliers->indices.size() < cloud->points.size() * finishRemovePlaneRatio){
		return true;
	}

	// 平面除去
	pcl::ExtractIndices extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true); // true にすると平面を除去、false にすると平面以外を除去
	extract.filter(*cloud);
	std::cout << "remove plane" << std::endl;

	return false;
}
