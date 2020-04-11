#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>
#include <pcl/io/png_io.h>
#include <ctime>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; // LASER_FRAME;
// LASER_FRAME
bool setUnseenToMaxRange = false;
bool live_update = false;
bool have_far_ranges = false;

// --------------
// -----Help-----
// --------------
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
		<< "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
		<< "-m           Treat all unseen points to max range\n"
		<< "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
		<< "-h           this help\n"
		<< "-f           far_ranges.pcd\n"
		<< "\n\n";
}

// --------------
// -----Main-----
// --------------
int
main(int argc, char** argv)
{
	// --------------------------------------
	// -----Parse Command Line Arguments-----
	// --------------------------------------

	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}
	if (pcl::console::find_argument(argc, argv, "-m") >= 0)
	{
		setUnseenToMaxRange = true;
		cout << "Setting unseen values in range image to maximum range readings.\n";
	}
	int tmp_far_ranges;
	if (pcl::console::parse(argc, argv, "-f", tmp_far_ranges) >= 0)
	{
		have_far_ranges = true;
		cout << "far_ranges equle true.\n";
	}
	int tmp_coordinate_frame;
	if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
	{
		coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
		cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
	}
	if (pcl::console::find_argument(argc, argv, "-l") >= 0)
	{
		live_update = true;
		cout << "Live update is on.\n";
	}

	if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
		cout << "Setting angular resolution to " << angular_resolution << "deg.\n";
	angular_resolution = pcl::deg2rad(angular_resolution);

	// ------------------------------------------------------------------
	// -----Read pcd file or create example point cloud if not given-----
	// ------------------------------------------------------------------
	pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;

	Eigen::Matrix4f T;
	T << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 500,
		0, 0, 0, 1;
	Eigen::Matrix4f T1;
	T1 << 0, -1, 0, 0,
		1, 0, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	Eigen::Affine3f Traw(T * T1);
	Eigen::Affine3f scene_sensor_pose = Traw;

	// Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());  //��������λ��
	std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
	if (!pcd_filename_indices.empty())
	{
		std::string filename = argv[pcd_filename_indices[0]];
		if (pcl::io::loadPCDFile(filename, point_cloud) == -1)   //���ļ�
		{
			cout << "Was not able to open file \"" << filename << "\".\n";
			printUsage(argv[0]);
			return 0;
		}

		std::cout << "��PCD�ļ����Ѽ������ݵ������ " << point_cloud.width * point_cloud.height
			<< "\n���Ϊ��" << point_cloud.width
			<< "\n�߶�Ϊ��" << point_cloud.height
			<< "\nǰ��������ϢΪ: \n";
		for (size_t i = 0; i < 5; ++i) 
			std::cout << "    " << point_cloud.points[i].x
			<< " " << point_cloud.points[i].y
			<< " " << point_cloud.points[i].z << std::endl;

		scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
			point_cloud.sensor_origin_[1],
			point_cloud.sensor_origin_[2])) *
			Eigen::Affine3f(point_cloud.sensor_orientation_);  //����任����

		if (have_far_ranges) 
		{
			std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
			if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
				std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
		}
	}
	else
	{
		cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
		for (float x = -0.5f; x <= 0.5f; x += 0.01f)      //���һ�����εĵ���
		{
			for (float y = -0.5f; y <= 0.5f; y += 0.01f)
			{
				PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
				point_cloud.points.push_back(point);
			}
		}
		point_cloud.width = (int)point_cloud.points.size();  point_cloud.height = 1;
	}

	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0;      //���ֲ���������
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange();

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer("3D Viewer");   //������ʼ�����ӻ�����
	viewer.setBackgroundColor(1, 1, 1);                      //���ñ�������Ϊ��ɫ
	viewer.addCoordinateSystem(1.0f);              //��������ϵ
	pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler(point_cloud_ptr, 0, 0, 0); //�����Զ�����ɫ
	viewer.addPointCloud(point_cloud_ptr, point_cloud_color_handler, "original point cloud");   //��ӵ���
	// PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
	// viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	// viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");

	// -------------------------
	// -----Extract borders��ȡ�߽�Ĳ���-----
	// -------------------------
	pcl::RangeImageBorderExtractor border_extractor(&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);     //��ȡ�߽����������

	// -------------------------------------------------------
	// -----Show points in 3D viewer��3D �ӿ�����ʾ����-----
	// ----------------------------------------------------
	pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),  //����߽�
		veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),     //veil�߽�
		shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);   //��Ӱ�߽�
	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
		& veil_points = *veil_points_ptr,
		& shadow_points = *shadow_points_ptr;

	for (int y = 0; y < (int)range_image.height; ++y)
	{
		for (int x = 0; x < (int)range_image.width; ++x)
		{
			if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
				border_points.points.push_back(range_image.points[y * range_image.width + x]);

			if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
				veil_points.points.push_back(range_image.points[y * range_image.width + x]);

			if (border_descriptions.points[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
				shadow_points.points.push_back(range_image.points[y * range_image.width + x]);
		}
	}
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0); // GREEN��ɫ ����
	viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0); // RED��ɫ �ڲ��
	viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255); // ��ɫ ��Ӱ����
	viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");


	//���ӻ����ͼ
	std::cout << "wty test 3" << std::endl;
	int imageSizeX = 600;
	int imageSizeY = 600;
	float centerX = imageSizeX / 2.0f;
	float centerY = imageSizeY / 2.0f;
	float LengthX = -800.0f;
	float LengthY = -800.0f;

	boost::shared_ptr<pcl::RangeImagePlanar> range_image_planar_ptr(new pcl::RangeImagePlanar);
	pcl::RangeImagePlanar& range_image_planar = *range_image_planar_ptr;
	range_image_planar.createFromPointCloudWithFixedSize(point_cloud, imageSizeX, imageSizeY,
		centerX, centerY, LengthX, LengthY,
		scene_sensor_pose, coordinate_frame, noise_level, min_range);

	pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
	range_image_widget.showRangeImage(range_image_planar);

	while (!viewer.wasStopped())
	{
		range_image_widget.spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);

		if (live_update)
		{
			scene_sensor_pose = viewer.getViewerPose();
			range_image_planar.createFromPointCloudWithFixedSize(point_cloud, imageSizeX, imageSizeY,
				centerX, centerY, LengthX, LengthY,
				scene_sensor_pose, coordinate_frame, noise_level, min_range);
			range_image_widget.showRangeImage(range_image_planar);
		}
	}

	//-------------------------------------
	// -----Show points on range image-----
	// ------------------------------------
	/*
	pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	 range_image_borders_widget =
		pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget(range_image, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false,
			border_descriptions, "Range image with borders");
	// range_image_borders_widget->showRangeImage(range_image);

	// -------------------------------------


	//--------------------
	// -----Main loop-----
	//--------------------
	int last_time = 0;
	int png_num = 0;

	while (!viewer.wasStopped())
	{
		range_image_borders_widget->spinOnce();
		viewer.spinOnce();
		pcl_sleep(0.01);

		if (live_update)
		{
			scene_sensor_pose = viewer.getViewerPose();
			range_image.createFromPointCloud(
				point_cloud, angular_resolution, angular_resolution,
				pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
				scene_sensor_pose, coordinate_frame,
				noise_level, min_range, border_size); // LASER_FRAME
			range_image_borders_widget->showRangeImage(range_image);

			int  now_time = clock() / CLOCKS_PER_SEC;
			if (now_time - last_time > 15) {
				last_time = now_time;
				cout << "now_time : " << now_time << endl;
				float* ranges = range_image.getRangesArray();
				unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, range_image.width, range_image.height);

				std::string png_name = "saveRangeImageRGB" + std::to_string(png_num) + ".png";
				pcl::io::saveRgbPNGFile(png_name, rgb_image, range_image.width, range_image.height);
				png_num++;
				// pcl::io::savePNGFile("saveRangeImage.png", point_cloud, "rgb");
			}
			
		}
	}
	*/
}