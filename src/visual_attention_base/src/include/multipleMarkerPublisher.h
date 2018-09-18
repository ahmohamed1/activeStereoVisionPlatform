#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>


struct targetInformation{
  cv::Point3f targetPose;
  float probability2D;
  float probability3D;
  float targetSize;
  float idea;
};


class MultipleMarkerPublisher{
public:
  MultipleMarkerPublisher(ros::NodeHandle nh){
    pub_markerArray = nh.advertise<visualization_msgs::MarkerArray>("targets_information", 1);
    world_frame = "camera_link";
  }

  void publishMarker(std::vector<targetInformation> targetInformation){
    visualization_msgs::MarkerArray marker_array_msg;
    for(int i=0; i < targetInformation.size(); i++){
      //Create Markers and add it to the multiple marker MarkerArray
      cv::Point3f centroid = targetInformation[i].targetPose;
      visualization_msgs::Marker markerText = createMarker(centroid.x, centroid.y, centroid.z, i,targetInformation[i], true);
      visualization_msgs::Marker marker = createMarker(centroid.x, centroid.y, centroid.z, i, targetInformation[i], false);
      marker_array_msg.markers.push_back(markerText);
      marker_array_msg.markers.push_back(marker);


      // Publish frame for each targets
      transform.setOrigin( tf::Vector3(centroid.x, centroid.y, centroid.z) );
      transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      stringstream ssl;
      ssl<<"Target_"<< i;
      string targetName = ssl.str();
      ssl.str("");
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, targetName));
    }
      pub_markerArray.publish(marker_array_msg);
      marker_array_msg.markers.clear();
  }

  visualization_msgs::Marker createMarker(float x, float y, float z, int ID ,targetInformation target, bool showText = true){
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp = ros::Time();
    marker.id = ID;
    if (showText){
      marker.ns = "textSpace";
      //Create string with pose and idea number
      stringstream ssl;
      ssl<<"Target ID: "<< ID
         <<"\n3D Position: (" << x <<"," << y <<"," << z << ")"
         <<"\n2D Probability: " << target.probability2D
         <<"\n3D RMS: " << target.probability3D;

      string markerInfo = ssl.str();
      ssl.str("");
      // Text create finish
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.text = markerInfo;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z - 0.05;
      marker.scale.z = 0.03;
      marker.scale.x = 0.03;
      marker.scale.y = 0.03;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }else{
      marker.ns = "markerSpace";
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.scale.z = 0.07;
      marker.scale.x = 0.07;
      marker.scale.y = 0.07;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    // marker.lifetime = ros::Duration(10);

    return marker;
  }

private:
  ros::Publisher pub_markerArray;
  ros::Publisher cluster_pub;
  std::string world_frame;
  // publish frames
  tf::TransformBroadcaster br;
  tf::Transform transform;
};
