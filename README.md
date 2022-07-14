# Localization_assignment_1

- ## Visualize raw pcd files with PCL Library and ROS2/RViz2

Bu adımı yapmak için ilk olarak "perception_pcl" adlı paketin kurulumunu yaptım. Bu paketin içinte bulunan
araçlardan "src/perception_pcl/pcl_ros/tools/pcd_to_pointcloud.cpp" olan araç ile odevde verilen pcd dosyaları
poincloud publisher'ına çevirildi.

Elde edilen pointcloud verileri RVIZ2 programında görüntülendi.

### Verilen pcd dosyalarının RVIZ2 görüntüleri

![2_pointclouds](https://user-images.githubusercontent.com/58399721/178893070-397dd889-f6fd-4f13-9428-1584ac9b9de3.png)

- ## Visualize combined point cloud and assign different colors to each point cloud which you used for registration

Algoritma olarak Iterative Closest Point algoritmasını kullandım.

![align](https://user-images.githubusercontent.com/58399721/178893147-8aba127c-e8d3-4b44-8264-c3aa09863bb5.png)


- ## Publish erros and accuracy values

![error](https://user-images.githubusercontent.com/58399721/178893187-7fe86f05-e40c-42a7-807a-698413539116.png)





