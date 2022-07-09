# Localization_assignment_1

- ## Visualize raw pcd files with PCL Library and ROS2/RViz2

Bu adımı yapmak için ilk olarak "perception_pcl" adlı paketin kurulumunu yaptım. Bu paketin içinte bulunan
araçlardan "src/perception_pcl/pcl_ros/tools/pcd_to_pointcloud.cpp" olan araç ile odevde verilen pcd dosyaları
poincloud publisher'ına çevirildi.

Elde edilen pointcloud verileri RVIZ2 programında görüntülendi.

### Verilen ilk pcd dosyası olan "capture0001.pcd" dosyasının pointcloud'a çevirilmiş hali.

![pc1](https://user-images.githubusercontent.com/58399721/178093724-a2e73ca6-8b51-4e3c-b77e-5ac3d2b5ac88.png)

### Verilen ikinci pcd dosyası olan "capture0002.pcd" dosyasının pointcloud'a çevirilmiş hali.

![pc2](https://user-images.githubusercontent.com/58399721/178093779-62d8344e-7f5a-48d0-b8f3-e87fc1e6337c.png)


- ## Find 3D transform between 2 pcd files

2 pcd dosyası arasındaki dönüşüm "src/pcl/src/register.cpp" dosyasında yapılmaktadır.

- ## Visualize combined point cloud and assign different colors to each point
cloud which you used for registration

### Registration işleminden önce pointcloud verilerine farklı renkler atanmıştır.

![pointcloud](https://user-images.githubusercontent.com/58399721/178094032-f6e0f271-e6d7-472e-9e59-1567b8278bdd.png)

### Renk atamalarının ardından registration işlemine başlanmıştır. Görselin sağ tarafında bulunan görüntüde registration işlemi gerçekleşmektedir.

Algoritma olarak Iterative Closest Point algoritmasını kullandım.
https://pcl.readthedocs.io/projects/tutorials/en/master/pairwise_incremental_registration.html#pairwise-incremental-registration

![registration](https://user-images.githubusercontent.com/58399721/178094073-6489a4e6-49ce-4d2b-bd0b-538e3754f35b.png)

### Görselin sağ tarafındaki görüntü registration işleminin son halini göstermektedir.

![regist](https://user-images.githubusercontent.com/58399721/178094098-d0c345ec-1bf1-4a00-a6bb-e465181773da.png)

- ## Publish erros and accuracy values


- ## Save combined point cloud as a pcd file

### Görseldeki kod ile pointcloud pcd dosyası şeklinde kaydedilmiş olmaktadır.

![pcdsave](https://user-images.githubusercontent.com/58399721/178094188-d52a2a89-f68e-49ca-bbb2-99591745ed0a.png)





