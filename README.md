# 3D-CONSTRUCTOR

> This project is under construction.

This is a 3D constructor for 3D reconstruction running on windows 10. By using depth-images received from kinect, 3D-CONSTRUCTOR first show renders it and then reconstructs it into a 3D model. 

## Platform

+ Windows 10
+ CMake

## Dependences
+ [CMake 3.6.1](https://www.cmake.org)
+ [Qt 5.7.0](https://www.qt.io)
+ [Kinect v2.0](https://developer.microsoft.com/zh-cn/windows/kinect)
+ [PCL 1.8](http://www.pointclouds.org)

## Attention

If you have any problems in compiling and installing PCL 1.8, [this page](http://leetsong.github.io/2016/08/05/post-6/) may help you in some degree.

## Present Situation

Now the project can achieve:

1. Receive data from kinect and render them by openGL in the (left) viewer window.
2. Load relevant and continuous point clouds files(\*.pcd and \*.ply) and registe them into one point cloud.
3. By data received from kinect, registe them into one point cloud, and then render then in the (right) viewer window.

## Compile and Run

+ Install the dependences

+ Download this project, or write in command line the following instruction:

```
git clone https://github.com/Leetsong/3D-CONSTRUCTOR.git
```

+ open CMake(GUI), set source and build as following(assume that the project locates in ` PATH/TO/PROJECT/3D-CONSTRUCTOR `):

` source: PATH/TO/PROJECT/3D-CONSTRUCTOR `

` build: PATH/TO/PROJECT/3D-CONSTRUCTOR/build  `

+ configure and generate it, and then open ` 3D-CONSTRUCTOR.sln ` in the directory ` build `, generate solution of ` 3D-CONSTRUCTOR `, and then ` 3D-CONSTRUCTOR.exe ` will be generated in ` PATH/TO/PROJECT/3D-CONSTRUCTOR/build/Debug/ `

+ run it

## Problems

1. (left) viewer window rendered by openGL is not as fine viewed as what I think.
2. (right) viewer window rendered by pcl runs slowly, it depends strongly on the size of your point cloud.
