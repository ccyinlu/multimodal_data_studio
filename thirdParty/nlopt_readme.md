## NLOPT matlab使用手册

#### 安装  
1. 下载  
Version 2.5.0 of NLopt is the latest version available from GitHub:  
[nlopt-2.5.0.tar.gz](https://github.com/stevengj/nlopt/archive/v2.5.0.tar.gz)  
2. 编译  
`tar xvf nlopt-2.5.0.tar.gz`  
`cd nlopt-2.5.0`  
`mkdir build`  
`cmake -DMatlab_ROOT_DIR=/usr/local/MATLAB/R2017b -DINSTALL_MEX_DIR=/usr/local/MATLAB/R2017b/mex ..`  
`make`  
`sudo make install`  
 