# 利用libft4222实现上位机同时读取四个角度传感器数据

for linux (ubuntu):
    先解压文件libft4222-linux-1.4.4.44  

----------;

解压完成后进入当前路径的终端；在终端运行下面命令
cd libft4222-linux-1.4.4.44  
sudo ./instal4222.sh  
cd ../spimTest  

mkdir build  
cd build  
cmake ..  
make  

 

---------------------;

## spimTest（4个单圈同时读取，打印出四个单圈传感器的角度数据）

sudo ./spimTest 

## spimTest2（4个多圈）

sudo ./spimTest2 

### 多圈传感器记得注意电压；我记得蒋老大画的板子只支持四个单圈传感器；一个多圈传感器也带不动，需要额外供电  
