
#include <webots/Display.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/gps.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stack>
#include <string>
#include <time.h>
#include <vector>

using namespace std;
using namespace webots;

#define SIZE 1000
#define SAMPLE_MAX 50
#define FINDPATH_FREQUENCY 50

//坐标表示一个点
typedef struct Coordinate
{
	int ID;
	int x;
	int y;
} Coord;

//起始点和终点
typedef struct startAndEnd
{
	int startx;
	int starty;
	int endx;
	int endy;
} SE;

int doubleToInt(double a)
{
	return int(a * 100) + SIZE / 2;
}
//处理离群点
bool isOutOfGroup(int x, int y, vector<vector<bool>> &matrix)
{
	// bool isCloseToSomePoints = false;
	int step = 3; //步长，当该点的  比如，-3~3矩阵范围内没有点时，才算离群点
	for (int i = x - step; i < x + step + 1; i++)
	{
		if (i < 0 || i >= SIZE)
			continue;
		for (int j = y - step; j < y + step + 1; j++)
		{
			if (j < 0 || j >= SIZE)
				continue;
			if (matrix[i][j])
				return false;
		}
	}
	return true;
}
//判断某个点是否靠近障碍物
bool isCloseToWall(int x, int y, vector<vector<bool>> &matrix, int step)
{
	 // step 步长，当该点的  比如，-20~20 * -20~20 的矩阵范围内没有点时，才算离群点
	for (int i = x - step; i < x + step + 1; i++)
	{
		if (i < 0 || i >= SIZE)
			continue;
		for (int j = y - step; j < y + step + 1; j++)
		{
			if (j < 0 || j >= SIZE)
				continue;
			if (matrix[i][j])
				return true;
		}
	}
	return false;
}
//随机取样
void Sample(vector<vector<bool>> &matrix, vector<Coordinate> &points, SE se)
{

	Coord temp;
	temp.x = se.startx;
	temp.y = se.starty;
	temp.ID = 0;
	points.push_back(temp);
	temp.x = se.endx;
	temp.y = se.endy;
	temp.ID = 1;
	points.push_back(temp);

	srand((unsigned)time(NULL));
	int cnt = 2;
	while (cnt < SAMPLE_MAX)
	{
		int rand_x = rand() % SIZE;
		int rand_y = rand() % SIZE;
		if (isCloseToWall(rand_x, rand_y, matrix, 10))
			continue;
		Coord temp;
		temp.x = rand_x;
		temp.y = rand_y;
		temp.ID = cnt;
		points.push_back(temp);
		cnt++;
	}
}
//欧几里得距离
int Distance(int x1, int y1, int x2, int y2)
{
	//cout << "distance error";
	return int(sqrt(double((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))));
}
//判断某个点是不是障碍物
bool isObstacle(int x, int y, vector<vector<bool>> &matrix)
{
	//检测是否是障碍物，不是障碍物时返回true
	if (matrix[x][y])
	{
		return true;
	}
	return false;
}
//检查路径
bool checkPath(int x1, int y1, int x2, int y2, vector<vector<bool>> &matrix)
{
	//计算两点之间的路径是否存在障碍，无障碍时返回true
	int step = max(abs(x1 - x2), abs(y1 - y2));
	double stepx = (double)abs(x1 - x2) / step;
	double stepy = (double)abs(y1 - y2) / step;
	double testx = min(x1, x2);
	double testy = min(y1, y2);
	if((testx == x1 && testy == y1) || (testx == x2 && testy == y2)){
        for (int i = 0; i < step; i++)
        {
            testx += stepx;
            testy += stepy;
            if(isCloseToWall(ceil(testx), ceil(testy), matrix, 1)){
                return false;
            }   
        }
    }else {
        if(testy == y2)testy = y1;
        else
            testy = y2;
        for (int i = 0; i < step; i++)
        {
            testx += stepx;
            testy -= stepy;
            if(isCloseToWall(ceil(testx), ceil(testy), matrix, 1)){
                return false;
            }   
        }
    }
	return true;
}

bool dijkstra(vector<vector<int>> &adMatrix, vector<int> &path)
{
	vector<int> dis(SAMPLE_MAX, INT_MAX);
	vector<bool> visited(SAMPLE_MAX, false);
	vector<int> pre(SAMPLE_MAX, -1);//记录前一个点    ->   箭头的左边

	//首先放入0号点
	visited[0] = true;
	int cur = 0;
	dis[cur] = 0;
	pre[cur] = 0;
	for (int i = 1; i < SAMPLE_MAX; i++)
	{
		//更新dis，并找出最小的没访问过的点
		int minnode = -1;
		int minval = INT_MAX;
		for (int j = 0; j < SAMPLE_MAX; j++)
		{
			if (!visited[j])
			{
				if (adMatrix[cur][j] != INT_MAX && dis[j] > dis[cur] + adMatrix[cur][j]){
					dis[j] = dis[cur] + adMatrix[cur][j];
					pre[j] = cur;
				}
				if (dis[j] < minval)
				{
					minnode = j;
					minval = dis[j];
				}
			}
		}
		//不能再找到新的可抵达的点
		if (minnode == -1)
			break;
		//cur等于新的点，并进入下一次循环
		visited[minnode] = true;
		cur = minnode;
	}
	
	//dis已经获取了源点到所有点的最短路径，现在我们要找到  0->...->1  的最短路径
	//且pre包含了每个点的前一个点
	stack<int> st;
	int curnode = 1;//从1开始
	while(curnode != 0){
		curnode = pre[curnode];
		if(curnode == -1)
			return false;
		st.push(curnode);
	}
	while(!st.empty()){
		cout << st.top() << "->";
		path.push_back(st.top());
		st.pop();
	}
	path.push_back(1);
	cout << "1" << endl;
	return true;
}

bool findShortestPath(vector<Coordinate> &points, vector<vector<bool>> &matrix, vector<int> &path)
{
	//生成一个邻接矩阵，将没有跨过障碍物的边全部加进来，无法抵达的用INT_MAX表示
	vector<vector<int>> adMatrix(SAMPLE_MAX, vector<int>(SAMPLE_MAX, INT_MAX));
	for (int i = 0; i < SAMPLE_MAX - 1; i++)
	{
		for (int j = i + 1; j < SAMPLE_MAX; j++)
		{
			//cout << points[i].x << " " << points[i].y << " " << points[j].x << " " << points[j].y << endl;
			if (checkPath(points[i].x, points[i].y, points[j].x, points[j].y, matrix))
			{
				adMatrix[i][j] = Distance(points[i].x, points[i].y, points[j].x, points[j].y);
				adMatrix[j][i] = adMatrix[i][j];
			}
		}
	}
	int flag = dijkstra(adMatrix, path);
	if(flag == false){
		return false;
	}else{
		int size = path.size();
		for (int i = 0; i < size-1; i ++){
			if (!checkPath(points[path[i]].x, points[path[i]].y, points[path[i+1]].x, points[path[i+1]].y, matrix))
			{
				return false;
			}
		}
		return true;
	}
}

void drawPath(vector<Coordinate> &points, vector<int> &path, Display* display){
	//绘图
	display->setColor(0xFFFFFF);
	int size = path.size();
	for (int i = 1; i < size; i++)
	{
		//display->drawPixel(points[path[i]].x, points[path[i]].y);
		display->drawLine(points[path[i-1]].x, points[path[i-1]].y,points[path[i]].x, points[path[i]].y);
	}
	return;
}

void deletePath(vector<Coordinate> &points, vector<int> &path, Display* display){
	//绘图
	display->setColor(0x000000);
	int size = path.size();
	for (int i = 1; i < size; i++)
	{
		display->drawLine(points[path[i-1]].x, points[path[i-1]].y,points[path[i]].x, points[path[i]].y);
	}
	return;
}

int main()
{

	Motor *motors[4]; //电机和键盘都要用webots给的类型
	webots::Keyboard keyboard;
	char wheels_names[4][8] = {"motor1", "motor2", "motor3", "motor4"}; //你在仿真器里面设置的名字

	Robot *robot = new Robot(); //使用webots的机器人主体
	keyboard.enable(1);			//运行键盘输入设置频率是1ms读取一次

	vector<vector<bool>> matrix(SIZE, vector<bool>(SIZE, false));
	vector<vector<bool>> pathMatrix(SIZE, vector<bool>(SIZE, false));
	//速度
	double speed1[4];
	double speed2[4];
	double velocity = 8;

	//初始化
	for (int i = 0; i < 4; i++)
	{
		motors[i] = robot->getMotor(wheels_names[i]); //按照你在仿真器里面设置的名字获取句柄
		motors[i]->setPosition(std::numeric_limits<double>::infinity());
		motors[i]->setVelocity(0.0);

		speed1[i] = 0;
		speed2[i] = 0;
	}

	//我列了一个小表格，当四个轮子按照下面这样转的时候，车子可以完成前后左右，转圈
	//斜着走是前后+左右  两个键同时按
	double speed_forward[4] = {velocity, velocity, velocity, velocity};
	
	double speed_leftward[4] = {-velocity, velocity, -velocity, velocity};
	double speed_rightward[4] = {velocity, -velocity, velocity, -velocity};

	double speed_leftCircle[4] = {-velocity, velocity, velocity, -velocity};
	double speed_rightCircle[4] = {velocity, -velocity, -velocity, velocity};

	int timeStep = (int)robot->getBasicTimeStep(); //获取你在webots设置一帧的时间
	cout << timeStep << endl;					   //默认为50
												   //timeStep=32;

	//enable IMU
	InertialUnit *imu;
	imu = robot->getInertialUnit("imu");
	imu->enable(timeStep);
	//enable gps
	GPS *gps;
	gps = robot->getGPS("gps");
	gps->enable(timeStep);

	//雷达相关
	Lidar *lidar;
	lidar = robot->getLidar("lidar");
	lidar->enable(timeStep);
	lidar->enablePointCloud(); //启动激光雷达云更新

	//地图
	Display *display;
	display = robot->getDisplay("display");
	int judge_count = 20; //多少帧后开始判断离群点

	//dijkstra计算最短路径，相关准备
	int findpath_frequency = 10;
	SE se = {0, 0, 9, 9};

	cout << se.startx << " " << se.starty << " " << endl;
	se.endx = 725;
	se.endy = 725;
	vector<Coordinate> points;
	vector<int> path;
	int moveframe = 0;
	while (robot->step(timeStep) != -1) //仿真运行一帧
	{
		bool turndir = false;
		
		if (judge_count > 0)
			judge_count--;
		if (findpath_frequency > 0)
			findpath_frequency--;
		//use gps to get the location
		double gx = gps->getValues()[0];
		double gy = gps->getValues()[1];
		se.startx = doubleToInt(gx);
		se.starty = doubleToInt(gy);
		//use imu to get the angle,由于修改了坐标体系，故选用pitch,即绕z轴的旋转角度
		double roll = imu->getRollPitchYaw()[0];
		//double pitch = imu->getRollPitchYaw()[1];
		//double yaw = imu->getRollPitchYaw()[2];
		//cout<< "IMU roll: " <<imu->getRollPitchYaw()[0]<< " pitch: " <<imu->getRollPitchYaw()[1]<< " yaw: " <<imu->getRollPitchYaw()[2] <<std::endl;

		//获取点云信息
		//const float *lidar_image=lidar->getRangeImageArray();
		const LidarPoint *pc = lidar->getLayerPointCloud(0);
		int pointnum = lidar->getNumberOfPoints();
		for (int i = 0; i < pointnum; i++)
		{
			//cout<<"point id:"<<i<<" x:"<< pc[i].x<<" y:"<<pc[i].y <<" z:"<<pc[i].z<<endl;
			double px = pc[i].x;
			double pz = pc[i].z;
			double d = sqrt(px * px + pz * pz);
			double sin_theta = px / d;
			double theta = asin(sin_theta);
			if (pc[i].z < 0)
				theta = M_PI - theta;
			double a = roll + theta - M_PI / 2.0;
			double global_x = gx + d * cos(a);
			double global_y = gy + d * sin(a);
			//坐标转换
			int new_x = doubleToInt(global_x);
			int new_y = doubleToInt(global_y);

			//cout << "x:" << new_x << "y:" << new_y <<endl;
			if (judge_count || !isOutOfGroup(new_x, new_y, matrix))
			{
				//额外用一个矩阵保存图像信息
				if (new_x > SIZE || new_x < 0)
					continue;
				if (new_y > SIZE || new_y < 0)
					continue;
				matrix[new_x][new_y] = true;
				//绘图
				display->setColor(0xFFFFFF);
				display->drawPixel(new_x, new_y);
			}
		}
		cout << findpath_frequency << endl;
		cout << "GPS Value X: " << gx << " Y: " << gy << endl;
		cout << "roll: " << roll << endl;

		if (findpath_frequency == 0)
		{
			//重置上一次随机生成的点，和路径
			deletePath(points, path, display);
			points.resize(0);
			path.resize(0);
			//随机生成点，取样的点不会太靠近墙壁
			Sample(matrix, points, se);
			//寻找最短路径，判断是否寻找成功
			bool isFindPath = findShortestPath(points, matrix, path);

			if (isFindPath)//成功则打印路径
            {
				moveframe = FINDPATH_FREQUENCY - 4;
				turndir = true;
				drawPath(points, path, display);
				findpath_frequency = FINDPATH_FREQUENCY;
            }
			else//失败则在下一帧再次寻找
			{
				cout << "Can't find the path!!!!" << endl;
			}
		}

		if(turndir || moveframe > 0){
            int nowx = points[0].x;
	        int nowy = points[0].y;
	        int nextx = points[path[1]].x;
	        int nexty = points[path[1]].y;
			int dis = Distance(nowx,nowy,nextx,nexty);
	        double sina = double((nexty-nowy)) / double(dis);
	        double a = asin(sina);
			if(nextx-nowx < 0 && nexty-nowy >0){
				a = M_PI - a;
			}else if(nextx-nowx < 0 && nexty-nowy <0){
        			           a = - M_PI - a;
			}
			double roll = imu->getRollPitchYaw()[0];
			cout <<roll <<" "<< a << " " <<roll - a << endl;
			//右转
			if(roll - a>0.1 ) {
				for (int i = 0; i < 4; i++)speed1[i] = speed_rightCircle[i];
				for (int i = 0; i < 4; i++)speed2[i] = 0;	
			moveframe--;
			}
			//左转
			else if(roll-a <-0.1) {
				for (int i = 0; i < 4; i++)speed1[i] = speed_leftCircle[i];
				for (int i = 0; i < 4; i++)speed2[i] = 0;
			moveframe--;
			}
			//直行
			else {
                for (int i = 0; i < 4; i++)speed1[i] = speed_forward[i];
				for (int i = 0; i < 4; i++)speed2[i] = speed_forward[i];
				
			}
			moveframe--;
		}
		//这一帧没有找到路径，小车不动
		else {
			    for (int i = 0; i < 4; i++)
				{
				speed1[i] = 0;
			    }
				for (int i = 0; i < 4; i++)
			    {
			  	speed2[i] = 0;
			    }
		}
        
		//让电机执行
		for (int i = 0; i < 4; i++)
		{
		 motors[i]->setVelocity(speed1[i] + speed2[i]);
		}

		// //wb_motor_set_velocity(wheels[0],right_speed);
	}

	return 0;
}
