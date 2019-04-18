#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stack>
#include <vector>
#include <cmath>
#include <queue>

using namespace cv;
using namespace std;

class node {
public:
	vector<vector<int>> *fscore;
	vector<int> loc;
	node(vector<int> loc, vector<vector<int>>* score){
		this->loc = loc;
		this->fscore = score;
	}
};

struct lessers{
	bool operator()(const node& lhs, const node& rhs) const{
		vector<vector<int>>& vecRef = *lhs.fscore;
		return vecRef[lhs.loc[0]][lhs.loc[1]] > vecRef[rhs.loc[0]][rhs.loc[1]];
	}
};
void astarList();
vector<int> left(Mat , vector<int>);
vector<int> right(Mat , vector<int> );
vector<int> up(Mat , vector<int> );
vector<int> down(Mat , vector<int> );
bool isWall(Mat image, int x, int y);
int getNextNode(vector<vector<int>> , vector<vector<int>>);

struct traceback {
	vector<int> prev;
};

void showPath(Mat, vector<vector<vector<int>>>, vector<int>);

float heuristicDist(vector<int>, vector<int>);

Mat image;

int startx, starty, endx, endy;
