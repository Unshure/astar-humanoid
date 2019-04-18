#include "astar.h"
#include <ctime>

int main( int argc, char** argv) {

    if( argc != 6)
    {
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return 0;
    }

    image = imread(argv[1]);   // Read the file

    startx = atoi(argv[2]);
    starty =  atoi(argv[3]);
    endx =  atoi(argv[4]);
    endy =  atoi(argv[5]);

    if(! image.data )   // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return 0;
    }

    clock_t t;
    t = clock();

    astarList();

    t = clock() - t;
    printf("time: %f\n", ((float)t)/ CLOCKS_PER_SEC);

    return 0;
}

void astarList()
{
    //namedWindow( "Display window", WINDOW_NORMAL );// Create a window for display.
    //imshow( "Display window", image );                   // Show our image inside it.
    //waitKey(0);  // Wait for a keystroke in the window

    cout << "Image rows: " << image.rows << " image cols: " << image.cols << endl;

    //Goal point of the image
    vector<int> goal {endy, endx};

    // Keep the score of the distance from start node to that node
    vector<vector<int>> fScore (image.rows, vector<int>(image.cols, INT_MAX));

    fScore[starty][startx] = heuristicDist(vector<int>{starty, startx}, goal);

    // Keep the score of the distance from start node to that node
    vector<vector<int>> gScore (image.rows, vector<int>(image.cols, INT_MAX));

    // Cost of going to start square is zero
    gScore[starty][startx] = 0;

    // Set of visited nodes
    vector<vector<bool>> closedSet (image.rows, vector<bool>(image.cols, false));

    // Set of nodes in open Set
    vector<vector<bool>> isOpenSet (image.rows, vector<bool>(image.cols, false));

    isOpenSet[starty][startx] = true;

    // Vector heap to store next nodes to visit
    vector<node> openSetVecq;

    openSetVecq.push_back(node(vector<int>{starty, startx}, &fScore));

    make_heap(openSetVecq.begin(), openSetVecq.end(), lessers());

    vector<vector<vector<int>>> cameFrom (image.rows, vector<vector<int>>(image.cols, vector<int>()));

    cameFrom[starty][startx] = vector<int>();

    while (!openSetVecq.empty()) {

        pop_heap(openSetVecq.begin(), openSetVecq.end(), lessers());
        node current = openSetVecq.back();
        openSetVecq.pop_back();

        //printf("x: %d, y: %d, f: %d\n", current.loc[0], current.loc[1], fScore[current.loc[0]][current.loc[1]]);

        if (current.loc == goal) {
            cout << "GOAL REACHED" << endl;
            showPath(image, cameFrom, current.loc);
            break;
        }

        isOpenSet[current.loc[0]][current.loc[1]] = false;
        closedSet[current.loc[0]][current.loc[1]] = true;

        vector<vector<int>> neighbors;

        // Check each neighbor of current
        vector<int> currUp = up(image, current.loc);
        vector<int> currDown = down(image, current.loc);
        vector<int> currLeft = left(image, current.loc);
        vector<int> currRight = right(image, current.loc);

        if(!currUp.empty()) {
            neighbors.push_back(currUp);
        }
        if(!currDown.empty()) {
            neighbors.push_back(currDown);
        }
        if(!currLeft.empty()) {
            neighbors.push_back(currLeft);
        }
        if(!currRight.empty()) {
            neighbors.push_back(currRight);
        }

        for (vector<int> neighbor: neighbors) {
            // Ignore neighbors that have already been evaluated
            if (!closedSet[neighbor[0]][neighbor[1]]) {
                // Calculate tentative distance
                float tentative_gScore = float(gScore[current.loc[0]][current.loc[1]]) + 1;

                if (!isOpenSet[neighbor[0]][neighbor[1]]) {

                    openSetVecq.push_back(node(neighbor, &fScore));
                    push_heap(openSetVecq.begin(), openSetVecq.end(), lessers());
                    isOpenSet[neighbor[0]][neighbor[1]] = true;

                } else if (tentative_gScore >= gScore[neighbor[0]][neighbor[1]]) {
                    continue;
                }

                cameFrom[neighbor[0]][neighbor[1]] = current.loc;
                gScore[neighbor[0]][neighbor[1]] = tentative_gScore;
                fScore[neighbor[0]][neighbor[1]] = tentative_gScore + heuristicDist(neighbor, goal);
                make_heap(openSetVecq.begin(), openSetVecq.end(), lessers());
            }
        }
    }

    imwrite( "path.jpeg", image );
}

void showPath(Mat image, vector<vector<vector<int>>> cameFrom, vector<int> current) {
    while(!cameFrom[current[0]][current[1]].empty()) {

        image.at<Vec3b>(current[0], current[1])[0] = 0;
        image.at<Vec3b>(current[0], current[1])[1] = 0;

        //cout << "CUrrent: " << current[0] << " " <<  current[1] << endl;
        current = cameFrom[current[0]][current[1]];


    }
    // namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    // imshow( "Display window", image );                   // Show our image inside it.
    // waitKey(0);  // Wait for a keystroke in the window
}


vector<int> left(Mat image, vector<int> point) {
    int x = point[0];
    if (x - 1 >= 0 && !isWall(image, x - 1, point[1])) {
        point[0]= x - 1;
        return point;
    }
    return vector<int>();
}
vector<int> right(Mat image, vector<int> point) {
    int x = point[0];
    if (x + 1 < image.rows && !isWall(image, x + 1, point[1])) {
        point[0] = x + 1;
        return point;
    }
    return vector<int>();
}
vector<int> up(Mat image, vector<int> point) {
    int y = point[1];
    if (y - 1 >= 0 && !isWall(image, point[0], y - 1)) {
        point[1] = y - 1;
        return point;
    }
    return vector<int>();
}
vector<int> down(Mat image, vector<int> point) {
    int y = point[1];
    if (y + 1 < image.cols && !isWall(image, point[0], y + 1)) {
        point[1] = y + 1;
        return point;
    }
    return vector<int>();
}
bool isWall(Mat image, int x, int y) {
    if (image.at<Vec3b>(x, y)[0] == 0) {
        // cout << "Detected wall at: " << x << " " << y << endl;
        return true;
    }  
    return false;
}
int getNextNode(vector<vector<int>> fScore,  vector<vector<int>> openSet) {
    int nextNode = 0;
    int x = openSet[0][0];
    int y = openSet[0][1];

    for (int i = 0; i < int(openSet.size()); i ++) {
        vector<int> node = openSet[i];
        if (fScore[x][y] > fScore[node[0]][node[1]]) {
            nextNode = i;
            x = openSet[i][0];
            y = openSet[i][1];
        }
    }
    return nextNode;
}


float heuristicDist(vector<int> space, vector<int> goal) {
    return sqrt(pow(goal[0] - space[0], 2) + pow(goal[1] - space[1], 2));
}
