#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
#include <iostream>
#define ROWS 400
#define COLS 640
// #define ROWS 600
// #define COLS 1000
#define length 10
#define breadth 10

using namespace std;
using namespace cv;
// Mat image = imread("aifa_assign2/maze.jpg", 1);
Mat image = imread("Test1.png", 1);
Mat img(image.rows, image.cols, CV_8UC1, Scalar(0));
Mat imgdil(image.rows, image.cols, CV_8UC1, Scalar(0));

//struct to store all info of the node
struct Nodes
{
    int row;
    int col;
    int parent_r;
    int parent_c;
    float heu; //for storing cost + heuristic
};
Nodes node[ROWS][COLS], current;

//Min heap for heuristic values and storing nodes in priority queue
struct myComparator
{
    bool operator()(Nodes const &n1, Nodes const &n2)
    {
        return n1.heu > n2.heu;
    }
};

//Cost function
float cost(int s0, int s1, int i0, int j0, int i1, int j1, int pr, int pc)
{
    return (sqrt(pow(s0 - i1, 2) + pow(s1 - j1, 2)) * 0.001);
}

//Heuristic
float heuristic(int s0, int s1, int i0, int j0, int i1, int j1, int pr, int pc)
{
    float heuristic, D = 1, D2 = sqrt(2);
    int dx = abs(i1 - i0);
    int dy = abs(j1 - j0);
    heuristic = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
    return (heuristic * 0 + cost(s0, s1, i0, j0, i1, j1, pr, pc)); //Dijkstra
}

// //Printing elements of priority queue
// void showpq(priority_queue <Nodes, vector<Nodes>, myComparator> gq)
// {
//     priority_queue <Nodes, vector<Nodes>, myComparator> g = gq;
//     while (!g.empty())
//     {
//         cout << g.top().row<<"|"<<g.top().col<<"|"<<g.top().heu<<"\n";
//         g.pop();
//     }
//     cout << '\n';
// }

int main()
{

    priority_queue<Nodes, vector<Nodes>, myComparator> open;
    stack<Nodes> closed, dr; //LIFO
    int end[2] = {0}, start[2] = {0};
    int stsum = 0, endsum = 0, cs = 0, ce = 0;
    bool isVisited[ROWS][COLS];
    memset(isVisited, false, sizeof(isVisited));

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {

            if (image.at<Vec3b>(i, j)[0] == 0 && image.at<Vec3b>(i, j)[1] == 255 && image.at<Vec3b>(i, j)[2] == 0)
            {
                //Initializing starting node and pushing its heuristic in open list
                start[0] += i;
                start[1] += j;
                cs++;
            }
            else if (image.at<Vec3b>(i, j)[0] == 0 && image.at<Vec3b>(i, j)[1] == 0 && image.at<Vec3b>(i, j)[2] == 255)
            {
                //Initializing end node
                end[0] += i;
                end[1] += j;
                ce++;
            }

            //Setting all distances to infinity at first
            node[i][j].row = i;
            node[i][j].col = j;
            node[i][j].heu = FLT_MAX;
            node[i][j].parent_r = -1;
            node[i][j].parent_c = -1;

            //Creating a binary equivalent of nodes for simplicity
            if (image.at<Vec3b>(i, j)[0] >= 127 && image.at<Vec3b>(i, j)[1] >= 127 && image.at<Vec3b>(i, j)[2] >= 127)
                img.at<uchar>(i, j) = 255;
            else
                img.at<uchar>(i, j) = 0;
        }
    }

    //Applying dilation to the binary image
    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            int c = 0, k = max(length / 2, breadth / 2);
            for (int x = i - k; x <= i + k && x < img.rows && x >= 0; x++)
            {
                for (int y = j - k; y <= j + k && y < img.cols && y >= 0; y++)
                {
                    if (img.at<uchar>(x, y) == 255)
                    {
                        c++;
                    }
                }
            }
            if (c > 0)
            {
                imgdil.at<uchar>(i, j) = 255;
            }
            else
            {
                imgdil.at<uchar>(i, j) = img.at<uchar>(i, j);
            }
        }
    }

    start[0] /= cs;
    start[1] /= cs;
    printf("Start node initialized at (%d, %d)\n", start[0], start[1]);
    node[start[0]][start[1]].row = start[0];
    node[start[0]][start[1]].col = start[1];
    node[start[0]][start[1]].parent_r = start[2];
    node[start[0]][start[1]].parent_c = start[1];
    node[start[0]][start[1]].heu = 0;
    open.push(node[start[0]][start[1]]);

    end[0] /= ce;
    end[1] /= ce;
    printf("Destination node initialized at (%d, %d)\n", end[0], end[1]);
    node[end[0]][end[1]].row = end[0];
    node[end[0]][end[1]].col = end[1];
    node[end[0]][end[1]].parent_r = -1; //to be set later
    node[end[0]][end[1]].parent_c = -1;
    node[end[0]][end[1]].heu = 0;
    /*-----------------------------------------------------------A-star Algorithm----------------------------------------------------------------*/
    bool foundDest = false;
    while (!open.empty() && !foundDest)
    {
        current = open.top();
        closed.push(current);
        open.pop();
        isVisited[current.row][current.col] = true;
        //Checking neighbours of the current node
        for (int x = current.row - 1; x <= current.row + 1 && x < image.rows && x >= 0; x++)
        {
            for (int y = current.col - 1; y <= current.col + 1 && y < image.cols && y >= 0; y++)
            {

                if (x != current.row || y != current.col && isVisited[x][y] == false)
                {

                    if (x == end[0] && y == end[1])
                    {
                        foundDest = true;
                        node[x][y].parent_r = current.row;
                        node[x][y].parent_c = current.col;
                        closed.push(node[x][y]);
                        isVisited[x][y] = true;
                        cout << "Destination reached" << endl;
                        break;
                    }
                    if (imgdil.at<uchar>(x, y) == 255)
                    {
                        continue;
                    }
                    //Push/Update condition
                    if (heuristic(start[0], start[1], end[0], end[1], x, y, current.row, current.col) < node[x][y].heu)
                    {
                        node[x][y].row = x;
                        node[x][y].col = y;
                        node[x][y].heu = heuristic(start[0], start[1], end[0], end[1], x, y, current.row, current.col);
                        node[x][y].parent_r = current.row;
                        node[x][y].parent_c = current.col;
                        open.push(node[x][y]);
                    }
                }
            }
            if (foundDest)
                break;
        }
        if (!foundDest && open.empty() && closed.top().row != end[0] && closed.top().col != end[1])
            cout << "Failed!"
                 << "\n";
    }
    cout << closed.top().row << "|" << closed.top().col << "\n";
    /*--------------------------------------------------------------------------------------------------------------------------------------------*/

    //Storing path points
    dr.push(closed.top());
    while (1)
    {
        int temp_r = dr.top().parent_r;
        int temp_c = dr.top().parent_c;
        dr.push(node[temp_r][temp_c]);
        if (temp_r == start[0] && temp_c == start[1])
        {
            cout << "Over\n";
            break;
        }
    }

    //Tracing out the path
    namedWindow("Win", WINDOW_NORMAL);
    while (!dr.empty())
    {
        Mat image1 = image.clone();
        for (int i = dr.top().row - breadth / 2; i <= dr.top().row + breadth / 2; i++)
        {
            image1.at<Vec3b>(i, dr.top().col - length / 2)[0] = 255;
            image1.at<Vec3b>(i, dr.top().col - length / 2)[1] = 255;
            image1.at<Vec3b>(i, dr.top().col - length / 2)[2] = 0;
            image1.at<Vec3b>(i, dr.top().col + length / 2)[0] = 255;
            image1.at<Vec3b>(i, dr.top().col + length / 2)[1] = 255;
            image1.at<Vec3b>(i, dr.top().col + length / 2)[2] = 0;
        }
        for (int j = dr.top().col - length / 2; j <= dr.top().col + length / 2; j++)
        {
            image1.at<Vec3b>(dr.top().row - breadth / 2, j)[0] = 255;
            image1.at<Vec3b>(dr.top().row - breadth / 2, j)[1] = 255;
            image1.at<Vec3b>(dr.top().row - breadth / 2, j)[2] = 0;
            image1.at<Vec3b>(dr.top().row + breadth / 2, j)[0] = 255;
            image1.at<Vec3b>(dr.top().row + breadth / 2, j)[1] = 255;
            image1.at<Vec3b>(dr.top().row + breadth / 2, j)[2] = 0;
        }
        image.at<Vec3b>(dr.top().row, dr.top().col)[0] = 255;
        image.at<Vec3b>(dr.top().row, dr.top().col)[1] = 255;
        image.at<Vec3b>(dr.top().row, dr.top().col)[2] = 0;
        imshow("Win", image1);

        // imgdil.at<uchar>(dr.top().row,dr.top().col)=127;
        // imshow("Win",imgdil);

        waitKey(5);
        dr.pop();
    }
    waitKey(0);
}
