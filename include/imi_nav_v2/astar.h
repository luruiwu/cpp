//
// Created on 18-10-23.
//

#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <vector>
#include "Common.h"
#include <nav_msgs/OccupancyGrid.h>

struct ANode
{
  int x;
  int y;
  float G;
  float H;
  float F;
  int idx;
  int fidx;
};

class AStar
{
public:
  static ANode popNode2D(std::vector<ANode> &NodeList){
    float value = 10000;
    int idx = 0;
    for(int i=0;i<NodeList.size();++i){
      float f=NodeList[i].G+NodeList[i].H;
      if(f<value){
        value = f;
        idx = i;
      }
    }
    ANode node = NodeList[idx];
    NodeList.erase(NodeList.begin()+idx);
    return node;
  }

  static bool calAStarPath(const  nav_msgs::OccupancyGrid &m_map, const SPose &start , const SPose &end , double &path_cost /*, vector<Node> &path*/){
    int width=m_map.info.width;
    int height=m_map.info.height;
    if(start.x<0 || start.x>=width || start.y<0 ||start.y>=height || end.x<0 || end.x>=width || end.y<0 ||end.y>=height)
    {
      return false;
    }
    if(m_map.data[MAP_IDX(m_map.info.width,start.x,start.y)]>0 || m_map.data[MAP_IDX(m_map.info.width,end.x,end.y)]>0)
    {
      return false;
    }
    int directions[8][2] = { { -1, 1 },{ 0, 1 },{ 1, 1 },{ -1, 0 },{ 1, 0 },{ -1, -1 },{ 0, -1 },{ 1, -1 } };
    int *isVisited =new int[height*width];
    for(int i=0;i<width*height;i++)
        isVisited[i]=0;
    ANode *nodes=new ANode[height*width];
    ANode startNode,endNode;
    startNode.x=start.x;
    startNode.y=start.y;
    startNode.G=0;
    startNode.H=abs(startNode.x-end.x)+abs(startNode.y-end.y);
    startNode.F=startNode.G+startNode.H;
    startNode.idx= startNode.y*width+startNode.x;
    startNode.fidx=-1;
    endNode.x=end.x;
    endNode.y=end.y;
    ANode nPred,nChild;
    int iPred,iChild;
    std::vector<ANode> OpenList,CloseList;
    OpenList.push_back(startNode);
    isVisited[startNode.idx]=1;
    nodes[startNode.idx]=startNode;
    while(!OpenList.empty()){
      nPred=popNode2D(OpenList);
      iPred=nPred.idx;
      if(nPred.x==end.x && nPred.y==end.y){
        //the whole path found
        path_cost=nPred.F;
//        printf("path_cost=%f",path_cost);

//        //get the path
//        Node tempNode=nPred;
//        while(tempNode.fidx>-1){
//          path.push_back(tempNode);
//          tempNode=nodes[tempNode.fidx];
//        }
//        path.push_back(tempNode);
//        int start=0;
//        int end=path.size()-1;
//        while (start>end) {
//          swap(path[start],path[end]);
//          ++start;
//          --end;
//        }
        delete[]isVisited;
        delete[]nodes;
        return true;
      }
      isVisited[iPred]=2;
      CloseList.push_back(nPred);
      for(char i=0;i<8;++i){
        int xChild=nPred.x+directions[i][0];
        int yChild=nPred.y+directions[i][1];
        iChild=yChild*width+xChild;
        if(xChild>=0 && xChild<width && yChild>=0 && yChild<height && m_map.data[MAP_IDX(m_map.info.width,xChild,yChild)]<1 && isVisited[iChild]!=2){
          nChild.x=xChild;
          nChild.y=yChild;
          nChild.idx=iChild;
          nChild.fidx=iPred;
          nChild.G=nPred.G+sqrt(pow(xChild-nPred.x,2)+pow(yChild-nPred.y,2));
          nChild.H=abs(nChild.x-endNode.x)+abs(nChild.y-endNode.y);
          nChild.F=nChild.G+nChild.H;
          if(isVisited[iChild]==0){
            isVisited[iChild]=1;
            nodes[iChild]=nChild;
            OpenList.push_back(nChild);
          }else if (isVisited[iChild]==1 && nChild.F+0.0001<nodes[iChild].F) {
            nodes[iChild]=nChild;
            for(int i=0;i<OpenList.size();++i){
              if(OpenList[i].x==nChild.x && OpenList[i].y==nChild.y){
                OpenList[i]=nChild;
                break;
              }
            }
          }
        }
      }
    }
    delete[]isVisited;
    delete[]nodes;
    return false;
  }
};


#endif // ASTAR_H
