#ifndef TSP_H
#define TSP_H

#include <vector>
#include <stdlib.h>

class TSP{
  public:
  static std::vector<std::vector<int> > cal_tsp(const double *a,const int &n){
    int count=1<<(n-1);
    double dp[n][count];
//    double** dp=(double**) malloc(n*sizeof(double*)) ;
    for(int i=0;i<n;++i){
//      dp[i]=(double*)malloc(count*sizeof(double));
      for(int j=0;j<count;++j)
        dp[i][j]=0x7ffff;
    }
    for(int i=0;i<n;++i)
      dp[i][0]=a[i*n];

    std::vector<std::vector<int> > path(count*n);
    for(int i=0;i<n;++i){
      path[i].push_back(i);
    }
    for(int j = 1;j < count;j++){
      for(int  i= 0;i < n;i++){
        if(((j >> (i - 1)) & 1) == 1){
          continue;
        }
        for(int k = 1;k < n;k++){
          if(((j >> (k - 1)) & 1) == 0){
            continue;
          }
          if(dp[i][j] > a[i*n+k] + dp[k][j ^ (1 << (k - 1))]){
            dp[i][j] = a[i*n+k] + dp[k][j ^ (1 << (k - 1))];
            path[j*n+i].clear();
            path[j*n+i].push_back(i);
            int l=j^(1<<(k-1));
            std::vector<int> path_kl=path[l*n+k];
            for(int q=0;q<path_kl.size();++q){
              path[j*n+i].push_back(path_kl[q]);
            }
          }
        }
      }
    }
    return path;
  }
};
#endif // TSP_H
